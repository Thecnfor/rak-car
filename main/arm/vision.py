"""main/arm/vision.py —— 机械臂视觉伺服客户端（详见 VISION_SERVO_DESIGN.md）。

Layer 2 的核心：Detection / TargetSelector / ArmVisionClient。
不动 runtime 一行代码 —— 所有硬件动作走 ArmClient 注入。
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from .labels import Label, LabelInfo, LABELS, LABEL_GROUPS  # noqa: F401


@dataclass(frozen=True)
class BBoxNorm:
    x_center: float
    y_center: float
    width: float
    height: float

    @property
    def is_centered(self) -> bool:
        return self.is_centered_at(0.05)

    def is_centered_at(self, tol: float) -> bool:
        return abs(self.x_center) <= tol and abs(self.y_center) <= tol


@dataclass(frozen=True)
class BBoxPixels:
    x1: int
    y1: int
    x2: int
    y2: int
    width: int
    height: int


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    track_id: Optional[int]
    class_id: Optional[int]
    bbox_norm: BBoxNorm
    bbox_pixels: Optional[BBoxPixels]
    fetched_at: float

    def __repr__(self) -> str:
        return (
            f"Detection({self.label}#{self.track_id} "
            f"score={self.score:.2f} cx={self.bbox_norm.x_center:+.2f})"
        )


def _parse_cache(raw: Dict[str, Any]) -> List[Detection]:
    """GET /v1/realtime/vision/task → List[Detection]（无 bbox_pixels）

    缓存字段命名约定（runtime 实际返回）：
      - det_id（cache 字段） / track_id（sync 字段） 都视作 track_id
      - cls_id / class_id 都视作 class_id
    """
    state = raw.get("task_state") or {}
    dets = state.get("detections") or []
    now = float(state.get("updated_at") or time.time())
    out: List[Detection] = []
    for d in dets:
        bn = d.get("bbox_norm") or {}
        out.append(Detection(
            label=str(d["label"]),
            score=float(d["score"]),
            track_id=d.get("det_id") or d.get("track_id"),
            class_id=d.get("cls_id") or d.get("class_id"),
            bbox_norm=BBoxNorm(
                float(bn["x_center"]), float(bn["y_center"]),
                float(bn.get("width", 0.0)), float(bn.get("height", 0.0)),
            ),
            bbox_pixels=None,
            fetched_at=now,
        ))
    return out


def _parse_sync(raw: Dict[str, Any]) -> List[Detection]:
    """POST /v1/vision/task → List[Detection]（含 bbox_pixels）

    同步字段命名：track_id / class_id / bbox_norm / bbox_pixels
    """
    dets = raw.get("detections") or []
    now = time.time()
    out: List[Detection] = []
    for d in dets:
        bn = d.get("bbox_norm") or {}
        bp = d.get("bbox_pixels") or None
        out.append(Detection(
            label=str(d["label"]),
            score=float(d["score"]),
            track_id=d.get("track_id"),
            class_id=d.get("class_id"),
            bbox_norm=BBoxNorm(
                float(bn["x_center"]), float(bn["y_center"]),
                float(bn.get("width", 0.0)), float(bn.get("height", 0.0)),
            ),
            bbox_pixels=BBoxPixels(
                int(bp["x1"]), int(bp["y1"]), int(bp["x2"]), int(bp["y2"]),
                int(bp["width"]), int(bp["height"]),
            ) if bp else None,
            fetched_at=now,
        ))
    return out


# ===== 选择策略 + 多目标选择器 =====


class SelectionStrategy(str, Enum):
    HIGHEST_SCORE      = "highest_score"
    CLOSEST_TO_CENTER  = "closest_to_center"
    LARGEST            = "largest"
    LEFTMOST           = "leftmost"
    RIGHTMOST          = "rightmost"
    TOPMOST            = "topmost"
    BOTTOMMOST         = "bottommost"
    LOCK_FIRST_SEEN    = "lock_first_seen"   # 首帧锁定 track_id


@dataclass(frozen=True)
class TargetSelector:
    """多目标选择器：label/track_id/group 过滤 + strategy 排序。"""
    label: Optional[str] = None
    track_id: Optional[int] = None
    strategy: str = SelectionStrategy.HIGHEST_SCORE.value
    group: Optional[str] = None

    @classmethod
    def for_label(cls, label, *, strategy: str = SelectionStrategy.HIGHEST_SCORE.value) -> "TargetSelector":
        return cls(
            label=str(label.value if isinstance(label, Label) else label),
            strategy=strategy,
        )

    @classmethod
    def for_group(cls, group: str, *, strategy: str = SelectionStrategy.HIGHEST_SCORE.value) -> "TargetSelector":
        if group not in LABEL_GROUPS:
            raise ValueError(f"未知 group: {group!r}（{list(LABEL_GROUPS)}）")
        return cls(label=None, strategy=strategy, group=group)

    def matches(self, det: Detection) -> bool:
        if self.group is not None:
            return det.label in [l.value for l in LABEL_GROUPS[self.group]]
        if self.label is not None:
            return det.label == self.label
        return True

    def apply_strategy(self, candidates: List[Detection]) -> Optional[Detection]:
        if not candidates:
            return None
        s = self.strategy
        if s == SelectionStrategy.HIGHEST_SCORE.value:
            return max(candidates, key=lambda d: d.score)
        if s == SelectionStrategy.CLOSEST_TO_CENTER.value:
            return min(candidates, key=lambda d: abs(d.bbox_norm.x_center) + abs(d.bbox_norm.y_center))
        if s == SelectionStrategy.LARGEST.value:
            return max(candidates, key=lambda d: d.bbox_norm.width * d.bbox_norm.height)
        if s == SelectionStrategy.LEFTMOST.value:
            return min(candidates, key=lambda d: d.bbox_norm.x_center)
        if s == SelectionStrategy.RIGHTMOST.value:
            return max(candidates, key=lambda d: d.bbox_norm.x_center)
        if s == SelectionStrategy.TOPMOST.value:
            return min(candidates, key=lambda d: d.bbox_norm.y_center)
        if s == SelectionStrategy.BOTTOMMOST.value:
            return max(candidates, key=lambda d: d.bbox_norm.y_center)
        # LOCK_FIRST_SEEN 由 find_target 循环内处理（首帧锁定 track_id）
        return candidates[0]