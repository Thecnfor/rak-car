"""main/arm/vision.py —— 机械臂视觉伺服客户端（详见 VISION_SERVO_DESIGN.md）。

Layer 2 的核心：Detection / TargetSelector / ArmVisionClient。
不动 runtime 一行代码 —— 所有硬件动作走 ArmClient 注入。
"""
from __future__ import annotations

import logging
import time
import dataclasses
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

from .labels import Label, LabelInfo, LABELS, LABEL_GROUPS  # noqa: F401

logger = logging.getLogger(__name__)


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
    """GET /v1/realtime/vision/task 或 WS subscribe_task_detection → List[Detection]（无 bbox_pixels）

    兼容两种 wrapper key：
      - HTTP cache 用 "task_state"
      - WS push 用 "data"（routes.py:1437）
    字段命名约定（runtime 实际返回）：
      - det_id（cache 字段） / track_id（sync 字段） 都视作 track_id
      - cls_id / class_id 都视作 class_id
    """
    state = raw.get("task_state") or raw.get("data") or raw
    # 第三种情况：state 直接发（无 wrapper）—— 用 "detections" 字段识别
    if "detections" not in state:
        state = raw
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


# ===== 视觉伺服循环 =====


@dataclass(frozen=True)
class ServoTrace:
    t_s: float
    iteration: int
    dx_norm: float
    dy_norm: float
    x_mm: float
    y_mm: float
    score: float
    selected_track_id: Optional[int]
    is_miss: bool = False           # 2026-07-31: 区分 hit / miss，调试连续性


@dataclass(frozen=True)
class ServoResult:
    converged: bool
    selector: TargetSelector
    x_mm: float
    y_mm: float
    confidence: float
    iterations: int
    elapsed_s: float
    final_detection: Optional[Detection]
    trace: Tuple[ServoTrace, ...]


class ArmVisionClient:
    """末端摄像头（side cam）视觉伺服客户端。

    主路径走 task_feed 30Hz 缓存（GET /v1/realtime/vision/task）；
    一次快照走 POST /v1/vision/task（含 bbox_pixels）。

    不动 runtime 一行代码 —— 所有硬件动作由 ArmClient 通过 move_fn 注入。
    """

    def __init__(self, http, *, default_timeout_s: float = 10.0):
        self.http = http
        self.default_timeout_s = default_timeout_s

    @staticmethod
    def labels() -> Tuple[LabelInfo, ...]:
        return LABELS

    @staticmethod
    def group(name: str) -> Tuple[Label, ...]:
        return LABEL_GROUPS[name]

    def get_state(self) -> List[Detection]:
        return _parse_cache(self.http.get_vision_task_cache())

    def get_state_filtered(self, selector: TargetSelector) -> List[Detection]:
        return [d for d in self.get_state() if selector.matches(d)]

    def snap(self, *, sort_pos=(0.0, 0.0), limit_x: float = 1.0,
             limit_y: float = 1.0, timeout: float = 20.0) -> List[Detection]:
        return _parse_sync(self.http.request_vision_task(
            sort_pos=sort_pos, limit_x=limit_x, limit_y=limit_y, timeout=timeout))

    def find_target(self, selector: TargetSelector, *,
                    x_mm: float, y_mm: float,
                    mm_per_norm: float = 30.0,
                    settle_tol_norm: float = 0.05,
                    min_step_mm: float = 1.0,
                    max_iter: int = 500,
                    timeout: float = 10.0,
                    on_missing_track: str = "abort",
                    move_fn: Optional[Callable[[float, float], dict]] = None) -> ServoResult:
        """视觉伺服主路径。

        循环：读缓存 → 应用 selector（label/group/track_id）→ 收敛判断 → dead-band
        → 通过 move_fn 下发 x/y 位移（默认走 http.execute_arm_action('goto_position')）。

        Args:
            selector: 多目标选择器（见 TargetSelector）
            x_mm, y_mm: 起始位姿
            mm_per_norm: bbox 归一化坐标 → mm 的转换系数
            settle_tol_norm: 收敛阈值（|dx|<tol AND |dy|<tol）
            min_step_mm: dead-band，避免抖动
            max_iter: 最大迭代次数（兜底）
            timeout: 总超时（秒）
            on_missing_track: 目标丢失行为（"abort"=5 帧无检测就 raise；"wait"=继续等）
            move_fn: 自定义 move 函数；默认走 arm.goto_position

        Returns:
            ServoResult(converged, x_mm, y_mm, confidence, iterations, elapsed_s, ...)

        Raises:
            RuntimeError: 连续 5 帧未检测到（on_missing_track="abort"）
            ValueError: y_mm 越界（move_fn 内部 raise）
        """
        t0 = time.time()
        trace: List[ServoTrace] = []
        locked_track_id: Optional[int] = None
        consecutive_misses = 0
        last_x_mm, last_y_mm = x_mm, y_mm
        last_detection: Optional[Detection] = None
        current_selector = selector

        for i in range(max_iter):
            if time.time() - t0 > timeout:
                break

            candidates = self.get_state_filtered(current_selector)

            if current_selector.strategy == SelectionStrategy.LOCK_FIRST_SEEN.value:
                if locked_track_id is None:
                    pick = current_selector.apply_strategy(candidates)
                    if pick is None:
                        consecutive_misses += 1
                        if consecutive_misses >= 5 and on_missing_track == "abort":
                            raise RuntimeError(
                                f"find_target: 首帧未检测到 {current_selector}"
                            )
                        continue
                    locked_track_id = pick.track_id
                    current_selector = dataclasses.replace(
                        current_selector, track_id=locked_track_id
                    )
                candidates = [d for d in candidates if d.track_id == locked_track_id]
            elif current_selector.track_id is not None:
                candidates = [d for d in candidates if d.track_id == current_selector.track_id]

            pick = current_selector.apply_strategy(candidates) if candidates else None
            if pick is None:
                consecutive_misses += 1
                trace.append(ServoTrace(
                    t_s=time.time() - t0, iteration=i,
                    dx_norm=0.0, dy_norm=0.0,
                    x_mm=last_x_mm, y_mm=last_y_mm,
                    score=0.0, selected_track_id=None,
                    is_miss=True,
                ))
                if on_missing_track == "abort" and consecutive_misses >= 5:
                    raise RuntimeError(
                        f"find_target: 连续 {consecutive_misses} 帧未检测到 {current_selector}"
                    )
                continue
            consecutive_misses = 0
            last_detection = pick

            dx_norm, dy_norm = pick.bbox_norm.x_center, pick.bbox_norm.y_center
            if pick.bbox_norm.is_centered_at(settle_tol_norm):
                trace.append(ServoTrace(
                    t_s=time.time() - t0, iteration=i,
                    dx_norm=dx_norm, dy_norm=dy_norm,
                    x_mm=last_x_mm, y_mm=last_y_mm,
                    score=pick.score, selected_track_id=pick.track_id))
                return ServoResult(
                    converged=True, selector=current_selector,
                    x_mm=last_x_mm, y_mm=last_y_mm,
                    confidence=pick.score, iterations=i + 1,
                    elapsed_s=time.time() - t0,
                    final_detection=pick, trace=tuple(trace))

            dx_mm = -dx_norm * mm_per_norm
            dy_mm = -dy_norm * mm_per_norm
            if abs(dx_mm) < min_step_mm:
                dx_mm = 0.0
            if abs(dy_mm) < min_step_mm:
                dy_mm = 0.0

            new_x_mm = last_x_mm + dx_mm
            new_y_mm = last_y_mm + dy_mm
            trace.append(ServoTrace(
                t_s=time.time() - t0, iteration=i,
                dx_norm=dx_norm, dy_norm=dy_norm,
                x_mm=new_x_mm, y_mm=new_y_mm,
                score=pick.score, selected_track_id=pick.track_id))

            if move_fn is not None:
                move_fn(new_x_mm, new_y_mm)
            else:
                self.http.execute_arm_action(
                    "goto_position",
                    x=new_x_mm / 1000.0, y=new_y_mm / 1000.0,
                    timeout=5.0, sync=True,
                )
            last_x_mm, last_y_mm = new_x_mm, new_y_mm

        return ServoResult(
            converged=False, selector=current_selector,
            x_mm=last_x_mm, y_mm=last_y_mm,
            confidence=last_detection.score if last_detection else 0.0,
            iterations=max_iter, elapsed_s=time.time() - t0,
            final_detection=last_detection, trace=tuple(trace))

    def find_targets_sequence(self, selectors: List[TargetSelector], *,
                              x_mm: float, y_mm: float, **kwargs) -> List[ServoResult]:
        """按顺序对每个 selector 调 find_target；返回 list of ServoResult"""
        return [self.find_target(sel, x_mm=x_mm, y_mm=y_mm, **kwargs) for sel in selectors]

    def pick_one(self, selectors: List[TargetSelector], *,
                 x_mm: float, y_mm: float, **kwargs) -> Optional[ServoResult]:
        """按优先级找第一个能匹配的目标并伺服；返回首个成功的 ServoResult 或 None"""
        for sel in selectors:
            try:
                result = self.find_target(sel, x_mm=x_mm, y_mm=y_mm, **kwargs)
                if result.converged:
                    return result
            except RuntimeError:
                continue
        return None

    # ===== 实时（WS push）路径 =====

    def find_target_realtime(self, selector: TargetSelector, *,
                             x_mm: float, y_mm: float,
                             hz: float = 30.0,
                             mm_per_norm: float = 30.0,
                             settle_tol_norm: float = 0.05,
                             min_step_mm: float = 1.0,
                             timeout: float = 10.0,
                             on_missing_track: str = "abort",
                             move_fn: Optional[Callable[[float, float], dict]] = None,
                             ws=None) -> ServoResult:
        """视觉伺服实时版本：用 WS subscribe_task_detection 推流（替代 HTTP 轮询）。

        与 find_target 行为一致，但检测来源是 WS 推送（task_feed 30Hz），
        延迟更低、多客户端不重复消费。

        Args:
            selector: 多目标选择器
            x_mm, y_mm: 起始位姿
            hz: WS 推送频率上限（默认 30.0）
            其他参数同 find_target
            ws: 注入 ws client（默认新建 RuntimeWsClient）

        Returns:
            ServoResult

        2026-07-31 修正（security review）：
        - 去 max_iter（误导；唯一兜底是 timeout）
        - 收敛/abort 检查挪进 WS callback（threading.Event 即时唤醒主线程）
        """
        import threading

        if ws is None:
            try:
                from main.ws_client import RuntimeWsClient
            except ImportError:  # pragma: no cover
                from ws_client import RuntimeWsClient  # type: ignore
            ws = RuntimeWsClient()

        stop_event = threading.Event()
        abort_reason: dict = {"reason": None}    # "converged" / "abort" / None

        state = {
            "x_mm": x_mm, "y_mm": y_mm,
            "last_detection": None,
            "consecutive_misses": 0,
            "locked_track_id": None,
            "current_selector": selector,
            "last_updated_at": None,
        }

        def _on_push(raw: dict) -> None:
            """WS 推送回调 —— 单帧处理；触发收敛/abort 时立即 set stop_event。"""
            if stop_event.is_set():
                return
            ts = raw.get("task_state") or {}
            updated_at = ts.get("updated_at")
            if updated_at is not None and updated_at == state["last_updated_at"]:
                return    # 同一帧 skip
            state["last_updated_at"] = updated_at

            try:
                dets = _parse_cache(raw)
            except Exception:
                return

            cur_sel = state["current_selector"]
            candidates = [d for d in dets if cur_sel.matches(d)]

            if cur_sel.strategy == SelectionStrategy.LOCK_FIRST_SEEN.value:
                if state["locked_track_id"] is None:
                    pick = cur_sel.apply_strategy(candidates)
                    if pick is None:
                        state["consecutive_misses"] += 1
                        if on_missing_track == "abort" and state["consecutive_misses"] >= 5:
                            abort_reason["reason"] = "miss_abort"
                            stop_event.set()
                        return
                    state["locked_track_id"] = pick.track_id
                    cur_sel = dataclasses.replace(cur_sel, track_id=pick.track_id)
                    state["current_selector"] = cur_sel
                candidates = [d for d in candidates if d.track_id == state["locked_track_id"]]
            elif cur_sel.track_id is not None:
                candidates = [d for d in candidates if d.track_id == cur_sel.track_id]

            pick = cur_sel.apply_strategy(candidates) if candidates else None
            if pick is None:
                state["consecutive_misses"] += 1
                if on_missing_track == "abort" and state["consecutive_misses"] >= 5:
                    abort_reason["reason"] = "miss_abort"
                    stop_event.set()
                return
            state["consecutive_misses"] = 0
            state["last_detection"] = pick

            dx_norm, dy_norm = pick.bbox_norm.x_center, pick.bbox_norm.y_center
            if pick.bbox_norm.is_centered_at(settle_tol_norm):
                abort_reason["reason"] = "converged"
                stop_event.set()
                return

            dx_mm = -dx_norm * mm_per_norm
            dy_mm = -dy_norm * mm_per_norm
            if abs(dx_mm) < min_step_mm:
                dx_mm = 0.0
            if abs(dy_mm) < min_step_mm:
                dy_mm = 0.0

            new_x = state["x_mm"] + dx_mm
            new_y = state["y_mm"] + dy_mm
            if move_fn is not None:
                move_fn(new_x, new_y)
            else:
                self.http.execute_arm_action(
                    "goto_position",
                    x=new_x / 1000.0, y=new_y / 1000.0,
                    timeout=5.0, sync=True,
                )
            state["x_mm"], state["y_mm"] = new_x, new_y

        stop = ws.subscribe_task_detection(_on_push, hz=hz)

        t0 = time.time()
        try:
            # 用 threading.Event.wait(timeout) 替代轮询：callback set 后立即唤醒
            stop_event.wait(timeout=timeout)
            elapsed = time.time() - t0
        finally:
            try:
                stop()    # RuntimeWsClient.subscribe_* 返回 callable
            except Exception:
                pass

        last = state["last_detection"]
        # iterations 估算：用 elapsed / 0.033（30Hz）做粗算（仅供参考，非硬上限）
        approx_iter = max(1, int(elapsed * 30.0))

        if abort_reason["reason"] == "miss_abort":
            raise RuntimeError(
                f"find_target_realtime: 连续 {state['consecutive_misses']} 帧未检测到 {selector}"
            )
        if abort_reason["reason"] == "converged" and last is not None:
            return ServoResult(
                converged=True, selector=selector,
                x_mm=state["x_mm"], y_mm=state["y_mm"],
                confidence=last.score, iterations=approx_iter,
                elapsed_s=elapsed,
                final_detection=last, trace=(),
            )
        return ServoResult(
            converged=False, selector=selector,
            x_mm=state["x_mm"], y_mm=state["y_mm"],
            confidence=last.score if last else 0.0,
            iterations=approx_iter, elapsed_s=elapsed,
            final_detection=last, trace=(),
        )