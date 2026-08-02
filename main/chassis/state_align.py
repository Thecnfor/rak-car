"""main/chassis/state_align.py
视觉微调专用数据 shape。底盘组的视觉微调控制律只接 ``AlignState``，不接 dict。

约定：
- 数值型 None 含义：feed 未就绪 / 数据缺失
- target_found=False 时 area / area_error / y_center / age_ms 都视作 None 含义，
  控制律会自然输出零速，调用方无需 try/except。

构造入口：``AlignState.from_task_payload(payload, ref_area=...)`` 接受：
  - 裸 payload: ``{"detections":[...], "count":N, "updated_at":..., ...}``
  - ``None`` / 非 dict: 视为无目标,字段全 None / target_found=False（不抛异常）

注：保持与 ``state.py`` 同档 dataclass 风格（fresh / age_ms / active），便于上层
统一调用。``state.py`` 故意不并进来——这是新功能的数据,等稳定后再统一收口。
"""
import time
from dataclasses import dataclass
from typing import Optional, List, Dict, Any


def _age_ms_from(updated_at, now: Optional[float] = None) -> Optional[float]:
    """根据 updated_at 时间戳计算 age_ms。None / 非数字 → None。"""
    if not isinstance(updated_at, (int, float)):
        return None
    if now is None:
        now = time.time()
    return max(0.0, (float(now) - float(updated_at)) * 1000.0)


def _safe_float(v) -> Optional[float]:
    try:
        return float(v) if v is not None else None
    except Exception:
        return None


def _bbox_area(det: Dict[str, Any]) -> Optional[float]:
    """从 detection dict 算 bbox 面积（优先用 bbox_norm，归一化宽×高）。

    兼容两种来源：
      - ``bbox_norm = {"width": w, "height": h, ...}``（task / front 模型）
      - ``bbox_pixels = {"width": w, "height": h, ...}``（task 也带）
    """
    if not isinstance(det, dict):
        return None
    bn = det.get("bbox_norm")
    if isinstance(bn, dict):
        w = _safe_float(bn.get("width"))
        h = _safe_float(bn.get("height"))
        if w is not None and h is not None:
            return float(w) * float(h)
    bp = det.get("bbox_pixels")
    if isinstance(bp, dict):
        w = _safe_float(bp.get("width"))
        h = _safe_float(bp.get("height"))
        if w is not None and h is not None:
            return float(w) * float(h)
    return None


def _bbox_y_center(det: Dict[str, Any]) -> Optional[float]:
    """从 detection dict 读 y_center（图像纵向归一化坐标）。"""
    if not isinstance(det, dict):
        return None
    bn = det.get("bbox_norm")
    if isinstance(bn, dict):
        return _safe_float(bn.get("y_center"))
    return None


def select_target(
    detections: List[Dict[str, Any]],
    *,
    label: Optional[str] = None,
) -> Optional[Dict[str, Any]]:
    """从 task detections 列表里选一个目标。

    规则：
      - 传了 ``label`` → 优先取 label 完全匹配且 score 最高的那一个
      - 没传 ``label`` → 取 ``bbox_norm.width × height`` 面积最大的那一个
      - 没检测到 / 全空 → None
    """
    if not detections:
        return None
    if label is not None:
        matched = [d for d in detections if isinstance(d, dict) and d.get("label") == label]
        if matched:
            return max(matched, key=lambda d: float(d.get("score") or 0.0))
        # label 没命中 → 退化为按面积选（业务层可以靠 score 兜底,但 0 分目标不该选）
    candidates = [d for d in detections if isinstance(d, dict)]
    if not candidates:
        return None
    with_area = [(d, _bbox_area(d)) for d in candidates]
    with_area = [(d, a) for d, a in with_area if a is not None]
    if with_area:
        return max(with_area, key=lambda da: da[1])[0]
    # 都没面积 → 退回 score 最高的
    return max(candidates, key=lambda d: float(d.get("score") or 0.0))


@dataclass
class AlignState:
    """视觉微调外环状态：当前帧选中的目标 + ref_area 误差。

    字段语义：
      - target_found: 本帧是否拿到有效目标（detection 非空 + 面积可解析）
      - label: 选中目标的 label（label 优先模式才有）
      - score: 选中目标的 score
      - area: 当前 bbox 面积（bbox_norm.width × height）
      - ref_area: 调参时记录的"目标到达期望位置"时的面积（标度阶段记录）
      - area_error: ref_area - area；正值=目标比期望位置更远,负值=更近
      - y_center: 当前 bbox 归一化 y 中心（用于标度阶段验证,控制律不依赖）
      - age_ms: feed 距今毫秒数（None = 未刷过）
    """

    target_found: bool = False
    label: Optional[str] = None
    score: Optional[float] = None
    area: Optional[float] = None
    ref_area: Optional[float] = None
    area_error: Optional[float] = None
    y_center: Optional[float] = None
    age_ms: Optional[float] = None

    @classmethod
    def from_task_payload(
        cls,
        payload,
        *,
        ref_area: Optional[float] = None,
        label: Optional[str] = None,
        now: Optional[float] = None,
    ) -> "AlignState":
        """从 task_feed payload 构造一个 AlignState。

        参数:
          payload    - ``/v1/realtime/vision/task`` 返回的 dict 或 None
          ref_area   - 期望面积（调参标度后填入）；None 时 area_error 也为 None
          label      - 优先选这个 label；None 时按面积最大选
          now        - 测试用：覆盖 time.time()
        """
        inner: Dict[str, Any] = {}
        if isinstance(payload, dict):
            # 兼容裸 inner / {"task_state": {...}} 两种 shape
            ts = payload.get("task_state")
            if isinstance(ts, dict):
                inner = ts
            else:
                inner = payload
        detections = inner.get("detections") if isinstance(inner, dict) else None
        if not isinstance(detections, list):
            detections = []
        updated_at = inner.get("updated_at") if isinstance(inner, dict) else None
        chosen = select_target(detections, label=label)
        area = _bbox_area(chosen) if chosen is not None else None
        y_center = _bbox_y_center(chosen) if chosen is not None else None
        score = _safe_float(chosen.get("score")) if chosen is not None else None
        chosen_label = chosen.get("label") if chosen is not None else None
        if area is None or ref_area is None:
            area_error = None
        else:
            area_error = float(ref_area) - float(area)
        return cls(
            target_found=chosen is not None and area is not None,
            label=chosen_label,
            score=score,
            area=area,
            ref_area=ref_area,
            area_error=area_error,
            y_center=y_center,
            age_ms=_age_ms_from(updated_at, now=now),
        )

    @property
    def is_fresh(self) -> bool:
        """<500ms 视为新鲜（与 LaneState / IrState 同档）。"""
        return self.age_ms is not None and self.age_ms < 500.0

    @property
    def has_error(self) -> bool:
        """目标找到 + area_error 数值有效，控制律可以算 vx。"""
        return self.target_found and self.area_error is not None


__all__ = ["AlignState", "select_target"]