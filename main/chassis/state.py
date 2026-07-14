"""main/chassis/state.py
外环数据 shape。底盘组的控制律只接 dataclass，不接 dict。
"""
from dataclasses import dataclass
from typing import Optional, List


@dataclass
class LaneState:
    """lane 误差缓存视图，来自 runtime 的 /v1/vision/lane/state。"""

    error_y: Optional[float] = None
    error_angle: Optional[float] = None
    forward: Optional[float] = None
    lateral: Optional[float] = None
    angular: Optional[float] = None
    distance: Optional[float] = None
    mode: Optional[str] = None
    age_ms: Optional[float] = None

    @classmethod
    def from_lane_state_payload(cls, payload: dict, now: Optional[float] = None) -> "LaneState":
        import time as _time

        updated_at = payload.get("updated_at")
        if now is None:
            now = _time.time()
        age_ms = None
        if isinstance(updated_at, (int, float)):
            age_ms = max(0.0, (float(now) - float(updated_at)) * 1000.0)
        return cls(
            error_y=payload.get("error_y"),
            error_angle=payload.get("error_angle"),
            forward=payload.get("forward_speed"),
            lateral=payload.get("lateral_speed"),
            angular=payload.get("angular_speed"),
            distance=payload.get("distance"),
            mode=payload.get("mode"),
            age_ms=age_ms,
        )

    @property
    def is_fresh(self) -> bool:
        return self.age_ms is not None and self.age_ms < 500.0

    @property
    def has_error(self) -> bool:
        return self.error_y is not None and self.error_angle is not None


@dataclass
class OdometryState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    distance: float = 0.0


@dataclass
class WheelsState:
    """四轮线速度 + 编码器读数（弧度累计）。"""

    speeds: List[float]
    encoders: List[float]
