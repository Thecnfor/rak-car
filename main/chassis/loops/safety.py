"""main/chassis/loops/safety.py
外环兜底：lane_state 太久没刷（车端 lane feed 卡死或推理慢）就急停。
"""
from ..state import LaneState


class EmergencyWatchdog:
    """`lane_state.updated_at` 超阈值时报警。"""

    def __init__(self, threshold_ms: float = 500.0) -> None:
        self.threshold_ms = float(threshold_ms)

    def should_stop(self, state: LaneState) -> bool:
        if state.age_ms is None:
            return False
        return state.age_ms > self.threshold_ms


class LostLineDetector:
    """误差值齐 0 持续 N 帧 → 报警（视作丢线）。"""

    def __init__(self, stable_ms: float = 300.0, zero_eps: float = 1e-3) -> None:
        self.stable_ms = float(stable_ms)
        self.zero_eps = float(zero_eps)
        self._zero_since: float = 0.0
        import time as _time
        self._now_fn = _time.monotonic

    def should_alert(self, state: LaneState) -> bool:
        if state.error_y is None or state.error_angle is None:
            return False
        near_zero = abs(state.error_y) < self.zero_eps and abs(state.error_angle) < self.zero_eps
        now = self._now_fn()
        if near_zero:
            if self._zero_since == 0.0:
                self._zero_since = now
            return (now - self._zero_since) * 1000.0 > self.stable_ms
        self._zero_since = 0.0
        return False
