"""main/chassis/controllers/stanley.py
Stanley 巡线：v_x 恒定；vy 与 omega 由误差 + 角度计算。
δ = error_angle + atan(k * error_y / v_x)
"""
import math
from typing import List

from ..state import LaneState
from .base import OuterLoop, mecanum_inverse


class StanleyOuterLoop(OuterLoop):
    """Stanley 控制律：仅供底盘同学参考实现，需要按场地再调。"""

    def __init__(self, k: float = 0.6, vx: float = 0.3, r_eff: float = 0.30):
        self.k = k
        self.vx = vx
        self.r_eff = r_eff

    def step(self, state: LaneState, dt: float) -> List[float]:
        if not state.has_error:
            return self._safe_zero()
        vy = -float(state.error_y)  # 视觉误差直接当横向修正（米）
        # Stanley 转向：δ = ea + atan(k·ey/vx)；麦轮上把 δ 当 omega、vy 当横移
        v = max(self.vx, 0.05)
        delta = float(state.error_angle) + math.atan(self.k * float(state.error_y) / v)
        omega = -delta
        return mecanum_inverse(self.vx, vy, omega, self.r_eff)
