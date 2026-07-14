"""main/chassis/controllers/p_controller.py
最简化版外环：对 error_y 做 P 控制。适合 demo / 起步。
不做横向 v_y，只做前向 vx + 转向 omega。
"""
from typing import List

from ..state import LaneState
from .base import OuterLoop, mecanum_inverse


class POuterLoop(OuterLoop):
    """外环=P：error_y 直给 vy；前向恒速；error_angle 进 omega。"""

    def __init__(self, kp_y: float = 0.4, kp_theta: float = 1.2, vx: float = 0.3, r_eff: float = 0.30):
        self.kp_y = kp_y
        self.kp_theta = kp_theta
        self.vx = vx
        self.r_eff = r_eff

    def step(self, state: LaneState, dt: float) -> List[float]:
        if not state.has_error:
            return self._safe_zero()
        vy = -self.kp_y * float(state.error_y)
        omega = -self.kp_theta * float(state.error_angle)
        return mecanum_inverse(self.vx, vy, omega, self.r_eff)
