"""main/chassis/controllers/base.py
所有外环控律都遵守这套接口。
"""
from abc import ABC, abstractmethod
from typing import List

from ..state import LaneState


def mecanum_inverse(vx: float, vy: float, omega: float, r: float) -> List[float]:
    """按 vehicle_to_wheel_matrix = [[1,-1,-1,1],[t,t,-t,-t],[r,r,r,r]] 推。
    这里只取机械混合部分（忽略 roller_angle），
    omega -> r*omega 中 r = half_track*tan(roller) + half_wheel_base。
    """
    r_eff = max(r, 1e-6)
    return [
        vx - vy + r_eff * omega,
        -vx + vy + r_eff * omega,
        -vx - vy + r_eff * omega,
        vx + vy + r_eff * omega,
    ]


class OuterLoop(ABC):
    """外环控制律接口。

    step(state, dt) -> [v1, v2, v3, v4] 四轮线速度（m/s）。
    state.error_y / error_angle 始终是底盘前摄的视觉误差。
    """

    @abstractmethod
    def step(self, state: LaneState, dt: float) -> List[float]:
        ...

    @staticmethod
    def _safe_zero() -> List[float]:
        return [0.0, 0.0, 0.0, 0.0]
