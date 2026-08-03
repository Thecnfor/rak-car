"""main/chassis/controllers/stanley.py
Stanley 巡线：v_x 恒定；vy 与 omega 由误差 + 角度计算。
δ = error_angle + atan(k * error_y / v_x)，omega = +δ。

2026-08-04 对齐本车实车：error_y>0=车在线右, +vy=物理左移;
error_angle>0=车头偏右, ω>0=逆时针左转。旧实现 w0/w3 手写反了（跟 SDK
轮序不一致），改用 mecanum_inverse 合成。仍是参考实现，需按场地再调。
"""
import math
from typing import List

from ..state import LaneState
from ..config.lane_follow import STANLEY_VX_FLOOR
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
        vy = +float(state.error_y)  # 视觉误差直接当横向修正（米）
        # Stanley 转向：delta>0 → ω>0 逆时针左转（对齐 curvature_adaptive ω=+kp_θ*ea）
        v = max(self.vx, STANLEY_VX_FLOOR)
        delta = float(state.error_angle) + math.atan(self.k * float(state.error_y) / v)
        omega = +delta
        return mecanum_inverse(self.vx, vy, omega, self.r_eff)
