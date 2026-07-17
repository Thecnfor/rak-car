"""main/chassis/controllers/pure_pursuit.py
Pure Pursuit 占位实现：以 LaneState.error_y 当成最近点横向偏差。

注意：
本底盘是前视车道线持续给"误差"，不是真正的轨迹；这里仅给 demo 用骨架，
底盘同学按真实目标点（list[Pose]）替换最近点计算逻辑。
"""
import math
from typing import List, Optional, Tuple

from ..state import LaneState
from .base import OuterLoop


class PurePursuitOuterLoop(OuterLoop):
    """Pure Pursuit demo：以 (error_y, error_angle) 推角速度。"""

    def __init__(self, look_ahead_m: float = 0.6, vx: float = 0.3, r_eff: float = 0.30):
        self.look_ahead_m = look_ahead_m
        self.vx = vx
        self.r_eff = r_eff

    def _target_from_lane(self, state: LaneState) -> Optional[Tuple[float, float]]:
        if not state.has_error:
            return None
        # 把视觉误差近似成一个前方目标点（车体坐标系）
        # x 正向 = 前；y 正向 = 左
        x = self.look_ahead_m
        y = -float(state.error_y)
        return x, y

    def step(self, state: LaneState, dt: float) -> List[float]:
        target = self._target_from_lane(state)
        if target is None:
            return self._safe_zero()
        x, y = target
        # 几何：alpha = atan(y/x)，曲率 = 2*sin(alpha)/Ld
        alpha = math.atan2(y, x)
        curvature = 2.0 * math.sin(alpha) / max(self.look_ahead_m, 1e-3)
        omega = self.vx * curvature
        return [
            self.vx + self.r_eff * omega,
            -self.vx + self.r_eff * omega,
            -self.vx + self.r_eff * omega,
            self.vx + self.r_eff * omega,
        ]
