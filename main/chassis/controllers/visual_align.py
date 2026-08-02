"""main/chassis/controllers/visual_align.py
视觉微调专用控制律：仅输出前向速度 ``vx``，**不准左右、不准旋转**。

设计要点：
- 控制律输入 ``AlignState``（来自 state_align.py）。
- 比例控制 ``vx = kp * area_error``，其中 ``area_error = ref_area - area``：
    - 当前面积 < ref_area → 车离目标比期望更远 → vx 为正 → 前进
    - 当前面积 > ref_area → 车离目标比期望更近 → vx 为负 → 后退
    - 实际方向（前进/后退）和 ref_area 在 y=-0.1 高度标度下的几何对应，
      由调参侧保证；控制器只负责比例缩放。
- vx 受 ``v_max`` 单向饱和（不限制符号）→ 经 ``WheelSmoother`` 做 slew rate 限幅。
- 几何意义：
    - **直接下发 4 轮全等 vx**（纯前后）；不通过 ``mecanum_inverse`` —
      SDK 的 ``calculate_wheel_velocities`` 在 vy=0/omega=0 时给的是
      ``[vx,-vx,-vx,vx]``（差速转向式），会让车原地打转而不是直线前进。
      视觉微调要求"不准旋转"，所以绕开 IK，直接发同速。
"""
from typing import List

from ..state_align import AlignState
from .base import OuterLoop


class VisualAlignOuterLoop(OuterLoop):
    """视觉微调控制律：**只前进/后退**。

    默认参数为"快档"(2026-08-02)：收敛灵敏 + 响应快。
    现场验证后可以调参；如果不要激进，调 ``kp`` / ``v_max`` 减小即可。

    参数:
      kp       - 比例增益（m/s 每单位 area_error）。默认 1.5 适合 cam2 bbox_norm.area
                 量级（一般 0~0.1）；调到 2.0+ 会冲过一点，调到 0.6 更平滑。
      v_max    - 速度上限（绝对值），默认 0.35 m/s。比 LANE_FOLLOW 慢一档,
                 比保守版 (0.20) 快了 75%。
      deadband - area_error 死区（绝对值小于此值视为 0 → 输出 0 速, 防止抖动）。
                 默认 0.005 ≈ 0.5% 画面面积的 bbox 变化。
    """

    def __init__(
        self,
        kp: float = 1.5,
        v_max: float = 0.35,
        deadband: float = 0.005,
    ) -> None:
        self.kp = float(kp)
        self.v_max = float(v_max)
        self.deadband = float(deadband)

    def step(self, state: AlignState, dt: float) -> List[float]:
        # 没拿到目标 / 数据缺失 → 安全零速
        if not state.has_error:
            return self._safe_zero()
        err = float(state.area_error)
        if abs(err) < self.deadband:
            return self._safe_zero()
        # 单向饱和：保留符号
        vx = self.kp * err
        if vx > self.v_max:
            vx = self.v_max
        elif vx < -self.v_max:
            vx = -self.v_max
        # 直接下发 4 轮全等 vx：物理意义是 4 个轮子同向转 → 整车直线前后；
        # vy/omega 都被绕开,确保"不准左右不准旋转"。
        return [vx, vx, vx, vx]


__all__ = ["VisualAlignOuterLoop"]