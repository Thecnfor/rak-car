"""main/chassis/controllers/straight.py
直道控制律：vx 定速巡航 + vy 横移回正正交合成（十字正交分解，odom theta 保持 0）。
step() 输出 mecanum_inverse(vx_cruise, vy, 0) 4 轮线速度，交给 DoubleLoopRunner
下发（runner 负责 smoother / watchdog / 标定 / 暂停恢复）。

vy = sign_y * kp_y * (ey - deadband_y)，过死区后按比例，|vy| ≤ strafe_v。
error_y 需经 runner 的 calibrator 标成米（--error-scale-y 等）。

弯道（里程计 theta 闭环 90° 转弯）后续再接回：OdomTurnPID 在 controllers/odom_turn.py 保留。
"""
from __future__ import annotations

from typing import List

from ..state import LaneState
from .base import OuterLoop, mecanum_inverse


class StraightOuterLoop(OuterLoop):
    """直道：vx 巡航 + vy 横移正交回正（θ 保持 0）。

    参数：
        vx_cruise  - 直道巡航前向速度 (m/s)。全程 omega=0, 只发前进。
        deadband_y - |error_y| 死区 (m, 标定后)。超过才启动 vy 横移回正。
        kp_y       - vy 横移通道比例增益 (m/s 每米误差)。过死区后 vy = sign_y*kp_y*(ey-deadband)。
        sign_y     - y 回正方向, -1 = error_y>0(目标在右) 左移回中; 实车反了改 +1。
        strafe_v   - |vy| 横移速度上限 (m/s)。
        r_eff      - mecanum_inverse 半径 (m), 算 4 轮速用。
    """

    def __init__(
        self,
        *,
        vx_cruise: float = 0.25,
        deadband_y: float = 0.01,
        kp_y: float = 0.2,
        sign_y: float = -1.0,
        strafe_v: float = 0.05,
        r_eff: float = 0.30,
    ) -> None:
        self.vx_cruise = float(vx_cruise)
        self.deadband_y = float(deadband_y)
        self.kp_y = float(kp_y)
        self.sign_y = -1.0 if float(sign_y) < 0 else 1.0
        self.strafe_v = float(strafe_v)
        self.r_eff = float(r_eff)
        self.corrections = 0  # vy 横移回正生效的帧数

    def _vy_from_ey(self, ey: float) -> float:
        """error_y → vy 横移通道（十字正交分解的 vy 轴）。过死区后按比例继续推，上限 strafe_v。"""
        if abs(ey) <= self.deadband_y:
            return 0.0
        e = ey - self.deadband_y if ey > 0 else ey + self.deadband_y
        vy = self.sign_y * self.kp_y * e
        return max(-self.strafe_v, min(self.strafe_v, vy))

    def step(self, state: LaneState, dt: float) -> List[float]:
        """直道: vx 定速巡航 + vy 横移回正正交合成（十字正交分解），
        ω 恒 0 → odom theta 保持 0。error_y 经 runner 标定后是米。"""
        vy = 0.0
        ey = state.error_y
        if ey is not None:
            vy = self._vy_from_ey(ey)
            if vy != 0.0:
                self.corrections += 1
        return mecanum_inverse(self.vx_cruise, vy, 0.0, self.r_eff)


__all__ = ["StraightOuterLoop"]
