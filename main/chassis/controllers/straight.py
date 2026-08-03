"""main/chassis/controllers/straight.py
直道控制律：vx 定速巡航 + vy 横移回正正交合成（十字正交分解，odom theta 保持 0）。
step() 输出 mecanum_inverse(vx_cruise, vy, 0) 4 轮线速度，交给 DoubleLoopRunner
下发（runner 负责 smoother / watchdog / 标定 / 暂停恢复）。

vy = sign_y * (kp_y * (ey - deadband_y) + kd_y * d(ey)/dt)，过死区后按比例，
再被 kd_y 阻尼压接近速度（防回正过头 → 回正后重新偏移的震荡），|vy| ≤ strafe_v。
error_y 需经 runner 的 calibrator 标成米（--error-scale-y 等）。

弯道（里程计 theta 闭环 90° 转弯）后续再接回：OdomTurnPID 在 controllers/odom_turn.py 保留。
"""
from __future__ import annotations

from typing import List, Optional

from ..state import LaneState
from .base import OuterLoop, mecanum_inverse


class StraightOuterLoop(OuterLoop):
    """直道：vx 巡航 + vy 横移正交回正（θ 保持 0）。

    参数：
        vx_cruise  - 直道巡航前向速度 (m/s)。全程 omega=0, 只发前进。
        deadband_y - |error_y| 死区 (m, 标定后)。超过才启动 vy 横移回正。
        kp_y       - vy 横移通道比例增益 (m/s 每米误差)。过死区后 vy = sign_y*kp_y*(ey-deadband)。
        kd_y       - vy 横移通道阻尼增益。误差快速归零（要冲过头）时压掉部分 vy，
                     抑制回正过头 → 回正后重新偏移的极限环。0 关闭。
        sign_y     - y 回正方向, +1 = error_y>0(车在线右) 左移回中; 实车反了改 -1。
        strafe_v   - |vy| 横移速度上限 (m/s)。
        r_eff      - mecanum_inverse 半径 (m), 算 4 轮速用。
    """

    def __init__(
        self,
        *,
        vx_cruise: float = 0.25,
        deadband_y: float = 0.3,
        kp_y: float = 0.2,
        kd_y: float = 0.2,
        sign_y: float = 1.0,
        strafe_v: float = 0.05,
        r_eff: float = 0.30,
    ) -> None:
        self.vx_cruise = float(vx_cruise)
        self.deadband_y = float(deadband_y)
        self.kp_y = float(kp_y)
        self.kd_y = float(kd_y)
        self.sign_y = -1.0 if float(sign_y) < 0 else 1.0
        self.strafe_v = float(strafe_v)
        self.r_eff = float(r_eff)
        self.corrections = 0  # vy 横移回正生效的帧数
        self._prev_ey: Optional[float] = None  # 上一帧 ey，算 D 阻尼用

    def _vy_from_ey(self, ey: float) -> float:
        """error_y → vy 横移通道 P 项（十字正交分解的 vy 轴）。过死区后按比例推，上限 strafe_v。"""
        if abs(ey) <= self.deadband_y:
            return 0.0
        e = ey - self.deadband_y if ey > 0 else ey + self.deadband_y
        vy = self.sign_y * self.kp_y * e
        return max(-self.strafe_v, min(self.strafe_v, vy))

    def step(self, state: LaneState, dt: float) -> List[float]:
        """直道: vx 定速巡航 + vy 横移回正正交合成（十字正交分解），
        ω 恒 0 → odom theta 保持 0。error_y 经 runner 标定后是米。

        vy 通道 PD：P 项过死区按比例推，D 项（-kd_y*d(ey)/dt）在误差快速归零、
        车要冲过头时把 vy 往回拉，抵消惯性。D 只在回正生效区间内叠加，最后整体
        钳到 ±strafe_v。
        """
        vy = 0.0
        ey = state.error_y
        if ey is not None:
            if abs(ey) > self.deadband_y:
                vy = self._vy_from_ey(ey)
                if self.kd_y and self._prev_ey is not None and dt > 0.0:
                    # ey 正在归零（prev_ey → ey，符号与 P 相反）→ 往回拉；反向则继续推
                    # sign_y 同时乘 P/D 两项：sign_y 翻号时阻尼语义不跟着翻（+1 下 `-kd*de/dt` 会反向助冲）
                    vy += self.sign_y * self.kd_y * (ey - self._prev_ey) / dt
                    vy = max(-self.strafe_v, min(self.strafe_v, vy))
            self._prev_ey = ey
            if vy != 0.0:
                self.corrections += 1
        return mecanum_inverse(self.vx_cruise, vy, 0.0, self.r_eff)


__all__ = ["StraightOuterLoop"]
