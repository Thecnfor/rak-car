"""main/chassis/controllers/straight.py
直道控制律：vx 定速巡航 + vy 横移回正 + ω 视觉航向纠正，三通道十字正交分解。
step() 输出 mecanum_inverse(vx_cruise, vy, omega) 4 轮线速度，交给 DoubleLoopRunner
下发（runner 负责 smoother / watchdog / 标定 / 暂停恢复）。

vy 通道：PD 修 error_y（横向偏差 → 左右纯平移），
    vy = sign_y * (kp_y * (ey - deadband_y) + kd_y * d(ey)/dt)，过死区后按比例，
    再被 kd_y 阻尼压接近速度（防回正过头 → 回正后重新偏移的震荡），|vy| ≤ strafe_v。
ω 通道：error_angle → 旋转纠正（PI），再加 error_y 的 cross-track 项，让车头收敛到
    与实际车道中心线**平行**：
    模型把 error_angle=0 量化为一段角度范围，零区内角度通道是瞎的（读 0 算无误差），
    只靠角度 PI 收敛不到真平行。cross-track 项 omega += sign_theta*k_ey_omega*error_y
    借用横向偏移反推：车头不平行 → 横向漂移 → error_y 变化 → ω 把车头转回，直到
    漂移归零 = 真平行。ea_target 只作可选额外偏置（默认 0）。
    角度项：误差 e = ea - ea_target，omega = sign_theta*(kp_theta*e + ki_theta*I(e))，
    I 指数衰减 + 硬 cap，加上 cross-track 后整体钳到 ±omega_max。
    直道右偏的根因就是旧版 ω 恒 0——只横移不转航向，车头一直歪着、横移在硬顶；
    加了 ω 后车会主动转到与线平行，横移只兜残余偏差。
error_y / error_angle 需经 runner 的 calibrator 标成物理量（--error-scale-y 等）。

弯道（里程计 theta 闭环 90° 转弯）后续再接回：OdomTurnPID 在 controllers/odom_turn.py 保留。
"""
from __future__ import annotations

import math
from dataclasses import asdict
from typing import List, Optional

from ..state import LaneState
from .base import OuterLoop, mecanum_inverse
from .orthogonal import OrthogonalDebug  # 直道 debug 复用 orthogonal 模板（vy 的 I 槽位是 D 项）


class StraightOuterLoop(OuterLoop):
    """直道：vx 巡航 + vy 横移回正 + ω 视觉航向纠正（三通道十字正交分解）。

    参数：
        vx_cruise  - 直道巡航前向速度 (m/s)。全程 vx 恒定。
        deadband_y - |error_y| 死区 (m, 标定后)。超过才启动 vy 横移回正。
        kp_y       - vy 横移通道比例增益 (m/s 每米误差)。过死区后 vy = sign_y*kp_y*(ey-deadband)。
        kd_y       - vy 横移通道阻尼增益。误差快速归零（要冲过头）时压掉部分 vy，
                     抑制回正过头 → 回正后重新偏移的极限环。0 关闭。
        sign_y     - y 回正方向, +1 = error_y>0(车在线右) 左移回中; 实车反了改 -1。
        strafe_v   - |vy| 横移速度上限 (m/s)。
        kp_theta   - ω 航向通道比例增益 (rad/s 每弧度航向误差)。
        ki_theta   - ω 航向通道积分增益。消稳态航向偏差（右偏就是稳态偏差）。
        omega_max  - |ω| 旋转速度上限 (rad/s)。直道巡航给比 orthogonal 小的值。
        ea_target  - 可选额外收敛偏置 (rad)。默认 0 = 以实际车道中心线方向为目标：
                     车头平行与否由 cross-track（k_ey_omega）反推，不用固定偏置。
        k_ey_omega - error_y → ω 的 cross-track 增益 (rad/s 每单位 error_y)。
                     error_angle 零区（模型把 0 量化为一段角度范围）内角度通道是瞎的，
                     靠横向偏移反推真实平行：车头不平行 → 横向漂移 → error_y 变化 →
                     ω 纠正到漂移归零 = 真平行。error_y>0（车在线右）→ 左转拉回。
                     需按 error_y 标定尺度调（与 kp_y 同一尺度）。
        ea_deadband- ω 通道死区 (rad)。|error_angle - ea_target| 小于它 → 角度项不输出。
        ea_int_decay - ω 积分指数衰减 (s^-1)，无误差时 I 自己归 0。
        ea_int_cap - ω 积分硬上限 (rad·s)。
        sign_theta - ω 方向, +1 = error_angle>0(车头偏右) 逆时针左转回正
                     （跟 orthogonal 的符号约定一致）。实车反了改 -1。
        r_eff      - mecanum_inverse 半径 (m), 算 4 轮速用。
    """

    def __init__(
        self,
        *,
        vx_cruise: float = 0.32,  # 巡线段直线速度 (0.25→0.30, 2026-08-07 用户: 任务间直线加快, 转向不变)
        deadband_y: float = 0.4,
        kp_y: float = 0.2,
        kd_y: float = 0.2,
        sign_y: float = 1.0,
        strafe_v: float = 0.10,
        kp_theta: float = 1.5,
        ki_theta: float = 0.15,
        omega_max: float = 0.25,
        ea_target: float = 0.0,
        k_ey_omega: float = 0.5,
        ea_deadband: float = 0.005,
        ea_int_decay: float = 0.5,
        ea_int_cap: float = 0.1,
        sign_theta: float = 1.0,
        r_eff: float = 0.30,
    ) -> None:
        self.vx_cruise = float(vx_cruise)
        self.deadband_y = float(deadband_y)
        self.kp_y = float(kp_y)
        self.kd_y = float(kd_y)
        self.sign_y = -1.0 if float(sign_y) < 0 else 1.0
        self.strafe_v = float(strafe_v)
        self.r_eff = float(r_eff)

        # ω 视觉航向通道（PI，同 orthogonal.py 旋转通道）
        self.kp_theta = float(kp_theta)
        self.ki_theta = float(ki_theta)
        self.omega_max = float(omega_max)
        self.ea_target = float(ea_target)  # 可选额外偏置；默认 0 = 收敛到真平行
        self.k_ey_omega = float(k_ey_omega)  # error_y → ω cross-track 增益
        self.ea_deadband = float(ea_deadband)
        self.ea_int_decay = float(ea_int_decay)
        self.ea_int_cap = float(ea_int_cap)
        self.sign_theta = -1.0 if float(sign_theta) < 0 else 1.0
        self._ea_integral = 0.0  # ω 通道积分

        self.corrections = 0  # vy 横移回正生效的帧数
        self._prev_ey: Optional[float] = None  # 上一帧 ey，算 D 阻尼用
        self._dbg: Optional[OrthogonalDebug] = None

    # ── 内部 ────────────────────────────────────────────────────
    def _vy_from_ey(self, ey: float) -> float:
        """error_y → vy 横移通道 P 项（十字正交分解的 vy 轴）。过死区后按比例推，上限 strafe_v。"""
        if abs(ey) <= self.deadband_y:
            return 0.0
        e = ey - self.deadband_y if ey > 0 else ey + self.deadband_y
        vy = self.sign_y * self.kp_y * e
        return max(-self.strafe_v, min(self.strafe_v, vy))

    def _omega_from_ea(self, ea: float, dt: float):
        """error_angle → ω 视觉航向通道 PI（取自 orthogonal.py 旋转通道 227-237）。

        目标 ``ea_target`` 默认 0（车头平行于车道）；零区内角度通道是瞎的，
        收敛到真平行靠 step() 里的 cross-track（error_y → ω）项反推。
        这里先把误差换成 ``e = ea - ea_target``（ea_target 只作可选额外偏置）。
        死区（拉普拉斯风格，过了就按比例）+ 积分指数衰减（无误差时 I 自己归 0，
        不留"上一段路"的积分）+ 硬 cap，最后整体钳到 ±omega_max。
        返回 (omega, ea_dz, p, i)，p/i 给 debug 用。
        """
        ea_dz = 0.0
        e = ea - self.ea_target
        if abs(e) > self.ea_deadband:
            ea_dz = e - self.ea_deadband if e > 0 else e + self.ea_deadband
        self._ea_integral = (
            self._ea_integral * math.exp(-self.ea_int_decay * dt) + ea_dz * dt
        )
        if self.ea_int_cap > 0:
            self._ea_integral = max(-self.ea_int_cap, min(self.ea_int_cap, self._ea_integral))
        p = self.kp_theta * ea_dz
        i = self.ki_theta * self._ea_integral
        omega = self.sign_theta * (p + i)
        if self.omega_max > 0:
            omega = max(-self.omega_max, min(self.omega_max, omega))
        return omega, ea_dz, p, i

    # ── 主 step ─────────────────────────────────────────────────
    def step(self, state: LaneState, dt: float) -> List[float]:
        """直道: vx 定速巡航 + vy 横移回正 + ω 视觉航向纠正（三通道十字正交分解）。
        error_y / error_angle 经 runner 标定后是物理量。

        vy 通道 PD：P 项过死区按比例推，D 项（+sign_y*kd_y*d(ey)/dt）在误差快速归零、
        车要冲过头时把 vy 往回拉，抵消惯性。D 只在回正生效区间内叠加，最后整体
        钳到 ±strafe_v。
        ω 通道：角度项 PI（error_angle → 主动转航向）+ cross-track 项（error_y → ω）。
        cross-track 是让车头收敛到与车道中心线**平行**的关键：error_angle 零区（模型
        把 0 量化为角度范围）内角度项是瞎的，车头不平行 → 横向漂移 → error_y 变化 →
        该项把车头转回，直到漂移归零 = 真平行。各通道独立，None（丢线 / feed 未就绪）
        时该通道输出 0。
        """
        vy, vy_p, vy_d, vy_dz = 0.0, 0.0, 0.0, 0.0
        ey = state.error_y
        if ey is not None:
            vy_p = self._vy_from_ey(ey)
            if abs(ey) > self.deadband_y:
                vy = vy_p
                vy_dz = ey - self.deadband_y if ey > 0 else ey + self.deadband_y
                if self.kd_y and self._prev_ey is not None and dt > 0.0:
                    # ey 正在归零（prev_ey → ey，符号与 P 相反）→ 往回拉；反向则继续推
                    # sign_y 同时乘 P/D 两项：sign_y 翻号时阻尼语义不跟着翻（+1 下 `-kd*de/dt` 会反向助冲）
                    vy_d = self.sign_y * self.kd_y * (ey - self._prev_ey) / dt
                    vy += vy_d
                    vy = max(-self.strafe_v, min(self.strafe_v, vy))
            self._prev_ey = ey
            if vy != 0.0:
                self.corrections += 1

        omega, ea_dz, omega_p, omega_i = 0.0, 0.0, 0.0, 0.0
        omega_ey = 0.0
        ea = state.error_angle
        if ea is not None:
            omega, ea_dz, omega_p, omega_i = self._omega_from_ea(ea, dt)
        # cross-track：error_y → ω。error_angle 零区（读 0 算无误差）内角度通道是瞎的，
        # 靠横向偏移反推真实平行：车头不平行 → 横向漂移 → error_y 变化 → 纠正到漂移归零。
        # error_y>0（车在线右）→ ω>0 左转拉回（与 sign_theta 同向翻）。
        if self.k_ey_omega and ey is not None:
            omega_ey = self.sign_theta * self.k_ey_omega * float(ey)
            omega += omega_ey
            if self.omega_max > 0:
                omega = max(-self.omega_max, min(self.omega_max, omega))

        wheels = mecanum_inverse(self.vx_cruise, vy, omega, self.r_eff)
        self._dbg = OrthogonalDebug(
            error_y=ey if ey is not None else 0.0,
            error_angle=ea if ea is not None else 0.0,
            dt=dt,
            vy_p_term=vy_p, vy_i_term=vy_d, vy_raw=vy, vy_dz=vy_dz,
            omega_p_term=omega_p, omega_i_term=omega_i, omega_raw=omega, omega_dz=ea_dz,
            vx=self.vx_cruise, vy=vy, omega=omega,
            wheels=wheels, locked_vx=False,  # 直道永远巡航
            omega_ey_term=omega_ey,
        )
        return wheels

    def debug_snapshot(self) -> dict:
        """给 lane_trace 的正交模板打 ω/vy P/I；无 debug 帧返回空 dict。"""
        if self._dbg is None:
            return {}
        d = asdict(self._dbg)
        d["type"] = "orthogonal"
        return d


__all__ = ["StraightOuterLoop"]
