"""main/chassis/controllers/orthogonal.py
麦轮"十字正交"控制律：横移 vy 与旋转 ω 两个通道解耦，分别独立修 d_e / d_a。

核心假设：麦轮的 (vx, vy, ω) 三个自由度完全独立（这正是"可以平移"的特点）。
所以把横向误差 d_e 只喂给 vy（左/右平移回中心线），
把角度误差 d_a 只喂给 ω（原地顺时针/逆时针转到摆正），
vx 强制 0 不前进——即"原地水平巡航"。

适用场景：
1. 不前进的静态/动态水平稳定（例如边做任务边沿中心线平行移动，或原地横移校准）
2. 单独调 vy（横移）与 ω（旋转）的增益、积分、死区等——
   两个通道彼此不影响，不会出"调 vy 时顺手把 ω 也串了"的耦合调参地狱。
3. 之后再把 vx 打开（正交模式的升级版就是"正交+巡航"：
   vx 用外部定速或曲率自定，d_e→vy，d_a→ω，三者完全独立、
   每通道各一个 PI）。

为什么叫"十字正交"：
把底盘三个控制输入 (vx, vy, ω) 看作 3D 向量空间的三个正交基：
- vx 轴：前进/后退（y 正方向为前）
- vy 轴：纯横移（x 正方向为右）
- ω 轴：纯旋转（右手定则，ω>0 为逆时针？现场按约定符号）
三者在麦轮上完全独立（不要求轮胎有 slip，麦轮正/横移/旋转
都不通过地面摩擦的侧向力耦合，靠辊子自己滚）。
所以每个通道对应一路 PI 即可——没有前馈 cross-term。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from typing import List

from .base import OuterLoop, mecanum_inverse
from ..state import LaneState


@dataclass
class OrthogonalDebug:
    """正交控制器的每帧内部量，给 telemetry / dry-run 读。"""

    # 输入
    error_y: float
    error_angle: float
    dt: float

    # 横移通道（d_e → vy）
    vy_p_term: float
    vy_i_term: float
    vy_raw: float
    vy_dz: float

    # 旋转通道（d_a → ω）
    omega_p_term: float
    omega_i_term: float
    omega_raw: float
    omega_dz: float

    # 最终输出
    vx: float
    vy: float
    omega: float
    wheels: List[float]

    # 模式：vx=0 就是"原地水平稳定"
    locked_vx: bool


class OrthogonalOuterLoop(OuterLoop):
    """十字正交控制律。

    - vx=0（默认锁死不前进；如后续要结合巡航，可设 ``vx_target``）
    - vy 通道：PI 修 d_e（横向偏差 → 左右纯平移）
    - ω 通道：PI 修 d_a（角度偏差 → 原地纯旋转）
    - 每通道独立的死区 / 积分抗饱和（指数 decay + 硬 cap）
    - 符号约定（与 curvature_adaptive 一致，跟车端 lane_pid 相反）：
        * error_y>0 说明车在"底盘视角的右边"（线在车左），
          所以应该 vy<0 横移向左去回中线 → 公式用 -kp*d_e
        * error_angle>0 说明车头偏左、需要顺时针回正（右手坐标系 ω<0），
          所以 ω 通道也用 -kp_θ*d_a
      如果现场发现"越打越偏"，改 ``flip_error_{y,angle}`` 即可，
      不用改控制律。
    """

    def __init__(
        self,
        *,
        # 模式：vx=0 就是"原地水平稳定"；打开巡航时可改 vx_target>0
        vx_target: float = 0.0,
        # 机械参数（和 base.mecanum_inverse 保持一致）
        r_eff: float = 0.30,

        # 横移通道（d_e → vy）
        kp_y: float = 1.20,            # 横移 P
        ki_y: float = 0.30,            # 横移 I
        vy_max: float = 0.45,          # 横移单通道上限 |vy|
        ey_int_decay: float = 0.50,    # 横移 I 指数衰减（秒^-1）
        ey_int_cap: float = 0.08,      # 横移 I 硬上限（米·秒）
        ey_deadband: float = 0.005,    # 横移死区（|d_e|<此值 → 不输出 vy，
                                       # 用于消除中线附近来回抽的抖动）

        # 旋转通道（d_a → ω）
        kp_theta: float = 2.20,        # 旋转 P
        ki_theta: float = 0.35,        # 旋转 I
        omega_max: float = 1.40,       # ω 单通道上限
        ea_int_decay: float = 0.40,    # 旋转 I 指数衰减
        ea_int_cap: float = 0.35,      # 旋转 I 硬上限（弧度·秒）
        ea_deadband: float = 0.005,    # 旋转死区（rad）

        # 快速改符号：现场发现方向反了切这里
        flip_error_y: bool = False,
        flip_error_angle: bool = False,
    ) -> None:
        # 模式/机械
        self.vx_target = float(vx_target)
        self.locked_vx = self.vx_target == 0.0
        self.r_eff = float(r_eff)

        # 横移通道
        self.kp_y = float(kp_y)
        self.ki_y = float(ki_y)
        self.vy_max = float(vy_max)
        self.ey_int_decay = float(ey_int_decay)
        self.ey_int_cap = float(ey_int_cap)
        self.ey_deadband = float(ey_deadband)
        self.flip_error_y = bool(flip_error_y)
        self._ey_integral = 0.0

        # 旋转通道
        self.kp_theta = float(kp_theta)
        self.ki_theta = float(ki_theta)
        self.omega_max = float(omega_max)
        self.ea_int_decay = float(ea_int_decay)
        self.ea_int_cap = float(ea_int_cap)
        self.ea_deadband = float(ea_deadband)
        self.flip_error_angle = bool(flip_error_angle)
        self._ea_integral = 0.0

        # 上一帧 debug（供 lane_trace 读）
        self._dbg: OrthogonalDebug | None = None

    # ── 对外工具 ────────────────────────────────────────────────
    @property
    def debug(self) -> OrthogonalDebug | None:
        return self._dbg

    def reset_integrals(self) -> None:
        """手动清零积分——pause/resume 或场地切换时用。"""
        self._ey_integral = 0.0
        self._ea_integral = 0.0

    def debug_snapshot(self) -> dict:
        if self._dbg is None:
            return {}
        d = asdict(self._dbg)
        d["wheels"] = list(self._dbg.wheels)  # list 本来就是 list，但 asdict 已保证
        d["type"] = "orthogonal"
        return d

    # ── 内部 ────────────────────────────────────────────────────
    @staticmethod
    def _apply_deadband(err: float, eps: float) -> float:
        if abs(err) < eps:
            return 0.0
        # 拉普拉斯风格，过了死区就按比例继续推（不是硬 0/阶跃）
        if err > 0:
            return err - eps
        return err + eps

    @staticmethod
    def _decay_and_accum(
        state: float,
        err: float,
        dt: float,
        decay: float,
        cap: float,
    ) -> float:
        # 指数衰减：无误差时 I 会自己归 0，防止"一弯的积分留到下一弯"
        state = state * math.exp(-decay * dt)
        state = state + err * dt
        # 硬 cap
        if cap > 0:
            state = max(-cap, min(cap, state))
        return state

    # ── 主 step ─────────────────────────────────────────────────
    def step(self, state: LaneState, dt: float) -> List[float]:
        if not state.has_error:
            # 没有误差帧（丢线或 lane_feed 还没首帧）：
            # 积分不累积（因为 I * exp(-decay*dt) 自己会衰减），
            # 输出也不做任何事（vx=0, vy=0, ω=0 → 原地停）
            wheels = mecanum_inverse(
                self.vx_target, 0.0, 0.0, r=self.r_eff
            )
            self._dbg = OrthogonalDebug(
                error_y=state.error_y if state.error_y is not None else 0.0,
                error_angle=state.error_angle if state.error_angle is not None else 0.0,
                dt=dt,
                vy_p_term=0.0, vy_i_term=0.0, vy_raw=0.0, vy_dz=0.0,
                omega_p_term=0.0, omega_i_term=0.0, omega_raw=0.0, omega_dz=0.0,
                vx=self.vx_target, vy=0.0, omega=0.0,
                wheels=wheels, locked_vx=self.locked_vx,
            )
            return wheels

        d_e = float(state.error_y)
        d_a = float(state.error_angle)

        # 快速翻符号：现场调参不碰控制律代码
        if self.flip_error_y:
            d_e = -d_e
        if self.flip_error_angle:
            d_a = -d_a

        # ── 横移通道：-kp*d_e + -ki*I(d_e) ─────────────────────
        ey_dz = self._apply_deadband(d_e, self.ey_deadband)
        self._ey_integral = self._decay_and_accum(
            self._ey_integral, ey_dz, dt,
            decay=self.ey_int_decay, cap=self.ey_int_cap,
        )
        p_y = -self.kp_y * ey_dz
        i_y = -self.ki_y * self._ey_integral
        vy_raw = p_y + i_y
        if self.vy_max > 0:
            vy_raw = max(-self.vy_max, min(self.vy_max, vy_raw))

        # ── 旋转通道：-kp_θ*d_a + -ki_θ*I(d_a) ─────────────────
        ea_dz = self._apply_deadband(d_a, self.ea_deadband)
        self._ea_integral = self._decay_and_accum(
            self._ea_integral, ea_dz, dt,
            decay=self.ea_int_decay, cap=self.ea_int_cap,
        )
        p_θ = -self.kp_theta * ea_dz
        i_θ = -self.ki_theta * self._ea_integral
        omega_raw = p_θ + i_θ
        if self.omega_max > 0:
            omega_raw = max(-self.omega_max, min(self.omega_max, omega_raw))

        # ── 合成：vx 固定（默认 0），vy / ω 完全独立 ────────────
        vx = self.vx_target
        vy = vy_raw
        omega = omega_raw
        wheels = mecanum_inverse(vx, vy, omega, r=self.r_eff)

        self._dbg = OrthogonalDebug(
            error_y=d_e, error_angle=d_a, dt=dt,
            vy_p_term=p_y, vy_i_term=i_y, vy_raw=vy_raw, vy_dz=ey_dz,
            omega_p_term=p_θ, omega_i_term=i_θ, omega_raw=omega_raw, omega_dz=ea_dz,
            vx=vx, vy=vy, omega=omega,
            wheels=wheels, locked_vx=self.locked_vx,
        )
        return wheels
