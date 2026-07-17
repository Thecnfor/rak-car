"""main/chassis/controllers/curvature_adaptive.py
弧度自适应巡线控律：

设计要点（按需求）：
1. 当 ``error_angle``（弧度偏差）大幅变动时，线速度 ``vx`` 立即放慢、转向角速度 ``omega`` 加力；
2. 只有当 ``error_angle`` 与 ``error_y`` 同时足够小、并稳定一段时间后，才恢复标称 ``vx``；
3. 横移 / 转向互斥（v6+）：直线段靠 ``vy`` 修偏差（朝向不变、质心斜向），
   弯道段靠 ``omega`` 转向（后轮做轴、前轮差速旋转），二者不同时存在。

实现策略：
- 把 "弧度偏差大幅变动" 量化为两个分量的加权和：
    a) ``|error_angle|``：当前视线相对车体轴线的偏离角度（弯道主线已经斜着过来）
    b) ``|d(error_angle)/dt|``：相邻帧间 error_angle 的变化率（突变 / 抖震）
- 用 EMA 平滑作为 curvature 估计 ``kappa``，避免单帧抖动把 vx 拉到底。
- vx 用指数衰减曲线映射 ``vx = v_min + (v_max - v_min) * exp(-kappa)``：
      kappa = 0  → vx = v_max（满速）
      kappa = 1  → vx ≈ v_min + 0.368 * (v_max - v_min)
      kappa → ∞  → vx → v_min
- omega 在 P 项 ``-kp_theta * error_angle`` 的基础上，乘以 ``1 + omega_gain * kappa``，
  在大幅变动时转向更猛；同时叠加一个 "进弯牵引" 项 ``-k_curvature * d(error_angle)/dt``，
  进一步把车头拉回主线。
- 恢复门控：维护 ``_straight_streak_ms``，仅当 ``|error_y| < ey_release`` 且
  ``|error_angle| < ea_release`` 持续 ``hold_ms`` 才把 vx 真正放回 ``v_max``；
  否则保持减速状态，避免在弯道末端一恢复就被下一波偏差甩飞。
- axis_mix 互斥权重（v6+）：把 ``kappa`` 喂给数值稳定 sigmoid 得到 ``axis_mix ∈ [0,1]``，
  当下发到 4 轮前按
        vy_decided   = (1 - axis_mix) * vy_raw
        omega_decided = axis_mix    * omega_raw
  缩放，保证同一帧内横向修正与转向不会同时全开，与底盘期望「横向偏差下走斜向、转弯时绕后轴」
  的行为一致。
- **vy_floor（v6+，2026-07-16 加）**：上面 ``vy_decided`` 写法在弯道里（axis_mix→1）把
  vy 完全压零，导致 error_y 在长弯里彻底没有修正机会，弯内侧（error_y 偏内一侧）
  会持续贴线/压边界。修复：保留 ``vy_floor``（默认 0.15）作为保底——
        vy_keep      = vy_floor + (1 - vy_floor) * (1 - axis_mix)
        vy_decided   = vy_keep * vy_raw
  这样 axis_mix=0（直线）保持 vy 全量、axis_mix=1（弯道）保留 15% 横向修正，
  过渡带由 sigmoid 保持平滑。``vy_floor`` 调大（→0.3）更像 Stanley 全时修正、
  error_y 收敛快但旋转略被稀释；调小（→0.05）更接近原「轴向互斥」行为。
- **横向 I 项（v6+，2026-07-16 加）**：纯 P 控制遇到恒定干扰（视觉 bias / 机械零漂 /
  地面不平）时永远需要一个稳态 error_y 才能维持修正力，看起来就是"过偏了才矫正"。
  修复：加 leaky 积分项——
        vy_raw = -kp_y * error_y - ki_y * I
        I(t+dt) = I(t) * exp(-ey_int_decay * dt) + error_y * dt
        I 夹到 [-ey_int_cap, +ey_int_cap]
  leaky 衰减保证曲线段积分残留不会带到直线段。``ki_y=0`` 退化到纯 P 控制。
  ``ki_y`` 调大（→1.0~1.5）消除稳态偏差更快，但过弯出弯后车头略晃；
  ``ey_int_decay`` 调大（→1.0）半衰期更短（≈ 0.7s），过弯残留衰减更快。
- 控制律在 chassis/state.py LaneState 之上，依赖 (state, dt) 即可，对底层 P / Stanley
  没有强耦合，可以单独替换使用。
"""
from __future__ import annotations

import math
import time
from typing import List, Optional

from ..state import LaneState
from .base import OuterLoop, mecanum_inverse


class CurvatureAdaptiveOuterLoop(OuterLoop):
    """弧度偏差自适应巡线：当 error_angle 大幅变动 → 降速 + 加强转向；
    误差回到小值并稳定 hold_ms 后再恢复标称速度。
    """

    def __init__(
        self,
        # --- 速度曲线 ---
        v_max: float = 0.30,
        v_min: float = 0.08,
        # --- 弧度偏差 → 速度映射 ---
        kappa_full: float = 0.6,
        # --- 弧度变化率 → 速度映射（rad/s 单位） ---
        dkappa_full: float = 1.5,
        # --- P 项（基础横向 + 转向） ---
        # kp_theta 从 1.8 降到 1.2：弯道转向不再"打满舵"，给内环 PID 留修正余地
        kp_y: float = 0.5,
        kp_theta: float = 1.2,
        # --- I 项（修复 2026-07-16 直行稳态误差）---
        # 纯 P 控制遇到恒定干扰（视觉 bias / 机械零漂 / 地面不平）时永远需要一个
        # 稳态 error_y 才能维持修正力，看起来就是"过偏了才矫正"。
        # 加 leaky 积分项消除稳态偏差：积分累积 - 衰减 → bias 长时间存在时积分增长，
        # 偏差回零后自动衰减，避免曲线段积分饱和把直线段冲飞。
        # 经验起步：ki_y=0.6 / ey_int_cap=0.10 / ey_int_decay=0.5
        ki_y: float = 0.6,
        ey_int_cap: float = 0.10,
        ey_int_decay: float = 0.5,
        # --- 弧度变化驱动的额外 omega 增益 ---
        # omega_gain 从 0.6 降到 0.35、kappa 上限从 2.0 收到 1.5：
        # 大弧度差时 boost 最大 ≈ 1.5x（之前 2.2x），单轮阶跃幅度减半
        omega_gain: float = 0.35,
        k_curvature: float = 0.25,
        # --- omega 软上限 ---
        # 防止 dkappa_ema 峰值瞬间把 omega 冲到 2.5+ rad/s，配合 r_eff=0.30
        # 就是单轮 0.75 m/s 的额外项，加上 vx/vy 直接撞电源。
        omega_cap: float = 1.8,
        # --- 平滑 ---
        ema_alpha: float = 0.35,
        # --- 恢复门控 ---
        ey_release: float = 0.02,
        ea_release: float = 0.05,
        hold_ms: float = 250.0,
        # --- 横移 / 转向 互斥权重 ---
        # axis_mix = sigmoid((kappa - kappa_axis_center) / kappa_axis_width)
        # axis_mix ≈ 0 → 横向滑动主导（vy 全量, ω=0，朝向不变）
        # axis_mix ≈ 1 → 转弯主导（vy=0, ω 全量，后轮做轴前轮差速旋转）
        kappa_axis_center: float = 1.0,
        kappa_axis_width: float = 0.5,
        # --- 弯道段 vy 保底比例（修复 2026-07-16 弯内侧压边界）---
        # 弯道段（axis_mix≈1）保留 vy_raw * vy_floor 的横向修正能力，
        # 让 error_y 在长弯里仍有机会被缓慢修正，否则弯内侧贴线/压边。
        # 经验区间：0.10~0.25。0 = 还原成完全互斥（不推荐）；>0.4 ≈ Stanley 全时修正。
        vy_floor: float = 0.15,
        # --- 麦轮几何 ---
        r_eff: float = 0.30,
    ) -> None:
        self.v_max = float(v_max)
        self.v_min = float(v_min)
        self.kappa_full = max(float(kappa_full), 1e-3)
        self.dkappa_full = max(float(dkappa_full), 1e-3)
        self.kp_y = float(kp_y)
        self.kp_theta = float(kp_theta)
        # 积分项：leaky 积分状态 + 抗饱和上限
        self.ki_y = float(ki_y)
        self.ey_int_cap = max(float(ey_int_cap), 0.0)
        self.ey_int_decay = max(float(ey_int_decay), 0.0)
        self._ey_integral: float = 0.0
        self.omega_gain = float(omega_gain)
        self.k_curvature = float(k_curvature)
        self.omega_cap = max(float(omega_cap), 1e-3)
        self.ema_alpha = float(ema_alpha)
        self.ey_release = float(ey_release)
        self.ea_release = float(ea_release)
        self.hold_ms = float(hold_ms)
        self.kappa_axis_center = float(kappa_axis_center)
        self.kappa_axis_width = max(float(kappa_axis_width), 1e-3)
        # vy 保底比例 ∈ [0, 1)；0 会让弯道里 vy 完全归零（原互斥行为）。
        self.vy_floor = max(0.0, min(float(vy_floor), 1.0))
        # 当前 axis_mix 缓存（每帧由 step() 刷新），供 debug_snapshot() 暴露
        self._axis_mix: float = 0.0
        self.r_eff = float(r_eff)

        # 弧度偏差估计（EMA 平滑）
        self._kappa_ema: float = 0.0
        # 上一次 error_angle + 采样时间，用于估计 d(error_angle)/dt
        self._prev_ea: Optional[float] = None
        self._prev_ea_t: Optional[float] = None
        self._dkappa_ema: float = 0.0
        # 恢复门控：误差小持续时间（ms）
        self._straight_streak_ms: float = 0.0

    # ---------- 弧度估计 ----------
    def _update_curvature(self, state: LaneState, now: float) -> float:
        """更新内部 curvature 估计，返回当前 kappa（已平滑）。"""
        ea = float(state.error_angle) if state.error_angle is not None else 0.0

        # (1) 弧度偏差本体 |ea| → kappa 项
        kappa_inst = abs(ea)

        # (2) 弧度偏差变化率 |d(ea)/dt|
        dkappa_inst = 0.0
        if (
            self._prev_ea is not None
            and self._prev_ea_t is not None
            and now > self._prev_ea_t
        ):
            dt = now - self._prev_ea_t
            # 抑制奇异帧：dt 太小（<5ms）或太大（>500ms，多半是状态空了一拍）当作不可信
            if 0.005 <= dt <= 0.5:
                dkappa_inst = abs((ea - self._prev_ea) / dt)
        self._prev_ea = ea
        self._prev_ea_t = now

        # EMA 平滑（两个独立 EMA）
        a = self.ema_alpha
        self._kappa_ema = (1 - a) * self._kappa_ema + a * kappa_inst
        self._dkappa_ema = (1 - a) * self._dkappa_ema + a * dkappa_inst

        # 加权合成 "弧度偏差大幅变动" 强度
        # 用各自满量程归一化后求和，再夹到 [0, +inf)
        kappa = (
            self._kappa_ema / self.kappa_full
            + self._dkappa_ema / self.dkappa_full
        )
        return max(kappa, 0.0)

    # ---------- 速度曲线 ----------
    def _vx_from_kappa(self, kappa: float) -> float:
        """kappa 越大 → vx 越接近 v_min；kappa=0 → vx=v_max。

        用 ``exp(-kappa)`` 单调衰减：
        - kappa = 0 → vx = v_max（满速）
        - kappa = 1 → vx = v_min + (v_max - v_min) * 0.368
        - kappa → ∞ → vx → v_min
        """
        scale = math.exp(-kappa)
        vx = self.v_min + (self.v_max - self.v_min) * scale
        return max(self.v_min, min(self.v_max, vx))

    # ---------- 横移 / 转向 互斥权重 ----------
    def _axis_mix_from_kappa(self, kappa: float) -> float:
        """数值稳定 sigmoid：kappa 越大 → 越偏向 ω 全权；越小 → 越偏向 vy 全权。

        返回值 ∈ [0, 1]，用作 vy / omega 的「互斥」权重：
            vy_decided   = (1 - axis_mix) * vy_raw
            omega_decided = axis_mix    * omega_raw
        """
        z = (kappa - self.kappa_axis_center) / self.kappa_axis_width
        if z >= 0:
            # exp(-z) 在 z 大时仍然有界；正向分支走 1/(1+exp(-z))
            ez = math.exp(-z)
            return 1.0 / (1.0 + ez)
        # 负向分支走 exp(z)/(1+exp(z))，避免 exp(-z) 数值溢出
        ez = math.exp(z)
        return ez / (1.0 + ez)

    # ---------- 恢复门控 ----------
    def _update_release(self, state: LaneState, dt: float) -> bool:
        """误差是否已经回到小值并稳定 hold_ms？返回 True → 允许全速。"""
        ey = state.error_y
        ea = state.error_angle
        if ey is None or ea is None:
            return False
        if abs(ey) < self.ey_release and abs(ea) < self.ea_release:
            self._straight_streak_ms += dt * 1000.0
        else:
            self._straight_streak_ms = 0.0
        return self._straight_streak_ms >= self.hold_ms

    # ---------- 横向 I 项（leaky 积分） ----------
    def _update_ey_integral(self, state: LaneState, dt: float) -> None:
        """leaky 积分 + 抗饱和。

        为什么 leaky 而不是纯积分：
        - 纯积分在曲线段会"记住"稳态偏差（error_y 不为零很正常），
          过弯后突然反向 → 直线段过冲。
        - leaky 衰减只在持续 bias 上累积：连续多个 frame 同号小偏差才积分起来。
        - 偏差回零后自动衰减（半衰期 ≈ ln(2)/decay），旧的偏差残留不带到下一段。

        抗饱和：用 ey_int_cap 把累积量夹到 [-cap, +cap]，避免 vision 卡死或
        长时间离线导致积分爆炸，下车后会自然衰减。
        """
        ey = float(state.error_y)
        # leaky 累积：I(t+dt) = I(t) * exp(-decay*dt) + ey * dt
        self._ey_integral = (
            self._ey_integral * math.exp(-self.ey_int_decay * dt)
            + ey * dt
        )
        if self._ey_integral > self.ey_int_cap:
            self._ey_integral = self.ey_int_cap
        elif self._ey_integral < -self.ey_int_cap:
            self._ey_integral = -self.ey_int_cap

    # ---------- 主 step ----------
    def step(self, state: LaneState, dt: float) -> List[float]:
        if not state.has_error:
            # 没误差也要清掉恢复门控和积分项，避免上一帧的 streak / 偏差残留
            self._straight_streak_ms = 0.0
            self._prev_ea = None
            self._prev_ea_t = None
            self._ey_integral = 0.0
            return self._safe_zero()

        now = time.monotonic()
        kappa = self._update_curvature(state, now)
        released = self._update_release(state, dt)
        self._update_ey_integral(state, dt)

        # 基础 vx 来自 curvature 曲线；如果误差已稳定放行 → 直接全速
        if released:
            vx = self.v_max
        else:
            vx = self._vx_from_kappa(kappa)

        # 横向修正 = P 项（瞬时偏差）+ I 项（消除稳态 bias）
        vy_raw = -self.kp_y * float(state.error_y) - self.ki_y * self._ey_integral

        # 转向 omega：
        #   基础 P 项 + 弧度大时的额外增益 + 弧度变化率进弯牵引
        # 符号约定：LaneState.error_angle "逆时针为正"，车体坐标系下逆时针 = 左转。
        # error_angle > 0 → 线相对车体轴向左偏 → 应左转（omega > 0），
        # 故 kp_theta 项取正号（与现场验证过的 examples/05_subscribe_lane_state.py 原版一致）。
        # kappa 封顶 1.5（之前 2.0），配合 omega_gain=0.35 → boost 最大 1.525x。
        boost = 1.0 + self.omega_gain * min(kappa, 1.5)
        omega_raw = (
            +self.kp_theta * float(state.error_angle) * boost
            + self.k_curvature * self._dkappa_ema * math.copysign(1.0, state.error_angle or 0.0)
        )
        # omega 软上限：避免急转弯瞬间单轮 r*omega 项把下位机电源拉爆。
        # 这是控制律层的"软闸"，下发层还有 WheelSmoother 做硬闸。
        if omega_raw > self.omega_cap:
            omega_raw = self.omega_cap
        elif omega_raw < -self.omega_cap:
            omega_raw = -self.omega_cap

        # axis_mix 互斥：横向修偏差 vs 转弯
        # 直线段 (kappa 小) → vy 接管、ω≈0 → 朝向不变，质心斜向滑动；
        # 弯道段 (kappa 大) → ω 接管、vy≈0 → 后轮做轴、前轮差速旋转。
        # 过渡带由 kappa_axis_width 控制，过分陡会让 vy/ω 互斥抖动。
        # 2026-07-16: 弯道段保留 vy_floor（默认 0.15）作为 error_y 最小修正能力，
        # 否则完全压零后弯内侧持续压边界。
        self._axis_mix = self._axis_mix_from_kappa(kappa)
        vy_keep = self.vy_floor + (1.0 - self.vy_floor) * (1.0 - self._axis_mix)
        vy_decided = vy_keep * vy_raw
        omega_decided = self._axis_mix * omega_raw

        return mecanum_inverse(vx, vy_decided, omega_decided, self.r_eff)

    # ---------- 调试辅助 ----------
    def debug_snapshot(self) -> dict:
        """给 on_tick 回调用：暴露内部 curvature 估计 / 恢复状态 / 轴向互斥权重 / 积分状态。"""
        vy_keep = self.vy_floor + (1.0 - self.vy_floor) * (1.0 - self._axis_mix)
        return {
            "kappa_ema": self._kappa_ema,
            "dkappa_ema": self._dkappa_ema,
            "straight_streak_ms": self._straight_streak_ms,
            "axis_mix": self._axis_mix,
            "vy_keep": vy_keep,
            "vy_floor": self.vy_floor,
            "ey_int": self._ey_integral,
            "ki_y": self.ki_y,
        }