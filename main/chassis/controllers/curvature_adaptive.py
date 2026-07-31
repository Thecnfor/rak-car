from __future__ import annotations
import math
import time
from typing import List, Optional

from ..state import LaneState
from ..config.lane_follow import (
    DT_VALID_MIN_S,
    DT_VALID_MAX_S,
    KAPPA_HARD_CAP,
)
from .base import OuterLoop, mecanum_inverse


class CurvatureAdaptiveOuterLoop(OuterLoop):
    def __init__(
        self,
        v_max: float = 0.30,
        v_min: float = 0.12,
        kappa_full: float = 0.9,
        dkappa_full: float = 1.4,
        kp_y: float = 0.80,
        kp_theta: float = 1.2,
        kp_theta_straight: float = 0.85,
        ki_y: float = 0.40,
        ey_int_cap: float = 0.10,
        ey_int_decay: float = 0.80,
        ki_theta: float = 0.30,
        ea_int_cap: float = 0.40,
        ea_int_decay: float = 0.50,
        omega_gain: float = 0.35,
        k_curvature: float = 0.25,
        omega_cap: float = 2.8,
        ema_alpha: float = 0.35,
        ey_release: float = 0.02,
        ea_release: float = 0.05,
        hold_ms: float = 250.0,
        kappa_axis_center: float = 1.0,
        kappa_axis_width: float = 0.5,
        vy_floor: float = 0.15,
        omega_floor: float = 0.15,
        r_eff: float = 0.30,
        ki_curve_boost: float = 1.5,
        # ---- 急弯专用参数（kappa >= sharp_kappa_threshold 时生效） ----
        sharp_kappa_threshold: float = 2.5,
        sharp_v_min: float = 0.06,
        sharp_kp_theta: float = 1.50,
        sharp_omega_gain: float = 0.90,
        sharp_ki_curve_boost: float = 2.5,
        sharp_k_curvature: float = 0.55,
    ) -> None:
        self.v_max = float(v_max)
        self.v_min = float(v_min)
        self.kappa_full = max(float(kappa_full), 1e-3)
        self.dkappa_full = max(float(dkappa_full), 1e-3)
        self.kp_y = float(kp_y)
        self.kp_theta = float(kp_theta)
        self.kp_theta_straight = float(kp_theta_straight)
        self.ki_y = float(ki_y)
        self.ey_int_cap = max(float(ey_int_cap), 0.0)
        self.ey_int_decay = max(float(ey_int_decay), 0.0)
        self._ey_integral: float = 0.0
        self.ki_theta = float(ki_theta)
        self.ea_int_cap = max(float(ea_int_cap), 0.0)
        self.ea_int_decay = max(float(ea_int_decay), 0.0)
        self._ea_integral: float = 0.0
        self.omega_gain = float(omega_gain)
        self.k_curvature = float(k_curvature)
        self.omega_cap = max(float(omega_cap), 1e-3)
        self.ema_alpha = float(ema_alpha)
        self.ey_release = float(ey_release)
        self.ea_release = float(ea_release)
        self.hold_ms = float(hold_ms)
        self.kappa_axis_center = float(kappa_axis_center)
        self.kappa_axis_width = max(float(kappa_axis_width), 1e-3)
        self.vy_floor = max(0.0, min(float(vy_floor), 1.0))
        self.omega_floor = max(0.0, min(float(omega_floor), 1.0))
        self._axis_mix: float = 0.0
        self.r_eff = float(r_eff)
        self.ki_curve_boost = max(0.0, float(ki_curve_boost))
        self.sharp_kappa_threshold = max(0.0, float(sharp_kappa_threshold))
        self.sharp_v_min = float(sharp_v_min)
        self.sharp_kp_theta = float(sharp_kp_theta)
        self.sharp_omega_gain = float(sharp_omega_gain)
        self.sharp_ki_curve_boost = max(0.0, float(sharp_ki_curve_boost))
        self.sharp_k_curvature = float(sharp_k_curvature)

        self._kappa_ema: float = 0.0
        self._prev_ea: Optional[float] = None
        self._prev_ea_t: Optional[float] = None
        # _prev_sign 单独存：丢线时 decay 而不是硬复位（#9），
        # 避免下一帧进入弯道符号默认变 +1，引入横跳。
        self._prev_sign: float = 1.0
        self._dkappa_ema: float = 0.0
        self._straight_streak_ms: float = 0.0

    def _update_curvature(self, state: LaneState, now: float) -> float:
        ea = float(state.error_angle) if state.error_angle is not None else 0.0

        kappa_inst = abs(ea)

        dkappa_inst = 0.0
        if (
            self._prev_ea is not None
            and self._prev_ea_t is not None
            and now > self._prev_ea_t
        ):
            dt = now - self._prev_ea_t
            if DT_VALID_MIN_S <= dt <= DT_VALID_MAX_S:
                dkappa_inst = abs((ea - self._prev_ea) / dt)
        self._prev_ea = ea
        self._prev_ea_t = now

        a = self.ema_alpha
        self._kappa_ema = (1 - a) * self._kappa_ema + a * kappa_inst
        self._dkappa_ema = (1 - a) * self._dkappa_ema + a * dkappa_inst

        kappa = (
            self._kappa_ema / self.kappa_full
            + self._dkappa_ema / self.dkappa_full
        )
        return max(kappa, 0.0)

    def _vx_from_kappa(self, kappa: float, v_min: Optional[float] = None) -> float:
        if v_min is None:
            v_min = self.v_min
        scale = math.exp(-kappa)
        vx = v_min + (self.v_max - v_min) * scale
        return max(v_min, min(self.v_max, vx))

    def _axis_mix_from_kappa(self, kappa: float) -> float:
        z = (kappa - self.kappa_axis_center) / self.kappa_axis_width
        if z >= 0:
            ez = math.exp(-z)
            return 1.0 / (1.0 + ez)
        ez = math.exp(z)
        return ez / (1.0 + ez)

    def _update_release(self, state: LaneState, dt: float) -> bool:
        ey = state.error_y
        ea = state.error_angle
        if ey is None or ea is None:
            return False
        if abs(ey) < self.ey_release and abs(ea) < self.ea_release:
            self._straight_streak_ms += dt * 1000.0
        else:
            self._straight_streak_ms = 0.0
        return self._straight_streak_ms >= self.hold_ms

    def _update_ey_integral(self, state: LaneState, dt: float) -> None:
        ey = float(state.error_y)
        self._ey_integral = (
            self._ey_integral * math.exp(-self.ey_int_decay * dt)
            + ey * dt
        )
        if self._ey_integral > self.ey_int_cap:
            self._ey_integral = self.ey_int_cap
        elif self._ey_integral < -self.ey_int_cap:
            self._ey_integral = -self.ey_int_cap

    def _update_ea_integral(self, state: LaneState, dt: float) -> None:
        ea = float(state.error_angle)
        self._ea_integral = (
            self._ea_integral * math.exp(-self.ea_int_decay * dt)
            + ea * dt
        )
        if self._ea_integral > self.ea_int_cap:
            self._ea_integral = self.ea_int_cap
        elif self._ea_integral < -self.ea_int_cap:
            self._ea_integral = -self.ea_int_cap

    def step(self, state: LaneState, dt: float) -> List[float]:
        if not state.has_error:
            self._straight_streak_ms = 0.0
            self._prev_ea = None
            self._prev_ea_t = None
            # _prev_sign 不复位，只 decay（#9）—— 短暂丢线后能记住上次弯向，
            # 重新获线时前馈项符号不会从 +1 跳到 -1（产生横跳）。
            self._prev_sign *= math.exp(-1.0 * dt)
            self._ey_integral = 0.0
            self._ea_integral = 0.0
            return self._safe_zero()

        now = time.monotonic()
        kappa = self._update_curvature(state, now)
        released = self._update_release(state, dt)
        self._update_ey_integral(state, dt)
        self._update_ea_integral(state, dt)

        # 急弯检测：kappa 超过阈值时切到强化参数组
        is_sharp = kappa >= self.sharp_kappa_threshold

        if released:
            vx = self.v_max
        else:
            eff_v_min = self.sharp_v_min if is_sharp else self.v_min
            vx = self._vx_from_kappa(kappa, v_min=eff_v_min)

        eff_ki_boost = self.sharp_ki_curve_boost if is_sharp else self.ki_curve_boost
        i_boost = 1.0 + eff_ki_boost * kappa

        vy_raw = -self.kp_y * float(state.error_y) - self.ki_y * self._ey_integral * i_boost

        eff_omega_gain = self.sharp_omega_gain if is_sharp else self.omega_gain
        boost = 1.0 + eff_omega_gain * min(kappa, KAPPA_HARD_CAP)
        # 前馈符号：用 state.error_angle 实时方向更新 _prev_sign，
        # 而不是用 _prev_ea 兜底（_prev_ea 在 has_error=False 时被复位会丢符号）。
        ea_now = float(state.error_angle)
        if ea_now != 0.0:
            self._prev_sign = math.copysign(1.0, ea_now)

        # 直道 kp_theta 用更小的 straight 值，弯道不变，急弯用 sharp
        self._axis_mix = self._axis_mix_from_kappa(kappa)
        if is_sharp:
            kp_theta_eff = self.sharp_kp_theta
        else:
            kp_theta_eff = self.kp_theta_straight + (self.kp_theta - self.kp_theta_straight) * self._axis_mix
        eff_k_curvature = self.sharp_k_curvature if is_sharp else self.k_curvature
        omega_raw = (
            +kp_theta_eff * ea_now * boost
            + eff_k_curvature * self._dkappa_ema * self._prev_sign
            + self.ki_theta * self._ea_integral * boost * i_boost
        )
        if omega_raw > self.omega_cap:
            omega_raw = self.omega_cap
        elif omega_raw < -self.omega_cap:
            omega_raw = -self.omega_cap

        vy_keep = self.vy_floor + (1.0 - self.vy_floor) * (1.0 - self._axis_mix)
        vy_decided = vy_keep * vy_raw
        omega_keep = self.omega_floor + (1.0 - self.omega_floor) * self._axis_mix
        omega_decided = omega_keep * omega_raw

        return mecanum_inverse(vx, vy_decided, omega_decided, self.r_eff)

    def debug_snapshot(self) -> dict:
        vy_keep = self.vy_floor + (1.0 - self.vy_floor) * (1.0 - self._axis_mix)
        omega_keep = self.omega_floor + (1.0 - self.omega_floor) * self._axis_mix
        kappa_now = max(
            self._kappa_ema / self.kappa_full
            + self._dkappa_ema / self.dkappa_full,
            0.0,
        )
        return {
            "kappa": kappa_now,
            "kappa_ema": self._kappa_ema,
            "dkappa_ema": self._dkappa_ema,
            "is_sharp": kappa_now >= self.sharp_kappa_threshold,
            "straight_streak_ms": self._straight_streak_ms,
            "axis_mix": self._axis_mix,
            "vy_keep": vy_keep,
            "vy_floor": self.vy_floor,
            "omega_keep": omega_keep,
            "omega_floor": self.omega_floor,
            "ey_int": self._ey_integral,
            "ki_y": self.ki_y,
            "ea_int": self._ea_integral,
            "ki_theta": self.ki_theta,
        }
