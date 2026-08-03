"""main/chassis/controllers/odom_turn.py
里程计 90° 转弯内环：θ_target = θ_start + 90°，ω = PID(θ_target - θ_now)，
|error| < tol_deg 判定转到位、输出 0 停。

输入是里程计 theta（外环读 odom_feed 缓存传进来，本控制器不做 IO），
输出是 ω（再经 mecanum_inverse 转 4 轮速）。弯道识别仍由外环视觉
(|error_angle| 阈值) 负责 —— 本控制器只负责"转 90°"这一段。

误差定义：err = wrap_pi(θ_target - θ_now)；ω = kp·err + ki·I + kd·D。
符号约定：err>0（还没转到 θ_target）→ ω>0（朝 theta 增大方向转）。
实车转向反了 → turn_deg 取反（-90°）。
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from .base import mecanum_inverse


def wrap_pi(angle: float) -> float:
    """把角度卷到 (-π, π]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


class OdomTurnPID:
    """里程计 theta 闭环转弯：θ_target = θ_start + turn_deg。

    用法（由外环驱动，每帧一次）::

        turn = OdomTurnPID(turn_deg=90.0)
        turn.start(odom.theta)                 # 弯道入口捕获 θ_start
        omega, done = turn.step(odom.theta, dt)  # 每帧
        if done:                               # |θ_target - θ_now| < tol_deg → 停
            ... 回直道巡航

    注意 theta 只用"增量"（θ_start → θ_start+90°），不依赖绝对 theta，
    所以不受实车 odom theta 整体漂移影响（drift 只在长期累计时出现）。
    """

    def __init__(
        self,
        *,
        turn_deg: float = 90.0,
        tol_deg: float = 2.0,
        kp: float = 2.2,
        ki: float = 0.35,
        kd: float = 0.06,
        omega_max: float = 1.4,
        int_decay: float = 0.4,
        int_cap: float = 0.35,
        r_eff: float = 0.30,
    ) -> None:
        self.turn_deg = float(turn_deg)
        self.tol = math.radians(abs(float(tol_deg)))
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.omega_max = float(omega_max)
        self.int_decay = float(int_decay)
        self.int_cap = float(int_cap)
        self.r_eff = float(r_eff)
        self._start: Optional[float] = None
        self._target: Optional[float] = None
        self._integral: float = 0.0
        self._prev_err: Optional[float] = None

    @property
    def active(self) -> bool:
        """是否已 start() 过（有 θ_target）。"""
        return self._target is not None

    @property
    def target(self) -> Optional[float]:
        return self._target

    def start(self, theta_start: float) -> None:
        """弯道入口调用：捕获 θ_start，定 θ_target = θ_start ± turn_deg。"""
        self._start = float(theta_start)
        delta = math.radians(self.turn_deg)
        self._target = self._start + delta
        self._integral = 0.0
        self._prev_err = None

    def error(self, theta_now: float) -> float:
        if self._target is None:
            return 0.0
        return wrap_pi(self._target - float(theta_now))

    def step(self, theta_now: float, dt: float) -> Tuple[float, bool]:
        """返回 (omega, done)。done=True 表示已转到 tol 内，omega 恒 0。"""
        err = self.error(theta_now)
        if abs(err) < self.tol:
            self._integral = 0.0
            return 0.0, True
        dt = max(float(dt), 1e-3)
        # 积分指数衰减 + 硬 cap（同 orthogonal.py，防单弯积分留到下一弯 / 风卷）
        self._integral = self._integral * math.exp(-self.int_decay * dt) + err * dt
        if self.int_cap > 0:
            self._integral = max(-self.int_cap, min(self.int_cap, self._integral))
        d = 0.0
        if self._prev_err is not None:
            d = (err - self._prev_err) / dt
        self._prev_err = err
        omega = self.kp * err + self.ki * self._integral + self.kd * d
        return max(-self.omega_max, min(self.omega_max, omega)), False

    def wheels(self, omega: float) -> List[float]:
        """纯旋转（vx=vy=0）的 4 轮线速度。"""
        return mecanum_inverse(0.0, 0.0, omega, self.r_eff)


__all__ = ["OdomTurnPID", "wrap_pi"]
