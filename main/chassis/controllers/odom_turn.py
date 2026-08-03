"""main/chassis/controllers/odom_turn.py
里程计 90° 转弯内环：θ_target = θ_start + 90°，ω = PID(θ_target - θ_now)，
|error| < tol_deg 判定转到位、输出 0 停。

输入是里程计 theta（外环读 odom_feed 缓存传进来，本控制器不做 IO），
输出是 ω（再经 mecanum_inverse 转 4 轮速）。弯道识别仍由外环视觉
(|error_angle| 阈值) 负责 —— 本控制器只负责"转 90°"这一段。

误差定义：err = wrap_pi(θ_target - θ_now)；ω = kp·err + ki·I + kd·D。
符号约定：err>0（还没转到 θ_target）→ ω>0（朝 theta 增大方向转）。
实车转向反了 → turn_deg 取反（-90°）。

弯角未知（地图只有 45/90/120）→ StaircaseTurn：目标 45→90→120 连续抬升，
配合 lane 回正校验；识别用 CurveDetector（|error_angle| 阈值 + 持续帧）。
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from .base import mecanum_inverse
from ..state import LaneState


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

    def extend(self, increment_deg: float) -> None:
        """连续抬升目标：θ_target += increment（StaircaseTurn 阶梯续转用，中间不停）。

        只动目标、清积分；下一帧 step 以新目标重算 err → ω 从低位直接拉起，
        不经过 0 速帧。"""
        if self._target is None:
            raise RuntimeError("extend() 前必须先 start()")
        self._target += math.radians(float(increment_deg))
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


class StaircaseTurn:
    """阶梯式 θ 闭环转弯：目标 45°→90°→120° 连续抬升，直到 lane 回正。

    适用：地图弯角 ∈ {45°, 90°, 120°}，进弯前不知道是哪个；且弯中 lane 可能
    短暂丢失。策略 = 盲转一段（OdomTurnPID）+ lane 校验，未回正就无缝续转到
    下一档（extend 抬目标，中间不停车）。

    每帧 step(theta_now, dt, lane)：
      1) 连续回正检测（lane 新鲜且已转过 ≥ min_align_rot）：
         |error_angle| ≤ straight_tol_deg 且 |error_y| ≤ straight_tol_y_m → done
         残余误差符号与识别时相反且大（过转）→ done，横向交给直道 vy
      2) 里程碑（|θ_target-θ_now| < tol_deg，目标已到）仍未回正：
         - lane 同向残余大误差 / lane 丢失 → 升档续转（45→90→120）
         - 已到 120 仍不回正 → fail

    回正里的 "error_y 居中" 是理想出口；若航向已平行但 y 未居中，同样 done——
    纯原地转改不了 error_y，死等无意义，横向残余交给直道 vy 通道。

    θ 由调用方喂（同 OdomTurnPID，本类不做 IO）；direction 实车符号未标取反。
    """

    def __init__(
        self,
        targets_deg=(45.0, 90.0, 120.0),
        *,
        tol_deg: float = 2.0,
        straight_tol_deg: float = 6.0,
        straight_tol_y_m: float = 0.05,
        min_align_rot_deg: float = 5.0,
    ) -> None:
        self._targets = [float(t) for t in targets_deg]
        self._increments = [self._targets[0]] + [
            b - a for a, b in zip(self._targets, self._targets[1:])
        ]
        self.tol = math.radians(float(tol_deg))
        self.tol_a = math.radians(float(straight_tol_deg))
        self.tol_y = float(straight_tol_y_m)
        self._min_align_rot = math.radians(float(min_align_rot_deg))
        self.phase: str = "idle"  # idle / turning / done / fail
        self.reason: str = ""     # done/fail 的原因，便于日志
        self._idx = 0
        self._direction = 1
        self._ea_ref = 1          # 弯道识别时刻 error_angle 的符号
        self._theta_start = 0.0
        self._turn = OdomTurnPID()

    @property
    def active(self) -> bool:
        return self.phase == "turning"

    def start(self, theta_start: float, direction: int = 1, ea_ref: int = 1) -> None:
        """弯道入口：direction=±1 转弯方向（实车反了取反）；ea_ref=识别时 error_angle 的符号。"""
        self._direction = 1 if float(direction) >= 0 else -1
        self._ea_ref = 1 if float(ea_ref) >= 0 else -1
        self._idx = 0
        self._theta_start = float(theta_start)
        self._turn = OdomTurnPID(turn_deg=self._direction * self._increments[0])
        self._turn.start(self._theta_start)
        self.phase = "turning"
        self.reason = ""

    def _rotated_enough(self, theta_now: float) -> bool:
        return abs(theta_now - self._theta_start) >= self._min_align_rot

    def _escalate(self) -> None:
        if self._idx + 1 < len(self._targets):
            self._idx += 1
            self._turn.extend(self._direction * self._increments[self._idx])
        else:
            self.phase, self.reason = "fail", "max_angle_still_not_straight"

    def wheels(self, omega: float) -> List[float]:
        """纯旋转（vx=vy=0）的 4 轮线速度，给 runner 直接下发。"""
        return self._turn.wheels(omega)

    def step(self, theta_now: float, dt: float, lane: Optional[LaneState]) -> Tuple[float, str]:
        """返回 (ω, phase)；phase ∈ turning/done/fail。done/fail 后 ω 恒 0。"""
        if self.phase in ("done", "fail"):
            return 0.0, self.phase
        theta_now = float(theta_now)
        # 1) 连续回正检测：转过一点后，lane 一露头说平行就停（不用凑里程碑）
        if (self._rotated_enough(theta_now)
                and lane is not None and lane.is_fresh and lane.error_angle is not None):
            ea = float(lane.error_angle)
            if abs(ea) <= self.tol_a:
                centered = lane.error_y is not None and abs(float(lane.error_y)) <= self.tol_y
                self.phase = "done"
                self.reason = "straight" if centered else "parallel_y_off"
                return 0.0, self.phase
            if abs(ea) > self.tol_a and math.copysign(1.0, ea) != self._ea_ref:
                # 过转：反向残余大误差 → 交给直道 vy 回正
                self.phase, self.reason = "done", "overshoot"
                return 0.0, self.phase
        # 2) 里程碑：当前 θ_target 已到仍未回正 → 升档 / fail
        if abs(self._turn.error(theta_now)) < self.tol:
            lane_ok = lane is not None and lane.is_fresh and lane.error_angle is not None
            if lane_ok:
                ea = float(lane.error_angle)
                if abs(ea) > self.tol_a and math.copysign(1.0, ea) == self._ea_ref:
                    self._escalate()  # 同向残余大误差：还需继续转
            else:
                self._escalate()      # lane 丢：盲升档，往转弯方向扫
            if self.phase == "fail":
                return 0.0, self.phase
        omega, _ = self._turn.step(theta_now, dt)
        return omega, self.phase


class CurveDetector:
    """纯视觉弯道识别：|error_angle| 持续 sustain 帧超 detect_tol → 触发。

    返回 direction（±1，喂给 StaircaseTurn.start）；entry_sign 属性记录识别
    时刻 error_angle 的符号（过转判定用）。sign_cal 修正 实车 error_angle→
    转向 映射（stanley 约定 error_angle>0=车头偏右；反了取 -1）。

    阈值（默认）：直道 error_angle 正常噪声 <10°，detect_tol=20° 留裕量；
    sustain=5 帧（50Hz 下 0.1s）去抖。按场地再调。
    """

    def __init__(self, tol_deg: float = 20.0, sustain: int = 5, sign_cal: int = 1) -> None:
        self.tol = math.radians(float(tol_deg))
        self.sustain = max(1, int(sustain))
        self.sign_cal = 1 if float(sign_cal) >= 0 else -1
        self._count = 0
        self.entry_sign: int = 1

    def reset(self) -> None:
        self._count = 0

    def update(self, lane: Optional[LaneState]) -> Optional[int]:
        """每帧调一次：返回 None 或 direction(±1)；entry_sign 同步更新。"""
        if lane is None or not lane.is_fresh or lane.error_angle is None:
            self._count = 0
            return None
        ea = float(lane.error_angle)
        if abs(ea) <= self.tol:
            self._count = 0
            return None
        self._count += 1
        if self._count < self.sustain:
            return None
        self._count = 0
        self.entry_sign = 1 if ea > 0 else -1
        return self.sign_cal * self.entry_sign


def demo():
    """离线自检：真弯 45/90/120、过转、超限 fail 各跑一遍。"""
    def run(real_deg, lane_at):
        turn = StaircaseTurn()
        turn.start(0.0, direction=1, ea_ref=1)
        theta = 0.0
        for _ in range(5000):
            deg = math.degrees(theta)
            ea, fresh = lane_at(deg)
            state = LaneState(error_angle=math.radians(ea), error_y=0.0,
                              age_ms=0 if fresh else 2000)
            omega, phase = turn.step(theta, 0.02, state)
            if phase != "turning":
                return phase, turn.reason, deg
            theta += omega * 0.02
        return "timeout", turn.reason, deg

    blind_until = lambda d, real: (0.0, d >= real - 2.0)
    # 真 45：43° 前盲转 → 45 里程碑回正 done
    phase, reason, deg = run(45, lambda d: blind_until(d, 45))
    assert phase == "done" and reason == "straight", (phase, reason, deg)
    assert 42 <= deg <= 46, deg
    # 真 90：45 里程碑盲升档 → 90 附近 lane 露头即 done
    phase, reason, deg = run(90, lambda d: blind_until(d, 90))
    assert phase == "done" and reason == "straight", (phase, reason, deg)
    assert 87 <= deg <= 92, deg
    # 真 120：逐档升到 120 → done
    phase, reason, deg = run(120, lambda d: blind_until(d, 120))
    assert phase == "done" and reason == "straight", (phase, reason, deg)
    assert 117 <= deg <= 122, deg
    # 过转（真 30，按 45 转）：45° 处 lane 露头反向残余 → overshoot done
    phase, reason, deg = run(30, lambda d: (0.0, False) if d < 43.0 else (-15.0, True))
    assert phase == "done" and reason == "overshoot", (phase, reason, deg)
    assert 42 <= deg <= 46, deg
    # 超限（真 180，ea 恒 +30 同向）：三档用尽 → fail
    phase, reason, deg = run(180, lambda d: (30.0, True))
    assert phase == "fail" and reason == "max_angle_still_not_straight", (phase, reason, deg)
    print("StaircaseTurn demo OK")


if __name__ == "__main__":
    demo()


__all__ = ["OdomTurnPID", "StaircaseTurn", "CurveDetector", "wrap_pi"]
