"""main/chassis/controllers/odom_turn.py
里程计 θ 闭环转弯原语 + 纯视觉弯道识别（遗留保留件，非弯道执行主路径）。

弯道执行统一走 turn_v2.TurnV2（转弯时继续视觉巡线 + 外/内侧差速过弯）；
本文件只保留两块被继续使用的部件：

- ``OdomTurnPID``：θ_target = θ_start + turn_deg，ω = PID(θ_target - θ_now)，
  |error| < tol_deg 判定转到位、输出 0 停。供任务固定角度盲转使用
  （orchestrator._turn_theta_deg：task1/task6 结束后掉头）。
- ``CurveDetector``：纯视觉弯道识别（|error_angle| 阈值 + 持续帧），
  喂给 turn_v2.TurnV2 触发。

误差定义：err = wrap_pi(θ_target - θ_now)；ω = kp·err + ki·I + kd·D。
符号约定：err>0（还没转到 θ_target）→ ω>0（朝 theta 增大方向转）。
实车转向反了 → turn_deg 取反（-90°）。
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

    用法（由任务/外环驱动，每帧一次）::

        turn = OdomTurnPID(turn_deg=90.0)
        turn.start(odom.theta)                 # 起点捕获 θ_start
        omega, done = turn.step(odom.theta, dt)  # 每帧
        if done:                               # |θ_target - θ_now| < tol_deg → 停
            ... 下一步

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
        self._d_filt: float = 0.0

    @property
    def active(self) -> bool:
        """是否已 start() 过（有 θ_target）。"""
        return self._target is not None

    @property
    def target(self) -> Optional[float]:
        return self._target

    def start(self, theta_start: float) -> None:
        """转弯入口调用：捕获 θ_start，定 θ_target = θ_start ± turn_deg。"""
        self._start = float(theta_start)
        delta = math.radians(self.turn_deg)
        self._target = self._start + delta
        self._integral = 0.0
        self._prev_err = None
        self._d_filt = 0.0

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
        # D 项：裸 D（odom 20Hz 积分/50Hz 读，θ 台阶波 D 有尖刺，需要时再加低通）
        if self._prev_err is None:
            self._d_filt = 0.0
        else:
            self._d_filt = (err - self._prev_err) / dt
        self._prev_err = err
        omega = self.kp * err + self.ki * self._integral + self.kd * self._d_filt
        return max(-self.omega_max, min(self.omega_max, omega)), False

    def wheels(self, omega: float) -> List[float]:
        """纯旋转（vx=vy=0）的 4 轮线速度。"""
        return mecanum_inverse(0.0, 0.0, omega, self.r_eff)


class CurveDetector:
    """纯视觉弯道识别：|error_angle| 持续 sustain 帧超 detect_tol → 触发。

    返回 direction（±1，喂给 TurnV2.start）；entry_sign 属性记录识别
    时刻 error_angle 的符号（过转判定用）。sign_cal 修正 实车 error_angle→
    转向 映射（stanley 约定 error_angle>0=车头偏右；反了取 -1）。

    阈值（默认）：直道 error_angle 正常噪声 <10°，detect_tol=20° 留裕量；
    sustain=5 帧（50Hz 下 0.1s）去抖。按场地再调。

    rearm_clean：触发后必须连续 rearm_clean 帧干净直道（|ea|≤tol）才允许再触发。
    弯道出口紧接着的十字路口 lane 是垃圾读数，没有冷却会在 0.1s 内重触发，把车转进
    十字路口反复过不去（2026-08-05 实车：45° 弯出口即十字路口）。
    """

    def __init__(self, tol_deg: float = 20.0, sustain: int = 5, sign_cal: int = 1,
                 rearm_clean: int = 0) -> None:
        self.tol = math.radians(float(tol_deg))
        self.sustain = max(1, int(sustain))
        self.sign_cal = 1 if float(sign_cal) >= 0 else -1
        self.rearm_clean = max(0, int(rearm_clean))
        self._count = 0
        self._rearm = 0          # 剩余待清帧：>0 时禁止再触发，只被干净直道递减
        self.entry_sign: int = 1

    def reset(self) -> None:
        self._count = 0
        self._rearm = 0

    def update(self, lane: Optional[LaneState]) -> Optional[int]:
        """每帧调一次：返回 None 或 direction(±1)；entry_sign 同步更新。"""
        if lane is None or not lane.is_fresh or lane.error_angle is None:
            self._count = 0
            return None          # lane 丢不算干净直道，rearm 保持阻塞
        ea = float(lane.error_angle)
        if abs(ea) <= self.tol:
            self._count = 0
            if self._rearm > 0:
                self._rearm -= 1
            return None
        self._count += 1
        if self._count < self.sustain:
            return None
        if self._rearm > 0:
            self._count = 0      # 冷却中：垃圾读数不触发，也不递减 rearm
            return None
        self._count = 0
        self._rearm = self.rearm_clean
        self.entry_sign = 1 if ea > 0 else -1
        return self.sign_cal * self.entry_sign


def demo():
    """离线自检：OdomTurnPID 正/反向转到目标角；CurveDetector 触发与冷却。"""
    # OdomTurnPID 90° 正转
    pid = OdomTurnPID(turn_deg=90.0)
    pid.start(0.0)
    theta = 0.0
    for _ in range(5000):
        omega, done = pid.step(theta, 0.02)
        if done:
            break
        theta += omega * 0.02
    assert done and math.degrees(theta) > 88.0, (done, math.degrees(theta))
    # -90° 反转
    pid = OdomTurnPID(turn_deg=-90.0)
    pid.start(0.0)
    theta = 0.0
    for _ in range(5000):
        omega, done = pid.step(theta, 0.02)
        if done:
            break
        theta += omega * 0.02
    assert done and math.degrees(theta) < -88.0, (done, math.degrees(theta))
    # CurveDetector：|ea|>tol 持续 sustain 帧触发；rearm 冷却内不重触发
    det = CurveDetector(rearm_clean=20)
    fresh = lambda ea: LaneState(error_angle=math.radians(ea), error_y=0.0, age_ms=0)
    for _ in range(4):
        assert det.update(fresh(30.0)) is None, "sustain 前不触发"
    assert det.update(fresh(30.0)) is not None, "第 5 帧触发"
    for _ in range(60):
        assert det.update(fresh(-28.0)) is None, "rearm 冷却内不重触发"
    for _ in range(20):
        det.update(fresh(2.0))          # 干净直道跑完冷却
    assert det.update(fresh(30.0)) is None, "冷却后仍需攒 sustain 帧"
    fired = None
    for _ in range(5):
        fired = det.update(fresh(30.0))
        if fired is not None:
            break
    assert fired is not None, "冷却结束后应能再触发"
    print("odom_turn demo OK")


if __name__ == "__main__":
    demo()


__all__ = ["OdomTurnPID", "CurveDetector", "wrap_pi"]
