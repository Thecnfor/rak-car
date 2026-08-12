"""main/chassis/controllers/turn_v2.py
新版本转弯架构：转弯时**继续视觉巡线**，靠**外/内侧差速**沿弧线过弯。

与旧盲转（odom_turn.OdomTurnPID 固定角度原地 θ 旋转）的根本区别：
  - 不原地转、不停车：弯中保持 vx 前向巡航（继续巡线），ω 由 lane 视觉实时驱动
    （``ω = sign_omega * kp_omega * error_angle``，与 orthogonal / curvature_adaptive
    同符号约定），车头跟着车道弯过去。
  - 差速机制：``wheels() = mecanum_inverse(vx_cruise, 0, omega)``。麦轮逆解（SDK
    vehicle_to_wheel 矩阵，base.py）给 4 轮 ``[vx+rω, -vx+rω, -vx+rω, vx+rω]``：
    对角两轮一组、两组线速度差 2·vx，ω 整体平移 4 轮（纯旋转=4 轮同速，SDK
    约定）→ 车沿半径 vx/ω 的弧线过弯，外侧两轮快、内侧两轮慢，而不是绕质心
    原地打转。
  - 出口 = 弯道结束（lane 重新回正），不是里程计目标角。

状态机（phase ∈ idle/turning/done/fail）：
  1. 识别（由外环 CurveDetector 负责）→ start() 入 turning
  2. turning 每帧：
     - lane 新鲜：回正(|ea|≤straight_tol、转过 min_align_rot、且实际转向已停 |θ̇|<tol_dot)
       连续 straight_sustain 帧 → done "straight"；过转(转过 min_align_rot 后反向大误差
       连续 overshoot_sustain 帧) → done "overshoot"（横向残余交给直道 vy 回正）；
       否则 ω = 视觉巡线继续过弯
     - lane 丢失：沿用最后一次视觉 ω 盲差速续转（从没拿到过视觉才用 fallback_omega），
       lane 回来即恢复视觉
     - 累计转过 max_turn_deg 仍不回正 → fail "max_turn_deg"（兜底）
  3. done/fail 后交还外环（closed_loop._compute_raw 走 outer）

接口与 DoubleLoopRunner 的 ``turn=`` 参数一致（closed_loop.py:205-213）：
  active / start(theta, direction, entry_sign) / step(theta, dt, lane) -> (omega, phase)
  / wheels(omega) / phase / reason。
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from ..state import LaneState
from .base import mecanum_inverse


class TurnV2:
    """转弯时继续视觉巡线 + 外/内侧差速过弯（占位骨架的策略实现）。

    用法（由 DoubleLoopRunner 驱动，每帧一次）::

        turn = TurnV2(vx_cruise=0.20)
        turn.start(theta, direction=1, entry_sign=1)   # 弯道入口（外环 CurveDetector 触发）
        omega, phase = turn.step(theta, dt, lane)      # 每帧
        if phase != "turning":
            ... 回直道巡航
    """

    def __init__(
        self,
        *,
        # 弯中前向巡航（继续巡线的 vx 分量；差速来自 omega，vx 恒定）
        vx_cruise: float = 0.2,
        # 视觉转向：ω = sign_omega * kp_omega * error_angle（同 orthogonal 符号约定，
        # 实车方向反了翻 sign_omega，不用改控制律）
        kp_omega: float = 1.5,
        sign_omega: float = 1,
        omega_max: float = 1.8,
        # lane 丢失且从没拿到过视觉时，盲差速续转 ω（拿到过视觉后沿用最后一次视觉 ω）
        fallback_omega: float = 0.8,
        # 出口判定：转过 min_align_rot 后 |error_angle|≤straight_tol 连续 sustain 帧 → done
        straight_tol_deg: float = 6.0,
        straight_sustain: int = 5,
        min_align_rot_deg: float = 8.0,
        # 出口再加"实际转向已停"门控：低通后的 |θ̇| < tol_dot 才算真回正。
        # 防宽弯里稳态 error_angle 本来就小（e_ss≈vx/(kp·R)，R 大就低于 straight_tol）
        # 被误判成直道提前退出；弯中车一直在转 θ̇≠0，弯真结束才 θ̇→0。
        tol_dot: float = 0.1,
        # 过转判定要连续 overshoot_sustain 帧反向大误差（防弯入口瞬时 ea 反号误杀）
        overshoot_sustain: int = 4,
        # 兜底：累计转过 max_turn_deg 仍不回正 → fail
        max_turn_deg: float = 135.0,
        r_eff: float = 0.30,
    ) -> None:
        self.vx_cruise = float(vx_cruise)
        self.kp_omega = float(kp_omega)
        self.sign_omega = -1.0 if float(sign_omega) < 0 else 1.0
        self.omega_max = float(omega_max)
        self.fallback_omega = float(fallback_omega)
        self.tol_a = math.radians(float(straight_tol_deg))
        self.sustain = max(1, int(straight_sustain))
        self.min_align_rot = math.radians(float(min_align_rot_deg))
        self.max_turn = math.radians(float(max_turn_deg))
        self.r_eff = float(r_eff)
        self.tol_dot = float(tol_dot)
        self.overshoot_sustain = max(1, int(overshoot_sustain))

        self.phase: str = "idle"   # idle / turning / done / fail
        self.reason: str = ""      # done/fail 的原因
        self._start: float = 0.0
        self._ea_ref: int = 1      # 弯道识别时刻 error_angle 的符号（过转判定用）
        self._straight_count: int = 0
        self._overshoot_count: int = 0
        self._last_omega: Optional[float] = None  # 最后一次视觉 ω（lane 丢失续转用）
        self._last_theta: Optional[float] = None  # θ̇ 低通用
        self._theta_dot: float = 0.0

    # ── 接口（closed_loop.py 的 turn= 契约） ────────────────────
    @property
    def active(self) -> bool:
        return self.phase == "turning"

    def start(self, theta_start: float, direction: int = 1, entry_sign: int = 1) -> None:
        """弯道入口：捕获 θ 起点与 error_angle 符号；进入 turning。"""
        self._start = float(theta_start)
        self._ea_ref = 1 if float(entry_sign) >= 0 else -1
        self._straight_count = 0
        self._overshoot_count = 0
        self._last_omega = None
        self._last_theta = float(theta_start)
        self._theta_dot = 0.0
        self.phase = "turning"
        self.reason = ""

    def _rotated_enough(self, theta_now: float) -> bool:
        return abs(float(theta_now) - self._start) >= self.min_align_rot

    def step(self, theta_now: float, dt: float, lane: Optional[LaneState]) -> Tuple[float, str]:
        """返回 (ω, phase)；phase ∈ turning/done/fail。done/fail 后 ω 恒 0。"""
        if self.phase in ("done", "fail"):
            return 0.0, self.phase
        dt = max(float(dt), 1e-3)
        # 实际转向速率（低通）：出口用它区分"宽弯里 ea 本来就小"和"真直道"
        if self._last_theta is not None:
            inst = (float(theta_now) - self._last_theta) / dt
            self._theta_dot += (inst - self._theta_dot) * min(1.0, dt * 8.0)
        self._last_theta = float(theta_now)
        # 兜底：转太多了还没回正
        if abs(float(theta_now) - self._start) >= self.max_turn:
            self.phase, self.reason = "fail", "max_turn_deg"
            return 0.0, self.phase

        if lane is not None and lane.is_fresh and lane.error_angle is not None:
            ea = float(lane.error_angle)
            # 回正：转过 min_align_rot、lane 平行、且实际转向已停（|θ̇| 小）连续 sustain 帧 → done。
            # θ̇ 门控防"宽弯里稳态 ea 小"被误判成直道（车还在转 θ̇≠0，弯真结束才 θ̇→0）。
            if (self._rotated_enough(theta_now)
                    and abs(ea) <= self.tol_a
                    and abs(self._theta_dot) < self.tol_dot):
                self._straight_count += 1
                if self._straight_count >= self.sustain:
                    self.phase, self.reason = "done", "straight"
                    return 0.0, self.phase
            else:
                self._straight_count = 0
            # 过转：转过 min_align_rot 后反向残余大误差，连续 overshoot_sustain 帧 → 交回直道 vy 回正。
            # 攒帧防弯入口瞬时 ea 反号误杀；攒帧期间冻结旋转（不往回打，那该直道 vy 处理）。
            if (self._rotated_enough(theta_now)
                    and abs(ea) > self.tol_a
                    and math.copysign(1.0, ea) != self._ea_ref):
                self._overshoot_count += 1
                if self._overshoot_count >= self.overshoot_sustain:
                    self.phase, self.reason = "done", "overshoot"
                    return 0.0, self.phase
                return 0.0, self.phase
            else:
                self._overshoot_count = 0
            # 视觉巡线：继续跟车道弯过去
            omega = self.sign_omega * self.kp_omega * ea
            self._last_omega = omega
        else:
            # lane 丢失：沿用最后一次视觉 ω 盲差速续转（沿用而不是降 ω，防大弯丢线往外漂）；
            # 从未拿到过视觉（start 后立刻丢线）才用 fallback_omega
            self._straight_count = 0
            self._overshoot_count = 0
            if self._last_omega is not None:
                omega = self._last_omega
            else:
                omega = self.sign_omega * self._ea_ref * self.fallback_omega
        omega = max(-self.omega_max, min(self.omega_max, omega))
        return omega, self.phase

    def wheels(self, omega: float) -> List[float]:
        """继续巡线（vx=vx_cruise）+ ω：麦轮逆解出外/内侧差速，车沿弧线过弯。"""
        return mecanum_inverse(self.vx_cruise, 0.0, float(omega), self.r_eff)


def demo():
    """离线自检：右弯回正、lane 丢失续转、过转、超限 fail、差速 wheel 分布。"""
    def run(turn: TurnV2, lane_at) -> Tuple[str, str, float]:
        turn.start(0.0, direction=1, entry_sign=1)   # 左弯：识别时 ea>0
        theta = 0.0
        for _ in range(10000):
            deg = math.degrees(theta)
            ea, fresh = lane_at(deg)
            state = LaneState(error_angle=math.radians(ea), error_y=0.0,
                              age_ms=0 if fresh else 2000)
            omega, phase = turn.step(theta, 0.02, state)
            if phase != "turning":
                return phase, turn.reason, deg
            theta += omega * 0.02
        return "timeout", turn.reason, deg

    def curve_then_straight(real):
        return lambda d: (35.0, True) if d < real - 2.0 else (0.0, True)

    # 真 90° 左弯：弯中 ea=35°(约 0.61rad) 视觉驱动差速过弯，弯末回正 → done straight
    phase, reason, deg = run(TurnV2(), curve_then_straight(90))
    assert phase == "done" and reason == "straight", (phase, reason, deg)
    assert 87 <= deg <= 92, deg
    # 真 120° 左弯（超过 90 也 OK，视觉跟着弯走，无升档上限）
    phase, reason, deg = run(TurnV2(), curve_then_straight(120))
    assert phase == "done" and reason == "straight", (phase, reason, deg)
    assert 117 <= deg <= 122, deg
    # lane 弯中丢失：盲差速续转，lane 回来自动恢复 → done
    def lane_lost(d):
        if d < 60:
            return 35.0, False   # 弯中丢 lane：盲续转
        if d < 88:
            return 35.0, True    # 60° 后恢复：继续视觉过弯
        return 0.0, True         # 88° 后回正 → done
    phase, reason, deg = run(TurnV2(), lane_lost)
    assert phase == "done" and reason == "straight", (phase, reason, deg)
    assert 85 <= deg <= 90, deg
    # 过转：真 45° 左弯按 90 转，45° 后 lane 反向残余大误差 → overshoot done
    overshoot = lambda d: (35.0, True) if d < 43 else (-25.0, True)
    phase, reason, deg = run(TurnV2(), overshoot)
    assert phase == "done" and reason == "overshoot", (phase, reason, deg)
    assert 42 <= deg <= 48, deg
    # 超限：lane 一直丢 → 盲续转到 max_turn → fail
    phase, reason, deg = run(TurnV2(), lambda d: (0.0, False))
    assert phase == "fail" and reason == "max_turn_deg", (phase, reason, deg)
    assert 133 <= deg <= 136, deg

    # 差速验证：wheels(ω>0) = [vx+rω, -vx+rω, -vx+rω, vx+rω] —— 对角两轮一组同速，
    # 两组差 2·vx（外侧快/内侧慢），ω 整体平移 4 轮
    t = TurnV2(vx_cruise=0.2)
    w = t.wheels(0.5)
    assert w[0] == w[3] == 0.2 + t.r_eff * 0.5, w          # 外侧组
    assert w[1] == w[2] == -0.2 + t.r_eff * 0.5, w         # 内侧组
    assert w[0] - w[1] == 2 * t.vx_cruise, w               # 组间差速 = 2*vx
    print("TurnV2 demo OK")


if __name__ == "__main__":
    demo()


__all__ = ["TurnV2"]
