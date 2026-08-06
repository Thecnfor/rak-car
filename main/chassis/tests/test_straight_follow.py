"""main/chassis/tests/test_straight_follow.py
直道控制律单测 (stdlib unittest, 离线无硬件)。

覆盖:
  - error_y 超死区 → vy 横移回正与 vx 巡航正交合成下发 (mecanum_inverse(vx, vy, 0))
  - error_y 在死区内 → vy=0, 纯巡航轮速
  - vy 超过 strafe_v → 钳位
  - kd_y 阻尼: 误差快速归零时压回 vy（防回正过头 → 回正后重新偏移），首帧不误触发
  - error_angle 超死区 → ω 视觉航向纠正与 vx/vy 正交合成下发
  - error_angle 在死区内 → ω=0；ω 超过 omega_max → 钳位
  - ω cross-track：error_angle 零区（模型把 0 量化为角度范围，角度通道瞎）内靠
    error_y → ω 反推真实平行——车头不平行→横向漂移→error_y 变化→纠正到漂移归零；
    error_y>0（车在线右）→ 左转拉回。解决"零区是角度范围 → 收敛不到正中"。
  - ω 积分: 无误差时指数衰减归 0；sign_theta 翻号 → 旋转方向相反
  - OdomTurnPID: θ_target=θ_start+90°, err>0→ω>0, |err|<2°→done（弯道控制器，后续接回）

TestStraightCorrection 组只测 vy 通道：构造时显式 ``ea_target=0.0`` + ``k_ey_omega=0.0``
把 ω 通道（角度项 + cross-track）整体关掉，隔离 vy。ω 收敛行为单独在
TestStraightHeadingTarget 测。
"""
import math
import unittest

from main.chassis.state import LaneState
from main.chassis.controllers.straight import StraightOuterLoop


class TestStraightCorrection(unittest.TestCase):
    def test_error_y_over_deadband_composes_vy_while_cruising(self):
        outer = StraightOuterLoop(deadband_y=0.01, kp_y=1.0, ea_target=0.0, k_ey_omega=0.0)
        composed = outer.step(LaneState(error_y=0.05, error_angle=0.0), 0.02)
        self.assertEqual(outer.corrections, 1)
        # ey=0.05, deadband=0.01 → vy = sign_y*kp_y*(ey-deadband) = +1*1*(0.04) = +0.04
        # 合成帧: mecanum_inverse(vx, +0.04, 0) = [vx+vy, -vx+vy, -vx-vy, vx-vy]
        vx = outer.vx_cruise
        self.assertAlmostEqual(composed[0], vx + 0.04, places=6)
        self.assertAlmostEqual(composed[1], -vx + 0.04, places=6)
        self.assertAlmostEqual(composed[2], -vx - 0.04, places=6)
        self.assertAlmostEqual(composed[3], vx - 0.04, places=6)

    def test_error_y_within_deadband_no_vy(self):
        outer = StraightOuterLoop(deadband_y=0.01, kp_y=1.0, ea_target=0.0, k_ey_omega=0.0)
        composed = outer.step(LaneState(error_y=0.005, error_angle=0.0), 0.02)
        self.assertEqual(outer.corrections, 0)
        # vy=0 → 纯巡航轮速: mecanum_inverse(vx,0,0) = [vx,-vx,-vx,vx]
        self.assertEqual(composed[0], outer.vx_cruise)
        self.assertEqual(composed[1], -outer.vx_cruise)

    def test_error_y_clamps_vy_at_strafe_v(self):
        outer = StraightOuterLoop(deadband_y=0.01, kp_y=1.0, strafe_v=0.20, ea_target=0.0, k_ey_omega=0.0)
        composed = outer.step(LaneState(error_y=0.5, error_angle=0.0), 0.02)
        self.assertEqual(outer.corrections, 1)
        # ey=0.5 → vy=+0.49 → 钳在 strafe_v=+0.20; wheel[1] = -vx+vy
        self.assertAlmostEqual(composed[1], -outer.vx_cruise + 0.20, places=6)

    def test_damping_opposes_overshoot(self):
        # 两帧: 第一帧 ey=0.05 车左移回正; 第二帧 ey=0.048（接近中线、速度沿原方向）。
        # 纯 P 第二帧 vy=+0.038; 带 kd 阻尼的应更小（压惯性，防冲过头）。
        plain = StraightOuterLoop(deadband_y=0.01, kp_y=1.0, kd_y=0.0, ea_target=0.0, k_ey_omega=0.0)
        damped = StraightOuterLoop(deadband_y=0.01, kp_y=1.0, kd_y=0.5, ea_target=0.0, k_ey_omega=0.0)
        for o in (plain, damped):
            o.step(LaneState(error_y=0.05, error_angle=0.0), 0.05)
        vy_plain = plain.step(LaneState(error_y=0.048, error_angle=0.0), 0.05)[0] - plain.vx_cruise
        vy_damped = damped.step(LaneState(error_y=0.048, error_angle=0.0), 0.05)[0] - damped.vx_cruise
        # 纯 P: vy = +kp*(ey-deadband) = +(0.048-0.01) = +0.038
        self.assertAlmostEqual(vy_plain, 0.038, places=6)
        # D: +sign_y*kd*(ey-prev)/dt = +1*0.5*(-0.002/0.05) = -0.02 → vy = +0.018
        self.assertAlmostEqual(vy_damped, 0.018, places=6)
        self.assertLess(abs(vy_damped), abs(vy_plain))

    def test_damping_does_not_fire_on_first_frame(self):
        outer = StraightOuterLoop(deadband_y=0.01, kp_y=1.0, kd_y=0.5, ea_target=0.0, k_ey_omega=0.0)
        composed = outer.step(LaneState(error_y=0.05, error_angle=0.0), 0.05)
        # 首帧无历史 ey → 只有 P 项: vy = +(0.05-0.01) = +0.04
        self.assertAlmostEqual(composed[0], outer.vx_cruise + 0.04, places=6)


class TestStraightHeadingCorrection(unittest.TestCase):
    """ω 视觉航向纠正通道（error_angle → ω，取自 orthogonal.py 旋转通道）。"""

    def test_error_angle_over_deadband_rotates_while_cruising(self):
        outer = StraightOuterLoop(ea_deadband=0.001, kp_theta=2.0, omega_max=0.5)
        composed = outer.step(LaneState(error_y=0.0, error_angle=0.05), 0.02)
        # ea=0.05 → dz=0.049, P=2*0.049=0.098, I 首帧≈0.15*0.049*0.02≈0.00015 → ω≈+0.098
        # 车头偏右(ea>0) → sign_theta=+1 逆时针左转 ω>0; wheel[0] = vx + r*ω
        omega = (composed[0] - outer.vx_cruise) / outer.r_eff
        self.assertGreater(omega, 0.0)
        self.assertLess(omega, 0.5)
        self.assertEqual(outer.corrections, 0)  # ω 帧不算 vy 横移回正

    def test_error_angle_within_deadband_no_omega(self):
        outer = StraightOuterLoop(ea_deadband=0.01, kp_theta=2.0)
        composed = outer.step(LaneState(error_y=0.0, error_angle=0.005), 0.02)
        # ea 在死区内 → ω=0, 纯巡航: [vx,-vx,-vx,vx]
        self.assertEqual(composed[0], outer.vx_cruise)
        self.assertEqual(composed[1], -outer.vx_cruise)

    def test_omega_clamps_at_omega_max(self):
        outer = StraightOuterLoop(ea_deadband=0.001, kp_theta=2.0, omega_max=0.2)
        composed = outer.step(LaneState(error_y=0.0, error_angle=0.5), 0.02)
        # P=2*0.499=0.998 → 钳在 omega_max=0.2 → wheel[0] = vx + r*0.2
        self.assertAlmostEqual(composed[0], outer.vx_cruise + outer.r_eff * 0.2, places=6)

    def test_omega_integral_decays_when_error_gone(self):
        outer = StraightOuterLoop(ea_deadband=0.001, kp_theta=0.0, ki_theta=1.0)
        outer.step(LaneState(error_y=0.0, error_angle=0.1), 0.02)
        outer.step(LaneState(error_y=0.0, error_angle=0.1), 0.02)
        self.assertGreater(outer._ea_integral, 0.0)
        # 误差归零后积分指数衰减（ea_int_decay=0.5 → τ=2s），每帧单调下降
        acc = outer._ea_integral
        for _ in range(50):
            outer.step(LaneState(error_y=0.0, error_angle=0.0), 0.02)
            self.assertLess(outer._ea_integral, acc)
            acc = outer._ea_integral

    def test_sign_theta_flips_rotation_direction(self):
        outer = StraightOuterLoop(ea_deadband=0.001, kp_theta=2.0, sign_theta=-1.0)
        composed = outer.step(LaneState(error_y=0.0, error_angle=0.05), 0.02)
        omega = (composed[0] - outer.vx_cruise) / outer.r_eff
        self.assertLess(omega, 0.0)

    def test_vy_and_omega_compose_independently(self):
        outer = StraightOuterLoop(
            deadband_y=0.01, kp_y=1.0, strafe_v=0.1,
            ea_deadband=0.001, kp_theta=2.0,
        )
        composed = outer.step(LaneState(error_y=0.05, error_angle=0.05), 0.02)
        # vy=+1*(0.05-0.01)=+0.04; ω≈+0.098 → wheel[0] = vx+0.04+0.3*0.098 > vx+0.04
        self.assertGreater(composed[0], outer.vx_cruise + 0.04)
        self.assertEqual(outer.corrections, 1)


class TestStraightHeadingTarget(unittest.TestCase):
    """ω 收敛到与实际车道中心线平行：error_angle 零区（模型把 0 量化为角度范围，
    角度通道读 0 算无误差）内靠 error_y 的 cross-track 项反推真实平行——
    车头不平行 → 横向漂移 → error_y 变化 → ω 纠正到漂移归零。ea_target 默认 0。
    """

    def test_centered_parallel_stays_straight(self):
        outer = StraightOuterLoop()  # ea_target=0, k_ey_omega=0.5
        composed = outer.step(LaneState(error_y=0.0, error_angle=0.0), 0.02)
        # 线上且车头平行：角度项 e=0，cross-track ey=0 → ω=0，纯巡航
        self.assertEqual(composed[0], outer.vx_cruise)
        self.assertEqual(composed[1], -outer.vx_cruise)

    def test_cross_track_turns_toward_line_when_angle_blind(self):
        outer = StraightOuterLoop()
        # error_angle=0（零区，角度通道瞎）但车偏右：cross-track 左转拉回
        right = outer.step(LaneState(error_y=0.05, error_angle=0.0), 0.02)
        omega_r = (right[0] - outer.vx_cruise) / outer.r_eff
        self.assertGreater(omega_r, 0.0)  # 车在线右 → ω>0 左转
        left = outer.step(LaneState(error_y=-0.05, error_angle=0.0), 0.02)
        omega_l = (left[0] - outer.vx_cruise) / outer.r_eff
        self.assertLess(omega_l, 0.0)  # 车在线左 → ω<0 右转
        self.assertAlmostEqual(omega_r, -omega_l, places=6)  # 相对中心对称

    def test_cross_track_respects_sign_theta(self):
        outer = StraightOuterLoop(sign_theta=-1.0)
        composed = outer.step(LaneState(error_y=0.05, error_angle=0.0), 0.02)
        omega = (composed[0] - outer.vx_cruise) / outer.r_eff
        self.assertLess(omega, 0.0)  # sign_theta=-1：cross-track 方向整体翻转

    def test_cross_track_clamped_at_omega_max(self):
        # deadband_y 拉大隔离 vy（error_y=0.45 < 0.5 → vy 不动作），只看 ω 钳位
        outer = StraightOuterLoop(omega_max=0.2, deadband_y=0.5)
        composed = outer.step(LaneState(error_y=0.45, error_angle=0.0), 0.02)
        # cross-track = 0.5*0.45 = 0.225 → 钳在 omega_max=0.2
        omega = (composed[0] - outer.vx_cruise) / outer.r_eff
        self.assertAlmostEqual(omega, 0.2, places=6)

    def test_angle_term_still_works_outside_zero_zone(self):
        outer = StraightOuterLoop()
        composed = outer.step(LaneState(error_y=0.0, error_angle=0.05), 0.02)
        # 零区外：角度项主导（kp_theta*ea≈1.5*0.045）+ cross-track(ey=0)=0 → ω>0 左转
        omega = (composed[0] - outer.vx_cruise) / outer.r_eff
        self.assertGreater(omega, 0.0)


class TestOdomTurnPID(unittest.TestCase):
    def test_pid_rotates_toward_target_and_reports_done(self):
        from main.chassis.controllers.odom_turn import OdomTurnPID
        turn = OdomTurnPID(turn_deg=90.0, tol_deg=2.0, kp=2.0, kd=0.0, omega_max=1.4)
        turn.start(0.0)
        self.assertAlmostEqual(turn.target, math.pi / 2, places=6)
        omega, done = turn.step(0.0, 0.02)   # err=90° → ω>0 正向转
        self.assertGreater(omega, 0.0)
        self.assertFalse(done)
        omega, done = turn.step(1.55, 0.02)  # err≈0.02 rad < 2°
        self.assertTrue(done)
        self.assertEqual(omega, 0.0)

    def test_wrap_pi(self):
        from main.chassis.controllers.odom_turn import wrap_pi
        self.assertAlmostEqual(wrap_pi(1.5 * math.pi), -0.5 * math.pi, places=9)
        self.assertAlmostEqual(wrap_pi(3 * math.pi), math.pi, places=9)
        self.assertAlmostEqual(wrap_pi(-3 * math.pi), math.pi, places=9)


if __name__ == "__main__":
    unittest.main()
