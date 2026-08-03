"""main/chassis/tests/test_straight_follow.py
直道控制律单测 (stdlib unittest, 离线无硬件)。

覆盖:
  - error_y 超死区 → vy 横移回正与 vx 巡航正交合成下发 (mecanum_inverse(vx, vy, 0))
  - error_y 在死区内 → vy=0, 纯巡航轮速
  - vy 超过 strafe_v → 钳位
  - OdomTurnPID: θ_target=θ_start+90°, err>0→ω>0, |err|<2°→done（弯道控制器，后续接回）
"""
import math
import unittest

from main.chassis.state import LaneState
from main.chassis.controllers.straight import StraightOuterLoop


class TestStraightCorrection(unittest.TestCase):
    def test_error_y_over_deadband_composes_vy_while_cruising(self):
        outer = StraightOuterLoop(deadband_y=0.01, kp_y=1.0)
        composed = outer.step(LaneState(error_y=0.05, error_angle=0.0), 0.02)
        self.assertEqual(outer.corrections, 1)
        # ey=0.05, deadband=0.01 → vy = sign_y*kp_y*(ey-deadband) = -1*1*(0.04) = -0.04
        # 合成帧: mecanum_inverse(0.25, -0.04, 0) = [vx+vy, -vx+vy, -vx-vy, vx-vy]
        self.assertAlmostEqual(composed[0], 0.25 - 0.04, places=6)
        self.assertAlmostEqual(composed[1], -0.25 - 0.04, places=6)
        self.assertAlmostEqual(composed[2], -0.25 + 0.04, places=6)
        self.assertAlmostEqual(composed[3], 0.25 + 0.04, places=6)

    def test_error_y_within_deadband_no_vy(self):
        outer = StraightOuterLoop(deadband_y=0.01, kp_y=1.0)
        composed = outer.step(LaneState(error_y=0.005, error_angle=0.0), 0.02)
        self.assertEqual(outer.corrections, 0)
        # vy=0 → 纯巡航轮速: mecanum_inverse(vx,0,0) = [vx,-vx,-vx,vx]
        self.assertEqual(composed[0], 0.25)
        self.assertEqual(composed[1], -0.25)

    def test_error_y_clamps_vy_at_strafe_v(self):
        outer = StraightOuterLoop(deadband_y=0.01, kp_y=1.0, strafe_v=0.20)
        composed = outer.step(LaneState(error_y=0.5, error_angle=0.0), 0.02)
        self.assertEqual(outer.corrections, 1)
        # ey=0.5 → vy=-0.49 → 钳在 strafe_v=-0.20; wheel[1] = -vx+vy = -0.45
        self.assertAlmostEqual(composed[1], -0.45, places=6)


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
