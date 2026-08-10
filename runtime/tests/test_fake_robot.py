#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""FakeRobotSim 运动学仿真 + 对齐策略注册表单测（纯标准库，无硬件）。"""
import math
import unittest

from runtime.services.fake_robot import (
    ALIGNMENT_STRATEGIES, CoarseFineAligner, FakeRobotSim, HeadingAligner,
    MultiAxisPlan, PidAligner, ProportionalAligner, StanleyAligner,
    _trapezoid, create_aligner, forward_kinematics, plan_axes,
)

POST_RESET_Y_MM = -150.0


class TrajectoryTests(unittest.TestCase):
    def test_trapezoid_short_move_falls_back_to_triangle(self):
        prof = _trapezoid(0.01, v_max=100.0, a_max=400.0)  # 太短达不到 v_max
        self.assertEqual(prof["t_run"], 0.0)
        self.assertLess(prof["v_peak"], 100.0)
        self.assertGreater(prof["t_total"], 0.0)

    def test_plan_reaches_target_exactly_at_T(self):
        starts = {"x_mm": 0.0, "y_mm": 0.0}
        targets = {"x_mm": 100.0, "y_mm": -120.0}
        v = {"x_mm": 100.0, "y_mm": 40.0}
        a = {"x_mm": 400.0, "y_mm": 200.0}
        plan = plan_axes(starts, targets, v, a)
        end = plan.evaluate(plan.T)
        self.assertAlmostEqual(end["x_mm"], 100.0, places=6)
        self.assertAlmostEqual(end["y_mm"], -120.0, places=6)

    def test_samples_monotonic(self):
        plan = plan_axes({"x_mm": 0.0}, {"x_mm": 80.0}, {"x_mm": 100.0},
                         {"x_mm": 400.0})
        vals = [plan.evaluate(t)["x_mm"] for t, _ in plan.samples(12)]
        self.assertEqual(vals, sorted(vals))
        self.assertEqual(vals[-1], 80.0)

    def test_zero_motion_plan_has_zero_T(self):
        plan = plan_axes({"x_mm": 5.0}, {"x_mm": 5.0}, {"x_mm": 100.0},
                         {"x_mm": 400.0})
        self.assertEqual(plan.T, 0.0)


class FakeRobotSimTests(unittest.TestCase):
    def setUp(self):
        self.sim = FakeRobotSim()

    def test_composite_move_clamps_and_ends_at_target(self):
        motion = self.sim.composite_move(
            {"x_mm": 9999.0, "arm_angle": 200.0, "hand_angle": -99.0})
        state = self.sim.arm_state_mm()
        self.assertEqual(state["x_mm"], 300.0)       # 限位 [-300, 300]
        self.assertEqual(state["arm_angle"], 150.0)  # 限位 [-150, 150]
        self.assertEqual(state["hand_angle"], -90.0)  # 限位 [-90, 10]
        self.assertGreater(motion.duration, 0.0)
        self.assertEqual(motion.final["arm_angle"], 150.0)

    def test_reset_position_ends_at_up_then_post_reset_y(self):
        self.sim.reset_position()
        state = self.sim.arm_state_mm()
        self.assertEqual(state["arm_angle"], 90.0)
        self.assertEqual(state["hand_angle"], -90.0)
        self.assertEqual(state["y_mm"], POST_RESET_Y_MM)
        self.assertEqual(state["x_mm"], 0.0)

    def test_reset_y_touches_bottom_then_rises(self):
        self.sim.reset_y()
        self.assertEqual(self.sim.joints["y_mm"].value, POST_RESET_Y_MM)

    def test_move_for_updates_odom_relative(self):
        self.sim.move_for(0.3, -0.1, 0.2)
        self.assertEqual(self.sim.odom["x"], 0.3)
        self.assertEqual(self.sim.odom["y"], -0.1)
        self.assertEqual(self.sim.odom["theta"], 0.2)
        self.assertEqual(self.sim.odom["distance"], 0.4)

    def test_velocity_mode_integrates_on_advance(self):
        self.sim.velocity_x(0.05)
        self.sim.advance(1.0)
        self.assertAlmostEqual(self.sim.joints["x_mm"].value, 50.0, places=6)

    def test_grasp_toggles(self):
        self.sim.grasped = True
        self.assertTrue(self.sim.posture_snapshot()["grasped"])
        self.sim.grasped = False
        self.assertFalse(self.sim.posture_snapshot()["grasped"])

    def test_feeds_default_on_lane_and_arm(self):
        snap = self.sim.posture_snapshot()
        self.assertTrue(self.sim.feeds["lane"])
        self.assertTrue(self.sim.feeds["arm"])
        self.assertFalse(self.sim.feeds["ir"])


class ForwardKinematicsTests(unittest.TestCase):
    def test_rest_posture_gripper_points_down(self):
        # arm=0 水平向前, hand=0 → gripper_angle = 0 - 90 - 0 = -90°（垂直向下）
        fk = forward_kinematics(0.0, 0.0, 0.0, 0.0, l1=0.20, l2=0.12)
        self.assertAlmostEqual(fk["gripper_deg"], -90.0)
        self.assertAlmostEqual(fk["ee_x_m"], 0.2)          # 肘在前
        self.assertAlmostEqual(fk["ee_y_m"], -0.12)        # 爪在肘下方 0.12m

    def test_arm_up_gripper_folds_along_arm(self):
        # arm=+90(UP), hand=-90 → gripper_angle = 90-90+90 = 90°（沿臂上折）
        fk = forward_kinematics(0.0, 0.0, 90.0, -90.0, l1=0.20, l2=0.12)
        self.assertAlmostEqual(fk["gripper_deg"], 90.0)
        self.assertAlmostEqual(fk["ee_x_m"], 0.0)
        self.assertAlmostEqual(fk["ee_y_m"], 0.20 + 0.12)  # 腕在最上

    def test_ee_matches_sim_end_effector_pose(self):
        sim = FakeRobotSim()
        sim.composite_move({"x_mm": 80.0, "y_mm": -60.0,
                            "arm_angle": 45.0, "hand_angle": -20.0})
        fk = forward_kinematics(*[sim.joints[n].value for n in
                                  ("x_mm", "y_mm", "arm_angle", "hand_angle")])
        self.assertAlmostEqual(fk["ee_x_m"], sim.end_effector_pose()["ee_x_m"])
        self.assertAlmostEqual(fk["ee_y_m"], sim.end_effector_pose()["ee_y_m"])


class AlignerTests(unittest.TestCase):
    def test_proportional_x_axis_packet(self):
        p = ProportionalAligner(kp=0.6, v_max=0.2)
        packet = p.step(0.05)
        self.assertEqual(packet["target"], "car")
        self.assertEqual(packet["name"], "set_chassis_velocity")
        self.assertAlmostEqual(packet["kwargs"]["vx"], 0.03)
        self.assertEqual(packet["kwargs"]["vy"], 0.0)
        self.assertEqual(packet["kwargs"]["wz"], 0.0)

    def test_proportional_clamps(self):
        p = ProportionalAligner(kp=0.6, v_max=0.2)
        self.assertAlmostEqual(p.step(100.0)["kwargs"]["vx"], 0.2)
        self.assertAlmostEqual(p.step(-100.0)["kwargs"]["vx"], -0.2)

    def test_pid_integrates_and_clamps(self):
        p = PidAligner(kp=0.5, ki=0.1, kd=0.05, v_max=0.2)
        for _ in range(50):                       # 恒定误差 50 步
            p.step(0.1)
        self.assertGreater(p._i, 0.0)             # 积分持续累积
        packet = p.step(0.1)
        self.assertLessEqual(abs(packet["kwargs"]["vx"]), 0.2)  # 限幅生效

    def test_coarse_fine_deadband(self):
        c = CoarseFineAligner(coarse=0.05, fine=0.005, v_coarse=0.2, v_fine=0.03)
        self.assertEqual(c.step(0.001)["kwargs"]["vx"], 0.0)
        self.assertAlmostEqual(c.step(0.01)["kwargs"]["vx"], 0.03)
        self.assertAlmostEqual(c.step(0.2)["kwargs"]["vx"], 0.2)

    def test_heading_axis_uses_wz(self):
        h = HeadingAligner(kp=1.5, wz_max=0.8)
        self.assertAlmostEqual(h.step(0.5)["kwargs"]["wz"], 0.75)
        self.assertAlmostEqual(h.step(10.0)["kwargs"]["wz"], 0.8)

    def test_stanley_outputs_vx_and_steer(self):
        s = StanleyAligner(v_forward=0.15, k_steer=2.0, max_steer=0.6)
        packet = s.step((0.1, 0.05))
        self.assertAlmostEqual(packet["kwargs"]["vx"], 0.15)
        self.assertLessEqual(abs(packet["kwargs"]["wz"]), 0.6)

    def test_registry_and_unknown(self):
        self.assertEqual(sorted(ALIGNMENT_STRATEGIES),
                         ["coarse_fine", "heading", "pid", "proportional", "stanley"])
        self.assertIsInstance(create_aligner("pid"), PidAligner)
        with self.assertRaises(KeyError):
            create_aligner("nope")


if __name__ == "__main__":
    unittest.main()
