#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""JointTrajectory 4-DOF 多关键点平滑测试（纯 Python，无硬件）。"""
import unittest

from main.arm.planning import (
    ARM_MAX_DEG, ARM_MIN_DEG, HAND_MAX_DEG, HAND_MIN_DEG,
    JointPose, plan_joint_trajectory,
)

A = JointPose(x_mm=0.0, y_mm=-100.0, arm_deg=-90.0, hand_deg=-90.0)
B = JointPose(x_mm=-120.0, y_mm=-80.0, arm_deg=0.0, hand_deg=-45.0)
C = JointPose(x_mm=-240.0, y_mm=-60.0, arm_deg=90.0, hand_deg=0.0)


class PlanBasicsTests(unittest.TestCase):
    def test_keypoints_passed_exactly_at_segment_boundaries(self):
        traj = plan_joint_trajectory([A, B, C])
        self.assertEqual(traj.sample(0.0), A)
        self.assertEqual(traj.sample(traj.segments[0].T), B)  # 段边界=B
        self.assertEqual(traj.sample(traj.total_time), C)     # 终点=goal

    def test_total_time_is_segment_sum(self):
        traj = plan_joint_trajectory([A, B, C])
        self.assertAlmostEqual(traj.total_time,
                               sum(seg.T for seg in traj.segments))
        self.assertGreater(traj.total_time, 0.0)

    def test_motion_is_monotonic_between_keypoints(self):
        traj = plan_joint_trajectory([A, B])
        prev = traj.sample(0.0)
        t = 0.0
        dt = traj.total_time / 50.0
        while t < traj.total_time:
            t += dt
            pose = traj.sample(t)
            # x 从 0 → -120 单调减
            self.assertLessEqual(pose.x_mm, prev.x_mm + 1e-6)
            # arm 从 -90 → 0 单调增
            self.assertGreaterEqual(pose.arm_deg, prev.arm_deg - 1e-6)
            prev = pose
        self.assertAlmostEqual(prev.x_mm, -120.0, places=6)
        self.assertAlmostEqual(prev.arm_deg, 0.0, places=6)

    def test_stop_waypoints_have_zero_pass_speed(self):
        traj = plan_joint_trajectory([A, B, C])  # 默认全 stop
        self.assertEqual(traj.segments[0].pass_speed, 0.0)
        self.assertEqual(traj.segments[1].pass_speed, 0.0)

    def test_non_stop_interior_waypoint_gets_pass_speed(self):
        b_through = JointPose(x_mm=-120.0, y_mm=-80.0,
                              arm_deg=0.0, hand_deg=-45.0, stop=False)
        traj = plan_joint_trajectory([A, b_through, C])
        self.assertGreater(traj.segments[0].pass_speed, 0.0)
        self.assertEqual(traj.segments[1].pass_speed, 0.0)  # 终点仍停车

    def test_dense_waypoints_include_every_keypoint(self):
        traj = plan_joint_trajectory([A, B, C])
        dense = traj.dense_waypoints(spacing_mm=3.0)
        poses = {(p.x_mm, p.y_mm, p.arm_deg, p.hand_deg) for p in dense}
        for kp in (A, B, C):
            self.assertIn((kp.x_mm, kp.y_mm, kp.arm_deg, kp.hand_deg), poses)
        self.assertEqual(dense[0], A)
        self.assertEqual(dense[-1], C)

    def test_max_speed_scale_slows_trajectory(self):
        t_fast = plan_joint_trajectory([A, C]).total_time
        t_slow = plan_joint_trajectory([A, C], max_speed_scale=0.5).total_time
        self.assertGreater(t_slow, t_fast)

    def test_single_keypoint_raises(self):
        with self.assertRaises(ValueError):
            plan_joint_trajectory([A])

    def test_sample_out_of_range_clamps(self):
        traj = plan_joint_trajectory([A, B])
        self.assertEqual(traj.sample(-10.0), A)
        self.assertEqual(traj.sample(1e9), B)


class JointPoseLimitTests(unittest.TestCase):
    def test_out_of_limit_arm_rejected(self):
        with self.assertRaises(ValueError):
            JointPose(x_mm=0, y_mm=-100, arm_deg=ARM_MAX_DEG + 1, hand_deg=0)
        with self.assertRaises(ValueError):
            JointPose(x_mm=0, y_mm=-100, arm_deg=ARM_MIN_DEG - 1, hand_deg=0)

    def test_out_of_limit_hand_rejected(self):
        with self.assertRaises(ValueError):
            JointPose(x_mm=0, y_mm=-100, arm_deg=0, hand_deg=HAND_MAX_DEG + 1)
        with self.assertRaises(ValueError):
            JointPose(x_mm=0, y_mm=-100, arm_deg=0, hand_deg=HAND_MIN_DEG - 1)

    def test_boundary_values_accepted(self):
        JointPose(x_mm=300.0, y_mm=-200.0, arm_deg=ARM_MAX_DEG, hand_deg=HAND_MAX_DEG)
        JointPose(x_mm=-300.0, y_mm=0.0, arm_deg=ARM_MIN_DEG, hand_deg=HAND_MIN_DEG)

    def test_nan_rejected(self):
        with self.assertRaises(ValueError):
            JointPose(x_mm=float("nan"), y_mm=-100, arm_deg=0, hand_deg=0)


class FromMappingTests(unittest.TestCase):
    def test_mapping_accepts_arm_angle_deg_alias(self):
        p = JointPose.from_mapping(
            {"x_mm": -70, "y_mm": 0, "arm_angle_deg": -95, "hand_angle_deg": 0})
        self.assertEqual(p.arm_deg, -95.0)
        self.assertEqual(p.hand_deg, 0.0)

    def test_mapping_defaults_zero(self):
        p = JointPose.from_mapping({})
        self.assertEqual((p.x_mm, p.y_mm, p.arm_deg, p.hand_deg), (0.0, 0.0, 0.0, 0.0))


class FakeRobotIntegrationTests(unittest.TestCase):
    """姿势库 route → plan_joint_trajectory → FakeRobotSim：关节真的动起来。"""

    def test_route_drives_sim_through_every_keypoint(self):
        from runtime.services.fake_robot import FakeRobotSim

        from main.arm.postures import load_postures

        lib = load_postures()
        traj = lib.plan(4, ["pose_p", "pick", "bin_blue", "pose_p"])
        sim = FakeRobotSim()
        # 每 20ms 把轨迹采样下发到仿真（真机等价于 composite_run 逐点喂）
        n = max(1, int(traj.total_time * 50))
        for i in range(n + 1):
            pose = traj.sample(traj.total_time * i / n)
            sim.composite_move({
                "x_mm": pose.x_mm, "y_mm": pose.y_mm,
                "arm_angle": pose.arm_deg, "hand_angle": pose.hand_deg,
            })
        end = sim.arm_state_mm()
        self.assertAlmostEqual(end["x_mm"], -295.0, places=0)   # pose_p.x
        self.assertAlmostEqual(end["y_mm"], -180.0, places=0)   # pose_p.y
        self.assertAlmostEqual(end["arm_angle"], 90.0, places=0)
        self.assertAlmostEqual(end["hand_angle"], 10.0, places=0)

    def test_each_keypoint_reached_during_sim(self):
        from runtime.services.fake_robot import FakeRobotSim

        from main.arm.postures import load_postures

        lib = load_postures()
        traj = lib.plan(4, ["pose_p", "pick", "bin_blue"])
        sim = FakeRobotSim()
        seen = []
        n = max(1, int(traj.total_time * 50))
        for i in range(n + 1):
            pose = traj.sample(traj.total_time * i / n)
            sim.composite_move({
                "x_mm": pose.x_mm, "y_mm": pose.y_mm,
                "arm_angle": pose.arm_deg, "hand_angle": pose.hand_deg,
            })
            seen.append(sim.arm_state_mm())
        # pick 关键点 (x=-240, y=-65, arm=90, hand=10) 在运动过程中出现过
        hit = any(abs(s["x_mm"] - (-240.0)) < 1.0 and abs(s["y_mm"] - (-65.0)) < 1.0
                  for s in seen)
        self.assertTrue(hit, "轨迹中途未经过 pick 关键点")


if __name__ == "__main__":
    unittest.main()
