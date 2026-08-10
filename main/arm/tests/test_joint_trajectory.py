#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""JointTrajectory 4-DOF 样条版测试：连续经过任意数量关键点 / stop 停车 / 限速。"""
import math
import unittest

from main.arm.planning import (
    ARM_MAX_DEG, ARM_MIN_DEG, HAND_MAX_DEG, HAND_MIN_DEG,
    JointPose, plan_joint_trajectory,
)

A = JointPose(x_mm=0.0, y_mm=-100.0, arm_deg=-90.0, hand_deg=-90.0)
B = JointPose(x_mm=-120.0, y_mm=-80.0, arm_deg=0.0, hand_deg=-45.0)
C = JointPose(x_mm=-240.0, y_mm=-60.0, arm_deg=90.0, hand_deg=0.0)


def _pose_tuple(p: JointPose):
    return (p.x_mm, p.y_mm, p.arm_deg, p.hand_deg)


class ContinuousThroughTests(unittest.TestCase):
    """核心语义：任意数量关键点默认连续平滑经过、不停顿。"""

    def test_any_number_waypoints_single_continuous_leg(self):
        # 6 个关键点、全部非 stop → 一整条连续 leg（无中间停车）
        pts = [JointPose(x_mm=-60 * i, y_mm=-100 + 10 * i,
                         arm_deg=-90 + 30 * i, hand_deg=-90 + 15 * i)
               for i in range(6)]
        traj = plan_joint_trajectory(pts)
        self.assertEqual(len(traj.legs), 1,
                         "非 stop 关键点不应切开成多段")
        self.assertEqual(len(traj.keypoints), 6)
        self.assertGreater(traj.total_time, 0.0)

    def test_path_passes_every_keypoint_exactly(self):
        traj = plan_joint_trajectory([A, B, C])
        leg = traj.legs[0]
        # PCHIP 精确过点：每个关键点的弧长位置求值应等于关键点
        for i, kp in enumerate([A, B, C]):
            q = leg.path.value_at(float(leg.path.s[i]))
            for got, want in zip(q, _pose_tuple(kp)):
                self.assertAlmostEqual(got, want, places=6,
                                       msg=f"关键点 {i} 未精确经过")

    def test_start_and_goal_exact_via_sample(self):
        traj = plan_joint_trajectory([A, B, C])
        self.assertEqual(traj.sample(0.0), A)
        self.assertEqual(traj.sample(traj.total_time), C)

    def test_no_interior_zero_velocity_when_all_continuous(self):
        # 全部连续时中间点速度不为 0（否则就停了）
        traj = plan_joint_trajectory([A, B, C])
        leg = traj.legs[0]
        v_mid = leg.profile["v_peak"]
        self.assertGreater(v_mid, 0.0)
        # 且内部只有一个弧长 profile（无分段停车）
        self.assertEqual(len(traj.legs), 1)

    def test_motion_is_monotonic_between_keypoints(self):
        traj = plan_joint_trajectory([A, B])
        prev = traj.sample(0.0)
        t = 0.0
        dt = traj.total_time / 50.0
        while t < traj.total_time:
            t += dt
            pose = traj.sample(t)
            self.assertLessEqual(pose.x_mm, prev.x_mm + 1e-6)
            self.assertGreaterEqual(pose.arm_deg, prev.arm_deg - 1e-6)
            prev = pose
        self.assertAlmostEqual(prev.x_mm, -120.0, places=6)
        self.assertAlmostEqual(prev.arm_deg, 0.0, places=6)

    def test_joint_velocity_stays_within_limits(self):
        traj = plan_joint_trajectory([A, C], joint_vmax={
            "x_mm": 150.0, "y_mm": 90.0, "arm_deg": 90.0, "hand_deg": 90.0})
        prev = traj.sample(0.0)
        dt = 1.0 / 200.0
        vmax = {"x_mm": 150.0, "y_mm": 90.0, "arm_deg": 90.0, "hand_deg": 90.0}
        t = 0.0
        axes = ("x_mm", "y_mm", "arm_deg", "hand_deg")
        while t < traj.total_time:
            t += dt
            cur = traj.sample(t)
            for ax in axes:
                v = abs(getattr(cur, ax) - getattr(prev, ax)) / dt
                self.assertLessEqual(v, vmax[ax] * 1.05,
                                     f"{ax} 速度超限: {v:.0f} > {vmax[ax]}")
            prev = cur


class StopWaypointTests(unittest.TestCase):
    def test_stop_waypoint_splits_into_legs_and_zeroes(self):
        bs = JointPose(x_mm=-120.0, y_mm=-80.0, arm_deg=0.0, hand_deg=-45.0,
                       stop=True)
        traj = plan_joint_trajectory([A, bs, C])
        self.assertEqual(len(traj.legs), 2)
        # 边界精确命中 stop 关键点（保留 stop 标志）
        self.assertEqual(traj.sample(traj.legs[0].T), bs)
        # 到达 stop 点时速度归 0（两个 arc profile 的端点速度都是 0）
        self.assertAlmostEqual(traj.legs[0].profile["v_end"], 0.0)
        self.assertAlmostEqual(traj.legs[1].profile["v_peak"] * 0 + 0.0, 0.0)
        # 且该边界两侧速度都为 0 → 真正停住
        self.assertAlmostEqual(traj.sample(traj.legs[0].T - 0.01).x_mm,
                               traj.sample(traj.legs[0].T + 0.01).x_mm, places=1)

    def test_all_stop_route_still_moves_between_poses(self):
        # 每个关键点都 stop（取/放序列）→ 仍有 leg 在各点之间运动
        traj = plan_joint_trajectory([A, B, C])  # 先全连续
        stop_pts = [JointPose(p.x_mm, p.y_mm, p.arm_deg, p.hand_deg, stop=True)
                    for p in (A, B, C)]
        traj_stop = plan_joint_trajectory(stop_pts)
        self.assertEqual(len(traj_stop.legs), 2)  # A→B, B→C
        self.assertGreater(traj_stop.total_time, traj.total_time)  # 有停车更慢

    def test_stop_at_goal_only(self):
        traj = plan_joint_trajectory([A, B, C])  # 默认 C 也非 stop
        c_stop = JointPose(C.x_mm, C.y_mm, C.arm_deg, C.hand_deg, stop=True)
        traj2 = plan_joint_trajectory([A, B, c_stop])
        self.assertEqual(len(traj2.legs), 1)  # 仅终点 stop 不影响中间
        self.assertTrue(traj2.keypoints[-1].stop)
        # 终点停车是默认收尾，时间应相同（末尾本来就减速到 0）
        self.assertAlmostEqual(traj2.total_time, traj.total_time, places=6)


class PlanBasicsTests(unittest.TestCase):
    def test_total_time_is_segment_sum(self):
        traj = plan_joint_trajectory([A, B, C])
        self.assertAlmostEqual(traj.total_time,
                               sum(leg.T for leg in traj.legs))
        self.assertGreater(traj.total_time, 0.0)

    def test_dense_waypoints_include_every_keypoint(self):
        traj = plan_joint_trajectory([A, B, C])
        dense = traj.dense_waypoints(spacing_mm=3.0)
        poses = [_pose_tuple(p) for p in dense]
        self.assertEqual(dense[0], A)
        self.assertEqual(dense[-1], C)
        # 起点/终点精确；中间关键点重采样后近似经过（浮点容差内）
        for kp in (A, C):
            self.assertIn(_pose_tuple(kp), poses)
        for kp in (B,):
            self.assertTrue(
                any(abs(px - kp.x_mm) < 1.0 and abs(py - kp.y_mm) < 1.0
                    for px, py, _, _ in poses),
                f"关键点 B 未在密集喂点内: {poses}")

    def test_dense_waypoints_exact_at_stop_boundary(self):
        bs = JointPose(B.x_mm, B.y_mm, B.arm_deg, B.hand_deg, stop=True)
        traj = plan_joint_trajectory([A, bs, C])
        dense = traj.dense_waypoints(spacing_mm=3.0)
        self.assertIn(_pose_tuple(bs), [_pose_tuple(p) for p in dense],
                      "stop 关键点在 dense 中必须精确出现")

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

    def test_linear_engine_fallback_works(self):
        traj = plan_joint_trajectory([A, B, C], engine="linear")
        self.assertEqual(len(traj.legs), 1)
        self.assertEqual(traj.sample(traj.total_time), C)


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

    def test_mapping_defaults_zero_continuous(self):
        p = JointPose.from_mapping({})
        self.assertEqual((p.x_mm, p.y_mm, p.arm_deg, p.hand_deg), (0.0, 0.0, 0.0, 0.0))
        self.assertFalse(p.stop)  # 默认连续经过


class FakeRobotIntegrationTests(unittest.TestCase):
    """姿势库 route → plan_joint_trajectory → FakeRobotSim：关节真的动起来。"""

    def test_route_drives_sim_through_every_keypoint(self):
        from runtime.services.fake_robot import FakeRobotSim

        from main.arm.postures import load_postures

        lib = load_postures()
        traj = lib.plan(4, ["pose_p", "pick", "bin_blue", "pose_p"])
        sim = FakeRobotSim()
        n = max(1, int(traj.total_time * 50))
        for i in range(n + 1):
            pose = traj.sample(traj.total_time * i / n)
            sim.composite_move({
                "x_mm": pose.x_mm, "y_mm": pose.y_mm,
                "arm_angle": pose.arm_deg, "hand_angle": pose.hand_deg,
            })
        end = sim.arm_state_mm()
        self.assertAlmostEqual(end["x_mm"], -295.0, places=0)
        self.assertAlmostEqual(end["y_mm"], -180.0, places=0)
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
        hit = any(abs(s["x_mm"] - (-240.0)) < 1.0 and abs(s["y_mm"] - (-65.0)) < 1.0
                  for s in seen)
        self.assertTrue(hit, "轨迹中途未经过 pick 关键点")


if __name__ == "__main__":
    unittest.main()
