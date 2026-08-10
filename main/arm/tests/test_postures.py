#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""姿势库测试：加载 / 校验 / 取姿 / 路由 / 保存。"""
import tempfile
import unittest
from pathlib import Path

from main.arm.postures import PostureLibrary, load_postures
from main.arm.planning import JointPose, JointTrajectory


class PostureLibraryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lib = load_postures()  # 仓库里的 main/arm/postures.yaml

    def test_loads_seeded_tasks(self):
        self.assertIn("task1_seeding", self.lib._data)
        self.assertIn("task4_harvest", self.lib._data)

    def test_resolve_key_int_and_name(self):
        self.assertEqual(self.lib.resolve_key(4), "task4_harvest")
        self.assertEqual(self.lib.resolve_key("task4_harvest"), "task4_harvest")
        with self.assertRaises(KeyError):
            self.lib.resolve_key(99)

    def test_pose_returns_valid_jointpose(self):
        p = self.lib.pose(4, "pose_p")
        self.assertIsInstance(p, JointPose)
        self.assertEqual((p.x_mm, p.y_mm, p.arm_deg, p.hand_deg),
                         (-295.0, -180.0, 90.0, 10.0))

    def test_missing_pose_raises_or_default(self):
        with self.assertRaises(KeyError):
            self.lib.pose(4, "nope")
        p = self.lib.pose(4, "nope", default={"x_mm": 0, "y_mm": -100})
        self.assertEqual(p.x_mm, 0.0)

    def test_scalar_threshold_accessible(self):
        self.assertEqual(self.lib.task(1)["init_y_mm"], -100)
        self.assertEqual(self.lib.task(4)["pick_y_mm"], -65)

    def test_route_returns_posture_list(self):
        r = self.lib.route(4, ["pose_p", "pick", "bin_blue"])
        self.assertEqual(len(r), 3)
        self.assertEqual(r[0].x_mm, -295.0)

    def test_route_close_loops_back_to_start(self):
        r = self.lib.route(4, ["pose_p", "pick"], close=True)
        self.assertEqual(len(r), 3)
        self.assertEqual(r[0], r[-1])  # goal→waypoint→goal 闭环

    def test_plan_builds_smooth_trajectory(self):
        traj = self.lib.plan(4, ["pose_p", "pick", "bin_blue", "pose_p"],
                             close=False)
        self.assertIsInstance(traj, JointTrajectory)
        self.assertGreater(traj.total_time, 0.0)
        # 终点精确回到 pose_p
        end = traj.sample(traj.total_time)
        self.assertEqual((end.x_mm, end.y_mm, end.arm_deg, end.hand_deg),
                         (-295.0, -180.0, 90.0, 10.0))

    def test_validation_rejects_bad_pose_on_load(self):
        bad = {"task9_x": {"pose": {"x_mm": 0, "y_mm": -100,
                                    "arm_deg": 999, "hand_deg": 0}}}
        with self.assertRaises(ValueError):
            PostureLibrary.validate(bad)

    def test_save_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "postures_copy.yaml"
            self.lib.save(str(out))
            reloaded = PostureLibrary(str(out))
            self.assertEqual(reloaded.pose(4, "pose_p"),
                             self.lib.pose(4, "pose_p"))


class TeachJsonTests(unittest.TestCase):
    """示教器导出 JSON（列表，首尾 goal，arm/hand 字段）→ JointPose。"""

    def test_load_teach_json_list_with_arm_hand_fields(self):
        import json
        from main.arm.postures import load_teach_json
        from main.arm.planning import plan_joint_trajectory

        data = [
            {"name": "pose_1", "x_mm": -223.7, "y_mm": -150.1,
             "arm": 90, "hand": -10, "ts": 123},
            {"name": "pose_2", "x_mm": -98.2, "y_mm": -60.3,
             "arm": 24, "hand": -22, "ts": 456},
            {"name": "pose_3", "x_mm": -87.7, "y_mm": -158.6,
             "arm": -82, "hand": -22, "ts": 789},
        ]
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "poses.json"
            p.write_text(json.dumps(data), encoding="utf-8")
            route = load_teach_json(str(p))
        self.assertEqual(len(route), 3)
        self.assertEqual(route[0].arm_deg, 90.0)   # arm → arm_deg 别名
        self.assertEqual(route[2].hand_deg, -22.0)
        # 首尾是 goal，中间是 waypoint → 直接规划成一条连续轨迹
        traj = plan_joint_trajectory(route)
        self.assertEqual(len(traj.legs), 1)
        self.assertEqual(traj.sample(traj.total_time), route[-1])

    def test_load_teach_json_rejects_non_list(self):
        from main.arm.postures import load_teach_json
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "bad.json"
            p.write_text('{"not": "a list"}', encoding="utf-8")
            with self.assertRaises(ValueError):
                load_teach_json(str(p))


if __name__ == "__main__":
    unittest.main()
