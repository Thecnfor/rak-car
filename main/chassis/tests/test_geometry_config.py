#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""几何路径配置解析/校验测试（task_config.yml waypoints 段 → PathWaypoint）。"""
import os
import tempfile
import unittest

from main.chassis.planning import load_waypoints_geometry, plan_smooth_path


class GeometryConfigTests(unittest.TestCase):
    def _write(self, body: str):
        handle = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yml", delete=False, encoding="utf-8")
        handle.write(body)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        return handle.name

    def test_parses_geometry_and_skips_mission_only_entries(self):
        path = self._write("""
waypoints:
  - name: start        # 任务触发条目, 无几何坐标 —— 应被跳过
    task_id: 1
    ir_threshold_m: 0.7
    ir_side: right
  - name: turn1
    x_m: 1.0
    y_m: 0.5
    heading_deg: 30.0
    speed_mps: 0.12
    stop: true
  - name: goal
    x_m: 2.0
    y_m: 0.0
""")
        waypoints, params = load_waypoints_geometry(path, spacing_m=0.1)
        self.assertEqual(len(waypoints), 2)
        self.assertEqual(waypoints[0].x_m, 1.0)
        self.assertEqual(waypoints[0].y_m, 0.5)
        self.assertAlmostEqual(waypoints[0].heading_rad, 30.0 * 3.141592653589793 / 180.0)
        self.assertEqual(waypoints[0].speed_mps, 0.12)
        self.assertTrue(waypoints[0].stop)
        self.assertIsNone(waypoints[1].heading_rad)
        self.assertEqual(params["spacing_m"], 0.1)
        self.assertEqual(params["max_speed_mps"], 0.2)

    def test_plans_through_parsed_geometry(self):
        path = self._write("""
waypoints:
  - name: a
    x_m: 1.0
    y_m: 1.0
  - name: b
    x_m: 2.0
    y_m: 0.0
""")
        waypoints, params = load_waypoints_geometry(path, spacing_m=0.25)
        from main.chassis.planning import Pose2D
        smooth = plan_smooth_path(Pose2D(0.0, 0.0), waypoints, Pose2D(3.0, 0.0),
                                  **params)
        self.assertGreater(smooth.length_m, 0.0)
        coords = {(round(s.pose.x_m, 4), round(s.pose.y_m, 4)) for s in smooth.samples}
        self.assertIn((1.0, 1.0), coords)
        self.assertIn((2.0, 0.0), coords)

    def test_rejects_duplicate_geometry_point(self):
        path = self._write("""
waypoints:
  - name: a
    x_m: 1.0
    y_m: 0.0
  - name: b
    x_m: 1.0
    y_m: 0.0
""")
        with self.assertRaises(ValueError):
            load_waypoints_geometry(path)

    def test_rejects_non_finite_or_negative_speed(self):
        path = self._write("""
waypoints:
  - name: a
    x_m: nan
    y_m: 0.0
""")
        with self.assertRaises(ValueError):
            load_waypoints_geometry(path)
        path2 = self._write("""
waypoints:
  - name: a
    x_m: 1.0
    y_m: 0.0
    speed_mps: -0.1
""")
        with self.assertRaises(ValueError):
            load_waypoints_geometry(path2)

    def test_mission_only_yaml_is_backward_compatible(self):
        path = self._write("""
waypoints:
  - name: task1
    task_id: 1
    ir_threshold_m: 0.7
    ir_side: right
    dis_at_least_m: 0.3
  - name: finish
    is_finish: true
""")
        waypoints, params = load_waypoints_geometry(path)
        self.assertEqual(waypoints, [])
        self.assertGreater(params["spacing_m"], 0.0)

    def test_missing_file_raises(self):
        with self.assertRaises(FileNotFoundError):
            load_waypoints_geometry("/nonexistent/task_config.yml")


if __name__ == "__main__":
    unittest.main()
