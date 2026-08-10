#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task1-8 端到端动作契约测试：跑在 fake runtime（FakeRobotSim 运动学仿真）上。

每个任务用一个独立 TaskHarness 实例 + 固定 fixture，断言：
  1. 任务在墙钟期限内终止（超时 → 急停兜底，视为失败）；
  2. 动作 trace 里出现了该任务的特征物理动作（target/action/args 由
     ActionRecorder 记录，含机械臂关节目标姿态）；
  3. 结束后四轮已归零（安全语义）。

外部能力（ERNIE/OCR/相机帧）按"预期 unsupported"注入确定性 stub：
  - task3/task8：stub subprocess.run，断言 wrapper 契约 + 启动命令；
  - task6：stub order_read_run（无 ERNIE → 无订单），验证任务在
    "读单失败仍继续" 路径上的动作；
  - task7：target OCR 无 cam2 帧 → 空名字 → 干净退出（无需 stub）。
"""
import subprocess
import unittest
from pathlib import Path

from main.testing.task_harness import TaskHarness

LIEBIAO_PATH = Path("main/arm/each_task/task6/.liebiao.json")


class FakeProc:
    """subprocess.run 的替身（记录命令，返回固定 returncode）。"""

    def __init__(self, returncode=0):
        self.returncode = returncode


class TaskEndToEndTests(unittest.TestCase):
    maxDiff = None

    def assert_clean_stop(self, res):
        """安全语义：任务结束时四轮必须归零。"""
        self.assertEqual(
            list(res.final_wheels), [0.0, 0.0, 0.0, 0.0],
            f"task{res.task_id} 结束轮速未归零: {res.final_wheels}",
        )

    def assert_action_counts(self, res, target, **expected):
        actual = dict(res.actions.get(target, {}))
        for action, minimum in expected.items():
            self.assertGreaterEqual(
                actual.get(action, 0), minimum,
                f"task{res.task_id} 缺少 {target}.{action} 动作 "
                f"(实际: {actual}, 至少需要 {minimum})",
            )

    # ---------------- task1：播种（移苗） ----------------

    def test_task1_seeding_pick_place(self):
        h = TaskHarness()
        h.setUp()
        try:
            res = h.run(1)
            self.assertTrue(res.done, "task1 超时未终止: " + res.summary())
            self.assertTrue(res.ok, "task1 ok=False: " + res.summary())
            self.assertTrue(res.result.get("completed"),
                            "task1 未完成任何列: " + res.summary())
            # 特征动作：机械臂复合运动 + 抓放 + 底盘相对位移 + 视觉伺服
            self.assert_action_counts(res, "arm", composite_run=1, grasp=1,
                                      move_y_position=1)
            self.assert_action_counts(res, "car", move_for=1, run_arm_servo=1)
            self.assert_clean_stop(res)
        finally:
            h.tearDown()

    # ---------------- task2：取水（水塔） ----------------

    def test_task2_water_tower(self):
        h = TaskHarness()
        h.setUp()
        try:
            res = h.run(2)
            self.assertTrue(res.done, "task2 超时未终止: " + res.summary())
            self.assertTrue(res.ok, "task2 ok=False: " + res.summary())
            self.assertTrue(res.result.get("completed"),
                            "task2 未处理任何水塔: " + res.summary())
            self.assert_action_counts(res, "arm", composite_run=1, move_y_position=1)
            self.assert_action_counts(res, "car", move_for=1, run_arm_servo=1)
            self.assert_clean_stop(res)
        finally:
            h.tearDown()

    # ---------------- task3：侦察（subprocess wrapper 契约） ----------------

    def test_task3_pest_scout_wrapper_contract(self):
        h = TaskHarness()
        h.setUp()
        try:
            launched = {}

            def fake_run(cmd, check=False, **kw):
                launched["cmd"] = list(cmd)
                return FakeProc(0)

            h.patch("subprocess.run", fake_run)
            res = h.run(3)
            self.assertTrue(res.done, "task3 超时未终止: " + res.summary())
            self.assertTrue(res.ok, "task3 ok=False: " + res.summary())
            self.assertEqual(res.result.get("status"), "ok")
            # wrapper 必须启动识别管线 subprocess，且带 --no-shoot --defer-judge
            self.assertIn("main.task.task3.task3_pipeline", launched["cmd"])
            self.assertIn("--no-shoot", launched["cmd"])
            self.assertIn("--defer-judge", launched["cmd"])
        finally:
            h.tearDown()

    # ---------------- task4：采收（creep 搜索 + 距离退出） ----------------

    def test_task4_harvest_creep_and_distance_exit(self):
        h = TaskHarness()
        h.setUp()
        try:
            # 无 IR 目标 → creep 到距离预算退出（fixtures 在 run 内 reset 后应用）
            res = h.run(4, fixtures=lambda svc: svc.set_ir_distances(left=None, right=None))
            self.assertTrue(res.done, "task4 超时未终止: " + res.summary())
            self.assertTrue(res.ok, "task4 ok=False: " + res.summary())
            detail = res.result.get("detail") or {}
            self.assertIn(detail.get("reason"),
                          ("distance_exit", "zone_cleared", "completed"),
                          "task4 退出原因异常: " + str(detail))
            # 特征动作：creep 速度下发 + 机械臂姿态 + 关仓
            self.assert_action_counts(res, "car", set_chassis_velocity=1,
                                      stop_arm_feed=1, start_arm_feed=1)
            self.assert_action_counts(res, "arm", composite_run=1)
            self.assert_clean_stop(res)
        finally:
            h.tearDown()

    # ---------------- task5：分拣（无视觉 → 降级走完） ----------------

    def test_task5_sort_degraded_trace(self):
        h = TaskHarness()
        h.setUp()
        try:
            res = h.run(5)
            self.assertTrue(res.done, "task5 超时未终止: " + res.summary())
            # 无注入视觉 → 高仓颜色 unknown → ok=False 但流程走完、动作可观测
            self.assertFalse(res.ok, "task5 无视觉应 ok=False: " + res.summary())
            self.assertEqual(res.result.get("rc"), 1)
            self.assert_action_counts(res, "arm", composite_run=1)
            self.assert_action_counts(res, "car", move_for=1,
                                      get_detection_results=1)
            self.assert_clean_stop(res)
        finally:
            h.tearDown()

    # ---------------- task6：接单（OCR stub → 读单失败仍走完） ----------------

    def test_task6_get_order_no_ocr(self):
        h = TaskHarness()
        h.setUp()
        try:
            called = {"n": 0}

            def fake_order_read():
                called["n"] += 1
                return {"ok": False, "orders": [], "error": "fake: 无 ERNIE/OCR"}

            h.patch("main.task.task6_get_order.order_read_run", fake_order_read)
            before = LIEBIAO_PATH.read_text(encoding="utf-8") if LIEBIAO_PATH.exists() else None
            try:
                res = h.run(6)
            finally:
                # 还原写盘副作用（任务把两轮读单写进 .liebiao.json）
                if before is None:
                    LIEBIAO_PATH.unlink(missing_ok=True)
                else:
                    LIEBIAO_PATH.write_text(before, encoding="utf-8")
            self.assertTrue(res.done, "task6 超时未终止: " + res.summary())
            # 读单 stub 应被调用（round1 + round2），不能静默跳过
            self.assertGreaterEqual(called["n"], 2,
                                    "order_read_run 未被调用两次")
            # 任务在"读单失败"下仍应走完推杆/移动/收尾
            self.assertTrue(res.ok, "task6 应 ok=True: " + res.summary())
            self.assert_action_counts(res, "arm", composite_run=1)
            self.assert_action_counts(res, "car", set_wheel_speeds=1)
            self.assert_clean_stop(res)
        finally:
            h.tearDown()

    # ---------------- task7：投放（OCR 空 → 干净退出） ----------------

    def test_task7_deliver_no_ocr(self):
        h = TaskHarness()
        h.setUp()
        try:
            res = h.run(7)
            self.assertTrue(res.done, "task7 超时未终止: " + res.summary())
            self.assertTrue(res.ok, "task7 ok=False: " + res.summary())
            self.assert_action_counts(res, "arm", composite_run=1)
            self.assert_action_counts(res, "car", move_for=1, set_storage_angle=1)
            self.assert_clean_stop(res)
        finally:
            h.tearDown()

    # ---------------- task8：射击（无确认害虫 → 直接跳过） ----------------

    def test_task8_shoot_no_pests(self):
        h = TaskHarness()
        h.setUp()
        try:
            launched = []

            def fake_run(cmd, check=False, **kw):
                launched.append(list(cmd))
                return FakeProc(0)

            h.patch("subprocess.run", fake_run)
            h.patch("main.task.task3_shoot._load_done_manifest",
                    lambda: {"pest_numbers": []})
            res = h.run(8)
            self.assertTrue(res.done, "task8 超时未终止: " + res.summary())
            self.assertTrue(res.ok, "task8 ok=False: " + res.summary())
            self.assertEqual(res.result.get("pest_numbers"), [])
            # 无害虫 → 不应启动射击 subprocess
            self.assertEqual(launched, [], "无害虫时不应启动 shoot_target")
            self.assert_clean_stop(res)
        finally:
            h.tearDown()


if __name__ == "__main__":
    unittest.main()
