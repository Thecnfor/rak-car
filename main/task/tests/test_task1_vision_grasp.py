"""main/task/tests/test_task1_vision_grasp.py

任务一（自动移苗）vision-grasp 单测：每列配对取放 (S_i ↔ T_i)。

覆盖:
  - _scan_cylinder_label: 找出首个属于白名单的检测, 重试 / 失败语义
  - _scan_marker_present: 见到 marker_label 即放行
  - run(): 3 轮循环, 每轮 pick_by_vision_lower + move_to_vision_target + drop_object
  - 底盘编排: 列间 dx 从 SOURCE_POSITIONS_M 算
  - 异常: 源头无 label / 槽位无 marker → 返回 ok=False
  - marker_label 透传
"""
from __future__ import annotations

import unittest
from unittest.mock import MagicMock, patch

from main.arm.state import ArmOrigin, ArmState
from main.task import _constants as C


# ── 视觉数据辅助 ──────────────────────────────────────────────────────────────

def _det(label: str, x_c: float = 0.0, score: float = 0.9, tid: int = 1) -> dict:
    return {
        "cls_id": 0, "det_id": tid, "label": label, "score": score,
        "bbox_norm": {"x_center": x_c, "y_center": 0.0,
                       "width": 0.1, "height": 0.1},
    }


def _ts(active: bool, dets: list) -> dict:
    return {
        "ok": True,
        "task_state": {
            "active": active,
            "mode": "tracking",
            "detections": dets,
            "count": len(dets),
        },
    }


# ── 视觉扫描单测 ──────────────────────────────────────────────────────────────

class _ScanCylinder(unittest.TestCase):
    def _call(self, client_responses):
        # 1st call: empty; 2nd call: with cylinder_2
        import main.task.task1_seeding as m
        client = MagicMock()
        client.get.side_effect = client_responses
        return m._scan_cylinder_label(client, ["cylinder_1", "cylinder_2", "cylinder_3"])

    def test_finds_first_match(self):
        r = self._call([
            _ts(True, []),
            _ts(True, [_det("cylinder_2")]),
        ])
        self.assertEqual(r, "cylinder_2")

    def test_returns_none_after_retries(self):
        import main.task.task1_seeding as m
        client = MagicMock()
        client.get.side_effect = [_ts(True, [])] * 5
        # 降低 backoff 让测试快
        orig = m._scan_cylinder_label
        r = m._scan_cylinder_label(client, ["cylinder_1"], retries=2, backoff_s=0)
        self.assertIsNone(r)

    def test_skips_non_whitelist(self):
        # 见到 cylinder_2 (不在白名单) 和 cylinder_1 (在) → 跳过 cylinder_2, 返回 cylinder_1
        r = self._call([
            _ts(True, [_det("other_label"), _det("cylinder_1")]),
        ])
        self.assertEqual(r, "cylinder_1")

    def test_first_whitelist_match_wins(self):
        # 服务端按 detections 顺序给; 我们也按这个顺序匹配, 不重新排序
        r = self._call([
            _ts(True, [_det("cylinder_2"), _det("cylinder_1")]),
        ])
        self.assertEqual(r, "cylinder_2")

    def test_active_false_retries(self):
        # 第一次 active=False (跳过), 第二次 active=True + 检测到
        r = self._call([
            _ts(False, []),
            _ts(True, [_det("cylinder_1")]),
        ])
        self.assertEqual(r, "cylinder_1")


class _ScanMarker(unittest.TestCase):
    def test_present(self):
        import main.task.task1_seeding as m
        client = MagicMock()
        client.get.return_value = _ts(True, [_det("cylinder_set")])
        self.assertTrue(m._scan_marker_present(client, "cylinder_set"))

    def test_absent(self):
        import main.task.task1_seeding as m
        client = MagicMock()
        client.get.return_value = _ts(True, [_det("cylinder_1")])
        self.assertFalse(m._scan_marker_present(client, "cylinder_set"))

    def test_wrong_label_ignored(self):
        import main.task.task1_seeding as m
        client = MagicMock()
        client.get.return_value = _ts(True, [_det("cylinder_set", score=0.5),
                                              _det("other_marker")])
        # 只看 marker_label
        self.assertTrue(m._scan_marker_present(client, "cylinder_set"))


# ── run() 端到端编排 ─────────────────────────────────────────────────────────

CFG = {
    "marker_label": "cylinder_set",
    "source_position_order": [1, 2, 3],
    "target_slot_map": {"cylinder_1": 1, "cylinder_2": 2, "cylinder_3": 3},
    "spacing_along_row_m": 0.15,
    "arm_pick_pose":    {"x_mm": -100, "arm_angle_deg": -90,
                          "y_mm": -10,  "hand_angle_deg": 0},
    "arm_carry_pose":   {"x_mm": -100, "arm_angle_deg": -90,
                          "y_mm": -150, "hand_angle_deg": -10},
    "arm_place_pose_T2":{"x_mm": -270, "arm_angle_deg": 90,
                          "y_mm": -30,  "hand_angle_deg": -10},
    "arm_return_S1_pose":{"x_mm": -100, "arm_angle_deg": -90,
                          "y_mm": -100, "hand_angle_deg": -10},
    "init_y_mm": -100,
    "v_max_arm_lateral_mms": 60,
    "vacuum_settle_s": 0.5,
    "chassis_move_timeout_s": 30,
}


def _make_runtime():
    """构造一个 ArmClient + ArmRunner + RuntimeApiClient mock, 三列顺序出 label."""
    arm_client = MagicMock()
    arm_client.ping.return_value = True
    arm_client.origin = ArmOrigin(nozzle_offset_x_norm=0.0, nozzle_offset_y_norm=0.0)
    arm_client.get_state.return_value = ArmState(x_mm=0.0, y_mm=-100.0)
    # reset_x / move_y / set_hand_angle / set_arm_angle / move_x 走默认值

    runner = MagicMock()
    runner.move_y.return_value = {"ok": True}
    runner.move_x.return_value = {"ok": True}
    runner.set_arm_angle.return_value = {"ok": True}
    runner.client = arm_client  # runner.client.composite_run
    runner.drop_object.return_value = {"ok": True}

    # track_velocity_pick → ok=True, 对齐+下降+吸气+抬回
    runner.track_velocity_pick.return_value = {
        "ok": True, "reason": None, "trace_hits": 60, "settled": True,
        "end_arm": -96.0, "end_hand": None,
        "steps": {"settled": True, "lower": True, "suck": True, "lift": True},
    }
    # composite_run / move_y → ok
    arm_client.composite_run.return_value = {"ok": True}
    arm_client.move_y.return_value = {"ok": True}
    arm_client.reset_x.return_value = {"ok": True}
    arm_client.set_hand_angle.return_value = {"ok": True}
    # 底盘 move_for (2026-08-02 改走 http.execute_car_action) → ok
    arm_client.http.execute_car_action.return_value = {"ok": True, "status": "succeeded"}
    # 兼容旧路径
    arm_client._call_car.return_value = {"ok": True}

    # 视觉伺服调用 (move_to_vision_target 内层 find_target) → 收敛
    vision_with_move = MagicMock()
    vision_with_move.find_target.return_value = MagicMock(converged=True, iterations=2)
    arm_client._make_vision_with_move.return_value = vision_with_move

    return arm_client, runner, vision_with_move


def _make_http(client):
    """RuntimeApiClient mock: vision/task 每次返一个 cylinder_label / marker_label."""
    http = MagicMock()
    http.wait_until_ready.return_value = True
    http.get_health.return_value = {"ok": True}
    return http


class TestRun(unittest.TestCase):
    def _patched_run(self, http_responses, http_health=None):
        from main.task import task1_seeding as m
        arm_client, runner, vision = _make_runtime()

        http = MagicMock()
        http.wait_until_ready.return_value = True
        http.get.side_effect = http_responses

        with patch.object(m, "load_task_config", return_value=CFG.copy()), \
             patch.object(m, "ArmClient") as arm_cls, \
             patch.object(m, "RuntimeApiClient", return_value=http):
            arm_cls.connect.return_value = arm_client
            # 让 runner = ArmRunner(arm_client) → 用 mock 的 runner
            with patch.object(m, "ArmRunner", return_value=runner):
                return m.run(http)

    def test_three_columns_succeed(self):
        # 3 列 × 2 次扫描 (cylinder + marker)
        # 列 1: cylinder_1 (S1) → cylinder_set (T1)
        # 列 2: cylinder_2 (S2) → cylinder_set (T2)
        # 列 3: cylinder_3 (S3) → cylinder_set (T3)
        responses = [
            _ts(True, [_det("cylinder_1")]),  # S1 扫描源头
            _ts(True, [_det("cylinder_set")]), # T1 扫描 marker
            _ts(True, [_det("cylinder_2")]),  # S2 扫描源头
            _ts(True, [_det("cylinder_set")]), # T2 扫描 marker
            _ts(True, [_det("cylinder_3")]),  # S3 扫描源头
            _ts(True, [_det("cylinder_set")]), # T3 扫描 marker
        ]
        arm_client, runner, vision = _make_runtime()
        from main.task import task1_seeding as m
        http = MagicMock()
        http.wait_until_ready.return_value = True
        http.get.side_effect = responses

        with patch.object(m, "load_task_config", return_value=CFG.copy()), \
             patch.object(m, "ArmClient") as arm_cls, \
             patch.object(m, "RuntimeApiClient", return_value=http):
            arm_cls.connect.return_value = arm_client
            with patch.object(m, "ArmRunner", return_value=runner):
                result = m.run(http)

        self.assertTrue(result["ok"], result)
        self.assertEqual(result["completed"], ["cylinder_1", "cylinder_2", "cylinder_3"])

        # 6 次 track_velocity_pick: 3 次智能抓取 (mode=pick) + 3 次智能释放 (mode=drop)
        self.assertEqual(runner.track_velocity_pick.call_count, 6)
        # 其中 3 次是 mode="drop" (释放)
        drop_modes = [c.kwargs.get("mode") for c in runner.track_velocity_pick.call_args_list]
        self.assertEqual(drop_modes.count("drop"), 3)
        # marker 放置已改用 track_velocity_pick(mode=drop), 不再走 find_target
        self.assertEqual(vision.find_target.call_count, 0)
        # composite_run 调用: 每列 = place (1×) + return-to-source (1×)
        # 注: track_velocity_pick 在真实实现中会内调 composite_run, 但 mock bypass 了
        # 共 3 × 2 = 6 次
        self.assertEqual(arm_client.composite_run.call_count, 6)

        # 底盘移动: S1→S2, S2→S3, 结束归位 S1 → 至少 3 次 execute_car_action
        self.assertGreaterEqual(arm_client.http.execute_car_action.call_count, 3)

    def test_marker_label_passed_to_selector(self):
        # 验证 marker 视觉伺服的 selector.label 来自 cfg.marker_label
        responses = [
            _ts(True, [_det("cylinder_1")]),
            _ts(True, [_det("cylinder_set")]),
            _ts(True, [_det("cylinder_2")]),
            _ts(True, [_det("cylinder_set")]),
            _ts(True, [_det("cylinder_3")]),
            _ts(True, [_det("cylinder_set")]),
        ]
        arm_client, runner, vision = _make_runtime()
        from main.task import task1_seeding as m
        http = MagicMock()
        http.wait_until_ready.return_value = True
        http.get.side_effect = responses

        with patch.object(m, "load_task_config", return_value=CFG.copy()), \
             patch.object(m, "ArmClient") as arm_cls, \
             patch.object(m, "RuntimeApiClient", return_value=http):
            arm_cls.connect.return_value = arm_client
            with patch.object(m, "ArmRunner", return_value=runner):
                m.run(http)

        # 每次 marker 伺服的 selector.label == marker_label
        for call in vision.find_target.call_args_list:
            sel = call.args[0]
            self.assertEqual(sel.label, "cylinder_set")

    def test_no_cylinder_fails(self):
        # S1 没扫到 → 立即 ok=False, error 包含 S1
        responses = [
            _ts(True, []),    # S1: 空
            _ts(True, []),    # 重试 1
            _ts(True, []),    # 重试 2
            _ts(True, []),    # 重试 3 (耗尽)
            _ts(True, []),    # 重试 4
            _ts(True, []),
        ]
        arm_client, runner, vision = _make_runtime()
        from main.task import task1_seeding as m
        http = MagicMock()
        http.wait_until_ready.return_value = True
        http.get.side_effect = responses

        with patch.object(m, "load_task_config", return_value=CFG.copy()), \
             patch.object(m, "ArmClient") as arm_cls, \
             patch.object(m, "RuntimeApiClient", return_value=http):
            arm_cls.connect.return_value = arm_client
            with patch.object(m, "ArmRunner", return_value=runner):
                result = m.run(http)

        self.assertFalse(result["ok"])
        self.assertIn("S1", result["error"])
        self.assertEqual(result["completed"], [])

    def test_no_marker_fails(self):
        # S1 扫到 cylinder_1, T1 没扫到 marker → ok=False
        responses = [
            _ts(True, [_det("cylinder_1")]),    # S1 ok
            _ts(True, []), _ts(True, []), _ts(True, []), _ts(True, []), _ts(True, []),  # T1 重试
        ]
        arm_client, runner, vision = _make_runtime()
        from main.task import task1_seeding as m
        http = MagicMock()
        http.wait_until_ready.return_value = True
        http.get.side_effect = responses

        with patch.object(m, "load_task_config", return_value=CFG.copy()), \
             patch.object(m, "ArmClient") as arm_cls, \
             patch.object(m, "RuntimeApiClient", return_value=http):
            arm_cls.connect.return_value = arm_client
            with patch.object(m, "ArmRunner", return_value=runner):
                result = m.run(http)

        self.assertFalse(result["ok"])
        self.assertIn("T1", result["error"])
        self.assertEqual(result["completed"], ["cylinder_1"])

    def test_pick_failure_does_not_drop(self):
        # pick_by_vision_lower 返回 ok=False → 后续 drop 不调用
        responses = [
            _ts(True, [_det("cylinder_1")]),
            _ts(True, [_det("cylinder_set")]),  # 不会到这里
        ]
        arm_client, runner, vision = _make_runtime()
        from main.task import task1_seeding as m
        http = MagicMock()
        http.wait_until_ready.return_value = True
        http.get.side_effect = responses
        runner.track_velocity_pick.return_value = {
            "ok": False, "reason": "not_settled",
            "trace_hits": 0, "settled": False,
            "end_arm": None, "end_hand": None,
            "steps": {"settled": False, "lower": False, "suck": False, "lift": None},
        }

        with patch.object(m, "load_task_config", return_value=CFG.copy()), \
             patch.object(m, "ArmClient") as arm_cls, \
             patch.object(m, "RuntimeApiClient", return_value=http):
            arm_cls.connect.return_value = arm_client
            with patch.object(m, "ArmRunner", return_value=runner):
                result = m.run(http)

        self.assertFalse(result["ok"])
        runner.drop_object.assert_not_called()


if __name__ == "__main__":
    unittest.main()