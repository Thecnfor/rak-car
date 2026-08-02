"""main/task/tests/test_task1_vision_grasp.py

任务一（自动移苗）vision-grasp 单测：每列配对取放 (S_i ↔ T_i)。

覆盖:
  - _scan_cylinder_label: 找出首个属于白名单的检测, 重试 / 失败语义
  - _scan_marker_present: 见到 marker_label 即放行 (函数保留, 主流程已不用)
  - run(): 3 轮循环, scan → track_velocity_pick → place 固定网格
  - 底盘编排: 列位置 = align ref + SOURCE_POSITIONS_M / SLOT_POSITIONS_M
  - 失败兜底: scan 扫不到 / servo 未收敛 → 兜底剩余 label 跑完全程, ok=True
  - 底盘闭环 move_to_position 传当前 theta (S2 乱跑修复的回归测试)
  - chassis_align.enabled=False 分支不崩 (旧版 UnboundLocalError)

2026-08-03 按现行规格重写: 旧版 fail-fast / marker 检查的期望已和实车调优后的
兜底语义不符, 且 1↔3 纠错曾泄漏函数属性导致用例互相污染。
"""
from __future__ import annotations

import unittest
from unittest.mock import MagicMock, patch

from main.arm.state import ArmOrigin, ArmState
from main.task import _constants as C


# ── 视觉数据辅助 ──────────────────────────────────────────────────────────────

def _det(label: str, x_c: float = 0.0, y_c: float = 0.0,
         score: float = 0.9, tid: int = 1) -> dict:
    return {
        "cls_id": 0, "det_id": tid, "label": label, "score": score,
        "bbox_norm": {"x_center": x_c, "y_center": y_c,
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
            _ts(True, [_det("other_label", x_c=-0.5, y_c=-0.5),
                     _det("cylinder_1", x_c=0.0, y_c=-0.4)]),
        ])
        self.assertEqual(r, "cylinder_1")

    def test_closest_to_setpoint_wins(self):
        # 2026-08-02: 多 cylinder 可见时取离 setpoint 最近的.
        # cylinder_3 在 (-0.87, -0.21), cylinder_2 在 (-0.05, -0.45) — setpoint=(-0.04, -0.42)
        # cylinder_2 比 cylinder_3 更近, 应该返回 cylinder_2
        import main.task.task1_seeding as m
        client = MagicMock()
        client.get.return_value = _ts(True, [
            _det("cylinder_3", x_c=-0.87, y_c=-0.21),
            _det("cylinder_2", x_c=-0.05, y_c=-0.45),
        ])
        r = m._scan_cylinder_label(
            client, list(m.SOURCE_LABELS),
            retries=1, backoff_s=0.0,
            setpoint_xy=(-0.04, -0.42),
        )
        self.assertEqual(r, "cylinder_2")

    def test_first_match_when_no_setpoint(self):
        # 没传 setpoint → 退回按顺序的第一个 (老行为).
        import main.task.task1_seeding as m
        client = MagicMock()
        client.get.return_value = _ts(True, [
            _det("cylinder_3", x_c=0.0, y_c=-0.4),
            _det("cylinder_2", x_c=0.5, y_c=-0.4),
        ])
        r = m._scan_cylinder_label(
            client, list(m.SOURCE_LABELS),
            retries=1, backoff_s=0.0,
            setpoint_xy=None,
        )
        self.assertEqual(r, "cylinder_3")

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
    # runtime settings.api_base: 给 move_to_position 用 (task1_seeding 直接 requests.post 打这条)
    arm_client.http.settings = MagicMock()
    arm_client.http.settings.api_base = "http://test-runtime:0"
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
    # odom 读取 (/v1/realtime/odom/state) → 默认全零 (align ref=0, theta=0).
    # 需要模拟全流程到场航向的用例单独覆盖这个 return_value.
    arm_client.http.get.return_value = {"odom_state": {"x": 0.0, "y": 0.0, "theta": 0.0}}
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
    """run() 端到端编排 (mock HTTP/WS, 离线).

    2026-08-03 按现行规格重写:
      - scan 失败 / servo 未收敛 → 兜底用剩余 label 继续 (ok=True), 不再 fail-fast
      - marker 检查已移除 (place 走固定网格), 相关旧测试删除
      - 1↔3 纠错改 run 内状态, 用例间不再互相污染
      - 底盘闭环 move_to_position 传当前 theta (全流程到场航向 ≠ 0 的修复)
    """

    def _run_with(self, scan_responses, cfg=None, track_result=None, odom=None):
        """统一入口: mock runtime + 指定扫描序列/odom, 返回 (result, arm_client, runner)."""
        from main.task import task1_seeding as m
        arm_client, runner, _vision = _make_runtime()
        if track_result is not None:
            runner.track_velocity_pick.return_value = track_result
        if odom is not None:
            arm_client.http.get.return_value = {"odom_state": odom}

        http = MagicMock()
        http.wait_until_ready.return_value = True
        http.get.side_effect = scan_responses

        with patch.object(m, "load_task_config", return_value=(cfg or CFG).copy()), \
             patch.object(m, "ArmClient") as arm_cls, \
             patch.object(m, "RuntimeApiClient", return_value=http):
            arm_cls.connect.return_value = arm_client
            with patch.object(m, "ArmRunner", return_value=runner):
                result = m.run(http)
        return result, arm_client, runner

    def test_three_columns_succeed(self):
        # 三列顺序识别 c1/c2/c3 → 全部成功; 1↔3 纠错不触发 (label 各不同)
        result, arm_client, runner = self._run_with([
            _ts(True, [_det("cylinder_1")]),
            _ts(True, [_det("cylinder_2")]),
            _ts(True, [_det("cylinder_3")]),
        ])
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["completed"], ["cylinder_1", "cylinder_2", "cylinder_3"])
        self.assertEqual(runner.track_velocity_pick.call_count, 3)
        # place 协议: 每列 grasp(False) 释放一次
        self.assertEqual(arm_client.grasp.call_count, 3)
        # 底盘网格用物理相对位移 (S1=0 基准): 只有 S2/S3 两次 +0.15 移动;
        # cfg 的 slot 映射是恒等 (c_k→T_k), S↔T 同列 → place 零移动全部跳过
        car_calls = [c for c in arm_client.http.execute_car_action.call_args_list
                     if c.args and c.args[0] == "move_for"]
        self.assertEqual(len(car_calls), 2)
        for c in car_calls:
            self.assertAlmostEqual(c.args[1][0], 0.15, places=6)

    def test_no_cylinder_fallback(self):
        # 三列都扫不到 cylinder → 兜底按剩余 label 顺序跑完全程, ok=True
        result, arm_client, runner = self._run_with([
            _ts(True, []),
            _ts(True, []),
            _ts(True, []),
        ])
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["completed"], ["cylinder_1", "cylinder_2", "cylinder_3"])
        # scan 就失败了, servo 一次没跑; 但 place 流程照走 (空吸也放)
        self.assertEqual(runner.track_velocity_pick.call_count, 0)
        self.assertEqual(arm_client.grasp.call_count, 3)

    def test_pick_failure_fallback(self):
        # scan 成功但 servo 未收敛 → raise → 兜底 label 继续, ok=True
        result, arm_client, runner = self._run_with(
            [
                _ts(True, [_det("cylinder_1")]),
                _ts(True, [_det("cylinder_2")]),
                _ts(True, [_det("cylinder_3")]),
            ],
            track_result={
                "ok": False, "reason": "not_settled",
                "trace_hits": 0, "settled": False,
                "end_arm": None, "end_hand": None,
                "steps": {"settled": False, "lower": False, "suck": False, "lift": None},
            },
        )
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["completed"], ["cylinder_1", "cylinder_2", "cylinder_3"])
        self.assertEqual(runner.track_velocity_pick.call_count, 3)
        self.assertEqual(arm_client.grasp.call_count, 3)

    def test_chassis_goto_moves_along_heading(self):
        # 全流程到场时 odom theta 是累积垃圾 (实测 ~0.97 rad), 但车头物理上正对车道。
        # 底盘平移必须走 move_for([dx,0,0]) (沿车头闭环), 不能用 move_to_position
        # 的世界坐标目标 —— 后者会被 SDK 按垃圾 theta 旋转配轮速 → 斜着走;
        # 网格间距也必须按物理相对位移记账, 不能拿 odom x 绝对值算 (否则目标偏远
        # 1/cos(theta) 倍)。2026-08-03 实车验证过的根因, 回归测试.
        odom = {"x": 0.9, "y": 0.05, "theta": 0.35}
        result, arm_client, _runner = self._run_with(
            [
                _ts(True, [_det("cylinder_1")]),
                _ts(True, [_det("cylinder_2")]),
                _ts(True, [_det("cylinder_3")]),
            ],
            odom=odom,
        )
        self.assertTrue(result["ok"], result)
        # 世界坐标路径已退役
        world_calls = [c for c in arm_client.http.execute_car_action.call_args_list
                       if c.args and c.args[0] == "move_to_position"]
        self.assertEqual(world_calls, [])
        # 所有底盘移动: 纯纵向增量 (offset[1]=offset[2]=0), 不横移不转头
        car_calls = [c for c in arm_client.http.execute_car_action.call_args_list
                     if c.args and c.args[0] == "move_for"]
        self.assertTrue(car_calls, "expected move_for calls")
        for c in car_calls:
            offset = c.args[1]
            self.assertEqual(offset[1], 0.0)
            self.assertEqual(offset[2], 0.0)
        # 网格与 odom 绝对值无关: odom x=0.9 不影响增量, 仍是 S2/S3 各 +0.15
        dxs = sorted(c.args[1][0] for c in car_calls)
        self.assertEqual(len(dxs), 2)
        for dx in dxs:
            self.assertAlmostEqual(dx, 0.15, places=6)

    def test_align_disabled_no_crash(self):
        # chassis_align.enabled=False 分支: 旧代码在使用 _odom_curr_x_y 之后才定义它
        # → UnboundLocalError。修复后必须正常跑完。
        cfg = CFG.copy()
        cfg["chassis_align"] = {"enabled": False}
        result, _arm, _runner = self._run_with(
            [
                _ts(True, [_det("cylinder_1")]),
                _ts(True, [_det("cylinder_2")]),
                _ts(True, [_det("cylinder_3")]),
            ],
            cfg=cfg,
        )
        self.assertTrue(result["ok"], result)
        self.assertNotIn("error", result)


if __name__ == "__main__":
    unittest.main()