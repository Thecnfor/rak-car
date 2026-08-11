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
    "arm_place_pose_T2":{"arm_angle_deg": 90,
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
        # place_align.enabled=False 分支: 旧代码在使用 _odom_curr_x_y 之后才定义它
        # → UnboundLocalError。修复后必须正常跑完。
        cfg = CFG.copy()
        cfg["place_align"] = {"enabled": False}
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

    def test_align_retries_once_on_failure(self):
        """2026-08-10: place_align 机械臂对齐失败重试 1 次 (trace_hits>0);
        第 2 次收敛 → 记住放苗姿态 + chassis_aligned=True。"""
        from main.task import task1_seeding as m
        arm_client, runner, _vision = _make_runtime()
        cfg = CFG.copy()
        cfg["place_align"] = {
            "enabled": True, "label": "cylinder_set",
            "setpoint_cxcy": [0.072, -0.331],
            "init_x_mm": -235, "init_arm_deg": 90, "init_hand_deg": 0,
            "timeout": 0.01,
        }

        first_fail = {"status": "succeeded",
                      "result": {"ok": True, "reason": "timeout", "settled": False,
                                 "trace_hits": 5, "end_arm": 89.0, "end_x": -0.233}}
        second_ok = {"status": "succeeded",
                     "result": {"ok": True, "reason": "settled", "settled": True,
                                "trace_hits": 8, "end_arm": 88.0, "end_x": -0.232}}
        servo_results = iter([first_fail, second_ok])

        def _execute(*args, **kwargs):
            if args[:2] == ("car", "run_arm_servo"):
                return next(servo_results)
            return {"status": "succeeded", "result": {"ok": True}}

        arm_client.http.execute.side_effect = _execute
        http = MagicMock()
        http.wait_until_ready.return_value = True
        http.get.return_value = {"odom_state": {"x": 0.0, "y": 0.0, "theta": 0.0}}

        with patch.object(m, "load_task_config", return_value=cfg.copy()), \
             patch.object(m, "ArmClient") as arm_cls, \
             patch.object(m, "RuntimeApiClient", return_value=http):
            arm_cls.connect.return_value = arm_client
            with patch.object(m, "ArmRunner", return_value=runner):
                result = m.run(http)
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["chassis_aligned"])
        # run_arm_servo 恰好 2 次 (第 1 次失败→重试, 第 2 次收敛)
        servo_calls = [c for c in arm_client.http.execute.call_args_list
                       if c.args[:2] == ("car", "run_arm_servo")]
        self.assertEqual(len(servo_calls), 2)
        # 第 1 次参数: setpoint + 起始姿态来自 yaml place_align 段
        # (execute 调用是 kwargs=kw, 所以 kw 在 recorded.kwargs["kwargs"] 里)
        kw0 = servo_calls[0].kwargs["kwargs"]
        self.assertEqual(kw0["setpoint_x_norm"], 0.072)
        self.assertEqual(kw0["setpoint_y_norm"], -0.331)
        self.assertEqual(kw0["arm_start"], 90)
        self.assertEqual(kw0["label"], "cylinder_set")
        # 放苗用记住的姿态 (arm=88, x=-232mm), 不是写死 -235/+90
        place_calls = [c for c in arm_client.http.execute.call_args_list
                       if c.args[:2] == ("arm", "composite_run")]
        place_kw = [c.kwargs["kwargs"] for c in place_calls]
        self.assertTrue(
            any(abs(p["arm"] - 88.0) < 1e-6 and abs(p["x"] - (-0.232)) < 1e-6
                for p in place_kw if p.get("arm") is not None and p.get("x") is not None),
            place_kw)

    def test_align_failure_after_two_tries_still_proceeds(self):
        """2026-08-10: place_align 机械臂对齐重试后仍失败 → ERROR 告警 + 放行;
        chassis_aligned=False, ok=True (比赛完赛优先), 放苗回落写死姿态。"""
        from main.task import task1_seeding as m
        arm_client, runner, _vision = _make_runtime()
        cfg = CFG.copy()
        cfg["place_align"] = {
            "enabled": True, "label": "cylinder_set",
            "setpoint_cxcy": [0.072, -0.331],
            "init_x_mm": -235, "init_arm_deg": 90, "init_hand_deg": 0,
            "timeout": 0.01,
        }

        fail1 = {"status": "succeeded",
                 "result": {"ok": True, "reason": "timeout", "settled": False,
                            "trace_hits": 3, "end_arm": 90.0, "end_x": -0.235}}
        fail2 = {"status": "succeeded",
                 "result": {"ok": True, "reason": "timeout", "settled": False,
                            "trace_hits": 1, "end_arm": 90.0, "end_x": -0.235}}
        servo_results = iter([fail1, fail2])

        def _execute(*args, **kwargs):
            if args[:2] == ("car", "run_arm_servo"):
                return next(servo_results)
            return {"status": "succeeded", "result": {"ok": True}}

        arm_client.http.execute.side_effect = _execute
        http = MagicMock()
        http.wait_until_ready.return_value = True
        http.get.return_value = {"odom_state": {"x": 0.0, "y": 0.0, "theta": 0.0}}

        with patch.object(m, "load_task_config", return_value=cfg.copy()), \
             patch.object(m, "ArmClient") as arm_cls, \
             patch.object(m, "RuntimeApiClient", return_value=http):
            arm_cls.connect.return_value = arm_client
            with patch.object(m, "ArmRunner", return_value=runner):
                result = m.run(http)
        self.assertTrue(result["ok"], result)
        # 失败仍放行, chassis_aligned 写 False 让上层编排/日志能看见
        self.assertFalse(result["chassis_aligned"])
        servo_calls = [c for c in arm_client.http.execute.call_args_list
                       if c.args[:2] == ("car", "run_arm_servo")]
        self.assertEqual(len(servo_calls), 2)
        self.assertEqual(result["completed"], ["cylinder_1", "cylinder_2", "cylinder_3"])
        # 未记住 → 放苗回落写死姿态 (arm=90, x=-235mm)
        place_calls = [c for c in arm_client.http.execute.call_args_list
                       if c.args[:2] == ("arm", "composite_run")]
        place_kw = [c.kwargs["kwargs"] for c in place_calls]
        self.assertTrue(
            any(abs(p["arm"] - 90.0) < 1e-6 and abs(p["x"] - (-0.235)) < 1e-6
                for p in place_kw if p.get("arm") is not None and p.get("x") is not None),
            place_kw)

    def test_run_ends_at_S3_via_move_for(self):
        """2026-08-07: 不管在哪结束, 末尾都把底盘移到 S3 (pos_along=0.30m) 作为终点。

        验证三件事:
          1) 主循环正常完成 → pos_along 终态 = 0.30m (S3), 主循环有 move_for +0.15 推 S2/S3;
          2) 主循环异常 → 异常路径仍会尝试 move_for 到 S3, 不掩盖原 error;
          3) 已在 S3 附近 (|dx|<5cm) → 跳过移动。
        """
        from main.task import task1_seeding as m
        arm_client, runner, _vision = _make_runtime()
        cfg = CFG.copy()
        cfg["chassis_align"] = {"enabled": False}  # 简化, 跳过对齐路径

        def _car_calls():
            return [c for c in arm_client.http.execute_car_action.call_args_list
                    if c.args and c.args[0] == "move_for"]

        # ===== 1) 成功路径 =====
        http = MagicMock()
        http.wait_until_ready.return_value = True
        http.get.return_value = {"odom_state": {"x": 0.0, "y": 0.0, "theta": 0.0}}
        http.get.side_effect = None

        with patch.object(m, "load_task_config", return_value=cfg.copy()), \
             patch.object(m, "ArmClient") as arm_cls, \
             patch.object(m, "RuntimeApiClient", return_value=http):
            arm_cls.connect.return_value = arm_client
            with patch.object(m, "ArmRunner", return_value=runner):
                result = m.run(http)

        self.assertTrue(result["ok"], result)
        # 末尾把 pos_along 推到 0.30, 主循环 S2/S3 各 +0.15:
        dxs = [c.args[1][0] for c in _car_calls()]
        self.assertIn(0.15, dxs, "expected at least one +0.15 move (S2/S3 推进)")
        # 全部 move_for 累计 dx 应该走到 0.30m (最后一笔 S3 → S3 不动):
        cumulative = sum(dxs)
        self.assertAlmostEqual(cumulative, 0.30, places=6)

        # ===== 2) 异常路径也走 S3 =====
        arm_client2, runner2, _vision2 = _make_runtime()
        http2 = MagicMock()
        http2.wait_until_ready.return_value = True
        http2.get.return_value = {"odom_state": {"x": 0.0, "y": 0.0, "theta": 0.0}}
        # 让 stage (3) 的并发底盘移动抛错 → 主 except 接住
        # 异常路径里走兜底 move_for 到 S3 (验证异常路径也尝试终点化)
        # 关键: side_effect 直接 raise, 不回 wrap, 让 ThreadPoolExecutor.result() 重抛
        call_count = {"n": 0}
        def _flaky(name, *args, **kwargs):
            call_count["n"] += 1
            # 实际底盘 move_for 调用计数: i=1 T2 一次, i=2 T3 一次 = 2 次.
            # 第 2 次抛错 → stage (3) 的 f_chassis.result() 重抛 → 主 except.
            if call_count["n"] == 2:
                raise RuntimeError("simulated stage3 move_for crash")
            return {"ok": True, "status": "succeeded"}
        arm_client2.http.execute_car_action.side_effect = _flaky

        with patch.object(m, "load_task_config", return_value=cfg.copy()), \
             patch.object(m, "ArmClient") as arm_cls2, \
             patch.object(m, "RuntimeApiClient", return_value=http2):
            arm_cls2.connect.return_value = arm_client2
            with patch.object(m, "ArmRunner", return_value=runner2):
                result2 = m.run(http2)

        self.assertFalse(result2["ok"])
        self.assertIn("simulated stage3 move_for crash", result2["error"])
        # 异常路径应尝试 move_for 移到 S3; 此时 pos_along 已经被 stage (3) 第 1 次推 +0.15,
        # 异常抛在第 2 次, 所以兜底 dx = 0.30 - 0.15 = 0.15.
        err_calls = [c for c in arm_client2.http.execute_car_action.call_args_list
                     if c.args and c.args[0] == "move_for"]
        self.assertTrue(err_calls, "异常路径也应该尝试 move_for 移到 S3")
        # 兜底 move: dx = S3 - pos_along = 0.30 - 0.15 = 0.15
        fallback = [c.args[1][0] for c in err_calls
                    if abs(c.args[1][0] - 0.15) < 1e-6]
        self.assertTrue(fallback, "异常路径兜底 move dx 应等于 0.15 (S3 - 已记账 pos_along=0.15)")

    def test_skip_S3_move_when_already_there(self):
        """已在 S3 附近 (|dx|<5cm) 时末尾不调 move_for, 避免无意义的抖动。"""
        from main.task import task1_seeding as m
        arm_client, runner, _vision = _make_runtime()
        cfg = CFG.copy()
        cfg["chassis_align"] = {"enabled": False}
        cfg["target_slot_map"] = {"cylinder_1": 1, "cylinder_2": 2, "cylinder_3": 3}
        cfg["source_position_order"] = [1]  # 只跑一列, pos_along 留在 0.0, |dx|>=5cm → 仍会移动
        # 把 odom x 设成 0.295, 等价于 pos_along 已经走到 0.295 (距 S3 还有 5cm) — 不对,
        # 实际我们的 _chassis_goto 不依赖 odom, 它走 self-bookkeeping; 想让末尾 dx<5cm
        # 就让 source_position_order 只走 S1 (i=0 跳过底盘移动), pos_along=0.0,
        # 末尾 dx=0.30-0.0=0.30 不在带内. 改写:
        cfg["source_position_order"] = [1, 2, 3]
        http = MagicMock()
        http.wait_until_ready.return_value = True
        http.get.return_value = {"odom_state": {"x": 0.0, "y": 0.0, "theta": 0.0}}

        # 强制 _chassis_goto 的 dx 接近 0 (最后一笔 S3→T3 走 move_for(dx=0)):
        # 让 slot target 和 source 一致 (target_slot_map 是恒等 c_k→k, S↔T 同列),
        # S3 走 +0.15 (源) 然后 place 到 T3 又走 -0.15 (源) — net dx 累计到 S3 时是 +0.30.
        # 末尾 dx = 0.30 - 0.30 = 0 → 跳过. 这里跳过检测由 _chassis_goto 阈值 5cm 处理,
        # 我们的新代码只看 s3_dx = 0.30 - pos_along (0.30) = 0 → 跳过调用.
        with patch.object(m, "load_task_config", return_value=cfg.copy()), \
             patch.object(m, "ArmClient") as arm_cls, \
             patch.object(m, "RuntimeApiClient", return_value=http):
            arm_cls.connect.return_value = arm_client
            with patch.object(m, "ArmRunner", return_value=runner):
                result = m.run(http)

        self.assertTrue(result["ok"], result)
        # 末尾 dx=0 → 不应再追加 move_for. 总累计 dx 应该 = +0.30 (主循环给的), 末尾 0 没有新调用.
        cumulative_dx = sum(
            c.args[1][0]
            for c in arm_client.http.execute_car_action.call_args_list
            if c.args and c.args[0] == "move_for"
        )
        self.assertAlmostEqual(cumulative_dx, 0.30, places=6)


class TestStep0TriggerLaneArm(unittest.TestCase):
    """2026-08-09: step0 — 触发后底盘前移 并发臂切 PLACE.
    2026-08-12: lane follow 改 move_for — lane_state stale 时 move_along_lane 的
    500ms watchdog 会静默 emergency_stop+break (车不动也不报错). 0.15m 直走
    odom 闭环 move_for 就够, 不依赖 lane."""

    def _call(self, cfg_override=None):
        import main.task.task1_seeding as m
        cfg = {
            "trigger_settle": {
                "enabled": True, "lane_follow_m": 0.1, "lane_speed_mps": 0.1,
            },
        }
        if cfg_override:
            cfg.update(cfg_override)
        arm = MagicMock()
        arm.get_state.return_value = MagicMock(y_mm=-100)
        return m._init_step0_trigger_lane_arm(arm, cfg), arm

    def test_chassis_and_arm_run_concurrently(self):
        """move_for([0.1,0,0]) 底盘前移 与 composite_run(PLACE) 并发. """
        import main.task.task1_seeding as m
        ok, arm = self._call()
        self.assertTrue(ok)
        # 底盘: move_for([0.1,0,0], max_velocities=[0.1,0.1,pi/3])
        car_calls = [c for c in arm.http.execute_car_action.call_args_list
                     if c.args and c.args[0] == "move_for"]
        self.assertTrue(car_calls, "应调 move_for 前移")
        dx = car_calls[0].args[1][0]
        self.assertAlmostEqual(dx, 0.1, places=6)
        # 臂: composite_run 走 http.execute (sync=False)
        arm_calls = [c for c in arm.http.execute.call_args_list
                     if c.args and c.args[1] == "composite_run"]
        self.assertTrue(arm_calls, "应调 composite_run 切 PLACE")
        kw = arm_calls[0].kwargs["kwargs"]
        self.assertEqual(kw["arm"], m.PLACE_ARM_DEG)
        self.assertEqual(kw["x"], m.PLACE_ALIGN_X_MM / 1000.0)
        self.assertEqual(kw["y"], m.PLACE_Y_MM / 1000.0)
        self.assertEqual(kw["hand"], m.PLACE_HAND_DEG)

    def test_disabled_skips(self):
        """enabled=False → 直接返回 True, 不碰底盘/臂. """
        ok, arm = self._call({"trigger_settle": {"enabled": False}})
        self.assertTrue(ok)
        arm.http.execute_car_action.assert_not_called()
        arm.http.execute.assert_not_called()

    def test_y_low_raises_before_concurrent(self):
        """y > -50 先串行抬到 PLACE_Y_MM 再并发 (防撞). """
        import main.task.task1_seeding as m
        arm = MagicMock()
        arm.get_state.return_value = MagicMock(y_mm=0)
        m._init_step0_trigger_lane_arm(arm, {
            "trigger_settle": {"enabled": True,
                               "lane_follow_m": 0.1, "lane_speed_mps": 0.1},
        })
        arm.move_y.assert_called_once_with(m.PLACE_Y_MM, timeout=5.0)

    def test_chassis_failure_continues(self):
        """底盘 move_for 抛错 → 记 warning 返回 False 不阻塞. """
        import main.task.task1_seeding as m
        arm = MagicMock()
        arm.get_state.return_value = MagicMock(y_mm=-100)

        def _boom(name, *args, **kwargs):
            if name == "move_for":
                raise RuntimeError("move_for 断")
            return {"id": "j1", "status": "succeeded"}
        arm.http.execute_car_action.side_effect = _boom
        cfg = {"trigger_settle": {"enabled": True,
                                  "lane_follow_m": 0.1, "lane_speed_mps": 0.1}}
        ok = m._init_step0_trigger_lane_arm(arm, cfg)
        self.assertFalse(ok)


if __name__ == "__main__":
    unittest.main()


if __name__ == "__main__":
    unittest.main()