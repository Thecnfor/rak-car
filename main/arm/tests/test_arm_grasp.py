"""grasp 方法 + pick_by_vision_lower 抓取流程 单测.

协议 (2026-08-01): y 降到 grasp_y_mm(默认 0) 才能吸; 下降后验证到位才吸气。
"""
import unittest
from unittest.mock import MagicMock
from main.arm.api import ArmClient
from main.arm.loops.runner import ArmRunner
from main.arm.state import ArmOrigin, ArmState
from main.arm.vision import TargetSelector, ServoResult, SelectionStrategy


class TestArmClientGrasp(unittest.TestCase):
    def test_grasp_calls_runtime_action_with_value(self):
        http = MagicMock()
        http.execute_arm_action.return_value = {"status": "succeeded", "result": {}}
        client = ArmClient(http=http)
        res = client.grasp(True, timeout=5.0)
        # 透传 runtime "grasp" action, 参数名 value (SDK arm_base.grasp(value))
        http.execute_arm_action.assert_called_once_with(
            "grasp", timeout=5.0, sync=True, value=True)
        self.assertTrue(res["on"])
        self.assertTrue(res["ok"])

    def test_grasp_false(self):
        http = MagicMock()
        client = ArmClient(http=http)
        client.grasp(False)
        http.execute_arm_action.assert_called_once_with(
            "grasp", timeout=10.0, sync=True, value=False)


def _servo_result(converged=True):
    sel = TargetSelector.for_label("cylinder_3")
    return ServoResult(converged=converged, selector=sel,
                       x_mm=0.0, y_mm=-100.0, confidence=0.9,
                       iterations=3, elapsed_s=0.5,
                       final_detection=None, trace=())


class TestPickByVisionLower(unittest.TestCase):
    def _runner(self, converged=True, grasp_y=0.0, states=None):
        """states: get_state 依次返回的 ArmState 列表 (默认 y_before=-100, 降到位=y_grasp)."""
        client = MagicMock()
        client.origin = ArmOrigin()
        finder = client._make_vision_with_move.return_value
        finder.find_target.return_value = _servo_result(converged)
        if states is None:
            states = [ArmState(y_mm=-100.0), ArmState(y_mm=grasp_y)]
        client.get_state.side_effect = states
        client.move_y.return_value = {"ok": True}
        client.grasp.return_value = {"ok": True}
        client.composite_run.return_value = {"ok": True}
        return ArmRunner(client), client, finder

    def test_sequence_composite_run_servo_lower_grasp_lift(self):
        runner, client, finder = self._runner(grasp_y=-20.0)
        result = runner.pick_by_vision_lower(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0,
            grasp_y_mm=-20.0)
        self.assertTrue(result["ok"])
        self.assertEqual(result["y_before"], -100.0)
        self.assertEqual(result["y_lower"], -20.0)
        # composite_run → find_target → move_y(降到-20) → 验证 → suck → move_y(抬回)
        client.composite_run.assert_called_once()
        finder.find_target.assert_called_once()
        client.grasp.assert_called_once_with(True, timeout=10.0)   # suck
        move_calls = client.move_y.call_args_list
        self.assertEqual(len(move_calls), 2)
        self.assertAlmostEqual(move_calls[0].args[0], -20.0)   # 降到位
        self.assertAlmostEqual(move_calls[1].args[0], -100.0)  # 抬回
        self.assertEqual(move_calls[0].kwargs.get("timeout"), 20.0)

    def test_not_converged_returns_fail_no_descent(self):
        runner, client, finder = self._runner(converged=False)
        result = runner.pick_by_vision_lower(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "servo_not_converged")
        client.move_y.assert_not_called()
        client.grasp.assert_not_called()

    def test_y_not_reached_blocks_suck(self):
        """协议强制: y 未降到目标位不吸气."""
        runner, client, finder = self._runner(
            states=[ArmState(y_mm=-100.0), ArmState(y_mm=-45.0)])  # 降到 -45 ≠ 0
        result = runner.pick_by_vision_lower(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "y未到位 err=45.0mm")
        client.grasp.assert_not_called()

    def test_lift_back_false_keeps_y_low(self):
        runner, client, finder = self._runner()
        result = runner.pick_by_vision_lower(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0,
            lift_back=False)
        self.assertTrue(result["ok"])
        # 只降不抬回
        client.move_y.assert_called_once()

    def test_lock_first_default_upgrades_selector(self):
        """多目标防跳变: 默认把 HIGHEST_SCORE selector 升级为 LOCK_FIRST_SEEN."""
        runner, client, finder = self._runner()
        runner.pick_by_vision_lower(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0)
        sel_used = finder.find_target.call_args.args[0]
        self.assertEqual(sel_used.strategy, SelectionStrategy.LOCK_FIRST_SEEN.value)

    def test_lock_first_false_keeps_highest_score(self):
        runner, client, finder = self._runner()
        runner.pick_by_vision_lower(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0,
            lock_first=False)
        sel_used = finder.find_target.call_args.args[0]
        self.assertEqual(sel_used.strategy, SelectionStrategy.HIGHEST_SCORE.value)

    def test_explicit_strategy_not_overridden(self):
        """显式指定了非默认策略 → 不覆盖."""
        runner, client, finder = self._runner()
        runner.pick_by_vision_lower(
            TargetSelector.for_label("cylinder_3",
                                     strategy=SelectionStrategy.CLOSEST_TO_CENTER.value),
            x_mm=0.0, y_mm=-100.0)
        sel_used = finder.find_target.call_args.args[0]
        self.assertEqual(sel_used.strategy, SelectionStrategy.CLOSEST_TO_CENTER.value)

    def test_reposition_false_skips_composite_run_uses_current_pose(self):
        """reposition=False → 不 composite_run, 用当前位姿当伺服起点."""
        client = MagicMock()
        client.origin = ArmOrigin()
        finder = client._make_vision_with_move.return_value
        finder.find_target.return_value = _servo_result()
        client.get_state.side_effect = [
            ArmState(x_mm=-260.0, y_mm=-133.0),   # 当前位姿 (reposition=False)
            ArmState(y_mm=-133.0),                # y_before
            ArmState(y_mm=0.0),                   # 降到位验证
        ]
        client.move_y.return_value = {"status": "succeeded"}
        client.grasp.return_value = {"ok": True}
        runner = ArmRunner(client)
        runner.pick_by_vision_lower(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0,
            reposition=False)
        client.composite_run.assert_not_called()
        kw = finder.find_target.call_args.kwargs
        self.assertAlmostEqual(kw["x_mm"], -260.0)   # 用当前位姿
        self.assertAlmostEqual(kw["y_mm"], -133.0)

    def test_align_false_skips_servo_descends_directly(self):
        """align=False → 跳过伺服, 直接在当前位姿降到 grasp_y 抓."""
        client = MagicMock()
        client.origin = ArmOrigin()
        finder = client._make_vision_with_move.return_value
        client.get_state.side_effect = [
            ArmState(x_mm=-182.0, y_mm=-169.0),   # 当前位姿
            ArmState(y_mm=-169.0),                # y_before
            ArmState(y_mm=0.0),                   # 降到位验证
        ]
        client.move_y.return_value = {"ok": True}
        client.grasp.return_value = {"ok": True}
        runner = ArmRunner(client)
        result = runner.pick_by_vision_lower(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0,
            reposition=False, align=False)
        finder.find_target.assert_not_called()
        client.composite_run.assert_not_called()
        client.grasp.assert_called_once_with(True, timeout=10.0)   # suck
        move_calls = client.move_y.call_args_list
        self.assertEqual(len(move_calls), 2)
        self.assertAlmostEqual(move_calls[0].args[0], 0.0)     # 降到 0
        self.assertAlmostEqual(move_calls[1].args[0], -169.0)  # 抬回
        self.assertTrue(result["ok"])


if __name__ == "__main__":
    unittest.main()
