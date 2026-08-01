"""ArmRunner.move_to_vision_target / pick_by_vision 单测"""
import unittest
from unittest.mock import MagicMock
from main.arm.loops.runner import ArmRunner
from main.arm.state import ArmOrigin
from main.arm.vision import TargetSelector, ServoResult


def _fake_servo_result(converged=True, x_mm=0.0, y_mm=-100.0, label="h_dou_jiao"):
    sel = TargetSelector.for_label(label)
    return ServoResult(
        converged=converged, selector=sel,
        x_mm=x_mm, y_mm=y_mm, confidence=0.9,
        iterations=3, elapsed_s=0.5,
        final_detection=None, trace=(),
    )


class TestMoveToVisionTarget(unittest.TestCase):
    def test_calls_composite_run_then_vision_servo(self):
        client = MagicMock()
        client.origin = ArmOrigin()  # runner 会读 origin 决定是否注入吸嘴 setpoint
        self.finder = client._make_vision_with_move.return_value
        self.finder.find_target.return_value = _fake_servo_result()

        runner = ArmRunner(client)
        result = runner.move_to_vision_target(
            TargetSelector.for_label("h_dou_jiao"),
            x_mm=100.0, y_mm=-150.0, arm_angle=-90.0,
        )

        client.composite_run.assert_called_once()
        kw = client.composite_run.call_args.kwargs
        self.assertEqual(kw["arm"], -90.0)
        self.assertEqual(kw["x_mm"], 100.0)
        self.assertEqual(kw["y_mm"], -150.0)
        self.assertEqual(kw["hand"], -90.0)
        self.finder.find_target.assert_called_once()
        self.assertTrue(result.converged)


class TestPickByVision(unittest.TestCase):
    def test_calls_3_actions_in_order(self):
        client = MagicMock()
        client.origin = ArmOrigin()  # runner 会读 origin 决定是否注入吸嘴 setpoint
        self.finder = client._make_vision_with_move.return_value
        self.finder.find_target.return_value = _fake_servo_result()
        client.composite_pick.return_value = {"ok": True, "steps": {}}

        runner = ArmRunner(client)
        result = runner.pick_by_vision(
            TargetSelector.for_label("h_dou_jiao"),
            x_mm=100.0, y_mm=-150.0, arm_angle=-90.0,
        )

        # 顺序：composite_run → find_target → composite_pick
        client.composite_run.assert_called_once()
        self.finder.find_target.assert_called_once()
        client.composite_pick.assert_called_once()
        self.assertTrue(result["ok"])


if __name__ == "__main__":
    unittest.main()