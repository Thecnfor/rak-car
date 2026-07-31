"""ArmClient.composite_run / composite_run_reset / vision 单测 —— 验参数透传"""
import unittest
from unittest.mock import MagicMock
from main.arm.vision import ArmVisionClient


class _FakeApiClient:
    """只暴露 api_client.RuntimeApiClient 接口（无完整实现）"""
    def __init__(self):
        self.api_prefix = "/v1"
        self.api_base = "http://test:5050"
        self.request_timeout = 30.0


class TestCompositeRun(unittest.TestCase):
    def setUp(self):
        from main.arm.api import ArmClient
        self.http = _FakeApiClient()
        self.captured = {}

        def fake_arm(name, *args, timeout=20.0, sync=True, **kwargs):
            self.captured["name"] = name
            self.captured["args"] = args
            self.captured["kwargs"] = kwargs
            self.captured["timeout"] = timeout
            self.captured["sync"] = sync
            # _read_raw_state 调 y_get_position / x_get_position；返回 -100mm / 0mm 让
            # get_state().y_mm = -100（出保护区）
            if name == "y_get_position":
                return {"result": -0.10}
            if name == "x_get_position":
                return {"result": 0.0}
            return {"ok": True}

        def fake_car(name, *args, timeout=20.0, sync=False, **kwargs):
            # composite_run 入口会 _check_y_protected → get_state → execute_car_action
            return {"result": {"y_mm": -100.0, "x_mm": 0.0,
                               "y_limit": True, "arm_angle": 0,
                               "side": "MID", "hand_angle": "UP"}}

        self.http.execute_arm_action = fake_arm
        self.http.execute_car_action = fake_car
        self.client = ArmClient(http=self.http)

    def test_composite_run_passes_all_four(self):
        self.client.composite_run(
            arm=30.0, x_mm=100.0, y_mm=-80.0, hand=-37.0, speed=80
        )
        self.assertEqual(self.captured["name"], "composite_run")
        kw = self.captured["kwargs"]
        self.assertEqual(kw["arm"], 30.0)
        self.assertEqual(kw["x"], 0.1)
        self.assertEqual(kw["y"], -0.08)
        self.assertEqual(kw["hand"], -37.0)
        self.assertEqual(kw["speed"], 80)

    def test_composite_run_passes_none_for_skipped(self):
        self.client.composite_run(arm=None, x_mm=None, y_mm=-100.0, hand=-90.0)
        kw = self.captured["kwargs"]
        self.assertIsNone(kw["arm"])
        self.assertIsNone(kw["x"])
        self.assertEqual(kw["y"], -0.1)
        self.assertEqual(kw["hand"], -90.0)

    def test_composite_run_reset(self):
        self.client.composite_run_reset(
            arm_angle=90.0, hand_angle=-90.0,
            x_direction="right", reset_x_velocity_mms=20.0, timeout=60.0
        )
        self.assertEqual(self.captured["name"], "composite_run_reset")
        kw = self.captured["kwargs"]
        self.assertEqual(kw["arm_angle"], 90.0)
        self.assertEqual(kw["hand_angle"], -90.0)
        self.assertEqual(kw["x_direction"], "right")
        self.assertEqual(kw["reset_x_velocity"], 0.03)

    def test_vision_property_lazy(self):
        v1 = self.client.vision
        v2 = self.client.vision
        self.assertIsInstance(v1, ArmVisionClient)
        self.assertIs(v1, v2)


if __name__ == "__main__":
    unittest.main()