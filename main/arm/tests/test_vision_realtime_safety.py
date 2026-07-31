"""回归测试：_make_vision_with_move 必须同时 wrap find_target + find_target_realtime。

Issue 1 (security review HIGH gate-bypass-sibling-path)：realtime 路径
之前未走 _check_safe / _check_y_protected，导致 poisoned detection 推 y 越界
时会下发越界指令。

测试目标：
  - find_target 被 wrap（注入 _safe_move 验证 _check_safe 被调）
  - find_target_realtime 同样被 wrap（同一 _safe_move）
  - 直接调 vision.find_target_realtime 不带 move_fn → 默认路径走裸
    execute_arm_action（这是 OK 的，caller 应该 wrap）；但 runner 通过
    _make_vision_with_move 注入 safe_move_fn 后必须 reject 越界。
"""
import unittest
from unittest.mock import MagicMock, patch
from main.api_client import RuntimeApiClient


class _FakeApiClient:
    api_prefix = "/v1"
    api_base = "http://test:5050"
    request_timeout = 30.0


class TestMakeVisionWithMove(unittest.TestCase):
    def setUp(self):
        from main.arm.api import ArmClient
        self.http = _FakeApiClient()
        self.captured = []

        def fake_arm(name, *args, timeout=20.0, sync=True, **kwargs):
            self.captured.append((name, args, kwargs))
            if name in ("y_get_position", "x_get_position"):
                return {"result": -0.10}    # y=-100mm 出保护区
            return {"ok": True}

        self.http.execute_arm_action = fake_arm
        self.http.execute_car_action = lambda *a, **kw: {
            "result": {"y_mm": -100.0, "x_mm": 0.0, "y_limit": True,
                       "arm_angle": 0, "side": "MID", "hand_angle": "UP"}
        }
        self.client = ArmClient(http=self.http)

    def test_find_target_realtime_is_wrapped(self):
        """_make_vision_with_move 必须 wrap find_target_realtime，否则越界会下发。"""
        vision = self.client._make_vision_with_move()
        # wrap 后 find_target_realtime 应该不再是原始方法
        from main.arm.vision import ArmVisionClient
        original = ArmVisionClient(self.http).find_target_realtime
        self.assertIsNot(vision.find_target_realtime, original)

    def test_find_target_is_wrapped(self):
        """find_target 也必须被 wrap（已有行为，不应回归）。"""
        vision = self.client._make_vision_with_move()
        from main.arm.vision import ArmVisionClient
        original = ArmVisionClient(self.http).find_target
        self.assertIsNot(vision.find_target, original)

    def test_poisoned_detection_realtime_rejected(self):
        """Issue 1 regression：realtime 路径推 y 越界 → _safe_move 必须 raise。

        检测 y_norm=+1.0（大正 → 车向上）→ mm_per_norm=30 → dy_mm=-30mm →
        new_y = -150 + (-30) = -180mm（仍在 -200..0 区间，没越界；改用更激进）。
        实际场景：检测 cy=+2.0 → dy_mm=-60mm → new_y=-210（越界）。
        """
        from main.arm.vision import (
            ArmVisionClient, TargetSelector, Detection, BBoxNorm,
        )

        class FakeWs:
            def __init__(self):
                self._cb = None
            def subscribe_task_detection(self, on_state, hz=30.0):
                self._cb = on_state
                # 推 1 帧：cy=+5.0 → dy_mm = -150mm → new_y = -150 + (-150) = -300mm（超界）
                on_state({"task_state": {
                    "detections": [{
                        "label": "cylinder_1", "score": 0.9,
                        "track_id": 0, "cls_id": 4,
                        "bbox_norm": {"x_center": 0.0, "y_center": 5.0,
                                       "width": 0.1, "height": 0.1},
                    }],
                    "updated_at": 1.0,
                }})
                return MagicMock()

        ws = FakeWs()
        vision = self.client._make_vision_with_move()
        # wrap 后的 find_target_realtime 必走 _safe_move
        with self.assertRaises(ValueError) as ctx:
            vision.find_target_realtime(
                TargetSelector.for_label("cylinder_1"),
                x_mm=0.0, y_mm=-150.0, mm_per_norm=30.0,
                timeout=2.0, ws=ws,
                # 注意：不传 move_fn，让 _make_vision_with_move 注入 _safe_move
            )
        self.assertIn("y_mm", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()