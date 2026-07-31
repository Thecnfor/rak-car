"""RuntimeApiClient vision 调用方法的单测 —— 用 mock 替代真实 HTTP"""
import unittest
from unittest.mock import MagicMock, patch
from main.api_client import RuntimeApiClient


class TestRequestVisionTask(unittest.TestCase):
    def setUp(self):
        # 跳过 __init__ 直接造一个 client，然后替换 _request
        self.client = RuntimeApiClient.__new__(RuntimeApiClient)
        # 创建一个 mock settings 让 @property api_prefix / api_base 能用
        self.client.settings = MagicMock()
        self.client.settings.api_prefix = "/v1"
        self.client.settings.api_base = "http://test:5050"
        self.client.settings.request_timeout = 30.0

    def test_request_vision_task_payload_shape(self):
        captured = {}
        def fake_request(method, path, payload=None, timeout=None):
            captured["method"] = method
            captured["path"] = path
            captured["payload"] = payload
            captured["timeout"] = timeout
            return {"ok": True, "detections": []}
        self.client._request = fake_request

        self.client.request_vision_task(
            sort_pos=(0.1, 0.2), limit_x=0.5, limit_y=0.8, timeout=15.0
        )

        self.assertEqual(captured["method"], "POST")
        self.assertEqual(captured["path"], "/v1/vision/task")
        self.assertEqual(captured["payload"]["sort_pos"], [0.1, 0.2])
        self.assertEqual(captured["payload"]["limit_x"], 0.5)
        self.assertEqual(captured["payload"]["limit_y"], 0.8)
        self.assertEqual(captured["payload"]["timeout"], 15.0)
        self.assertGreaterEqual(captured["timeout"], 15.0)  # outer timeout + 5s

    def test_get_vision_task_cache_calls_correct_path(self):
        captured = {}
        def fake_request(method, path, payload=None, timeout=None):
            captured["method"] = method
            captured["path"] = path
            return {"ok": True, "task_state": {"detections": []}}
        self.client._request = fake_request

        result = self.client.get_vision_task_cache()

        self.assertEqual(captured["method"], "GET")
        self.assertEqual(captured["path"], "/v1/realtime/vision/task")
        self.assertIn("task_state", result)


if __name__ == "__main__":
    unittest.main()