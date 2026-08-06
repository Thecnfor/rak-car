"""main/arm/tests/test_state_io_zerocopy.py — 0-copy 短 TTL 缓存行为测试.

2026-08-07 新增: 验证 _read_arm_state_raw() 50ms 窗口内复用同一 HTTP GET,
三个 _read_*_realtime + get_state 共用 1 次底层 HTTP.
"""
import time
import unittest
from unittest.mock import MagicMock

from main.arm.api.state_io import StateIOMixin


class _FakeClient(StateIOMixin):
    """最小 StateIOMixin 测试桩。"""

    def __init__(self, http):
        self.http = http
        self.origin = None
        # 模拟 _call_arm / _call_car 路径用
        self.http_call_history = []


class TestArmStateZeroCopy(unittest.TestCase):
    def setUp(self):
        self.http = MagicMock()
        # 每次 get_arm_state 返回同一 dict 30 次
        self.http.get_arm_state.return_value = {
            "ok": True,
            "arm_state": {
                "active": True,
                "mode": "arm_feed",
                "x_mm": 12.5,
                "y_mm": -150.0,
                "arm_angle": 30,
                "hand_angle": -10,
                "side": "RIGHT",
                "y_limit": True,
            },
        }
        self.client = _FakeClient(self.http)

    def test_three_readers_share_one_http(self):
        """同 tick 内 _read_x/y/arm_angle_realtime 共用 1 次 HTTP GET."""
        x = self.client._read_x_mm_realtime()
        y = self.client._read_y_mm_realtime()
        angle = self.client._read_arm_angle_realtime()
        self.assertEqual(x, 12.5)
        self.assertEqual(y, -150.0)
        self.assertEqual(angle, 30)
        # 关键断言: 一次 _read_arm_state_raw 命中 cache, 后续 2 次不走 HTTP
        self.assertEqual(self.http.get_arm_state.call_count, 1)

    def test_get_state_shares_cache(self):
        """get_state() 与 _read_*_realtime 在 50ms 内共享同一 HTTP GET."""
        x = self.client._read_x_mm_realtime()
        self.assertEqual(self.http.get_arm_state.call_count, 1)
        # 业务层调 get_state 必须复用同一 HTTP
        st = self.client.get_state()
        self.assertEqual(x, st.x_mm)
        self.assertEqual(self.http.get_arm_state.call_count, 1)

    def test_cache_invalidation_on_invalidate(self):
        """invalidate_arm_state_cache 强制下次重新 HTTP."""
        self.client._read_x_mm_realtime()
        self.assertEqual(self.http.get_arm_state.call_count, 1)
        self.client.invalidate_arm_state_cache()
        self.client._read_x_mm_realtime()
        self.assertEqual(self.http.get_arm_state.call_count, 2)

    def test_cache_ttl_expires(self):
        """TTL 50ms 过期后下次重新 HTTP."""
        self.client._read_x_mm_realtime()
        self.assertEqual(self.http.get_arm_state.call_count, 1)
        # 等 60ms (TTL 50ms + 缓冲)
        time.sleep(0.06)
        self.client._read_x_mm_realtime()
        self.assertEqual(self.http.get_arm_state.call_count, 2)

    def test_cache_returns_none_on_runtime_error(self):
        """runtime 失败返回 None, 失败原因存 _last_rt_err."""
        self.http.get_arm_state.side_effect = ConnectionError("nope")
        x = self.client._read_x_mm_realtime()
        y = self.client._read_y_mm_realtime()
        self.assertIsNone(x)
        self.assertIsNone(y)
        # 失败也共享 cache (1 次 HTTP)
        self.assertEqual(self.http.get_arm_state.call_count, 1)
        self.assertIn("ConnectionError", self.client.last_realtime_error())

    def test_cache_returns_none_when_arm_feed_inactive(self):
        """arm_feed 未启 (active=False) 返回 None."""
        self.http.get_arm_state.return_value = {
            "ok": True,
            "arm_state": {"active": False, "mode": "init"},
        }
        x = self.client._read_x_mm_realtime()
        self.assertIsNone(x)
        self.assertIn("arm_feed 未启", self.client.last_realtime_error())


class TestArmStateZeroCopyThreadSafety(unittest.TestCase):
    """0-copy 缓存是 per-instance, 多线程实例各自独立。

    不是 thread-safe (lock-free), 但应用层每个 ArmClient 实例只在
    一个 thread 用 (visual servo / task main loop), 这是合理契约。
    """

    def test_two_clients_independent(self):
        http1 = MagicMock()
        http1.get_arm_state.return_value = {
            "ok": True,
            "arm_state": {"active": True, "x_mm": 1.0, "y_mm": -1.0, "arm_angle": 0},
        }
        http2 = MagicMock()
        http2.get_arm_state.return_value = {
            "ok": True,
            "arm_state": {"active": True, "x_mm": 2.0, "y_mm": -2.0, "arm_angle": 0},
        }
        c1 = _FakeClient(http1)
        c2 = _FakeClient(http2)
        self.assertEqual(c1._read_x_mm_realtime(), 1.0)
        self.assertEqual(c2._read_x_mm_realtime(), 2.0)
        self.assertEqual(http1.get_arm_state.call_count, 1)
        self.assertEqual(http2.get_arm_state.call_count, 1)


if __name__ == "__main__":
    unittest.main()
