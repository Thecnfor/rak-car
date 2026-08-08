"""main/chassis/tests/test_lane_push.py
lane_state WS 推送订阅共享缓存单测（stdlib unittest，离线无硬件）。

覆盖 ChassisClient 的推送优先读取路径（2026-08-09）：
  - start_lane_subscription 幂等 + 接收 stop
  - _on_lane_push 写共享缓存
  - read_lane：缓存新鲜 → 零 RTT 读缓存（不碰 req/resp）
  - read_lane：订阅线程死了 → 回退 req/resp
  - read_lane：缓存过期 → 回退 req/resp
  - stop_lane_subscription / close 停订阅
  - DoubleLoopRunner.__init__ 自动起订阅（getattr 守卫，mock api 不炸）
"""
import time
import unittest
from unittest.mock import MagicMock, patch

from main.chassis.api import ChassisClient
from main.chassis.state import LaneState


class FakeWs:
    """模拟 RuntimeWsClient：订阅状态 + req/resp 通道可独立 mock。"""

    def __init__(self):
        self.lane_subscription_active = False
        self.subscribed_on_state = None
        # 模拟真实 realtime_lane_state 的解包结果（inner lane_state dict）
        self.realtime_lane_state = MagicMock(
            return_value={"error_y": 0.5, "error_angle": 0.1,
                          "mode": "lane_feed", "updated_at": 0.0}
        )
        self.close = MagicMock()

    def subscribe_lane(self, on_state, hz=50.0):
        self.subscribed_on_state = on_state
        self.lane_subscription_active = True

        def _stop():
            self.lane_subscription_active = False
            self.subscribed_on_state = None
        return _stop


class TestLanePush(unittest.TestCase):
    def _client(self):
        http = MagicMock()
        ws = FakeWs()
        client = ChassisClient(http=http, ws=ws, ws_ready=True)
        return client, ws

    def test_start_subscription_receives_stop(self):
        """start_lane_subscription 返回 True 且存下 stop。"""
        client, ws = self._client()
        ok = client.start_lane_subscription(hz=50.0)
        self.assertTrue(ok)
        self.assertTrue(ws.lane_subscription_active)
        self.assertIsNotNone(client._lane_sub_stop)

    def test_start_subscription_exception_returns_false(self):
        """订阅抛异常 → False，静默回退 req/resp。"""
        client, ws = self._client()

        def _boom(*a, **k):
            raise RuntimeError("ws down")
        ws.subscribe_lane = _boom
        ok = client.start_lane_subscription()
        self.assertFalse(ok)

    def test_on_push_writes_cache(self):
        """_on_lane_push 写共享缓存 + 本地单调时间戳。"""
        client, _ = self._client()
        with patch("main.chassis.api.time.monotonic", return_value=123.0):
            client._on_lane_push({"error_y": 0.3, "error_angle": -0.1})
        self.assertEqual(client._latest_lane_state["error_y"], 0.3)
        self.assertEqual(client._latest_lane_mono, 123.0)

    def test_read_lane_uses_cache_when_fresh(self):
        """缓存新鲜 → read_lane 零 RTT 读缓存，不碰 req/resp。"""
        client, ws = self._client()
        client.start_lane_subscription()
        # 推送一帧：error_y=0.3, error_angle=-0.1
        with patch("main.chassis.api.time.monotonic", return_value=100.0):
            client._on_lane_push({"error_y": 0.3, "error_angle": -0.1,
                                  "mode": "lane_feed", "updated_at": 0.0})
        # 10ms 后读 → 新鲜
        with patch("main.chassis.api.time.monotonic", return_value=100.01):
            state = client.read_lane()
        self.assertEqual(state.error_y, 0.3)
        self.assertEqual(state.error_angle, -0.1)
        self.assertTrue(state.has_error)
        self.assertLess(state.age_ms, 500.0)
        # req/resp 路径未被调用
        ws.realtime_lane_state.assert_not_called()

    def test_read_lane_falls_back_when_subscription_dead(self):
        """订阅线程死了（active=False）→ 回退 req/resp。"""
        client, ws = self._client()
        client.start_lane_subscription()
        # 缓存有一帧，但订阅被停（线程退出）
        client._on_lane_push({"error_y": 0.3, "error_angle": -0.1,
                              "mode": "lane_feed", "updated_at": 0.0})
        ws.lane_subscription_active = False
        state = client.read_lane()
        ws.realtime_lane_state.assert_called_once()
        self.assertAlmostEqual(state.error_y, 0.5)  # FakeWs 的 req/resp 值

    def test_read_lane_falls_back_when_cache_stale(self):
        """缓存超过 500ms → 回退 req/resp。"""
        client, ws = self._client()
        client.start_lane_subscription()
        with patch("main.chassis.api.time.monotonic", return_value=100.0):
            client._on_lane_push({"error_y": 0.3, "error_angle": -0.1,
                                  "mode": "lane_feed", "updated_at": 0.0})
        # 600ms 后读 → 过期
        with patch("main.chassis.api.time.monotonic", return_value=100.6):
            state = client.read_lane()
        ws.realtime_lane_state.assert_called_once()
        self.assertAlmostEqual(state.error_y, 0.5)

    def test_read_lane_no_subscription_uses_reqresp(self):
        """从未订阅 → 直接 req/resp。"""
        client, ws = self._client()
        state = client.read_lane()
        ws.realtime_lane_state.assert_called_once()

    def test_read_lane_exception_returns_empty(self):
        """req/resp 抛异常 → 空 LaneState（控制律零速）。"""
        client, ws = self._client()
        ws.realtime_lane_state = MagicMock(side_effect=RuntimeError("boom"))
        state = client.read_lane()
        self.assertFalse(state.has_error)
        self.assertIsNone(state.error_y)

    def test_stop_subscription(self):
        """stop_lane_subscription 停订阅且幂等。"""
        client, ws = self._client()
        client.start_lane_subscription()
        client.stop_lane_subscription()
        self.assertFalse(ws.lane_subscription_active)
        self.assertIsNone(client._lane_sub_stop)
        client.stop_lane_subscription()  # 二次调用安全

    def test_close_stops_subscription_and_ws(self):
        """close() 停订阅 + 关 ws + 发零速。"""
        client, ws = self._client()
        client.start_lane_subscription()
        client.close()
        self.assertFalse(ws.lane_subscription_active)
        ws.close.assert_called_once()


class TestDoubleLoopRunnerAutoSubscribe(unittest.TestCase):
    def test_runner_auto_starts_subscription(self):
        """DoubleLoopRunner 构造自动起订阅。"""
        from main.chassis.loops.closed_loop import DoubleLoopRunner
        from main.chassis.controllers.base import OuterLoop

        api = MagicMock()
        api.start_lane_subscription = MagicMock(return_value=True)
        outer = MagicMock(spec=OuterLoop)
        runner = DoubleLoopRunner(api=api, outer=outer, hz=50.0)
        api.start_lane_subscription.assert_called_once()

    def test_runner_old_client_no_method(self):
        """无 start_lane_subscription 的旧/mock client → 构造不炸。"""
        from main.chassis.loops.closed_loop import DoubleLoopRunner

        api = MagicMock()  # 没有 start_lane_subscription 属性
        del api.start_lane_subscription
        outer = MagicMock()
        DoubleLoopRunner(api=api, outer=outer, hz=50.0)  # 不应抛


if __name__ == "__main__":
    unittest.main()
