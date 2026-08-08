"""main/chassis/tests/test_wait_wheels_stopped.py
RuntimeApiClient.wait_wheels_stopped 单测 (2026-08-09)。

对齐后"等底盘真正停稳"的判停: 真实 MC602 编码器弧度双采样, 两读数 4 轮总位移
< 1.0 rad (≈0.5mm) → 停稳。端点异常/超时保守放行。与 target4.rearm 判停同构。
"""
import unittest
from unittest.mock import MagicMock

from main.api_client import RuntimeApiClient


class TestWaitWheelsStopped(unittest.TestCase):
    def setUp(self):
        self.client = RuntimeApiClient.__new__(RuntimeApiClient)
        self.client.settings = MagicMock()
        self.client.settings.api_prefix = "/v1"
        self.client.settings.api_base = "http://test:5050"
        self.client.settings.request_timeout = 30.0

    def test_stopped_returns_true(self):
        """两次读数相同 (轮子没动) → 立即判定停稳 True。"""
        enc = [1.0, 2.0, 3.0, 4.0]
        self.client.realtime_wheel_encoders = MagicMock(return_value={"encoders": enc})
        ok = self.client.wait_wheels_stopped(settle_s=0.0, timeout_s=0.5)
        self.assertTrue(ok)

    def test_moving_times_out_false(self):
        """两次读数持续不同 (轮子在动) → 直到超时 → False。"""
        calls = [0]

        def _moving():
            calls[0] += 1
            return {"encoders": [float(calls[0])] * 4}  # 每次读数都变

        self.client.realtime_wheel_encoders = MagicMock(side_effect=_moving)
        ok = self.client.wait_wheels_stopped(settle_s=0.0, timeout_s=0.05)
        self.assertFalse(ok)
        self.assertGreaterEqual(calls[0], 2)  # 确实做了双采样

    def test_stops_polling_when_wheels_quit_moving(self):
        """第 1 轮在动 (delta 大), 第 2 轮停 → 第 2 轮返回 True (不傻等到超时)。"""
        encs = [
            {"encoders": [0.0, 0.0, 0.0, 0.0]},   # 采样1
            {"encoders": [5.0, 5.0, 5.0, 5.0]},   # 采样2: 动了
            {"encoders": [5.0, 5.0, 5.0, 5.0]},   # 下一轮采样1
            {"encoders": [5.0, 5.0, 5.0, 5.0]},   # 下一轮采样2: 没动
        ]
        self.client.realtime_wheel_encoders = MagicMock(side_effect=encs)
        ok = self.client.wait_wheels_stopped(settle_s=0.0, timeout_s=0.5)
        self.assertTrue(ok)

    def test_exception_conservative_pass(self):
        """端点异常 → 保守放行 True (不阻塞任务)。"""
        self.client.realtime_wheel_encoders = MagicMock(
            side_effect=RuntimeError("car 未初始化"))
        ok = self.client.wait_wheels_stopped(settle_s=0.0, timeout_s=0.5)
        self.assertTrue(ok)

    def test_malformed_payload_pass(self):
        """encoders 非 4 元 → 无法判停, 保守放行 True。"""
        self.client.realtime_wheel_encoders = MagicMock(
            return_value={"encoders": [1.0, 2.0]})
        ok = self.client.wait_wheels_stopped(settle_s=0.0, timeout_s=0.5)
        self.assertTrue(ok)


if __name__ == "__main__":
    unittest.main()
