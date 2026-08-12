"""main/arm/tests/test_task4_track.py
task4 _track_leftmost_ball 新版 (2026-08-11): 4s → 超时加时 3s → 返回结果。

原五段式 (软成功/软重试/宽成) 已废除; 上层不因未对齐阻塞 (失败也继续)。
"""
import unittest
from unittest.mock import patch

from main.arm.each_task.task4.target4 import _track_leftmost_ball
from main.chassis.loops.visual_track import TrackChassisResult, TrackFrame


def _res(reason="timeout", arrived=False, stop_ok=True):
    ff = TrackFrame(target_found=True, cx_err=0.03, cy_err=0.02,
                    label="ball_yellow")
    return TrackChassisResult(
        arrived=arrived, reason=reason, final_frame=ff,
        frames=10, elapsed_s=1.0, stop_ok=stop_ok)


class TestTask4Track(unittest.TestCase):
    def test_arrived_no_extend(self):
        """已到位 → 不再加时, 原样返回."""
        with patch("main.arm.each_task.task4.track_align.track_chassis",
                   return_value=_res(reason="arrived", arrived=True)) as track:
            res = _track_leftmost_ball(max_seconds=4.0, extend_seconds=3.0, dry_run=False)
        self.assertTrue(res.arrived)
        self.assertEqual(track.call_count, 1)

    def test_timeout_triggers_extend(self):
        """4s 超时未到位 → 加时 3s 再跑一次 (共 2 次), 返回加时结果."""
        first = _res(reason="timeout", arrived=False)
        second = _res(reason="timeout", arrived=True)
        with patch("main.arm.each_task.task4.track_align.track_chassis",
                   side_effect=[first, second]) as track:
            res = _track_leftmost_ball(max_seconds=4.0, extend_seconds=3.0, dry_run=False)
        self.assertEqual(track.call_count, 2)
        self.assertTrue(res.arrived)
        # 第一次 4s, 第二次 3s
        self.assertEqual(track.call_args_list[0].kwargs["max_seconds"], 4.0)
        self.assertEqual(track.call_args_list[1].kwargs["max_seconds"], 3.0)

    def test_extend_still_timeout_returns_not_arrived(self):
        """加时后仍超时 → 返回未到位 (上层继续, 不抛)."""
        with patch("main.arm.each_task.task4.track_align.track_chassis",
                   return_value=_res(reason="timeout", arrived=False)):
            res = _track_leftmost_ball(max_seconds=4.0, extend_seconds=3.0, dry_run=False)
        self.assertFalse(res.arrived)
        self.assertEqual(res.reason, "timeout")

    def test_no_extend_when_reason_not_timeout(self):
        """control_lost / no_target 不是 timeout → 不加时, 原样返回."""
        for reason in ("control_lost", "no_target"):
            with patch("main.arm.each_task.task4.track_align.track_chassis",
                       return_value=_res(reason=reason, arrived=False)) as track:
                res = _track_leftmost_ball(max_seconds=4.0, extend_seconds=3.0, dry_run=False)
            self.assertEqual(track.call_count, 1)
            self.assertEqual(res.reason, reason)

    def test_track_chassis_exception_does_not_crash(self):
        """runtime 无响应 (ReadTimeout/ConnectionReset) → 捕获返回失败结果, 不抛 (2026-08-11 实车复现)."""
        with patch("main.arm.each_task.task4.track_align.track_chassis",
                   side_effect=TimeoutError("read timeout")) as track:
            res = _track_leftmost_ball(max_seconds=4.0, extend_seconds=3.0, dry_run=False)
        self.assertEqual(track.call_count, 1)
        self.assertFalse(res.arrived)
        self.assertEqual(res.reason, "error")
        self.assertIsNone(res.final_frame)

    def test_extend_exception_keeps_first_result(self):
        """首次超时, 加时调用抛异常 → 保留首次结果, 不抛."""
        first = _res(reason="timeout", arrived=False)
        with patch("main.arm.each_task.task4.track_align.track_chassis",
                   side_effect=[first, TimeoutError("read timeout")]) as track:
            res = _track_leftmost_ball(max_seconds=4.0, extend_seconds=3.0, dry_run=False)
        self.assertEqual(track.call_count, 2)
        self.assertFalse(res.arrived)
        self.assertEqual(res.reason, "timeout")

    def test_track_requests_field_tuned_parameters(self):
        """task4 底盘对齐必须显式使用现场调好的参数 (2026-08-11: v_max 0.04 / v_slew 0.01 /
        kp 0.05 / deadband 0.08 / 开 decouple_xy 单轴优先防麦轮打滑)."""
        with patch("main.arm.each_task.task4.track_align.track_chassis",
                   return_value=_res(reason="timeout", arrived=False)) as track:
            _track_leftmost_ball(max_seconds=4.0, extend_seconds=3.0, dry_run=False)
        kwargs = track.call_args.kwargs
        self.assertTrue(kwargs["decouple_xy"])
        self.assertEqual(kwargs["kp"], 0.05)
        self.assertEqual(kwargs["v_max"], 0.04)
        self.assertEqual(kwargs["v_slew"], 0.01)
        self.assertEqual(kwargs["deadband"], 0.08)
        self.assertEqual(kwargs["hold_frames"], 3)
        self.assertEqual(kwargs["select_mode"], "leftmost")
        self.assertEqual(kwargs["setpoint_cxcy"], (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
