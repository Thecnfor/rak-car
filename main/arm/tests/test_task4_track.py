"""main/arm/tests/test_task4_track.py
task4 _track_leftmost_ball 在下沉后的 stop_ok 透传 (2026-08-09)。

track_chassis 现在走 runtime (单次阻塞 HTTP), 返回带 stop_ok 的 TrackChassisResult。
5 段式软成功重构 TrackChassisResult 时必须透传 stop_ok, 否则上层 step_target4
看不到"零速没发出去" → 不会显式停稳 → 抓取时车在滑 / 死底盘白烧预算。
"""
import unittest
from unittest.mock import patch

from main.arm.each_task.task4.track_align import _track_leftmost_ball
from main.chassis.loops.visual_track import TrackChassisResult, TrackFrame


def _res(reason="timeout", arrived=False, cx_err=0.03, cy_err=0.02, stop_ok=True):
    ff = TrackFrame(target_found=True, cx_err=cx_err, cy_err=cy_err,
                    label="ball_yellow")
    return TrackChassisResult(
        arrived=arrived, reason=reason, final_frame=ff,
        frames=10, elapsed_s=1.0, stop_ok=stop_ok)


class TestTask4TrackStopOk(unittest.TestCase):
    def test_soft_success_preserves_stop_ok_false(self):
        """timeout + 软死区 → near_arrived_soft; stop_ok=False 必须透传.

        否则上层以为车停了、不显式停稳 → 抓取时车在滑 (下沉后零速失败不可见)。
        """
        with patch("main.arm.each_task.task4.track_align.track_chassis",
                   return_value=_res(cx_err=0.03, cy_err=0.02, stop_ok=False)):
            res = _track_leftmost_ball(max_seconds=12.0, dry_run=False)
        self.assertTrue(res.arrived)
        self.assertEqual(res.reason, "near_arrived_soft")
        self.assertFalse(res.stop_ok)

    def test_soft_success_default_stop_ok_true(self):
        """正常软成 → stop_ok 默认 True."""
        with patch("main.arm.each_task.task4.track_align.track_chassis",
                   return_value=_res(cx_err=0.03, cy_err=0.02)):
            res = _track_leftmost_ball(max_seconds=12.0, dry_run=False)
        self.assertTrue(res.stop_ok)

    def test_track_requests_dual_axis_tuned_parameters(self):
        """task4 首轮对齐必须显式使用双轴和现场调好的速度参数。"""
        with patch("main.arm.each_task.task4.track_align.track_chassis",
                   return_value=_res(reason="control_lost", arrived=False)) as track:
            _track_leftmost_ball(max_seconds=12.0, dry_run=False)
        kwargs = track.call_args.kwargs
        self.assertFalse(kwargs["decouple_xy"])
        self.assertEqual(kwargs["kp"], 0.20)
        self.assertEqual(kwargs["v_max"], 0.12)
        self.assertEqual(kwargs["v_slew"], 0.04)
        self.assertEqual(kwargs["deadband"], 0.05)
        self.assertEqual(kwargs["hold_frames"], 3)

    def test_control_lost_returned_as_is(self):
        """control_lost → 5 段式不参与, 原样返回 (reason 非 timeout)."""
        with patch("main.arm.each_task.task4.track_align.track_chassis",
                   return_value=_res(reason="control_lost", stop_ok=False)):
            res = _track_leftmost_ball(max_seconds=12.0, dry_run=False)
        self.assertEqual(res.reason, "control_lost")
        self.assertFalse(res.stop_ok)
        self.assertFalse(res.arrived)

    def test_no_target_returned_as_is(self):
        """no_target 不是 timeout → 不触发软成功逻辑, 原样返回."""
        with patch("main.arm.each_task.task4.track_align.track_chassis",
                   return_value=_res(reason="no_target", stop_ok=True)):
            res = _track_leftmost_ball(max_seconds=12.0, dry_run=False)
        self.assertEqual(res.reason, "no_target")


if __name__ == "__main__":
    unittest.main()
