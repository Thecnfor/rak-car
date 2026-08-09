"""main/arm/tests/test_task4_arm_servo.py

task4 机械臂智能抓取 (2026-08-10, 替换底盘对齐):
  - _pick_by_arm_servo 走 runner.track_velocity_pick (大臂控 cx + x 十字控 cy,
    y 锁 0), 高位伺服 → 最后盲降。
  - _pick_and_store = 臂伺服 pick + 放 bin (transit → bin_x → 降 → 放气)。
"""
import unittest
from unittest.mock import Mock

from main.arm.each_task.task4.pick_store import (
    _pick_and_store, _pick_by_arm_servo,
)
from main.arm.each_task.task4 import constants as C


class TestPickByArmServo(unittest.TestCase):
    def _runner(self, ok=True, reason=None):
        r = Mock()
        r.track_velocity_pick.return_value = {
            "ok": ok, "reason": reason, "trace_hits": 5,
            "settled": ok, "end_arm": None, "end_hand": None,
        }
        return r

    def test_success_uses_arm_cross_with_hardcoded_setpoint(self):
        """成功: 大臂+x 轴, y 锁 0, setpoint=constants 硬编码, 高位伺服→盲降。"""
        r = self._runner(ok=True)
        res = _pick_by_arm_servo(
            r, color="blue",
            servo_x_start_mm=-295.0, servo_y_start_mm=-160.0,
            servo_arm_start_deg=90.0, servo_hand_start_deg=10.0,
        )
        self.assertTrue(res["ok"])
        self.assertIsNone(res["error"])
        # 关键调用参数
        kw = r.track_velocity_pick.call_args.kwargs
        self.assertEqual(r.track_velocity_pick.call_args.args[0], "ball_blue")
        self.assertEqual(kw["setpoint_x_norm"], C.TASK4_SETPOINT_X_NORM)
        self.assertEqual(kw["setpoint_y_norm"], C.TASK4_SETPOINT_Y_NORM)
        self.assertEqual(kw["grasp_y_mm"], C.Y_PICK_MM)  # 最后盲降目标
        self.assertEqual(kw["y_start"], -160.0)          # 高位伺服起始
        self.assertEqual(kw["arm_start"], 90.0)
        self.assertEqual(kw["sign_arm"], C.TASK4_SERVO_SIGN_ARM)
        self.assertEqual(kw["sign_x"], C.TASK4_SERVO_SIGN_X)
        self.assertTrue(kw["skip_pose_align"])           # 主循环已摆到 P 姿态
        # 两种球同尺寸 → 黄球同一 setpoint
        r2 = self._runner(ok=True)
        _pick_by_arm_servo(r2, color="yellow", servo_x_start_mm=-295.0)
        kw2 = r2.track_velocity_pick.call_args.kwargs
        self.assertEqual(r2.track_velocity_pick.call_args.args[0], "ball_yellow")
        self.assertEqual(kw2["setpoint_x_norm"], C.TASK4_SETPOINT_X_NORM)

    def test_failure_propagates_reason(self):
        """伺服未收敛 → ok=False, error 带 reason, 不透传误成功。"""
        r = self._runner(ok=False, reason="not_settled")
        res = _pick_by_arm_servo(r, color="blue", servo_x_start_mm=-295.0)
        self.assertFalse(res["ok"])
        self.assertIn("not_settled", res["error"])


class TestPickAndStore(unittest.TestCase):
    def _mocks(self, servo_ok=True):
        runner = Mock()
        runner.track_velocity_pick.return_value = {
            "ok": servo_ok, "reason": None if servo_ok else "not_settled",
            "trace_hits": 5, "settled": servo_ok,
            "end_arm": None, "end_hand": None,
        }
        arm = Mock()
        return runner, arm

    def test_servo_failure_skips_store(self):
        """臂伺服抓取失败 → 直接失败, 不执行放 bin (composite_run 不调)。"""
        runner, arm = self._mocks(servo_ok=False)
        res = _pick_and_store(arm, runner, color="blue",
                              return_x_mm=None, pick_timeout_s=30.0)
        self.assertFalse(res["ok"])
        arm.composite_run.assert_not_called()
        runner.grasp.assert_not_called()

    def test_success_pick_then_store_bin(self):
        """成功: 臂伺服 pick → 中转 → bin_x → 降 → 放气。"""
        runner, arm = self._mocks(servo_ok=True)
        res = _pick_and_store(arm, runner, color="blue",
                              return_x_mm=None, pick_timeout_s=30.0)
        self.assertTrue(res["ok"])
        # 臂伺服被调用 (label=ball_blue)
        self.assertEqual(runner.track_velocity_pick.call_args.args[0], "ball_blue")
        # 放 bin: 中转 x → bin_x → 降 → 放气
        xs = [c.kwargs.get("x_mm") for c in arm.composite_run.call_args_list]
        self.assertIn(C.BIN_X_MM["blue"], xs)  # 蓝 bin x=0 必现
        self.assertTrue(any(c.kwargs.get("y_mm") is not None
                            for c in arm.composite_run.call_args_list))
        runner.grasp.assert_called_with(False, timeout=5.0)

    def test_yellow_bin_column(self):
        """黄球放黄 bin 列 (BIN_X_MM['yellow'])。"""
        runner, arm = self._mocks(servo_ok=True)
        _pick_and_store(arm, runner, color="yellow",
                        return_x_mm=None, pick_timeout_s=30.0)
        xs = [c.kwargs.get("x_mm") for c in arm.composite_run.call_args_list]
        self.assertIn(C.BIN_X_MM["yellow"], xs)


if __name__ == "__main__":
    unittest.main()
