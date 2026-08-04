"""十字路口弯按弯号换加固转弯的单测（stdlib unittest，离线无硬件）。

覆盖 main/chassis/loops/closed_loop.py 的 crossroad_turn 逻辑：
  - 第 crossroad_turn 个弯触发前自动换上加固的 StaircaseTurn/CurveDetector
    （上一个弯 done 后的第一个 detector 帧就装配好，等第 2 弯的触发）
  - 该弯结束（done/fail）后换回普通对，且计数递增
  - 其他弯（计数器不匹配）不换，用原版对
"""
import math
import unittest
from types import SimpleNamespace

from main.chassis.loops.closed_loop import DoubleLoopRunner
from main.chassis.controllers.odom_turn import CurveDetector, StaircaseTurn
from main.chassis.state import LaneState


def _lane(ea_deg: float) -> LaneState:
    return LaneState(error_angle=math.radians(ea_deg), error_y=0.0, age_ms=0)


class _Api:
    """只实现 _sense_odom 用到的最小面；theta 由测试直接设。"""
    def __init__(self):
        self.theta = 0.0

    def get_odometry_state(self):
        return SimpleNamespace(theta=self.theta)


class _Outer:
    def step(self, state, dt):
        return [0.1, 0.1, 0.1, 0.1]


class CrossroadSwapTest(unittest.TestCase):
    def _runner(self, crossroad_turn=2):
        return DoubleLoopRunner(
            api=_Api(),
            outer=_Outer(),
            watchdog_ms=None,
            lost_line_ms=None,
            dry_run=True,
            turn=StaircaseTurn(),
            detector=CurveDetector(),
            crossroad_turn=crossroad_turn,
        )

    def _fire_turn(self, runner):
        """喂 6 帧垃圾 ea 让当前 detector 触发一个转弯；返回触发帧后 runner.turn。"""
        runner.api.theta = 0.0   # 每弯从 0 起算，避免上个弯残留 theta 污染 theta_start
        for _ in range(6):
            runner._compute_raw(_lane(30.0))
        self.assertTrue(runner.turn.active, "detector 应已触发转弯")
        return runner.turn

    def _complete_turn(self, runner):
        """把 theta 推到当前弯目标刚过一点并喂干净 lane，让当前弯 done。"""
        runner.api.theta = runner.turn._turn.target + 0.01
        for _ in range(4):
            runner._compute_raw(_lane(0.0))
        self.assertFalse(runner.turn.active, "转弯应已完成")

    def test_second_turn_uses_crossroad_pair_and_reverts(self):
        r = self._runner(crossroad_turn=2)
        self.assertIsNone(r._cross_turn, "初始未装配加固对")

        # 第 1 个弯：普通对触发
        self.assertIs(self._fire_turn(r), r._normal_turn, "第 1 弯应为原版对")
        self._complete_turn(r)
        self.assertEqual(r._turn_seq, 1)
        # 上一个弯结束 → 下一个（第 2）弯是十字路口弯，detector 帧上就换加固对
        self.assertIs(r.turn, r._cross_turn, "第 1 弯结束后应立即装配加固对")
        self.assertEqual(r.detector.rearm_clean, 20, "加固 detector 应带 rearm 冷却")

        # 第 2 个弯：加固对触发 + 完成 → 卸载并换回普通对
        self.assertIs(self._fire_turn(r), r._cross_turn, "第 2 弯应为加固对")
        self._complete_turn(r)
        self.assertEqual(r._turn_seq, 2)
        self.assertIsNone(r._cross_turn, "十字路口弯结束应卸载加固对")
        self.assertIs(r.turn, r._normal_turn, "结束应换回普通对")

        # 第 3 个弯：不匹配 → 仍用普通对
        self.assertIs(self._fire_turn(r), r._normal_turn, "第 3 弯不应加固")
        self._complete_turn(r)
        self.assertEqual(r._turn_seq, 3)

    def test_no_crossroad_config_keeps_original(self):
        r = self._runner(crossroad_turn=None)
        for _ in range(3):
            self.assertIs(self._fire_turn(r), r._normal_turn, "未声明十字路口弯时全部走原版")
            self._complete_turn(r)
        self.assertEqual(r._turn_seq, 3)

    def test_cross_pair_fires_on_three_frame_signal(self):
        """加固 detector（tol=12, sustain=3）必须抓住 sub-20° 的 3 帧短信号。

        2026-08-05 实车：45° 弯出口接十字路口，弯道那 3 帧 error_angle 只有
        ~0.3rad（≈17°，在默认 20° 阈值之下）→ 默认 tol=20 全按"干净直道"清零、
        永不触发、直冲十字路口。降到 12° 才能在信号内判成弯道 + sustain=3 触发。
        本用例用 ea=18°（0.31rad，低于旧 20° 阈值）——旧配置下必然失败。
        """
        r = self._runner(crossroad_turn=2)
        # 完成第 1 弯 → 加固对装配（同 test_second_turn_uses_crossroad_pair_and_reverts）
        self._fire_turn(r)
        self._complete_turn(r)
        self.assertIs(r.turn, r._cross_turn, "第 1 弯后应装配加固对")
        # 只喂 3 帧 ea=18°（sub-20°、正好是实测信号量级）→ 加固对必须触发
        r.api.theta = 0.0
        for _ in range(3):
            r._compute_raw(_lane(18.0))
        self.assertTrue(r.turn.active, "3 帧 sub-20° 弯道信号应触发加固转弯")
        self.assertIs(r.turn, r._cross_turn, "触发应是加固对而非原版对")


if __name__ == "__main__":
    unittest.main()
