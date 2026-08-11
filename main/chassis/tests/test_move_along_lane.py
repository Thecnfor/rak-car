"""main/chassis/tests/test_move_along_lane.py
``move_along_lane`` 单测 (stdlib unittest, 离线无硬件)。

核心不变量: 控制律 = ``StraightOuterLoop(vx_cruise=vx, strafe_v=0.0)``
  - **vy 锁死** → 物理上只有 vx 平移（前进 vx>0 / 后退 vx<0），不横移 / 不侧滑。
    判据: (w0+w1)-(w2+w3) = 4*vy（w0/w1 带 +vy, w2/w3 带 -vy）。
  - **ω 照常视觉对齐** → strafe_v 不影响 ω 通道（error_angle PI + error_y cross-track）。
"""
import sys
import unittest
from pathlib import Path

# 路径: main/chassis/tests/ → repo_root（同 main/task/tests 的 bootstrap 写法,
# 让 `python3 main/chassis/tests/test_move_along_lane.py` 也能直接跑）
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.chassis.state import LaneState
from main.chassis.controllers.straight import StraightOuterLoop
from main.chassis.controllers.move_along_lane import _make_distance_stop


class TestLaneLineForwardOnly(unittest.TestCase):
    def test_vy_locked_zero_with_big_errors(self):
        # 大横向误差 + 大角度误差: vy 仍必须为 0, 只有 vx + ω。
        outer = StraightOuterLoop(vx_cruise=0.20, strafe_v=0.0)
        w = outer.step(LaneState(error_y=0.5, error_angle=0.2), 0.02)
        self.assertAlmostEqual(w[0] + w[1], w[2] + w[3], places=9)

    def test_backward_vx_flips_wheel_pattern(self):
        # vx<0 = 后退; 无误差 → 纯倒车轮速 [vx,-vx,-vx,vx]
        outer = StraightOuterLoop(vx_cruise=-0.20, strafe_v=0.0)
        w = outer.step(LaneState(error_y=0.0, error_angle=0.0), 0.02)
        self.assertEqual(w, [-0.20, 0.20, 0.20, -0.20])

    def test_omega_still_aligns_when_vx_locked(self):
        # ω 通道不受 strafe_v 影响: 车头偏右(ea>0) → ω>0 左转, w0 = vx + r*ω > vx
        outer = StraightOuterLoop(vx_cruise=0.20, strafe_v=0.0,
                                  ea_deadband=0.001, kp_theta=2.0)
        w = outer.step(LaneState(error_y=0.0, error_angle=0.05), 0.02)
        self.assertGreater(w[0], 0.20)
        self.assertGreater(w[0] + w[1], 0.0)  # w0+w1 = 2*r*ω > 0


class TestMoveAlongLaneExposed(unittest.TestCase):
    def test_importable_from_controllers_package(self):
        # 方法在 controllers/ 目录下可经包直接 import（暴露入口）
        from main.chassis.controllers import move_along_lane
        self.assertTrue(callable(move_along_lane))


class _FakeRunner:
    """记录 stop() 是否被调。"""

    def __init__(self) -> None:
        self.stopped = False

    def stop(self) -> None:
        self.stopped = True


class TestDistanceStop(unittest.TestCase):
    """``_make_distance_stop``：累计 lane_state.distance 到 target 就 stop。

    distance 是单调路径长（前进/后退都累计）；runtime auto-init 重置 odometry
    时 distance 回跳 → 重新记账。
    """

    def _tick_with(self, runner, distance_m=1.0, user=None):
        holder = {"runner": runner}
        return _make_distance_stop(distance_m, user, holder)

    def test_stops_when_path_length_reaches_target(self):
        runner = _FakeRunner()
        tick = self._tick_with(runner, distance_m=1.0)
        tick(LaneState(error_y=0.0, error_angle=0.0, distance=0.1), [])
        self.assertFalse(runner.stopped)  # 起始帧只记账（start=0.1）
        tick(LaneState(error_y=0.0, error_angle=0.0, distance=0.6), [])
        self.assertFalse(runner.stopped)  # 0.5m < 1.0m
        tick(LaneState(error_y=0.0, error_angle=0.0, distance=1.2), [])
        self.assertTrue(runner.stopped)   # 1.1m >= 1.0m

    def test_reinit_resets_accumulator(self):
        runner = _FakeRunner()
        tick = self._tick_with(runner, distance_m=1.0)
        tick(LaneState(error_y=0.0, error_angle=0.0, distance=3.0), [])  # 记账 start=3.0
        # runtime 重置 odometry → distance 回跳到 0.2 → 重新从 0.2 记账
        tick(LaneState(error_y=0.0, error_angle=0.0, distance=0.2), [])
        self.assertFalse(runner.stopped)
        tick(LaneState(error_y=0.0, error_angle=0.0, distance=1.1), [])  # 0.9m < 1.0m
        self.assertFalse(runner.stopped)
        tick(LaneState(error_y=0.0, error_angle=0.0, distance=1.3), [])  # 1.1m >= 1.0m
        self.assertTrue(runner.stopped)

    def test_none_distance_does_not_stop(self):
        runner = _FakeRunner()
        tick = self._tick_with(runner, distance_m=1.0)
        tick(LaneState(error_y=0.0, error_angle=0.0, distance=None), [])
        self.assertFalse(runner.stopped)


if __name__ == "__main__":
    unittest.main()
