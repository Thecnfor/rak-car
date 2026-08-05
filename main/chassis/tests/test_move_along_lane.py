"""main/chassis/tests/test_move_along_lane.py
``move_along_lane`` 单测 (stdlib unittest, 离线无硬件)。

核心不变量: 控制律 = ``StraightOuterLoop(vx_cruise=vx, strafe_v=0.0)``
  - **vy 锁死** → 物理上只有 vx 平移（前进 vx>0 / 后退 vx<0），不横移 / 不侧滑。
    判据: (w0+w1)-(w2+w3) = 4*vy（w0/w1 带 +vy, w2/w3 带 -vy）。
  - **ω 照常视觉对齐** → strafe_v 不影响 ω 通道（error_angle PI + error_y cross-track）。
"""
import unittest

from main.chassis.state import LaneState
from main.chassis.controllers.straight import StraightOuterLoop


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


if __name__ == "__main__":
    unittest.main()
