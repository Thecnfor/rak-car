"""curvature_adaptive 前瞻 + ω 阻尼单测。

- lookahead_s：ey_pred = ey + vx·sin(ea)·T 喂 vy/积分通道
- omega_lag_alpha：ω 一阶滞后，α=1 关闭，α=0 冻结在首帧

关键：默认参数（lookahead_s=0.15, omega_lag_alpha=0.40）下行为已开启，
但两个参数都能设回关闭值（0 / 1.0）完全还原旧行为。
"""
import unittest

from main.chassis.state import LaneState
from main.chassis.controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop


class TestLookahead(unittest.TestCase):
    def test_disabled_uses_raw_ey(self):
        c = CurvatureAdaptiveOuterLoop(lookahead_s=0.0)
        c.step(LaneState(error_y=0.04, error_angle=0.5), 0.02)
        self.assertAlmostEqual(c.debug_snapshot()["ey_used"], 0.04, places=9)

    def test_positive_ea_adds_forward_drift(self):
        # ea>0（车头朝右偏，本车约定）→ 侧向漂移为正，ey_used 应比原始 ey 大
        c = CurvatureAdaptiveOuterLoop(lookahead_s=0.2)
        c.step(LaneState(error_y=0.0, error_angle=0.3), 0.02)
        self.assertGreater(c.debug_snapshot()["ey_used"], 0.0)

    def test_negative_ea_subtracts_drift(self):
        c = CurvatureAdaptiveOuterLoop(lookahead_s=0.2)
        c.step(LaneState(error_y=0.0, error_angle=-0.3), 0.02)
        self.assertLess(c.debug_snapshot()["ey_used"], 0.0)

    def test_bigger_T_more_drift(self):
        c_big = CurvatureAdaptiveOuterLoop(lookahead_s=0.4)
        c_small = CurvatureAdaptiveOuterLoop(lookahead_s=0.1)
        st = LaneState(error_y=0.0, error_angle=0.3)
        c_big.step(st, 0.02)
        c_small.step(st, 0.02)
        self.assertGreater(c_big.debug_snapshot()["ey_used"],
                           c_small.debug_snapshot()["ey_used"])

    def test_ey_used_feeds_integral(self):
        # lookahead 开启时积分基于 ey_used；关闭时基于原始 ey
        c_on = CurvatureAdaptiveOuterLoop(lookahead_s=0.2)
        c_off = CurvatureAdaptiveOuterLoop(lookahead_s=0.0)
        st = LaneState(error_y=0.01, error_angle=0.3)
        for _ in range(10):
            c_on.step(st, 0.02)
            c_off.step(st, 0.02)
        # 正 ea → ey_used > ey → 开启时积分更正的快
        self.assertGreater(c_on.debug_snapshot()["ey_int"],
                           c_off.debug_snapshot()["ey_int"])


class TestOmegaLag(unittest.TestCase):
    def test_alpha_one_is_off(self):
        # α=1 = 关闭滞后：omega_lagged 不跟踪（None），ω 用原始目标
        c = CurvatureAdaptiveOuterLoop(omega_lag_alpha=1.0)
        c.step(LaneState(error_y=0.01, error_angle=0.5), 0.02)
        c.step(LaneState(error_y=-0.01, error_angle=-0.5), 0.02)
        self.assertIsNone(c.debug_snapshot()["omega_lagged"])

    def test_alpha_zero_freezes_at_first_frame(self):
        c = CurvatureAdaptiveOuterLoop(omega_lag_alpha=0.0)
        c.step(LaneState(error_y=0.01, error_angle=0.5), 0.02)
        frozen = c.debug_snapshot()["omega_lagged"]
        self.assertIsNotNone(frozen)
        c.step(LaneState(error_y=-0.01, error_angle=-0.5), 0.02)
        self.assertAlmostEqual(c.debug_snapshot()["omega_lagged"], frozen, places=6)

    def test_alpha_half_smooths_vs_freeze(self):
        # 同样两帧输入，α=0.5 的 ω 量级应比 α=0（冻结在首帧）小——被上一帧同号拖住
        c_half = CurvatureAdaptiveOuterLoop(omega_lag_alpha=0.5)
        c_freeze = CurvatureAdaptiveOuterLoop(omega_lag_alpha=0.0)
        for c in (c_half, c_freeze):
            c.step(LaneState(error_y=0.01, error_angle=0.5), 0.02)
            c.step(LaneState(error_y=-0.01, error_angle=-0.5), 0.02)
        self.assertLess(abs(c_half.debug_snapshot()["omega_lagged"]),
                        abs(c_freeze.debug_snapshot()["omega_lagged"]))

    def test_reset_on_lost_line(self):
        c = CurvatureAdaptiveOuterLoop(omega_lag_alpha=0.0)
        c.step(LaneState(error_y=0.01, error_angle=0.5), 0.02)
        self.assertIsNotNone(c.debug_snapshot()["omega_lagged"])
        c.step(LaneState(), 0.02)  # 丢线
        self.assertIsNone(c.debug_snapshot()["omega_lagged"])


if __name__ == "__main__":
    unittest.main()
