"""误差标定层单测 — ErrorCalibrator 的数学正确性 + no-op 兼容。"""
import unittest

from main.chassis.state import LaneState
from main.chassis.controllers.calibration import ErrorCalibrator


def _state(ey=0.1, ea=0.05):
    return LaneState(error_y=ey, error_angle=ea)


class TestNoop(unittest.TestCase):
    def test_default_is_noop(self):
        c = ErrorCalibrator()
        self.assertTrue(c.is_noop)

    def test_calibrate_returns_same_object_when_noop(self):
        c = ErrorCalibrator()
        st = _state()
        out = c.calibrate(st)
        self.assertIs(out, st)  # 不配置时零拷贝透传

    def test_no_error_frame_returns_unchanged(self):
        c = ErrorCalibrator(scale_y=1000.0)  # 有标定但没误差帧
        st = LaneState(error_y=None, error_angle=None)
        self.assertFalse(st.has_error)
        out = c.calibrate(st)
        self.assertIs(out, st)


class TestAffine(unittest.TestCase):
    def test_scale_y(self):
        c = ErrorCalibrator(scale_y=1000.0)
        ey, ea = c.step(0.0012, 0.0)
        self.assertAlmostEqual(ey, 1.2, places=6)
        self.assertAlmostEqual(ea, 0.0, places=6)

    def test_offset_then_scale(self):
        # 零点漂移：静止在中线时模型输出 0.002，先减 offset 再乘 scale
        c = ErrorCalibrator(scale_y=100.0, offset_y=0.002)
        ey, _ = c.step(0.002, 0.0)  # 静止 → 标定后应为 0
        self.assertAlmostEqual(ey, 0.0, places=9)
        ey, _ = c.step(0.102, 0.0)  # 偏离 0.1(模型单位) → 0.1*100
        self.assertAlmostEqual(ey, 10.0, places=6)

    def test_scale_angle(self):
        c = ErrorCalibrator(scale_angle=2.0)
        _, ea = c.step(0.0, 0.3)
        self.assertAlmostEqual(ea, 0.6, places=9)

    def test_negative_scale_flips_sign(self):
        # 现场发现方向反了，可以直接用负 scale，不用改控制律
        c = ErrorCalibrator(scale_y=-1.0)
        ey, _ = c.step(0.05, 0.0)
        self.assertAlmostEqual(ey, -0.05, places=9)


class TestEma(unittest.TestCase):
    def test_first_frame_seeds_no_smoothing(self):
        c = ErrorCalibrator(scale_y=1.0, ema_alpha=0.2)
        ey, _ = c.step(0.100, 0.0)
        self.assertAlmostEqual(ey, 0.100, places=9)

    def test_smooths_following_frames(self):
        c = ErrorCalibrator(ema_alpha=0.2)
        c.step(0.100, 0.0)                      # 播种 0.100
        ey, _ = c.step(0.000, 0.0)              # 突跳 0 → 平滑后 0.08
        self.assertAlmostEqual(ey, 0.08, places=9)
        ey, _ = c.step(0.000, 0.0)              # → 0.064
        self.assertAlmostEqual(ey, 0.064, places=9)

    def test_none_resets_ema_state(self):
        c = ErrorCalibrator(ema_alpha=0.2)
        c.step(0.100, 0.0)
        c.step(None, 0.0)                       # 丢线
        ey, _ = c.step(0.500, 0.0)              # 恢复 → 直接播种，不被旧 0.1 拖住
        self.assertAlmostEqual(ey, 0.500, places=9)

    def test_reset_clears_ema(self):
        c = ErrorCalibrator(ema_alpha=0.2)
        c.step(0.100, 0.0)
        c.reset()
        ey, _ = c.step(0.900, 0.0)
        self.assertAlmostEqual(ey, 0.900, places=9)

    def test_invalid_alpha_raises(self):
        with self.assertRaises(ValueError):
            ErrorCalibrator(ema_alpha=1.5)
        with self.assertRaises(ValueError):
            ErrorCalibrator(ema_alpha=0.0)


class TestCalibrateOnState(unittest.TestCase):
    def test_replaces_only_errors(self):
        c = ErrorCalibrator(scale_y=10.0)
        st = LaneState(error_y=0.01, error_angle=0.04,
                       forward=0.3, distance=1.2, mode="external_feed", age_ms=10.0)
        out = c.calibrate(st)
        self.assertAlmostEqual(out.error_y, 0.1, places=9)
        self.assertAlmostEqual(out.error_angle, 0.04, places=9)
        # 其余字段透传
        self.assertEqual(out.forward, 0.3)
        self.assertEqual(out.distance, 1.2)
        self.assertEqual(out.mode, "external_feed")
        self.assertEqual(out.age_ms, 10.0)

    def test_original_state_untouched(self):
        c = ErrorCalibrator(scale_y=10.0)
        st = _state(ey=0.01, ea=0.04)
        c.calibrate(st)
        self.assertAlmostEqual(st.error_y, 0.01, places=9)  # 不改原对象


if __name__ == "__main__":
    unittest.main()
