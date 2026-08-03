"""main/chassis/tests/test_heading.py
HeadingEstimator 单测 — 离线，无硬件。

覆盖场景：
  1. 直道无漂移：heading 稳定在 0
  2. 直道有漂移：da 锚定把 heading 拉回
  3. 弯道：odom 真转 + da 跟踪，heading 跟上弯道
  4. 丢线：置信度衰减，heading 保持最后好值
  5. 丢线后重锚：置信度恢复
  6. x/y 重积分：直行后 x 增长、y 不动
  7. TrackMap 插值
  8. reset
"""
import math
import unittest

from main.chassis.heading import HeadingEstimator, HeadingState, TrackMap, _wrap_pi


class TestWrapPi(unittest.TestCase):
    def test_zero(self):
        self.assertAlmostEqual(_wrap_pi(0.0), 0.0)

    def test_positive_wrap(self):
        self.assertAlmostEqual(_wrap_pi(math.pi + 0.1), -math.pi + 0.1, places=10)

    def test_negative_wrap(self):
        self.assertAlmostEqual(_wrap_pi(-math.pi - 0.1), math.pi - 0.1, places=10)

    def test_2pi(self):
        self.assertAlmostEqual(_wrap_pi(2 * math.pi), 0.0, places=10)


class TestTrackMap(unittest.TestCase):
    def test_straight(self):
        tm = TrackMap.straight()
        self.assertAlmostEqual(tm.psi(0), 0.0)
        self.assertAlmostEqual(tm.psi(50), 0.0)

    def test_segments_interpolation(self):
        # 0-10m 朝东(0), 10-20m 朝北(π/2)
        tm = TrackMap.from_segments([(0, 10, 0.0), (10, 20, math.pi / 2)])
        self.assertAlmostEqual(tm.psi(0), 0.0)
        self.assertAlmostEqual(tm.psi(5), math.pi / 4, places=5)  # 中点
        self.assertAlmostEqual(tm.psi(10), math.pi / 2)
        self.assertAlmostEqual(tm.psi(15), math.pi / 2)

    def test_extrapolate_before(self):
        tm = TrackMap([(5.0, 0.3)])
        self.assertAlmostEqual(tm.psi(0), 0.3)

    def test_extrapolate_after(self):
        tm = TrackMap([(0.0, 0.0), (10.0, 0.5)])
        self.assertAlmostEqual(tm.psi(99), 0.5)

    def test_empty_raises(self):
        with self.assertRaises(ValueError):
            TrackMap([])


class TestHeadingEstimator(unittest.TestCase):
    def _make(self, **kw):
        return HeadingEstimator(track_map=TrackMap.straight(), **kw)

    def test_straight_no_drift(self):
        """直道无漂移：theta_odom 恒 0，da 恒 0 → heading 稳在 0。"""
        est = self._make(alpha=0.2)
        for i in range(100):
            st = est.update(
                theta_odom=0.0, distance=float(i) * 0.02,
                da=0.0, da_fresh=True,
            )
        self.assertAlmostEqual(st.heading, 0.0, places=5)
        self.assertTrue(st.anchored)
        self.assertAlmostEqual(st.confidence, 1.0)

    def test_drift_correction(self):
        """直道有漂移：theta_odom 每帧 +0.002 rad（累计 0.2 rad），da 恒 0。
        互补滤波应把 heading 拉回接近 0。"""
        est = self._make(alpha=0.2)
        drift_per_frame = 0.002
        for i in range(200):
            theta_odom = drift_per_frame * i  # 纯漂移，车实际没转
            st = est.update(
                theta_odom=theta_odom, distance=float(i) * 0.02,
                da=0.0, da_fresh=True,  # da 说"没偏"
            )
        # 没有修正的话 heading 会漂到 0.4 rad
        # 有修正后应该远小于 0.4
        self.assertLess(abs(st.heading), 0.1,
                        f"漂移未压住: heading={st.heading:.4f}")
        # 漂移率应该是正的
        self.assertGreater(st.drift_rate, 0.0)

    def test_real_turn(self):
        """弯道：theta_odom 真转 + da 跟踪弯道切线。
        车在直道(ψ=0)上走了 50 帧后，进入右弯(ψ从0变到π/2)，
        da 应该从 0 变到负值（车头滞后于弯道切线），heading 应跟上。"""
        # 弯道地图：0-10m 直(0), 10-14m 弯(0→π/2)
        track = TrackMap.from_segments([(0, 10, 0.0), (10, 14, math.pi / 2)])
        est = HeadingEstimator(track_map=track, alpha=0.3)

        # 先走直道
        for i in range(50):
            est.update(theta_odom=0.0, distance=i * 0.02, da=0.0, da_fresh=True)

        # 进弯：odom theta 真转 + da 报告车头相对切线的偏差
        total_turn = math.pi / 2
        n_turn_frames = 100
        for i in range(n_turn_frames):
            frac = i / n_turn_frames
            theta_odom = total_turn * frac  # odom 真转（无漂移）
            distance = 10.0 + 4.0 * frac
            # da = heading_true - ψ_lane(s)
            # heading_true ≈ θ_odom（无漂移），ψ_lane 由地图给
            psi = track.psi(distance)
            da = theta_odom - psi  # 车头相对切线
            st = est.update(
                theta_odom=theta_odom, distance=distance,
                da=da, da_fresh=True,
            )
        # 弯道结束后 heading 应接近 π/2
        self.assertAlmostEqual(st.heading, math.pi / 2, places=1,
                               msg=f"弯道后 heading={st.heading:.4f}, 期望≈{math.pi/2:.4f}")

    def test_lost_line_decay(self):
        """丢线：da=None → 置信度衰减，heading 保持。"""
        est = self._make(alpha=0.2, confidence_decay=0.95)
        # 先锚定
        for i in range(20):
            est.update(theta_odom=0.0, distance=i * 0.02, da=0.0, da_fresh=True)
        self.assertAlmostEqual(est.confidence, 1.0)

        # 丢线 10 帧
        heading_before = est.heading
        for i in range(10):
            st = est.update(theta_odom=0.0, distance=(20 + i) * 0.02, da=None, da_fresh=False)
        self.assertLess(st.confidence, 0.7)
        self.assertAlmostEqual(st.heading, heading_before, places=5)

    def test_reanchor(self):
        """丢线后重锚：置信度恢复。"""
        est = self._make(alpha=0.2, confidence_decay=0.9)
        for i in range(20):
            est.update(theta_odom=0.0, distance=i * 0.02, da=0.0, da_fresh=True)
        # 丢线
        for i in range(30):
            est.update(theta_odom=0.0, distance=(20 + i) * 0.02, da=None, da_fresh=False)
        self.assertLess(est.confidence, 0.1)
        # 重锚
        st = est.update(theta_odom=0.0, distance=50 * 0.02, da=0.0, da_fresh=True)
        self.assertAlmostEqual(st.confidence, 1.0)

    def test_xy_reintegration_straight(self):
        """直行：x 增长，y 不动（heading=0 → cos=1, sin=0）。"""
        est = self._make(alpha=0.2)
        for i in range(50):
            st = est.update(
                theta_odom=0.0, distance=i * 0.02,
                x_odom=float(i) * 0.02, y_odom=0.0,
                da=0.0, da_fresh=True,
            )
        self.assertAlmostEqual(st.x, 0.98, places=2)  # 49 × 0.02
        self.assertAlmostEqual(st.y, 0.0, places=5)

    def test_xy_reintegration_rotated(self):
        """朝北走(heading=π/2)：x 不动，y 增长。
        SDK 的 x_odom/y_odom 是世界系（用 theta_odom 旋转），这里模拟：
        车体每帧走 (0.02, 0)，SDK 用 theta=π/2 旋转 → 世界增量 (0, +0.02)。
        修正后 heading 也是 π/2，重积分应得 y 增长、x 不动。"""
        track = TrackMap.straight(heading=math.pi / 2)
        est = HeadingEstimator(track_map=track, alpha=0.5)
        theta = math.pi / 2
        # 锚定：da=0, ψ_lane=π/2 → heading→π/2
        for i in range(30):
            est.update(theta_odom=theta, distance=i * 0.02, da=0.0, da_fresh=True)
        # 朝北走：SDK 输出世界系 x/y（车体 dx=0.02 用 theta=π/2 旋转 → 世界 dy=+0.02）
        x_odom = 0.0
        y_odom = 0.0
        for i in range(50):
            x_odom += 0.02 * math.cos(theta)   # ≈ 0
            y_odom += 0.02 * math.sin(theta)   # ≈ +0.02
            st = est.update(
                theta_odom=theta, distance=(30 + i) * 0.02,
                x_odom=x_odom, y_odom=y_odom,
                da=0.0, da_fresh=True,
            )
        self.assertAlmostEqual(st.x, 0.0, delta=0.05)
        self.assertAlmostEqual(st.y, 0.98, delta=0.05)

    def test_reset(self):
        est = self._make()
        est.update(theta_odom=0.1, distance=1.0, da=0.0, da_fresh=True)
        self.assertTrue(est.anchored)
        est.reset(heading=1.0, x=5.0, y=3.0)
        self.assertFalse(est.anchored)
        self.assertAlmostEqual(est.heading, 1.0)
        self.assertAlmostEqual(est.confidence, 0.0)

    def test_none_theta_skips_delta(self):
        """theta_odom=None 时不更新航位推算。"""
        est = self._make()
        est.update(theta_odom=0.0, distance=0.0, da=0.0, da_fresh=True)
        st = est.update(theta_odom=None, distance=0.02, da=0.0, da_fresh=True)
        self.assertAlmostEqual(st.heading, 0.0)


if __name__ == "__main__":
    unittest.main()
