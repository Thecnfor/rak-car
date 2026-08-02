"""视觉微调单测（stdlib unittest，离线无硬件）。

覆盖：
  - select_target：label 优先 / 面积兜底 / 空 detections
  - AlignState.from_task_payload：裸 dict / task_state 包裹 / None / 无面积
  - VisualAlignOuterLoop：纯前后（4 轮全等 vx）、零速边界、死区、饱和、快档默认
  - AlignConvergenceDetector：3 帧死区内 / 启动保护区 / 目标丢失重置
  - AlignRunResult / make_align_runner：builder 入口
  - VisualAlignRunner.run():mock api 时跑完整循环 + 判收敛返回 AlignRunResult
"""
import unittest
from unittest.mock import MagicMock

from main.chassis.state_align import AlignState, select_target
from main.chassis.controllers.visual_align import VisualAlignOuterLoop
from main.chassis.controllers.base import WheelSmoother
from main.chassis.loops.visual_align import (
    AlignConvergenceDetector,
    AlignRunResult,
    VisualAlignRunner,
    make_align_runner,
)


def _det(label="hopper", score=0.9, w=0.20, h=0.20, y_center=0.0):
    return {
        "label": label,
        "score": score,
        "bbox_norm": {"x_center": 0.0, "y_center": y_center, "width": w, "height": h},
        "bbox_pixels": {"x1": 0, "y1": 0, "x2": int(w * 640), "y2": int(h * 480),
                        "width": int(w * 640), "height": int(h * 480)},
    }


class TestSelectTarget(unittest.TestCase):
    def test_empty_returns_none(self):
        self.assertIsNone(select_target([]))
        self.assertIsNone(select_target(None))

    def test_label_priority_picks_label_match_with_highest_score(self):
        dets = [
            _det(label="hopper", score=0.5, w=0.30, h=0.30),
            _det(label="other",  score=0.99, w=0.40, h=0.40),  # 面积最大但 label 不对
            _det(label="hopper", score=0.8, w=0.20, h=0.20),
        ]
        chosen = select_target(dets, label="hopper")
        self.assertEqual(chosen["score"], 0.8)

    def test_no_label_falls_back_to_largest_area(self):
        dets = [
            _det(label="a", score=0.9, w=0.10, h=0.10),
            _det(label="b", score=0.5, w=0.40, h=0.40),
            _det(label="c", score=0.7, w=0.20, h=0.20),
        ]
        chosen = select_target(dets)  # 无 label
        self.assertEqual(chosen["label"], "b")

    def test_label_miss_falls_back_to_area(self):
        # label 找不到 → 退化到按面积选
        dets = [
            _det(label="a", score=0.9, w=0.40, h=0.40),
            _det(label="b", score=0.5, w=0.10, h=0.10),
        ]
        chosen = select_target(dets, label="nope")
        self.assertEqual(chosen["label"], "a")


class TestAlignStateFromPayload(unittest.TestCase):
    def test_none_payload_is_empty(self):
        s = AlignState.from_task_payload(None, ref_area=0.04)
        self.assertFalse(s.target_found)
        self.assertIsNone(s.area)
        self.assertIsNone(s.area_error)
        self.assertIsNone(s.label)

    def test_naked_inner_dict(self):
        payload = {
            "count": 1,
            "updated_at": 1000.0,
            "detections": [_det(label="hopper", w=0.20, h=0.20)],
        }
        s = AlignState.from_task_payload(payload, ref_area=0.04, now=1000.5)
        self.assertTrue(s.target_found)
        self.assertEqual(s.label, "hopper")
        self.assertAlmostEqual(s.area, 0.04, places=9)
        self.assertAlmostEqual(s.area_error, 0.0, places=9)  # 0.04 - 0.04 = 0
        self.assertAlmostEqual(s.age_ms, 500.0, places=3)

    def test_task_state_wrapped_dict(self):
        # /v1/realtime/vision/task 返回的形状是 {"task_state": {...}}
        payload = {
            "task_state": {
                "count": 1,
                "updated_at": 1000.0,
                "detections": [_det(label="x", w=0.10, h=0.10)],
            }
        }
        s = AlignState.from_task_payload(payload, ref_area=0.04, now=1000.0)
        self.assertTrue(s.target_found)
        self.assertAlmostEqual(s.area, 0.01, places=9)
        self.assertAlmostEqual(s.area_error, 0.03, places=9)  # 0.04 - 0.01

    def test_area_error_signs(self):
        # area < ref_area → 误差为正 → 车应前进
        s_far = AlignState.from_task_payload(
            {"updated_at": 0.0, "detections": [_det(w=0.05, h=0.05)]},
            ref_area=0.04,
        )
        self.assertGreater(s_far.area_error, 0.0)
        # area > ref_area → 误差为负 → 车应后退
        s_near = AlignState.from_task_payload(
            {"updated_at": 0.0, "detections": [_det(w=0.30, h=0.30)]},
            ref_area=0.04,
        )
        self.assertLess(s_near.area_error, 0.0)

    def test_ref_area_none_disables_error(self):
        s = AlignState.from_task_payload(
            {"updated_at": 0.0, "detections": [_det(w=0.20, h=0.20)]},
            ref_area=None,
        )
        self.assertTrue(s.target_found)
        self.assertAlmostEqual(s.area, 0.04, places=9)
        self.assertIsNone(s.area_error)  # 没 ref_area → 不算 error
        self.assertFalse(s.has_error)     # 控制律安全零速

    def test_no_area_disables_target_found(self):
        # 没有 bbox_norm 也没有 bbox_pixels → 选不出来
        payload = {"updated_at": 0.0, "detections": [{"label": "x", "score": 0.5}]}
        s = AlignState.from_task_payload(payload, ref_area=0.04)
        self.assertFalse(s.target_found)


class TestVisualAlignOuterLoop(unittest.TestCase):
    def _assert_pure_longitudinal(self, speeds):
        """断言 4 轮下发是纯前后:vy=0, omega=0 → 4 轮全等。"""
        self.assertEqual(len(speeds), 4)
        vx = speeds[0]
        for v in speeds:
            self.assertAlmostEqual(v, vx, places=9,
                                   msg="vy/omega 必须为 0,4 轮全等 vx")

    def test_no_target_returns_zero(self):
        c = VisualAlignOuterLoop()
        speeds = c.step(AlignState(), 0.05)
        self.assertEqual(speeds, [0.0, 0.0, 0.0, 0.0])

    def test_no_ref_area_returns_zero(self):
        c = VisualAlignOuterLoop()
        s = AlignState(target_found=True, area=0.04, area_error=None)
        speeds = c.step(s, 0.05)
        self.assertEqual(speeds, [0.0, 0.0, 0.0, 0.0])

    def test_deadband_kills_small_error(self):
        c = VisualAlignOuterLoop(kp=0.6, deadband=0.002)
        s = AlignState(target_found=True, area=0.039, area_error=0.001)  # 0.001 < 0.002
        speeds = c.step(s, 0.05)
        self.assertEqual(speeds, [0.0, 0.0, 0.0, 0.0])

    def test_positive_error_moves_forward(self):
        # area 比 ref_area 小 → 车离目标远 → vx > 0 → 前进
        c = VisualAlignOuterLoop(kp=0.6, v_max=0.20)
        s = AlignState(target_found=True, area=0.01, area_error=0.03)  # 0.04 - 0.01
        speeds = c.step(s, 0.05)
        self._assert_pure_longitudinal(speeds)
        self.assertGreater(speeds[0], 0.0)
        self.assertAlmostEqual(speeds[0], 0.6 * 0.03, places=9)

    def test_negative_error_moves_backward(self):
        # area 比 ref_area 大 → 车离目标近 → vx < 0 → 后退
        c = VisualAlignOuterLoop(kp=0.6, v_max=0.20)
        s = AlignState(target_found=True, area=0.09, area_error=-0.05)
        speeds = c.step(s, 0.05)
        self._assert_pure_longitudinal(speeds)
        self.assertLess(speeds[0], 0.0)
        self.assertAlmostEqual(speeds[0], 0.6 * -0.05, places=9)

    def test_v_max_saturates_both_directions(self):
        c = VisualAlignOuterLoop(kp=0.6, v_max=0.10)
        # 大正误差 → vx 饱和到 +0.10
        s_pos = AlignState(target_found=True, area=0.0, area_error=1.0)
        speeds = c.step(s_pos, 0.05)
        self._assert_pure_longitudinal(speeds)
        self.assertAlmostEqual(speeds[0], 0.10, places=9)
        # 大负误差 → vx 饱和到 -0.10
        s_neg = AlignState(target_found=True, area=1.0, area_error=-1.0)
        speeds = c.step(s_neg, 0.05)
        self._assert_pure_longitudinal(speeds)
        self.assertAlmostEqual(speeds[0], -0.10, places=9)

    def test_fast_defaults(self):
        """2026-08-02 用户要求"灵敏+快速"，默认参数应该比保守版快一档。"""
        c = VisualAlignOuterLoop()  # 全用默认
        self.assertEqual(c.kp, 1.5)
        self.assertEqual(c.v_max, 0.35)
        self.assertEqual(c.deadband, 0.005)
        # 大误差 → 直接达到 v_max=0.35(不是保守的 0.20)
        s = AlignState(target_found=True, area=0.0, area_error=1.0)
        speeds = c.step(s, 0.05)
        self.assertAlmostEqual(speeds[0], 0.35, places=9)


class TestAlignConvergenceDetector(unittest.TestCase):
    def _st(self, err, target_found=True, area=0.04):
        return AlignState(
            target_found=target_found, area=area, area_error=err,
            ref_area=0.04,
        )

    def test_three_consecutive_in_deadband_arrives(self):
        d = AlignConvergenceDetector(tol=0.005, required_frames=3, settle_skip_frames=5)
        # 前 5 帧是启动保护区,无论 err 多小都不到达
        for _ in range(5):
            self.assertFalse(d.update(self._st(0.0)))
        # 第 6、7 帧在死区内(连续 1、2)
        self.assertFalse(d.update(self._st(0.001)))
        self.assertFalse(d.update(self._st(0.001)))
        # 第 8 帧在死区内 → 连续 3 帧 → arrival
        self.assertTrue(d.update(self._st(0.001)))

    def test_out_of_deadband_resets_counter(self):
        d = AlignConvergenceDetector(tol=0.005, required_frames=3, settle_skip_frames=2)
        # 越过启动保护区
        for _ in range(2):
            d.update(self._st(0.01))
        # 死区内累计 2 帧
        d.update(self._st(0.001))
        self.assertFalse(d.update(self._st(0.001)))
        # 突然出死区 → 计数清零
        d.update(self._st(0.05))
        # 再累计 3 帧才能到达
        self.assertFalse(d.update(self._st(0.001)))
        self.assertFalse(d.update(self._st(0.001)))
        self.assertTrue(d.update(self._st(0.001)))

    def test_target_lost_resets_counter(self):
        d = AlignConvergenceDetector(tol=0.005, required_frames=3, settle_skip_frames=2)
        for _ in range(2):
            d.update(self._st(0.001))
        # 已死区内 1 帧,然后目标丢了
        d.update(self._st(0.001))
        d.update(self._st(0.0, target_found=False))
        # 再回死区,要从头数 3 帧
        self.assertFalse(d.update(self._st(0.001)))
        self.assertFalse(d.update(self._st(0.001)))
        self.assertTrue(d.update(self._st(0.001)))

    def test_settle_skip_protects_against_false_positive(self):
        """启动时车可能瞬间就在死区内(目标已经在画面正中),不能立刻 arrival。"""
        d = AlignConvergenceDetector(tol=0.005, required_frames=3, settle_skip_frames=5)
        # 前 5 帧就算全部 err=0 也不算到达
        for _ in range(5):
            self.assertFalse(d.update(self._st(0.0)))
        # 第 6 帧开始正式计数
        self.assertFalse(d.update(self._st(0.0)))
        self.assertFalse(d.update(self._st(0.0)))
        self.assertTrue(d.update(self._st(0.0)))


class TestMakeAlignRunner(unittest.TestCase):
    def test_builder_uses_fast_defaults(self):
        """make_align_runner 不传参时,默认是快档。"""
        runner = make_align_runner(ref_area=0.04)
        self.assertIsInstance(runner.outer, VisualAlignOuterLoop)
        self.assertEqual(runner.outer.kp, 1.5)
        self.assertEqual(runner.outer.v_max, 0.35)
        self.assertEqual(runner.outer.deadband, 0.005)
        # smoother 也是快档
        self.assertIsInstance(runner.smoother, WheelSmoother)
        self.assertEqual(runner.smoother.max_abs, 0.40)
        self.assertEqual(runner.smoother.max_accel, 0.15)
        self.assertEqual(runner.smoother.max_decel, 0.25)
        # 收敛是 3 帧死区 + 5 帧启动保护区
        self.assertEqual(runner.convergence.required_frames, 3)
        self.assertEqual(runner.convergence.settle_skip_frames, 5)
        self.assertTrue(runner.arrival_enabled)

    def test_builder_accepts_overrides(self):
        runner = make_align_runner(
            ref_area=0.04, kp=0.6, v_max=0.20, label="x",
            arrival_required_frames=8, arrival_settle_skip_frames=10,
            arrival_enabled=False,
        )
        self.assertEqual(runner.outer.kp, 0.6)
        self.assertEqual(runner.outer.v_max, 0.20)
        self.assertEqual(runner.label, "x")
        self.assertEqual(runner.convergence.required_frames, 8)
        self.assertEqual(runner.convergence.settle_skip_frames, 10)
        self.assertFalse(runner.arrival_enabled)


def _make_mock_api(payload_for):
    """构造一个 mock ChassisClient：每次 _sense() 调一次 get_vision_task_cache()
    返回下一个 payload。"""
    api = MagicMock()
    api.http.get_vision_task_cache.side_effect = payload_for
    api.set_wheel_speeds = MagicMock()
    api.close = MagicMock()
    api.ws_ready = False
    return api


class TestVisualAlignRunnerRun(unittest.TestCase):
    def test_run_returns_align_run_result_on_timeout(self):
        """run() 返回 AlignRunResult,timeout 时 arrived=False。"""
        # 一直返回 area 远超 ref_area 的 payload → 永远不到达
        payload_seq = [
            {"updated_at": 0.0, "detections": [_det(w=0.05, h=0.05)]}
            for _ in range(20)
        ]
        api = _make_mock_api(iter(payload_seq).__next__)
        api.http.get_vision_task_cache.side_effect = lambda: payload_seq.pop(0) \
            if payload_seq else {"updated_at": 0.0, "detections": []}

        outer = VisualAlignOuterLoop()
        runner = VisualAlignRunner(
            api=api, outer=outer, hz=50.0,
            ref_area=0.04,
            max_seconds=0.05,  # 50ms 内必超时
            arrival_enabled=True,
            watchdog_ms=None,  # 关闭 watchdog 干扰
            dry_run=True,  # 不真发轮速
        )
        result = runner.run()
        self.assertIsInstance(result, AlignRunResult)
        self.assertFalse(result.arrived)
        self.assertEqual(result.reason, "timeout")
        self.assertGreater(result.frames, 0)
        self.assertGreater(result.elapsed_s, 0.0)

    def test_run_arrives_after_3_frames_in_deadband(self):
        """连续 3 帧 err 在死区内 → arrived=True, reason="arrived"。"""
        # 前 5 帧大误差(启动保护区外计数),后 3 帧误差 0
        seq = [
            {"updated_at": 0.0, "detections": [_det(w=0.10, h=0.10)]}  # area=0.01, err=0.03
            for _ in range(5)
        ] + [
            {"updated_at": 0.0, "detections": [_det(w=0.20, h=0.20)]}  # area=0.04, err=0
            for _ in range(3)
        ] + [
            # 多备几个,防止 run() 跑超
            {"updated_at": 0.0, "detections": [_det(w=0.20, h=0.20)]}
            for _ in range(20)
        ]
        api = _make_mock_api(None)
        api.http.get_vision_task_cache.side_effect = lambda: seq.pop(0) \
            if seq else {"updated_at": 0.0, "detections": []}

        outer = VisualAlignOuterLoop()
        runner = VisualAlignRunner(
            api=api, outer=outer, hz=50.0,
            ref_area=0.04,
            max_seconds=5.0,
            arrival_enabled=True,
            watchdog_ms=None,
            dry_run=True,
        )
        result = runner.run()
        self.assertTrue(result.arrived, "应该 3 帧内到达,结果=%r" % result)
        self.assertEqual(result.reason, "arrived")
        # 5 帧保护区 + 3 帧死区 = 8 帧
        self.assertGreaterEqual(result.frames, 8)


if __name__ == "__main__":
    unittest.main()