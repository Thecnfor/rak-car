"""ArmVisionClient.find_target 单测 —— 用 mock http 模拟检测序列"""
import unittest
from typing import List
from main.arm.vision import (
    ArmVisionClient, TargetSelector, Detection, BBoxNorm,
)


class FakeHttp:
    """模拟 RuntimeApiClient.get_vision_task_cache 行为"""
    def __init__(self, frames: List[List[Detection]]):
        self.frames = list(frames)
        self.call_count = 0

    def get_vision_task_cache(self):
        if self.call_count >= len(self.frames):
            self.call_count += 1
            return {"task_state": {"detections": [
                self._det_to_dict(d) for d in self.frames[-1]
            ]}}
        frame = self.frames[self.call_count]
        self.call_count += 1
        return {"task_state": {"detections": [self._det_to_dict(d) for d in frame]}}

    @staticmethod
    def _det_to_dict(d: Detection) -> dict:
        return {
            "label": d.label,
            "score": d.score,
            "track_id": d.track_id,
            "cls_id": d.class_id,
            "bbox_norm": {
                "x_center": d.bbox_norm.x_center,
                "y_center": d.bbox_norm.y_center,
                "width": d.bbox_norm.width,
                "height": d.bbox_norm.height,
            },
        }


def _det(cx, cy, label="h_dou_jiao", score=0.9, tid=0):
    return Detection(
        label=label, score=score, track_id=tid, class_id=8,
        bbox_norm=BBoxNorm(cx, cy, 0.1, 0.1),
        bbox_pixels=None, fetched_at=0.0,
    )


class TestFindTarget(unittest.TestCase):
    def test_converges_when_centered(self):
        # 第 0 帧：目标在右 (dx=0.3) → 应 move -9mm
        # 第 1 帧：目标居中 → 收敛
        frames = [
            [_det(cx=0.3, cy=0.0)],
            [_det(cx=0.01, cy=0.01)],
        ]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)
        move_log: list = []
        def move(x, y):
            move_log.append((x, y))
            return {}
        sel = TargetSelector.for_label("h_dou_jiao")
        result = vision.find_target(
            sel, x_mm=0.0, y_mm=-100.0,
            mm_per_norm=30.0, settle_tol_norm=0.05, min_step_mm=1.0,
            timeout=5.0, move_fn=move,
        )
        self.assertTrue(result.converged)
        self.assertGreaterEqual(result.iterations, 2)
        self.assertEqual(len(move_log), 1)
        self.assertAlmostEqual(move_log[0][0], -9.0, places=1)

    def test_timeout_returns_unconverged(self):
        frames = [[_det(cx=0.5, cy=0.5)] for _ in range(100)]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)
        sel = TargetSelector.for_label("h_dou_jiao")
        result = vision.find_target(
            sel, x_mm=0.0, y_mm=-100.0,
            mm_per_norm=30.0, timeout=0.05, max_iter=100,
            move_fn=lambda x, y: {},
        )
        self.assertFalse(result.converged)
        self.assertGreater(result.iterations, 0)

    def test_min_step_dead_band(self):
        frames = [
            [_det(cx=0.01, cy=0.01)],
            [_det(cx=0.01, cy=0.01)],
        ]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)
        move_log: list = []
        result = vision.find_target(
            TargetSelector.for_label("h_dou_jiao"),
            x_mm=0.0, y_mm=-100.0, min_step_mm=1.0,
            move_fn=lambda x, y: move_log.append((x, y)) or {},
        )
        self.assertTrue(result.converged)
        self.assertEqual(len(move_log), 0)

    def test_label_filter_excludes_other_labels(self):
        frames = [
            [_det(cx=0.0, cy=0.0, label="animal")],
        ]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)
        with self.assertRaises(RuntimeError) as ctx:
            vision.find_target(
                TargetSelector.for_label("h_dou_jiao"),
                x_mm=0.0, y_mm=-100.0,
                on_missing_track="abort",
                move_fn=lambda x, y: {},
                max_iter=10, timeout=5.0,
            )
        self.assertIn("连续", str(ctx.exception))


class TestFindTargetsSequence(unittest.TestCase):
    def test_runs_each_selector(self):
        frames = [
            [_det(cx=0.0, cy=0.0, label="h_dou_jiao")],
            [_det(cx=0.0, cy=0.0, label="h_fan_qie")],
        ]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)
        results = vision.find_targets_sequence(
            [TargetSelector.for_label("h_dou_jiao"),
             TargetSelector.for_label("h_fan_qie")],
            x_mm=0.0, y_mm=-100.0, timeout=2.0,
            move_fn=lambda x, y: {},
        )
        self.assertEqual(len(results), 2)
        self.assertTrue(all(r.converged for r in results))


class TestPickOne(unittest.TestCase):
    def test_picks_first_matching(self):
        frames = [
            [_det(cx=0.0, cy=0.0, label="h_fan_qie")],
        ]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)
        result = vision.pick_one(
            [TargetSelector.for_label("h_dou_jiao"),
             TargetSelector.for_label("h_fan_qie")],
            x_mm=0.0, y_mm=-100.0, timeout=2.0,
            move_fn=lambda x, y: {},
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.selector.label, "h_fan_qie")


class TestStaticMetadata(unittest.TestCase):
    def test_labels_returns_twenty(self):
        labels = ArmVisionClient.labels()
        self.assertEqual(len(labels), 20)

    def test_group_returns_tuple(self):
        veg = ArmVisionClient.group("vegetable")
        self.assertEqual(len(veg), 9)


if __name__ == "__main__":
    unittest.main()