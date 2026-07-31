"""稳定收敛单测 — settle_stable_frames 控制."""
import unittest
from unittest.mock import MagicMock
from main.arm.vision import ArmVisionClient, TargetSelector


def _det_dict(dx=0.0, dy=0.0, label="h_dou_jiao", score=0.9):
    return {
        "label": label, "score": score, "det_id": 1, "cls_id": 1,
        "bbox_norm": {"x_center": dx, "y_center": dy, "width": 0.05, "height": 0.05},
    }


def _make_http_with_dets(dicts):
    http = MagicMock()
    counter = [0]
    pool = [d for d in dicts]

    def _next():
        counter[0] += 1
        return {
            "task_state": {
                "detections": [{k: v for k, v in d.items()} for d in pool],
                "updated_at": float(counter[0]),
            }
        }

    http.get_vision_task_cache.side_effect = _next
    return http


class TestSettleStable(unittest.TestCase):
    def test_three_frames_centered_converges(self):
        """3 帧都居中 → settle_stable=True + converged=True."""
        det = _det_dict(dx=0.01, dy=0.01, label="h_dou_jiao")
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("h_dou_jiao")
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            settle_tol_norm=0.05,
            settle_stable_frames=3,
            timeout=0.5, max_iter=10,
        )
        self.assertTrue(result.converged)
        self.assertTrue(result.settle_stable)

    def test_short_iter_does_not_settle(self):
        """max_iter=2 不够 3 帧 → settle_stable=False."""
        det = _det_dict(dx=0.01, dy=0.01, label="h_dou_jiao")
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("h_dou_jiao")
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            settle_tol_norm=0.05,
            settle_stable_frames=3,
            timeout=0.3, max_iter=2,
        )
        self.assertFalse(result.settle_stable)

    def test_settle_stable_frames_one_converges_immediately(self):
        """frames=1 → 单帧居中即收敛 (兼容旧行为)."""
        det = _det_dict(dx=0.01, dy=0.01, label="h_dou_jiao")
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("h_dou_jiao")
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            settle_tol_norm=0.05,
            settle_stable_frames=1,  # 立即
            timeout=0.3, max_iter=2,
        )
        self.assertTrue(result.converged)
        self.assertTrue(result.settle_stable)

    def test_legacy_keeps_old_behavior(self):
        """find_target_legacy: 单帧居中即 converged=True (旧行为)."""
        det = _det_dict(dx=0.01, dy=0.01, label="h_dou_jiao")
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("h_dou_jiao")
        result = client.find_target_legacy(
            sel, x_mm=0.0, y_mm=-100.0,
            settle_tol_norm=0.05,
            timeout=0.5, max_iter=10,
        )
        self.assertTrue(result.converged)
        # legacy 没有 settle_stable 概念, 默认 False
        self.assertFalse(result.settle_stable)
