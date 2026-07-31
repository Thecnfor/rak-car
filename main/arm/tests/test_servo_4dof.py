"""4 自由度策略单测 — on_strategic_4dof 回调触发."""
import unittest
from unittest.mock import MagicMock
from main.arm.vision import ArmVisionClient, TargetSelector


def _det_dict(dx=0.0, dy=0.0, label="h_dou_jiao", score=0.9, height=100):
    return {
        "label": label, "score": score, "det_id": 1, "cls_id": 1,
        "bbox_norm": {"x_center": dx, "y_center": dy, "width": 0.1, "height": 0.1},
        "bbox_pixels": {"x1": 0, "y1": 0, "x2": height, "y2": height,
                         "width": height, "height": height},
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


class Test4DOFStrategy(unittest.TestCase):
    def test_large_offset_triggers_arm_rotate(self):
        """|dx_norm|=0.5 > 0.3 → on_strategic_4dof 被调, event='arm_rotate'."""
        det = _det_dict(dx=0.5, dy=0.0, label="h_dou_jiao", height=100)
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("h_dou_jiao")
        events = []
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            kp=1.0, ki=0.0, kd=0.0,
            settle_tol_norm=0.05,
            settle_stable_frames=99,
            target_real_height_m=0.20,
            arm_dx_threshold_norm=0.3,
            on_strategic_4dof=lambda evt, det: events.append(evt),
            timeout=0.5, max_iter=3,
        )
        self.assertIn("arm_rotate", events)

    def test_small_offset_no_trigger(self):
        """|dx_norm|=0.1 < 0.3 → 不触发 arm_rotate."""
        det = _det_dict(dx=0.1, dy=0.0, label="h_dou_jiao", height=100)
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("h_dou_jiao")
        events = []
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            kp=1.0, ki=0.0, kd=0.0,
            settle_tol_norm=0.05,
            settle_stable_frames=99,
            target_real_height_m=0.20,
            arm_dx_threshold_norm=0.3,
            on_strategic_4dof=lambda evt, det: events.append(evt),
            timeout=0.3, max_iter=2,
        )
        self.assertNotIn("arm_rotate", events)

    def test_arm_rotate_only_fires_once(self):
        """arm_rotate 一次性触发, 后续帧即使 |dx| 仍大也不重复."""
        det = _det_dict(dx=0.5, dy=0.0, label="h_dou_jiao", height=100)
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("h_dou_jiao")
        events = []
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            kp=1.0, ki=0.0, kd=0.0,
            settle_tol_norm=0.05,
            settle_stable_frames=99,
            target_real_height_m=0.20,
            arm_dx_threshold_norm=0.3,
            on_strategic_4dof=lambda evt, det: events.append(evt),
            timeout=0.5, max_iter=5,
        )
        # 多次 hit, 但 arm_rotate 只一次
        self.assertEqual(events.count("arm_rotate"), 1)

    def test_no_callback_no_trigger_error(self):
        """无回调时不报错 (默认 None)."""
        det = _det_dict(dx=0.5, dy=0.0, label="h_dou_jiao", height=100)
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("h_dou_jiao")
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            kp=1.0, ki=0.0, kd=0.0,
            settle_tol_norm=0.05,
            settle_stable_frames=99,
            target_real_height_m=None,
            arm_dx_threshold_norm=0.3,
            on_strategic_4dof=None,  # 无回调
            timeout=0.3, max_iter=2,
        )
        # 不报错
        self.assertIsNotNone(result)
