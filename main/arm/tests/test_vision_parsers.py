"""vision.py 数据类型 + 解析器单测"""
import unittest
from main.arm.vision import (
    BBoxNorm, BBoxPixels, Detection,
    _parse_cache, _parse_sync,
)


CACHE_FIXTURE = {
    "ok": True,
    "task_state": {
        "active": True,
        "detections": [
            {
                "cls_id": 15,
                "det_id": 3,
                "label": "h_dou_jiao",
                "score": 0.88,
                "bbox_norm": {
                    "x_center": 0.10,
                    "y_center": -0.05,
                    "width": 0.22,
                    "height": 0.15,
                },
            },
        ],
        "count": 1,
        "updated_at": 1785488303.73,
    },
}


SYNC_FIXTURE = {
    "ok": True,
    "model": "task",
    "camera": "cam2",
    "detections": [
        {
            "index": 0,
            "class_id": 3,
            "track_id": 0,
            "label": "cylinder_1",
            "score": 0.95,
            "bbox_norm": {"x_center": 0.0, "y_center": 0.0, "width": 0.1, "height": 0.2},
            "bbox_pixels": {"x1": 320, "y1": 240, "x2": 384, "y2": 336, "width": 64, "height": 96},
        },
    ],
    "count": 1,
    "frame_shape": [480, 640, 3],
}


class TestBBoxNorm(unittest.TestCase):
    def test_is_centered_within_tol(self):
        b = BBoxNorm(x_center=0.02, y_center=-0.03, width=0.1, height=0.1)
        self.assertTrue(b.is_centered_at(0.05))

    def test_is_centered_outside_tol(self):
        b = BBoxNorm(x_center=0.20, y_center=-0.30, width=0.1, height=0.1)
        self.assertFalse(b.is_centered_at(0.05))

    def test_is_centered_default_tol(self):
        b = BBoxNorm(x_center=0.02, y_center=-0.02, width=0.1, height=0.1)
        # default 0.05
        self.assertTrue(b.is_centered)


class TestParseCache(unittest.TestCase):
    def test_parse_returns_one_detection(self):
        dets = _parse_cache(CACHE_FIXTURE)
        self.assertEqual(len(dets), 1)
        d = dets[0]
        self.assertEqual(d.label, "h_dou_jiao")
        self.assertEqual(d.score, 0.88)
        self.assertEqual(d.track_id, 3)
        self.assertEqual(d.class_id, 15)
        self.assertIsNone(d.bbox_pixels)
        self.assertAlmostEqual(d.bbox_norm.x_center, 0.10)

    def test_parse_handles_empty_detections(self):
        dets = _parse_cache({"ok": True, "task_state": {"detections": []}})
        self.assertEqual(dets, [])


class TestParseSync(unittest.TestCase):
    def test_parse_returns_detection_with_pixels(self):
        dets = _parse_sync(SYNC_FIXTURE)
        self.assertEqual(len(dets), 1)
        d = dets[0]
        self.assertEqual(d.label, "cylinder_1")
        self.assertIsNotNone(d.bbox_pixels)
        self.assertEqual(d.bbox_pixels.x1, 320)
        self.assertEqual(d.bbox_pixels.width, 64)

    def test_parse_handles_missing_pixels(self):
        raw = {**SYNC_FIXTURE}
        raw["detections"] = [{**SYNC_FIXTURE["detections"][0]}]
        raw["detections"][0].pop("bbox_pixels")
        dets = _parse_sync(raw)
        self.assertIsNone(dets[0].bbox_pixels)


if __name__ == "__main__":
    unittest.main()