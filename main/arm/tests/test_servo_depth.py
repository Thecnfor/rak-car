"""ArmVisionClient.compute_depth 单测 — 深度估计边界."""
import unittest
from main.arm.vision import ArmVisionClient
from main.arm.vision.types import BBoxPixels


class TestComputeDepth(unittest.TestCase):
    def test_basic_depth(self):
        """基础公式: depth = real_height * focal / bbox_height"""
        bp = BBoxPixels(0, 0, 100, 100, 100, 100)
        d = ArmVisionClient.compute_depth(bp, target_real_height_m=0.10, focal_length_px=600.0)
        # 0.10 * 600 / 100 = 0.6m
        self.assertAlmostEqual(d, 0.6, places=3)

    def test_zero_bbox_height_fallback(self):
        bp = BBoxPixels(0, 0, 100, 0, 100, 0)  # height=0
        d = ArmVisionClient.compute_depth(bp, target_real_height_m=0.10, focal_length_px=600.0)
        self.assertAlmostEqual(d, ArmVisionClient.DEFAULT_REF_DEPTH_M)  # 0.30m

    def test_zero_target_height_fallback(self):
        bp = BBoxPixels(0, 0, 100, 100, 100, 100)
        d = ArmVisionClient.compute_depth(bp, target_real_height_m=0.0, focal_length_px=600.0)
        self.assertAlmostEqual(d, ArmVisionClient.DEFAULT_REF_DEPTH_M)

    def test_none_bbox_fallback(self):
        d = ArmVisionClient.compute_depth(None, target_real_height_m=0.10, focal_length_px=600.0)
        self.assertAlmostEqual(d, ArmVisionClient.DEFAULT_REF_DEPTH_M)

    def test_far_object_small_bbox(self):
        """小 bbox → 远距离 (depth 大)."""
        bp = BBoxPixels(0, 0, 30, 30, 30, 30)
        d = ArmVisionClient.compute_depth(bp, target_real_height_m=0.30, focal_length_px=600.0)
        # 0.30 * 600 / 30 = 6.0m
        self.assertAlmostEqual(d, 6.0, places=3)

    def test_near_object_large_bbox(self):
        """大 bbox → 近距离."""
        bp = BBoxPixels(0, 0, 200, 200, 200, 200)
        d = ArmVisionClient.compute_depth(bp, target_real_height_m=0.20, focal_length_px=600.0)
        # 0.20 * 600 / 200 = 0.6m
        self.assertAlmostEqual(d, 0.6, places=3)

    def test_constants(self):
        self.assertEqual(ArmVisionClient.DEFAULT_FOCAL_LENGTH_PX, 600.0)
        self.assertEqual(ArmVisionClient.DEFAULT_REF_DEPTH_M, 0.30)
