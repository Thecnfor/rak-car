"""main/arm 公开符号 import 薄烟测 — 拆完验证可访问性."""
import unittest


class TestPublicImports(unittest.TestCase):
    def test_arm_client_importable(self):
        from main.arm import ArmClient
        self.assertTrue(callable(ArmClient.connect))
        # 拆完 ArmClient 路径应在 main.arm.api
        self.assertEqual(ArmClient.__module__, "main.arm.api")

    def test_arm_runner_importable(self):
        from main.arm import ArmRunner
        self.assertTrue(callable(ArmRunner))

    def test_arm_vision_client_importable(self):
        from main.arm import ArmVisionClient
        self.assertTrue(callable(ArmVisionClient))

    def test_dataclasses_importable(self):
        from main.arm import (ArmClient, ArmState, ArmOrigin, TrajectoryGenerator,
                                TargetSelector, SelectionStrategy,
                                Detection, BBoxNorm, BBoxPixels,
                                ServoTrace, ServoResult,
                                Label, LabelInfo, LABELS, LABEL_GROUPS)
        # 全部 import 成功, sanity check
        self.assertTrue(callable(ArmClient))
        self.assertEqual(len(LABELS), 20)

    def test_origin_calibrator_importable(self):
        from main.arm import OriginCalibrator, run_calibrator
        self.assertTrue(callable(OriginCalibrator))
        self.assertTrue(callable(run_calibrator))
