"""ArmClient / ArmVisionClient MRO 顺序验证.

拆完必须确保:
  1. 所有 mixin 都在 MRO 里
  2. SafetyMixin 在 MotionMixin/CompositeMixin/SettersMixin 之前
  3. ArmClient.method 全部可达
"""
import unittest


class TestArmClientMRO(unittest.TestCase):
    def test_arm_client_inherits_all_mixin(self):
        from main.arm import ArmClient
        from main.arm.api.safety import SafetyMixin
        from main.arm.api.motion import MotionMixin
        from main.arm.api.setters import SettersMixin
        from main.arm.api.composite import CompositeMixin
        from main.arm.api.reset_ops import ResetOpsMixin
        from main.arm.api.storage import StorageMixin
        from main.arm.api.state_io import StateIOMixin
        from main.arm.api.vis_servo import VisServoMixin
        mro = ArmClient.__mro__
        for mixin in (SafetyMixin, MotionMixin, SettersMixin, CompositeMixin,
                      ResetOpsMixin, StorageMixin, StateIOMixin, VisServoMixin):
            self.assertIn(mixin, mro, f"{mixin.__name__} missing from MRO")

    def test_safety_first_in_mro(self):
        """SafetyMixin 在 Motion 之前 (避免 MRO 菱形冲突的次序)."""
        from main.arm import ArmClient
        from main.arm.api.safety import SafetyMixin
        from main.arm.api.motion import MotionMixin
        from main.arm.api.composite import CompositeMixin
        from main.arm.api.setters import SettersMixin
        mro = ArmClient.__mro__
        self.assertLess(mro.index(SafetyMixin), mro.index(MotionMixin))
        self.assertLess(mro.index(SafetyMixin), mro.index(CompositeMixin))
        self.assertLess(mro.index(SafetyMixin), mro.index(SettersMixin))

    def test_arm_client_has_all_methods(self):
        """所有原 ArmClient.method 仍可达."""
        from main.arm import ArmClient
        expected = [
            "set_pose", "move_xy", "move_x", "move_y",
            "set_arm_angle", "set_hand_angle",
            "composite_pick", "composite_release", "composite_go_home",
            "composite_run", "composite_run_reset",
            "reset_y", "reset_x", "reset_all", "reset_origin",
            "set_storage", "get_storage", "set_storage_angle",
            "get_state", "get_pose_mm", "get_x_mm", "get_y_mm",
            "emergency_stop", "ping", "save_origin",
        ]
        names = set(dir(ArmClient))
        for m in expected:
            self.assertIn(m, names, f"ArmClient.{m} missing")

    def test_safety_mixin_alone_no_error(self):
        """SafetyMixin 可独立实例化 (mixin 单测)."""
        from main.arm.api.safety import SafetyMixin
        m = SafetyMixin()
        # _check_step_loss 是 staticmethod
        SafetyMixin._check_step_loss("x", 100.0, 102.0, 5.0)  # 偏差 2mm < 5mm → 不 warn
        # _check_safe 在没有 origin 时用 ArmOrigin() 默认
        m.origin = None
        m._check_safe(y_mm=-100.0)  # 不抛

    def test_motion_mixin_alone_no_error(self):
        """MotionMixin 单独 import 不破坏 MRO."""
        from main.arm.api.motion import MotionMixin
        self.assertTrue(callable(MotionMixin))
