"""麦轮逆解一致性单测 — 手写 IK vs SDK matrix。

判定标准：runtime ``set_wheel_speeds`` 把 [v1..v4] 直通 WheelWrap（port[1,2,3,4]），
与 SDK ``set_velocity`` 的 ``inverse_kinematics``（car @ vehicle_to_wheel_matrix）
喂同一个物理轮序。因此对同一 (vx, vy, ω)，手写 ``mecanum_inverse`` 必须与 SDK
矩阵产生**相同数组**才物理一致。

2026-08-01 修复后（stable tag 之后）：
  - 纯前进 / 纯横移 / 纯旋转 / 组合：与 SDK 一致（tan_r≈1 近似，量级差 <10%）
  - 用 SDK 正运动学做闭环自检：fk(ik(v)) ≈ v —— 旧版纯横移反解出 0 的问题已消除。
"""
import math
import unittest

from main.chassis.controllers.base import mecanum_inverse


def _sdk_matrix_ik(vx, vy, wz):
    """复刻 SDK inverse_kinematics：car_velocity @ vehicle_to_wheel_matrix。"""
    tan_r = math.tan(math.pi / 4.0 * 1.052)
    r_c = 0.30 / 2.0 * tan_r + 0.28 / 2.0
    return [
        vx + vy * tan_r + wz * r_c,
        -vx + vy * tan_r + wz * r_c,
        -vx - vy * tan_r + wz * r_c,
        vx - vy * tan_r + wz * r_c,
    ]


def _sdk_forward_kinematics(wheels):
    """复刻 SDK forward_kinematics：wheel_velocity @ wheel_to_vehicle_matrix。

    用于闭环自检 fk(ik(v)) ≈ v，验证逆解是否真的产生目标速度。
    """
    tan_r = math.tan(math.pi / 4.0 * 1.052)
    r_c = 0.30 / 2.0 * tan_r + 0.28 / 2.0
    w = list(wheels)
    vx = (w[0] - w[1] - w[2] + w[3]) / 4.0
    vy = (w[0] + w[1] - w[2] - w[3]) / (4.0 * tan_r)
    wz = (w[0] + w[1] + w[2] + w[3]) / (4.0 * r_c)
    return vx, vy, wz


class TestMatchesSdk(unittest.TestCase):
    def test_pure_forward_matches_sdk(self):
        for vx in (0.1, 0.3, 0.5):
            s = _sdk_matrix_ik(vx, 0.0, 0.0)
            c = mecanum_inverse(vx, 0.0, 0.0, r=0.30)
            for a, b in zip(s, c):
                self.assertAlmostEqual(a, b, places=4)

    def test_pure_strafe_matches_sdk(self):
        """修复核心：纯横移的 vy 轮序必须与 SDK 一致（旧版这里反解出 0）。"""
        for vy in (-0.15, -0.1, 0.1, 0.15):
            s = _sdk_matrix_ik(0.0, vy, 0.0)
            c = mecanum_inverse(0.0, vy, 0.0, r=0.30)
            # 符号必须逐位一致（这是本质）；量级差 tan_r≈1 vs 1.08 <10%
            self.assertEqual([1 if a > 0 else -1 for a in s],
                             [1 if b > 0 else -1 for b in c])
            for a, b in zip(s, c):
                self.assertAlmostEqual(a, b, delta=0.02)

    def test_pure_rotation_matches_sdk(self):
        for wz in (-0.5, 0.5):
            s = _sdk_matrix_ik(0.0, 0.0, wz)
            c = mecanum_inverse(0.0, 0.0, wz, r=0.30)
            self.assertEqual([1 if v > 0 else -1 for v in s],
                             [1 if v > 0 else -1 for v in c])
            for a, b in zip(s, c):
                self.assertAlmostEqual(a, b, delta=0.01)

    def test_combined_matches_sdk(self):
        for vx, vy, wz in [(0.3, 0.1, 0.0), (0.0, 0.1, 0.5), (0.2, -0.1, 0.3)]:
            s = _sdk_matrix_ik(vx, vy, wz)
            c = mecanum_inverse(vx, vy, wz, r=0.30)
            self.assertEqual([1 if a > 0 else -1 for a in s],
                             [1 if b > 0 else -1 for b in c])
            for a, b in zip(s, c):
                self.assertAlmostEqual(a, b, delta=0.02)


class TestClosedLoopSelfConsistent(unittest.TestCase):
    """闭环自检：fk(ik(v)) ≈ v。旧版纯横移这步会得 0，修复后必须还原目标速度。"""

    def test_pure_forward_roundtrip(self):
        vx, vy, wz = 0.3, 0.0, 0.0
        fvx, fvy, fwz = _sdk_forward_kinematics(mecanum_inverse(vx, vy, wz, r=0.30))
        self.assertAlmostEqual(fvx, vx, places=6)
        self.assertAlmostEqual(fvy, vy, places=6)
        self.assertAlmostEqual(fwz, wz, places=6)

    def test_pure_strafe_roundtrip(self):
        """关键回归：vy 命令必须产生真实横向速度（旧版为 0）。"""
        for vy in (0.05, 0.1, 0.2):
            fvx, fvy, fwz = _sdk_forward_kinematics(
                mecanum_inverse(0.0, vy, 0.0, r=0.30)
            )
            self.assertAlmostEqual(fvy, vy, delta=0.02)   # 横向真的动了
            self.assertAlmostEqual(fvx, 0.0, places=6)     # 不带前进
            self.assertAlmostEqual(fwz, 0.0, places=6)     # 不带旋转

    def test_combined_roundtrip(self):
        for vx, vy, wz in [(0.2, 0.1, 0.0), (0.15, -0.1, 0.3)]:
            fvx, fvy, fwz = _sdk_forward_kinematics(
                mecanum_inverse(vx, vy, wz, r=0.30)
            )
            self.assertAlmostEqual(fvx, vx, delta=0.02)
            self.assertAlmostEqual(fvy, vy, delta=0.02)
            self.assertAlmostEqual(fwz, wz, delta=0.02)


if __name__ == "__main__":
    unittest.main()
