"""麦轮逆解一致性单测 — 手写 IK vs SDK matrix。

背景：runtime ``set_wheel_speeds`` 把 [v1..v4] 直通 WheelWrap（port[1,2,3,4]），
与 SDK ``set_velocity`` 的 ``inverse_kinematics``（car @ vehicle_to_wheel_matrix）
喂同一个物理轮序。因此对同一 (vx, vy, ω)，两套逆解必须产生**相同数组**才物理一致。

实测结论（2026-08-01）：
  - 纯前进：一致 ✓（vy=ω=0，只有 vx 模式 [1,-1,-1,1]）
  - 纯横移 / 前进+横移：**不一致 ✗** —— 手写 IK 的元素 0/3 的 vy 符号
    与其引用的矩阵推导相反。
  哪个物理正确需实车确认（本测试只把差距钉死，避免未来有人"顺手修掉"
  却没有意识到它在改所有控制律的横移行为）。
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


class TestForwardConsistent(unittest.TestCase):
    def test_pure_forward_matches_sdk(self):
        """纯前进两套必须一致（这是实车能跑的前提）。"""
        for vx in (0.1, 0.3, 0.5):
            s = _sdk_matrix_ik(vx, 0.0, 0.0)
            c = mecanum_inverse(vx, 0.0, 0.0, r=0.30)
            for a, b in zip(s, c):
                self.assertAlmostEqual(a, b, places=4)

    def test_pure_rotation_matches_sdk(self):
        """纯旋转只差 r 系数（0.3028 vs 0.30），符号必须一致（全同向）。"""
        s = _sdk_matrix_ik(0.0, 0.0, 0.5)
        c = mecanum_inverse(0.0, 0.0, 0.5, r=0.30)
        self.assertEqual(all(v > 0 for v in s), all(v > 0 for v in c))


class TestStrafeKnownGap(unittest.TestCase):
    """把已知的横移差异钉死：当前手写 IK 的 vy 轮序 ≠ SDK 推导。

    这不是"断言代码必须错"，而是防止无意识改动：任何人修这个差异时，
    都意味着所有 4 个控制律的 vy 通道行为一起变化，必须先实车 A/B。
    """

    def test_strafe_diverges_from_sdk(self):
        s = _sdk_matrix_ik(0.0, 0.1, 0.0)
        c = mecanum_inverse(0.0, 0.1, 0.0, r=0.30)
        # 两套确实不一致（这是 2026-08-01 记录的已知差距）
        self.assertNotEqual(
            [round(v, 4) for v in s],
            [round(v, 4) for v in c],
            "手写 IK 与 SDK 矩阵纯横移一致了？如果是，说明横移轮序问题已被修复，"
            "请更新本测试与 diag_lane_error 的提示。",
        )

    def test_gap_is_vy_sign_flip_on_wheels_0_and_3(self):
        """差距的本质：纯横移时元素 0/3 的 vy 符号与 SDK 相反。

        SDK vy 系数 [+, +, -, -]；chassis 实际输出 [-, +, -, +]——
        即元素 0 和 3 的 vy 符号反了（roller_angle 近似只影响量级，不影响此结论）。
        """
        c = mecanum_inverse(0.0, 0.1, 0.0, r=0.30)
        s = _sdk_matrix_ik(0.0, 0.1, 0.0)
        c_sign = [1 if v > 0 else -1 for v in c]
        s_sign = [1 if v > 0 else -1 for v in s]
        self.assertEqual(s_sign, [1, 1, -1, -1])   # SDK：元素 0/1 同向
        self.assertEqual(c_sign, [-1, 1, -1, 1])   # chassis：元素 0/3 与 SDK 相反

    def test_sdk_matrix_with_tan_r_is_self_consistent(self):
        """基准校验：_sdk_matrix_ik 自身数学自洽（tan_r 项必须参与 vy 贡献）。"""
        tan_r = math.tan(math.pi / 4.0 * 1.052)
        r_c = 0.30 / 2.0 * tan_r + 0.28 / 2.0
        vx, vy, wz = 0.3, 0.1, 0.0
        expected = [
            vx + vy * tan_r + wz * r_c,
            -vx + vy * tan_r + wz * r_c,
            -vx - vy * tan_r + wz * r_c,
            vx - vy * tan_r + wz * r_c,
        ]
        for a, b in zip(_sdk_matrix_ik(vx, vy, wz), expected):
            self.assertAlmostEqual(a, b, places=9)


if __name__ == "__main__":
    unittest.main()
