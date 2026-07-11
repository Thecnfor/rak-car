# -*- coding: utf-8 -*-
"""
机械臂 气泵 / 抓取 调试脚本

对应 ArmController 方法：
- grasp(value: bool)
"""

import time
from smartcar.whalesbot.vehicle.arm.arm_base import ArmController


def test_grasp(arm: ArmController, value=True):
    """
    控制气泵

    Args:
        value: True=吸取, False=释放
    """
    print(f"[GRASP] grasp({value})")
    arm.grasp(value)
    time.sleep(0.5)


def test_grasp_cycle(arm: ArmController, cycles=2, hold=1.0):
    """循环测试气泵吸取/释放"""
    for i in range(cycles):
        print(f"--- 循环 {i+1}/{cycles} ---")
        test_grasp(arm, value=True)
        time.sleep(hold)
        test_grasp(arm, value=False)
        time.sleep(hold)


if __name__ == "__main__":
    arm = ArmController()
    test_grasp_cycle(arm, cycles=2, hold=1.0)