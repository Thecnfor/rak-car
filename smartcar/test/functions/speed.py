# -*- coding: utf-8 -*-
"""
机械臂速度调试脚本

对应 ArmController 方法：
- y_speed(velocity)
- x_speed(velocity)
"""

import time
from smartcar.whalesbot.vehicle.arm.arm_base import ArmController


def test_y_speed(arm: ArmController, velocity=0.05, duration=1.0):
    """以指定速度竖直移动"""
    print(f"[SPEED] y_speed({velocity}) for {duration}s")
    arm.y_speed(velocity)
    time.sleep(duration)
    arm.y_speed(0)
    print("[SPEED] y 停止")


def test_x_speed(arm: ArmController, velocity=0.05, duration=1.0):
    """以指定速度水平移动"""
    print(f"[SPEED] x_speed({velocity}) for {duration}s")
    arm.x_speed(velocity)
    time.sleep(duration)
    arm.x_speed(0)
    print("[SPEED] x 停止")


if __name__ == "__main__":
    arm = ArmController()
    test_y_speed(arm, velocity=0.05, duration=1.0)
    time.sleep(0.5)
    test_x_speed(arm, velocity=0.05, duration=1.0)