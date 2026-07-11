# -*- coding: utf-8 -*-
"""
机械臂姿态调试脚本

对应 ArmController 方法：
- set_arm_pose(x, y, arm, hand)
- set_arm_angle(angle, speed)
- set_hand_angle(angle, speed)
- go_for(x_offset, y_offset, time_run, speed)
- goto_position(x, y, time_run, speed)
"""

import time
from smartcar.whalesbot.vehicle.arm.arm_base import ArmController


def test_set_arm_pose(arm: ArmController, x=0.0, y=0.2, arm_side="LEFT", hand="DOWN"):
    """设置机械臂位姿"""
    print(f"[POSE] set_arm_pose(x={x}, y={y}, arm={arm_side}, hand={hand})")
    arm.set_arm_pose(x=x, y=y, arm=arm_side, hand=hand)


def test_set_arm_angle(arm: ArmController, angle="RIGHT", speed=80):
    """设置手臂角度"""
    print(f"[POSE] set_arm_angle({angle}, speed={speed})")
    arm.set_arm_angle(angle, speed)


def test_set_hand_angle(arm: ArmController, angle="UP", speed=80):
    """设置手部角度"""
    print(f"[POSE] set_hand_angle({angle}, speed={speed})")
    arm.set_hand_angle(angle, speed)


def test_goto_position(arm: ArmController, x=0.15, y=0.1, speed=[0.15, 0.04]):
    """移动到指定 (x, y)"""
    print(f"[POSE] goto_position(x={x}, y={y}, speed={speed})")
    arm.goto_position(x=x, y=y, speed=speed)


def test_go_for(arm: ArmController, x_offset=0.05, y_offset=0.0, time_run=1.0):
    """相对当前位置偏移"""
    print(f"[POSE] go_for(dx={x_offset}, dy={y_offset}, time={time_run})")
    arm.go_for(x_offset, y_offset, time_run=time_run)


if __name__ == "__main__":
    arm = ArmController()
    test_set_arm_pose(arm, x=0.0, y=0.2, arm_side="LEFT", hand="DOWN")
    time.sleep(1)
    test_set_arm_pose(arm, arm_side="RIGHT", hand="UP")