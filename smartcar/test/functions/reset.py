# -*- coding: utf-8 -*-
"""
机械臂复位调试脚本

对应 ArmController 方法：
- reset_position()
- reset_y()
- reset_x()
- switch_side(side)
- set_position_start(y_position)
"""

import time
from smartcar.whalesbot.vehicle.arm.arm_base import ArmController


def test_reset_position(arm: ArmController):
    """整体复位（Y/X 同时归零 + 手爪向上 + 手臂向右）"""
    print("[RESET] reset_position()")
    arm.reset_position()
    print("[RESET] 完成")


def test_reset_y(arm: ArmController):
    """仅复位 Y 轴"""
    print("[RESET] reset_y()")
    arm.reset_y()
    print("[RESET] Y 轴已归零")


def test_reset_x(arm: ArmController):
    """仅复位 X 轴"""
    print("[RESET] reset_x()")
    arm.reset_x()
    print("[RESET] X 轴已归零")


def test_switch_side(arm: ArmController, side="LEFT"):
    """切换机械臂方向"""
    print(f"[RESET] switch_side({side})")
    arm.switch_side(side)


def test_set_position_start(arm: ArmController, y_position=0.0):
    """设置当前位置为起点"""
    print(f"[RESET] set_position_start(y={y_position})")
    arm.set_position_start(y_position)


if __name__ == "__main__":
    arm = ArmController()
    test_reset_position(arm)