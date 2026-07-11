# -*- coding: utf-8 -*-
"""
机械臂 Y 轴（竖直方向）调试脚本

对应 ArmController 方法：
- move_y_position(target)
- y_pid_moveto(target)
- y_reset_check()
- y_get_position()
- y_params_init(...)
"""

import time
from smartcar.whalesbot.vehicle.arm.arm_base import ArmController


def test_y_get_position(arm: ArmController):
    """读取当前竖直位置"""
    pos = arm.y_get_position()
    print(f"[Y] 当前竖直位置: {pos}")
    return pos


def test_move_y_position(arm: ArmController, target=0.1):
    """移动到指定竖直位置"""
    print(f"[Y] move_y_position -> {target}")
    arm.move_y_position(target)
    print(f"[Y] 实际位置: {arm.y_get_position()}")


def test_y_pid_moveto(arm: ArmController, target=0.1):
    """单步 PID 推到目标位置"""
    print(f"[Y] y_pid_moveto -> {target}")
    ok = arm.y_pid_moveto(target)
    print(f"[Y] 是否到位: {ok}")


def test_y_reset_check(arm: ArmController):
    """检查竖直方向是否触底"""
    flag = arm.y_reset_check()
    print(f"[Y] y_reset_check: {flag}")
    return flag


if __name__ == "__main__":
    arm = ArmController()
    test_y_get_position(arm)
    test_y_reset_check(arm)
    test_move_y_position(arm, target=0.1)
    time.sleep(0.5)
    test_move_y_position(arm, target=0.2)