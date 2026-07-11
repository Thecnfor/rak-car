# -*- coding: utf-8 -*-
"""
机械臂 X 轴（水平方向）调试脚本

对应 ArmController 方法：
- move_x_position(target, out_time)
- x_pid_moveto(target)
- x_stop_check()
- x_get_position()
- x_params_init(...)
"""

import time
from smartcar.whalesbot.vehicle.arm.arm_base import ArmController


def test_x_get_position(arm: ArmController):
    """读取当前水平位置"""
    pos = arm.x_get_position()
    print(f"[X] 当前水平位置: {pos}")
    return pos


def test_move_x_position(arm: ArmController, target=0.15, out_time=6.0):
    """移动到指定水平位置"""
    print(f"[X] move_x_position -> {target}, out_time={out_time}")
    arm.move_x_position(target, out_time=out_time)
    print(f"[X] 实际位置: {arm.x_get_position()}")


def test_x_pid_moveto(arm: ArmController, target=0.15):
    """单步 PID 推到目标位置"""
    print(f"[X] x_pid_moveto -> {target}")
    ok = arm.x_pid_moveto(target)
    print(f"[X] 是否到位: {ok}")


def test_x_stop_check(arm: ArmController):
    """检查水平方向是否停止"""
    flag = arm.x_stop_check()
    print(f"[X] x_stop_check: {flag}")
    return flag


if __name__ == "__main__":
    arm = ArmController()
    test_x_get_position(arm)
    test_x_stop_check(arm)
    test_move_x_position(arm, target=0.15)
    time.sleep(0.5)
    test_move_x_position(arm, target=0.0)