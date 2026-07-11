# -*- coding: utf-8 -*-
"""
任务: init()

对应 car_task_function.py 中的 init(reset_arm=False)
机械臂相关：arm.reset_position()（可选）
"""

import time
from car_wrap_2026 import MyCar, kill_other_python


def init(reset_arm=False):
    """系统初始化"""
    time.sleep(1)
    global my_car
    my_car = MyCar()
    my_car.STOP_PARAM = False
    my_car.beep()
    time.sleep(1)
    if reset_arm:
        my_car.arm.reset_position()
    my_car.reset_position()


if __name__ == "__main__":
    init(reset_arm=False)