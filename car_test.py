#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import threading
import os
import platform
import signal
from camera import Camera
import numpy as np
# from vehicle import SensorAi, SerialWrap, ArmBase, ScreenShow, Key4Btn
from simple_pid import PID
import cv2
# from task_func import MyTask


if __name__ == "__main__":
    # kill_other_python()
    # my_car = MyCar()
    # my_car.beep()
    # time.sleep(0.2)
    # my_car.lane_time(0.3, 5)
    import math, datetime
    now_time = datetime.datetime.now().strftime('%Y%m%d%H%M%S.%f')[2:-3]  # 当前日期格式化
    print(now_time)
    time.sleep(0.2)
    now_time = datetime.datetime.now().strftime('%Y%m%d%H%M%S.%f')[2:-3]  # 当前日期格式化
    print(now_time)
    # from vehicle import MecanumBase
    # car = MecanumBase()
    # car.beep()
    # print(car.get_odom())
    # car.set_pos_offset([0, 0, math.pi/2], 2)
    # car.stop()
    # print(car.get_odom())
    # 前进
    # print("forward")
    # car.set_pos_offset([0.3, 0, 0])
    # # 后退
    # print("backward")
    # car.set_pos_offset([-0.3, 0, 0])
    # car.set_pos_offset([-0.3, -0.3, -2], 5)
    # while True:
    #     car.set_pos([0.3, 0.2, math.pi/2], 2)
    #     time.sleep(1)
    #     car.set_pos([0, 0, 0], 2)
    #     time.sleep(1)
    # while True:

    #     car.turn(1.5, 1.57)
    #     time.sleep(1)

    #     car.turn(1.5, -1.57)
    #     time.sleep(1)
    # car = ()
    # while True:
    #     car.move_closed([-0.15, 0.07], 1)
    #     time.sleep(2)
    #     car.move_closed([0.15, - 0.07], 1)
    #     time.sleep(2)
    # while True:
    #     if my_car.key.get_key() == 3:
    #         my_car.task.arm.switch_side(-1)
    #         my_car.task.arm.reset()
    #         my_car.task.arm.set(-0.05, 0, 1.5)
    #         # while True:

    #             # my_car.lane_forward(0.2, 0.0, 0.1)
    #         my_car.lane_time(0.2, 1)
    #         my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    #         my_car.lane_task_location(0.3, 2)
            
    # my_car.task.pick_up_block()
    # my_car.task.put_down_self_block()
    # my_car.lane_time(0.2, 2)
    # my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    # my_car.lane_task_location(0.3, 2)
    # my_car.task.pick_up_block()
    # my_car.close()
    # print(time.time())
    # my_car.lane_task_location(0.3, 2)


    # my_car.debug()
    # programs = [func1, func2, func3, func4, func5, func6]
    # my_car.manage(programs)
    # import sys
    # test_ord = 0
    # if len(sys.argv) >= 2:
    #     test_ord = int(sys.argv[1])
    # print("test:", test_ord)
    # car_test(test_ord)
