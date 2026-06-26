#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import threading
import os
import numpy as np
from task_func import MyTask
from log_info import logger
from car_wrap import MyCar
from tools import CountRecord
import math
import sys, os
# 添加上本文件对应目录
sys.path.append(os.path.abspath(os.path.dirname(__file__))) 

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    my_car.lane_dis_offset(0.3, 1)
    my_car.set_vel_time(-0.3, 0, 0, 0.8)
    '''
    my_car.task.arm.switch_side(-1)
    my_car.lane_dis_offset(0.2, 0.58)
    my_car.task.arm.set(0.265,0)
    tar = [9, 0, 'text_det', 0, 0.0390625, 0.7916666666666666, 0.37, 0.39166666666666666]
    my_car.lane_det_location_v4(0.2, tar, side=-1, dis_out=5)
    my_car.set_pose_offset([-0.7,0,0],2)
    '''