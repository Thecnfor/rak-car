#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 开始编码格式和运行环境选择

import math, threading
import numpy as np


from threading import Thread
import yaml, os, sys

import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))) 
# 导入自定义log模块
from log_info import logger
from vehicle import MotorConvert, Motors
from tools import PID

# 把该文件夹目录加入环境变量
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

def get_path_relative(*args):
    local_dir = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(local_dir, *args)

s_th = math.sin(math.pi/4)
r = 0.1115
# 根据小车四轮运动计算小车运动, 正解
transform_forward = np.array([[ 1/4/s_th,  1/4/s_th, 1/r/4],
                            [-1/4/s_th,  1/4/s_th, 1/r/4],
                            [-1/4/s_th, -1/4/s_th, 1/r/4],
                            [ 1/4/s_th, -1/4/s_th, 1/r/4]])
def linear2rad(linear):
    # return sp_car2world(linear_vx, linear_vy, angle_car)
    return np.dot(np.array(linear), transform_forward)

ret = linear2rad([0.17514379, 0.17514379, 0.17514379, 0.17514379])
print(ret)