import time
import threading
import os
import numpy as np
from task_func import MyTask,task_reset
from log_info import logger
from car_wrap import MyCar
from tools import CountRecord
import math
import sys, os

sys.path.append(os.path.abspath(os.path.dirname(__file__)))

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    my_car.task.arm.set(0.15,0.13)#手臂高一点不要拍到下面的
    
    my_car.task.arm.switch_side(1)#手臂向左
    
    tar=[5, 0, 'text_det', 0.9527782797813416, 0.0140625, 0.3145833333333333, 0.396875, 0.42916666666666664]#目标位置
    
    my_car.lane_det_location_v4(0.2, tar, side=1,dis_out=1)#巡航找文本
    
    my_car.set_pose_offset([-0.04,0,0])#第二个选项
    
    #my_car.set_pose_offset([-0.12,0,0])#第一个选项

    my_car.task.arm.grap(1)#吸物体
    
    time.sleep(3)

    my_car.task.arm.set(0.15,0.07)#下降
    
    my_car.task.arm.set(0.25,0.07)#碰选项