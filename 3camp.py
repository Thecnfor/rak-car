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

sys.path.append(os.path.abspath(os.path.dirname(__file__))) 

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    # my_car.task.reset()
   
    
####################################################################################################
    #my_car.task.arm.set(0,0)
     #my_car.task.arm.grap(1)
    #my_car.task.arm.set_hand_angle(80)
    angle_offset = -math.pi/2*0.82
    # dis_angle = -math.pi/2*0.3
    # dis = 1.
    dis_x = 1.36
    dis_y = -0.87
    # print(dis_x, dis_y)
    angle_now = my_car.get_odometry()[2]
    x_offset = dis_x*math.cos(angle_now) - dis_y*math.sin(angle_now)
    y_offset = dis_y*math.cos(angle_now) + dis_x*math.sin(angle_now)
    angle_tar = angle_now - math.pi*2 + angle_offset
    pose = my_car.get_odometry().copy()
    pose[0] = pose[0] + x_offset
    pose[1] = pose[1] + y_offset
    pose[2] = angle_tar
    # print(pose)
    # return

    my_car.lane_sensor(0.3, value_h=1, sides=-1)
    # time.sleep(25)
    # my_car.lane_sensor()
    my_car.lane_dis_offset(0.3, 0.5)
    my_car.set_vel_time(0.3, 0, -0.5, 8)
    my_car.lane_dis_offset(0.3, 4)
    
    #my_car.set_pose(pose, vel=[0.2, -0.2, math.pi/3])