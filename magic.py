
# -*- coding: utf-8 -*-
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
from ernie_bot.base import answer

sys.path.append(os.path.abspath(os.path.dirname(__file__)))



if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False

    
    
    angle_offset = -math.pi / 2 * 0.82
    # dis_angle = -math.pi/2*0.3
    # dis = 1.
    dis_x = 1.36
    dis_y = -0.87
    # print(dis_x, dis_y)
    angle_now = my_car.get_odometry()[2]
    x_offset = dis_x * math.cos(angle_now) - dis_y * math.sin(angle_now)
    y_offset = dis_y * math.cos(angle_now) + dis_x * math.sin(angle_now)
    angle_tar = angle_now - math.pi * 2 + angle_offset
    pose = my_car.get_odometry().copy()
    pose[0] = pose[0] + x_offset
    pose[1] = pose[1] + y_offset
    pose[2] = angle_tar
    # print(pose)
    # return
    
    my_car.lane_sensor(0.3, value_h=1, sides=-1)  # 
    # time.sleep(25)
    # my_car.lane_sensor()
    
    my_car.task.arm.switch_side(1)
    my_car.task.arm.set(0.27, 0.1)
   # my_car.task.arm.switch_side(1)
    
    # my_car.lane_sensor(0.3, value_h=1, sides=-1)
    
    # time.sleep(25)
    # my_car.lane_sensor()
    
    #my_car.task.arm.set(0.27, 0.1)  ############


    my_car.lane_dis_offset(0.3, 0.5)
    
    ## my_car.set_vel_time(0.3, -0.01, -0.53, 6)
    ## my_car.set_vel_time(0.3, -0.02, -0.52, 3)
    ## my_car.lane_dis_offset(0.3, 3.0)
    
    my_car.set_vel_time(0.3, -0.02, -0.53, 3)
    
    my_car.lane_dis_offset(0.3, 1)
    
    my_car.set_vel_time(0.3, -0.005, -0.53, 3)
    
   # my_car.lane_dis_offset(0.3, 2.7)
    my_car.lane_dis_offset(0.3, 1.1)
    
    # my_car.lane_dis_offset(0.3, 4.58)
    
    #my_car.set_pose_offset([0, -0.03, 0], 0.2)
    #my_car.set_pose_offset([-0.03,0,0],0.2)
    #my_car.set_pose_offset([0, 0, math.pi / 30], 1)
    
######magic
    #my_car.task.arm.set(0.17, 0.15)
    #my_car.task.arm.switch_side(1)
    
    my_car.lane_sensor(0.25, value_h=0.2, sides=1)
    #my_car.set_pose_offset([0.24, 0, 0],1.2)
    my_car.set_vel_time(0.235, 0, 0, 0.85)
    
    my_car.task.arm.switch_side(1)
    my_car.task.arm.set_hand_angle(60)
    
    my_car.task.arm.set(0.255, 0.1)
    my_car.task.arm.grap(1)
    my_car.task.arm.set(0.255, 0.08)
    time.sleep(0.5)
    my_car.task.arm.set(0.255, 0.1)
    my_car.task.arm.switch_side(-1)
    my_car.task.arm.set(0.17, 0.1)
    my_car.task.arm.set_arm_angle(-115)
    my_car.task.arm.set(0.17, 0)
    my_car.task.arm.grap(0)
    
    my_car.task.arm.set(0.17, 0.1)
    
    
    
#def eject1():
    # finetune_car_position=my_car.get_odometry()
    # print(finetune_car_position)
    # print(finetune_car_position[2]-start_car_position[2])
    # my_car.set_pose_offset([0, 0, (finetune_car_position[2]-start_car_position[2]) ] , 0.2)

    my_car.lane_dis_offset(0.3, 1)
    my_car.set_pose_offset([0, 0, math.pi / 35], 1)
    
    my_car.task.eject(1)
    time.sleep(0.5)
    # my_car.set_pose_offset([0.7, 0, 0],0.5)
    my_car.lane_dis_offset(0.3, 0.7)

    print("finish camp task!!!!!")
    
    

    
    