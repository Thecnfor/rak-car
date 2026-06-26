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

index_form = {
0: 'turn_right',
1: 'tofu',
2: 'chili',
3: 'turn_left',
4: 'cylinder1',
5: 'tomato',
6: 'potato',
7: 'cylinder3',
8: 'cylinder2',
9: 'text_det',
10: 'chicken',
11: 'meat',
12: 'celery',
13: 'egg',
14: 'mushroom',
15: 'green_beans',
16: 'cauliflower',
17: 'greens'
}

# 反向查找函数
def get_key_by_value(d, value):
    for k, v in d.items():
        if v == value:
            return k
    return None




if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    # my_car.task.reset()
   
    
############################################################################################


'''
######## camp
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
    my_car.set_vel_time(0.3, 0, -0.5, 4)
    my_car.lane_dis_offset(0.3, 4.65)  
    
    #my_car.set_pose(pose, vel=[0.2, -0.2, math.pi/3])
    
    
    

######## eject1
    my_car.task.eject(1)
    '''
    
    
    
    
######## food
    my_car.task.arm.switch_side(-1)  # 手臂向右
    my_car.task.arm.set(0.265, 0)  # 手臂向后退到能看到文字的位置
    tar = [9, 0, 'text_det', 0.9334853291511536, 0.0390625, 0.7916666666666666, 0.334375, 0.39166666666666666]
    my_car.lane_det_location_v4(0.2, tar, side=-1, dis_out=2)  # 巡航找文字
    # text=my_car.get_ocr_list()[0] #获取文字
    # foods=answer.ask1(text)
    # the_food=foods['answer']
    # print(the_food)
    the_food = 'egg'
    my_car.set_pose_offset([-0.25, 0, 0])  # 后退一步
    my_car.task.arm.set(0.135, 0)  # 调整手臂定位食材
    index = get_key_by_value(index_form, the_food)
    tar = [index, 0, the_food, 0.9151089787483215, 0, 0.2, 0.3, 0.7]
    my_car.lane_det_location_vert(0.135, tar, side=-1, dis_out=1)
    img_side = my_car.cap_side.read()
    dets_ret = my_car.task_det(img_side)
    print("检测目标是", dets_ret)

    for det_ret in dets_ret:
        if det_ret[0] == index:
            y = det_ret[5]
    if y < 0 or y == 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.135, 0.1)  # 手臂上抬
        my_car.task.arm.set_hand_angle(-45)  # 掌心向前
        my_car.task.arm.set(0, 0.1)  # 向前抓
        time.sleep(0.5)
        my_car.task.arm.set(0.265, 0.1)  # 往后缩
        my_car.task.arm.set_arm_angle(90)  # 手臂向外
        time.sleep(0.5)
        my_car.task.arm.grap(0)
        # time.sleep(0.5)
        # my_car.task.arm.set(0.265,0.03)#手臂向下放食材
        # my_car.task.arm.grap(0)#松手

    if y > 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.135, 0.01)  # 手臂下降
        my_car.task.arm.set_hand_angle(-45)  # 掌心向前
        my_car.task.arm.set(0, 0.03)  # 向前抓
        time.sleep(0.5)
        my_car.task.arm.set(0.265, 0.05)  # 往后缩

        my_car.task.arm.set_arm_angle(90)  # 手臂向外
        time.sleep(0.5)
        my_car.task.arm.grap(0)

        # my_car.task.arm.set_hand_angle(90)#掌心向下
        # time.sleep(0.5)
        # my_car.task.arm.set(0.265,0.03)#手臂向下放食材
        # my_car.task.arm.grap(0)#松

    my_car.set_pose_offset([-0.5, 0, 0])  # 后退取第二个食材

    my_car.task.arm.switch_side(1)  # 手臂向左
    my_car.task.arm.set(0, 0)  # 达到摄像头能看到文字的位置
    tar = [9, 0, 'text_det', 0.9307849407196045, -0.0546875, 0.7645833333333333, 0.31, 0.30416666666666664]
    my_car.lane_det_location_v4(0.2, tar, side=1, dis_out=1)  # 巡航找文本
    # text=my_car.get_ocr_list()[0]   #获取文字
    # foods=answer.ask1(text)
    # the_food=foods['answer']
    # print(the_food)
    the_food = 'tomato'

    my_car.set_pose_offset([-0.25, 0, 0])  # 后退
    my_car.task.arm.set(0.135, 0)  # 调整手臂定位食材

    index = get_key_by_value(index_form, the_food)
    tar = [index, 0, the_food, 0.7483181953430176, 0, 0.2, 0.31, 0.7]
    # tar=[index, 0, answer, 0.5542138814926147, 0, 0.192, 0.18, 0.375]
    my_car.lane_det_location_vert(0.15, tar, side=1, dis_out=1)
    img_side = my_car.cap_side.read()
    dets_ret = my_car.task_det(img_side)
    print("检测目标是", dets_ret)

    for det_ret in dets_ret:
        if det_ret[0] == index:
            y = det_ret[5]
    if y < 0 or y == 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.135, 0.1)  # 手臂上抬
        my_car.task.arm.set_hand_angle(-45)  # 掌心向前
        my_car.task.arm.set(0.27, 0.1)  # 向前抓
        time.sleep(0.5)
        my_car.task.arm.set(0, 0.1)  # 往后缩
        # time.sleep(0.5)
        # my_car.task.arm.set_hand_angle(90)#掌心向下
        # time.sleep(0.5)

        # my_car.task.arm.set(0,0.03)#手臂向下放食材
        # my_car.task.arm.grap(0)#松手

    if y > 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.135, 0.01)  # 手臂下降
        my_car.task.arm.set_hand_angle(-45)  # 掌心向前
        my_car.task.arm.set(0.27, 0.01)  # 向前抓
        time.sleep(0.5)
        my_car.task.arm.set(0, 0.01)  # 往后缩
        # time.sleep(0.5)
        # my_car.task.arm.set_hand_angle(90)#掌心向下
        # time.sleep(0.5)
        # my_car.task.arm.set(0,0.03)#手臂向下放食材
        # my_car.task.arm.grap(0)#松

    
    
    