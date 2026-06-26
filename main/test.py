import sys

sys.path.append("/home/jetson/workspace/vehicle_wbt/")

import time
import threading
import os
import numpy as np
from task_func import MyTask
from log_info import logger
from car_wrap import MyCar
from tools import CountRecord
import math
from ernie_bot.base import answer

import cv2
import numpy as np
import numpy as np
index_form = {
    0: 'cylinder3',
    1: 'turn_right',
    2: 'chili',
    3: 'tofu',
    4: 'turn_left',
    5: 'cylinder2',
    6: 'cylinder1',
    7: 'potato',
    8: 'tomato',
    9: 'text_det',
    10: 'meat',
    11: 'chicken',
    12: 'celery',
    13: 'egg',
    14: 'green_beans',
    15: 'mushroom',
    16: 'cauliflower',
    17: 'greens'
}


def get_key_by_value(d, value):
    for k, v in d.items():
        if v == value:
            return k
    return None



if __name__ == '__main__':
    my_car = MyCar()
    my_car.STOP_PARAM = False
    my_car.task.arm.grap(1)

    # the_food1 = 'egg'
    # the_food2 = 'tomato'
    # my_car.lane_dis_offset(0.3, 1)
    # 巡航找文字
    '''the_food1="tomato"
    the_food2="egg"
    my_car.task.arm.switch_side(-1)

    my_car.task.arm.set(0.15, 0.01)  # 摄像头看得到两个文字的高度

    tar = [16, 0, 'text_det', 0.9494228959083557, 0.075, -0.2520833333333333, 0.54, 0.4875]

    my_car.lane_det_location_v4(0.2, tar, side=-1, dis_out=0.5)  # 巡航到第一列有文字的地方
    time.sleep(0.5)
    try:
        text1 = my_car.get_ocr_list_plus()  # 获取文字
        print(text1)
    except Exception as e:
        print("/n", e, "/n")
        text1 = ['表面裹着红亮的酱汁呈深褐色，表面略带光泽，酸甜味突出外酥里嫩，酱汁浓郁，口感层次丰富。',
                 '金黄色与鲜红色混合食材软烂出汁，整体呈现红黄相间的色彩。']

    # 识别文字，这个先不写
    my_car.set_vel_time(0.2, 0, 0, 2)
    # my_car.set_pose_offset([0.4, 0, 0])  # 向前走点,不要看到第一列文字

    my_car.lane_det_location_v4(0.2, tar, side=-1, dis_out=0.5)  # 巡航到第二列有文字的地方
    time.sleep(1)

    try:

        text2 = my_car.get_ocr_list_plus()  # 获取文字
        print(text2)
    except Exception as e:
        print("/n", e, "/n")
        text2 = ['表面油亮，整体呈现鲜亮的绿色，口感清脆，带有淡淡的蔬菜清甜，味道清淡爽口。',
                 '深绿色的切片与浅棕色的肉块混合，肉片略带焦香，带有明显的辛辣味，咸香嫩滑整体口感鲜辣开胃间的色彩。']

    try:
        text_all = the_food1 + ',' + the_food2 + ',1.' + text1[0] + ',2.' + text1[1] + ',3.' + text2[0] + ',4.' + text2[
            1]
        answer_dish = answer.ask2(text_all)
        print(answer_dish)
        answer_dish = answer_dish['answer']
        answer_dish=int(answer_dish)
    except Exception as e:
        print("/n", e, "/n")
        answer_dish = 3
    # my_car.set_pose_offset([-0.1,0,0])#向后退一点

    if answer_dish == 1 or answer_dish == 3:
        if answer_dish == 1:
            back = -0.24
        if answer_dish == 3:
            back = -0.09
        # 放上栏 第一个
        # my_car.set_pose_offset([back, 0, 0])
        my_car.set_vel_time(back / 0.5, 0, 0, 0.5)
        my_car.task.arm.grap(1)  # 吸
        time.sleep(1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前
        my_car.task.arm.set(0.1, 0.135)  # 抬起手臂
        my_car.task.arm.set(0.0, 0.135)  # 伸手去放物品
        my_car.set_vel_time(0.1, 0, 0, 0.1) #向前挪一点点
        my_car.task.arm.grap(0)  # 松手
        my_car.task.arm.set(0.19, 0.132)  # 缩手

        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        #my_car.task.arm.set_arm_angle(-120)  # 手臂里弯
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.26, 0)  # 向下拿
        my_car.task.arm.set(0.26, 0.132)
        my_car.task.arm.switch_side(-1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前

        my_car.set_vel_time(-0.1, 0, 0, 0.65)
        my_car.task.arm.set(0, 0.132)  # 伸手去放物品
        my_car.set_vel_time(-0.1, 0, 0, 0.1)
        my_car.task.arm.grap(0)  # 松手
        my_car.task.arm.set(0.266, 0.132)

    if answer_dish == 2 or answer_dish == 4:
        if answer_dish == 2:
            back = -0.24
        if answer_dish == 4:
            back = -0.09
        # 放下栏 第一个
        # my_car.set_pose_offset([back, 0, 0])
        my_car.set_vel_time(back / 0.5, 0, 0, 0.5)
        my_car.task.arm.set(0.15, 0.03)
        my_car.task.arm.grap(1)  # 吸
        time.sleep(1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前
        my_car.task.arm.set(0.1, 0.03)  # 压低手臂
        my_car.task.arm.set(0, 0.02)  # 伸手去放物品
        my_car.set_vel_time(0.1, 0, 0, 0.1)
        my_car.task.arm.grap(0)  # 松手
        my_car.task.arm.set(0.19, 0.06)  # 缩手

        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        #my_car.task.arm.set_arm_angle(-120)  # 手臂里弯
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.26, 0)  # 向下拿
        my_car.task.arm.set(0.26, 0.05)
        my_car.task.arm.switch_side(-1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前

        my_car.set_vel_time(-0.1, 0, 0, 0.65)
        my_car.task.arm.set(0.1, 0.03)  # 压低手臂
        my_car.task.arm.set(0, 0.02)  # 伸手去放物品
        my_car.set_vel_time(-0.1, 0, 0, 0.1)
        my_car.task.arm.grap(0)  # 松手
        my_car.task.arm.set(0.266, 0.03)  # 缩j/'''
        
    '''
    my_car.task.arm.switch_side(-1)
    my_car.task.arm.set(0.26,0.15)
    my_car.task.arm.set_hand_angle(60)
    my_car.task.arm.grap(1)
    my_car.task.arm.set(0.26,0)
    my_car.task.arm.set(0.26,0.15)
    '''    
    
    
    
    
    #沙包测试
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

    # my_car.task.arm.set(0.27, 0.1)  ############

    my_car.lane_dis_offset(0.3, 0.5)

    ## my_car.set_vel_time(0.3, -0.01, -0.53, 6)
    ## my_car.set_vel_time(0.3, -0.02, -0.52, 3)
    ## my_car.lane_dis_offset(0.3, 3.0)

    my_car.set_vel_time(0.3, -0.02, -0.53, 3)

    my_car.lane_dis_offset(0.3, 1)

    my_car.set_vel_time(0.3, -0.005, -0.53, 3)

    # my_car.lane_dis_offset(0.3, 2.7)
    my_car.lane_dis_offset(0.3, 1.1)
    
    
    
    
    my_car.lane_dis_offset(0.3, 1.3)
    my_car.set_pose_offset([0, 0, math.pi / 35], 1)

    my_car.task.eject(1)

        
    
    
