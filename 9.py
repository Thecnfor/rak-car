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

sys.path.append(os.path.abspath(os.path.dirname(__file__)))

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False


    #my_car.task.arm.set(0,0)

    #my_car.task.arm.set(0.1,0) detect
    '''
    #side==-1
    
    #pick0
    
    my_car.task.arm.set(0.1,0.1)
    my_car.task.arm.set(0.27,0.1)
    my_car.task.arm.set_offset(0, -0.032)
    my_car.task.arm.grap(1)
    time.sleep(0.5)
    my_car.task.arm.set(0.27,0.18)
    my_car.task.arm.set(0.27,0.1)
    my_car.task.arm.grap(0)
    
    
    
    #pick1
    
    my_car.task.arm.set(0.1,0.1)
    my_car.task.arm.set(0.27,0.1)
    my_car.task.arm.set_offset(0, -0.032)
    my_car.task.arm.grap(1)
    time.sleep(0.5)    
    my_car.task.arm.set(0.27,0.18)
    my_car.task.arm.set(0.27,0.12)
    my_car.task.arm.grap(0)
    my_car.task.arm.set(0.27,0.18)
    
    
    
    #pick2
    
    my_car.task.arm.set(0.1,0.1)
    my_car.task.arm.set(0.27,0.1)
    my_car.task.arm.set_offset(0, -0.032)
    my_car.task.arm.grap(1)
    time.sleep(0.5)    
    
    my_car.task.arm.set(0.27,0.205)
    my_car.task.arm.set(0.27,0.18)
    my_car.task.arm.grap(0)
    my_car.task.arm.set(0.27,0.205)
    '''
    
    '''
    #side==1
    
    #pick0
    
    my_car.task.arm.set(0.1,0.1)
    my_car.task.arm.set(0,0.1)
    my_car.task.arm.set_offset(0, -0.032)
    my_car.task.arm.grap(1)
    time.sleep(0.5)
    my_car.task.arm.set(0,0.18)
    my_car.task.arm.set(0,0.1)
    my_car.task.arm.grap(0)
    
    
    
    #pick1
    
    my_car.task.arm.set(0.1,0.1)
    my_car.task.arm.set(0,0.1)
    my_car.task.arm.set_offset(0, -0.032)
    my_car.task.arm.grap(1)
    time.sleep(0.5)    
    my_car.task.arm.set(0,0.18)
    my_car.task.arm.set(0,0.12)
    my_car.task.arm.grap(0)
    my_car.task.arm.set(0,0.18)
    
    
    '''
    #pick2
    
    '''my_car.task.arm.set(0.1,0.1)
    my_car.task.arm.set(0,0.1)
    my_car.task.arm.set_offset(0, -0.032)
    my_car.task.arm.grap(1)
    time.sleep(0.5)    
    
    my_car.task.arm.set(0,0.205)
    my_car.task.arm.set(0,0.18)
    my_car.task.arm.grap(0)
    my_car.task.arm.set(0,0.205)'''
    #my_car.task.arm.set(0,0,speed=[0.15, 0.2])
    #my_car.task.arm.set(0,0.18,speed=[0.15, 0.2])
    
    
    my_car.task.arm.grap(1)

    # the_food1 = 'egg'
    # the_food2 = 'tomato'
    # my_car.lane_dis_offset(0.3, 1)
    # 巡航找文字
    my_car.task.arm.switch_side(-1)

    my_car.task.arm.set(0.15, 0.01)  # 摄像头看得到两个文字的高度

    tar = [9, 0, 'text_det', 0.9494228959083557, 0.075, -0.2520833333333333, 0.535, 0.4875]

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
        answer_dish = 2
    # my_car.set_pose_offset([-0.1,0,0])#向后退一点
    answer_dish = 4
    if answer_dish == 1 or answer_dish == 3:
        if answer_dish == 1:
            back = -0.24
        if answer_dish == 3:
            back = -0.1
        # 放上栏 第一个
        # my_car.set_pose_offset([back, 0, 0])
        my_car.set_vel_time(back / 0.5, 0, 0, 0.5)
        my_car.task.arm.grap(1)  # 吸
        time.sleep(1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前
        my_car.task.arm.set(0.1, 0.131)  # 抬起手臂
        my_car.task.arm.set(0.0, 0.131)  # 伸手去放物品
        my_car.set_vel_time(0.1, 0, 0, 0.15)
        my_car.task.arm.grap(0)  # 松手
        my_car.task.arm.set(0.19, 0.130)  # 缩手

        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        my_car.task.arm.set_arm_angle(-120)  # 手臂里弯
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.17, 0)  # 向下拿
        my_car.task.arm.set(0.17, 0.130)
        my_car.task.arm.switch_side(-1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前
        my_car.set_vel_time(-0.1, 0, 0, 0.5)
        my_car.task.arm.set(0, 0.131)  # 伸手去放物品
        my_car.set_vel_time(-0.1, 0, 0, 0.15)
        
        my_car.task.arm.grap(0)  # 松手
        my_car.task.arm.set(0.266, 0.130)

    if answer_dish == 2 or answer_dish == 4:
        if answer_dish == 2:
            back = -0.24
        if answer_dish == 4:
            back = -0.1
        # 放下栏 第一个
        # my_car.set_pose_offset([back, 0, 0])
        my_car.set_vel_time(back / 0.5, 0, 0, 0.5)
        my_car.task.arm.set(0.15, 0.03)
        my_car.task.arm.grap(1)  # 吸
        time.sleep(1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前
        my_car.task.arm.set(0.1, 0.03)  # 压低手臂
        my_car.task.arm.set(0, 0.015)  # 伸手去放物品
        my_car.set_vel_time(0.1, 0, 0, 0.15)
        my_car.task.arm.grap(0)  # 松手
        my_car.task.arm.set(0.19, 0.06)  # 缩手

        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        my_car.task.arm.set_arm_angle(-120)  # 手臂里弯
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.17, 0)  # 向下拿
        my_car.task.arm.set(0.17, 0.05)
        my_car.task.arm.switch_side(-1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前

        my_car.set_vel_time(-0.1, 0, 0, 0.5)
        my_car.task.arm.set(0.1, 0.03)  # 压低手臂
        my_car.task.arm.set(0, 0.015)  # 伸手去放物品
        my_car.set_vel_time(-0.1, 0, 0, 0.15)
        my_car.task.arm.grap(0)  # 松手
        my_car.task.arm.set(0.266, 0.03)  # 缩
        
    
    
  