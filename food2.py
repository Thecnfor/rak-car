# -*- coding: utf-8 -*-
import re
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

from multiprocessing import Process
import time
from ernie_bot.base import ernie_test
from ernie_bot.base import answer

'''index_form = {
  0:"turn_right",
  1:"turn_left",
  2:"chicken",
  3:"tofu",
  4:"celery",
  5:"text_det",
  6:"meat",
  7:"chili",
  8:"tomato",
  9:"potato",
  10:"cauliflower",
  11:"egg",
  12:"mushroom",
  13:"greens",
  14:"green_beans",
  15:"cylinder1",
  16:"cylinder3",
  17:"cylinder2"
}'''
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


# 示例

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    
    
    #拿食材
    
    

    my_car.task.arm.switch_side(-1)  # 手臂向右
    my_car.task.arm.set(0.265, 0)  # 手臂向后退到能看到文字的位置
    tar = [9, 0, 'text_det', 0.9334853291511536, 0.0390625, 0.7916666666666666, 0.334375, 0.39166666666666666]
    my_car.lane_det_location_v4(0.2, tar, side=-1, dis_out=1)  # 巡航找文字
    try:
        text=my_car.get_ocr_list()[0] #获取文字
        foods=answer.ask1(text)
        the_food1=foods['answer']
        print(the_food1)
    except Exception as e:
        print(e)
        the_food1='egg'
    my_car.set_pose_offset([-0.25, 0, 0])  # 后退一步
    my_car.task.arm.set(0.135, 0)  # 调整手臂定位食材
    index = get_key_by_value(index_form, the_food1)
    tar = [index, 0, the_food1, 0.9151089787483215, 0, 0.2, 0.3, 0.7]
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
    try:
        text=my_car.get_ocr_list()[0]   #获取文字
        foods=answer.ask1(text)
        the_food2=foods['answer']
        print(the_food2)
    except Exception as e:
        print(e)
        the_food2 = 'tomato'

    my_car.set_pose_offset([-0.25, 0, 0])  # 后退
    my_car.task.arm.set(0.135, 0)  # 调整手臂定位食材

    index = get_key_by_value(index_form, the_food2)
    tar = [index, 0, the_food2, 0.7483181953430176, 0, 0.2, 0.31, 0.7]
    print(f"tar:{tar}")
    # tar=[index, 0, answer, 0.5542138814926147, 0, 0.192, 0.18, 0.375]
    my_car.lane_det_location_vert(0.135, tar, side=1, dis_out=1)
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
      
      
      
      


    #做题
    
    
    
    
    
    
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
    
    
    
    
    
    
    
    
    #放食物
    
    
    
    
    
    
    
    
    my_car.task.arm.switch_side(-1)
    
    my_car.task.arm.set(0.15,0.01) #摄像头看得到两个文字的高度
    
    tar=[9, 0, 'text_det', 0.9494228959083557, 0.075, -0.2520833333333333, 0.4375, 0.4875]
    
    my_car.lane_det_location_v4(0.2, tar, side=-1,dis_out=1)#巡航到第一列有文字的地方
    try:
    
        text1=my_car.get_ocr_list() #获取文字
    except Exception as e:
        print(e)
        text1=['表面裹着红亮的酱汁呈深褐色，表面略带光泽，酸甜味突出外酥里嫩，酱汁浓郁，口感层次丰富。','金黄色与鲜红色混合食材软烂出汁，整体呈现红黄相间的色彩。']
    
    
    #识别文字，这个先不写
    
    my_car.set_pose_offset([0.4,0,0]) #向前走点,不要看到第一列文字
    
    my_car.lane_det_location_v4(0.2, tar, side=-1,dis_out=1)#巡航到第二列有文字的地方
    
    try:
    
        text2=my_car.get_ocr_list() #获取文字
    except Exception as e:
        print(e)
        text2=['表面油亮，整体呈现鲜亮的绿色，口感清脆，带有淡淡的蔬菜清甜，味道清淡爽口。','深绿色的切片与浅棕色的肉块混合，肉片略带焦香，带有明显的辛辣味，咸香嫩滑整体口感鲜辣开胃间的色彩。']
        
        
        
        
    text_all=the_food1+','+the_food2+',1.'+text1[0]+',2.'+text1[1]+',3.'+text2[0]+',4.'+text2[1]
    
    try:
        answer_dish=answer.ask2(text_all)
    except Exception as e:
        print(e)
        answer_dish=3
    #my_car.set_pose_offset([-0.1,0,0])#向后退一点
    
    
    
    if answer_dish==1 or answer_dish==3:
    
        if answer_dish==1:
            back=-0.1
        
        if answer_dish==3:
            back=-0.27
        #放上栏 第一个
        my_car.set_pose_offset([back,0,0])
        
        my_car.task.arm.grap(1) #吸
        
        time.sleep(3)
        
        my_car.task.arm.set_hand_angle(-45)#手心向前
        
        my_car.task.arm.set(0.1,0.132) #抬起手臂
        
        
        my_car.task.arm.set(0.02,0.132) #伸手去放物品
        
        my_car.task.arm.grap(0) #松手
        
        my_car.task.arm.set(0.266,0.132) #缩手
        
        
    if answer_dish==2 or answer_dish==4:    
        
        if answer_dish==2:
            back=-0.1
        
        if answer_dish==4:
            back=-0.27
        
        #放下栏 第一个
        my_car.set_pose_offset([back,0,0])
        
        my_car.task.arm.set(0.15,0.03)
        
        my_car.task.arm.grap(1) #吸
        time.sleep(3)
        
        my_car.task.arm.set_hand_angle(-45)#手心向前
        
        my_car.task.arm.set(0.1,0.03) #压低手臂
        
        
        my_car.task.arm.set(0,0.03) #伸手去放物品
        
        my_car.task.arm.grap(0) #松手
        
        my_car.task.arm.set(0.266,0.03) #缩手
      
      
    
    
    
    
   
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    