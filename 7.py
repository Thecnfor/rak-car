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
import subprocess
from ernie_bot.base import answer
from main.qqq import camp

sys.path.append(os.path.abspath(os.path.dirname(__file__)))

index_form = {
    0: "cauliflower",
    1: "greens",
    2: "cylinder3",
    3: "turn_right",
    4: "chili",
    5: "tofu",
    6: "turn_left",
    7: "eggplant",
    8: "wax_gourd",
    9: "cylinder2",
    10: "cylinder1",
    11: "potato",
    12: "tomato",
    13: "cabbage",
    14: "meat",
    15: "sparerib",
    16: "text_det",
    17: "chicken",
    18: "celery",
    19: "egg",
    20: "mushroom",
    21: "green_beans",
}

def get_key_by_value(d, value):
    for k, v in d.items():
        if v == value:
            return k
    return None


def get_food():
    my_car.task.arm.switch_side(-1)  # 手臂向右
    my_car.task.arm.set(0.265, 0)  # 手臂向后退到能看到文字的位置
    tar = [
        16,
        0,
        "text_det",
        0,
        0.0390625,
        0.7916666666666666,
        0.37,
        0.39166666666666666,
    ]
    # time.sleep(1)
    # my_car.set_velocity(0.3, 0, 0)
    time.sleep(0.5)
    flag, offset = my_car.lane_det_location_v4(
        0.2, tar, side=-1, dis_out=1
    )  # 巡航找文字
    time.sleep(0.7)
    try:
        text = my_car.get_ocr_list_plus()
        # 获取文字
        print(text)
        text = text[0]
        foods = answer.ask1(text)
        the_food1 = foods["answer"]
        print("\n食物是\n", the_food1)
    except Exception as e:
        print(e)
        the_food1 = "egg"
    # my_car.set_vel_time(-0.5,0,0,0.5)
    v = -0.5
    t = -offset / v
    v = -0.48
    print("时间是", t)
    my_car.set_vel_time(v, 0, 0, t)  # 后退
    my_car.set_vel_time(0.3, 0, 0, 0.6)

    # my_car.set_pose_offset([-0.25, 0, 0])  # 后退一步
    my_car.task.arm.set(0.135, 0)  # 调整手臂定位食材
    index = get_key_by_value(index_form, the_food1)
    if index == None:
        index = 12
    #tar = [index, 0, the_food1, 0.9151089787483215, 0, 0.2, 0.33, 0.7]
    tar = [index, 0, the_food1, 0.9151089787483215, 0, 0.2, 0.342, 0.7]
    
    flag, offset = my_car.lane_det_location_vert(0.15, tar, side=-1, dis_out=1)

    try:
        img_side = my_car.cap_side.read()
        dets_ret = my_car.task_det(img_side)
        print("object is", dets_ret)
        for det_ret in dets_ret:
            if det_ret[0] == index:
                y = det_ret[5]
    except:
        y = 0
    if flag == False:
        y = -1

    if y < 0 or y == 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.135, 0.095)  # 手臂上抬
        my_car.task.arm.set_hand_angle(-80)  # 掌心向前
        my_car.task.arm.set(0, 0.115)  # 向前抓
        time.sleep(0.5)

        my_car.task.arm.set(0.268, 0.1)  # 往后缩
        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        my_car.task.arm.set_arm_angle(-95)
        my_car.task.arm.set(0.27, 0)
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松
        my_car.task.arm.set(0.27, 0.05)
        time.sleep(1)
        """my_car.task.arm.set(0.18, 0.1)  # 往后缩
        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        
        my_car.task.arm.set_arm_angle(-121)  # 手臂里弯
        my_car.task.arm.set(0.18, 0.02)  # 手臂下放
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松手
        time.sleep(1)"""

    if y > 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.135, 0)  # 手臂下降
        my_car.task.arm.set_hand_angle(-80)  # 掌心向前
        my_car.task.arm.set(0, 0.012)  # 向前抓
        time.sleep(0.5)

        my_car.task.arm.set(0.268, 0.05)  # 往后缩
        my_car.task.arm.set_hand_angle(60)
        my_car.task.arm.set_arm_angle(-95)
        my_car.task.arm.set(0.268, 0)
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松
        my_car.task.arm.set(0.268, 0.05)
        time.sleep(1)
        """my_car.task.arm.set(0.18, 0.05)  # 往后缩
        my_car.task.arm.set_hand_angle(60)  # 掌心向下
        my_car.task.arm.set_arm_angle(-115)  # 手臂里弯
        my_car.task.arm.set(0.18, 0.02)  # 手臂下放
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松
        time.sleep(1)"""

    v = -0.5
    t = -offset / v
    v = -0.5
    print("时间是", t)
    my_car.set_vel_time(v, 0, 0, t)  # 后退
    my_car.task.arm.switch_side(1)  # 手臂向左
    my_car.task.arm.grap(0)
    my_car.task.arm.set(0.268, 0.1)
    my_car.task.arm.set(0, 0.06)
    my_car.task.arm.set_hand_angle(-80)
    my_car.task.arm.set(0, 0.02)  # 达到摄像头能看到文字的位置
    my_car.task.arm.set_hand_angle(-80)
    tar = [
        16,
        0,
        "text_det",
        0.9307849407196045,
        -0.0546875,
        0.7645833333333333,
        0.34,
        0.30416666666666664,
    ]
    time.sleep(1)
    flag, offset = my_car.lane_det_location_v4(
        0.2, tar, side=1, dis_out=1
    )  # 巡航找文本
    time.sleep(0.7)
    try:
        text = my_car.get_ocr_list_plus()[0]  # 获取文字
        print(text)
        foods = answer.ask1(text)
        the_food2 = foods["answer"]
        print("\n第二个食物是\n", the_food2)
    except Exception as e:
        print(e)
        the_food2 = "tomato"
    if flag == False:
        y = -1

    v = -0.5
    t = -offset / v
    v = -0.48
    print("时间是", t)
    my_car.set_vel_time(v, 0, 0, t)  # 后退

    my_car.task.arm.set(0.10, 0.02)  # 调整手臂定位食材

    index = get_key_by_value(index_form, the_food2)
    if index == None:
        index = 19
    tar = [index, 0, the_food2,0.8597215414047241, 0.028125, 0.37291666666666665, 0.40, 0.2625]  #   0.7483181953430176, 0, 0.2, 0.27, 0.7
    print(f"tar:{tar}")
    # tar=[index, 0, answer, 0.5542138814926147, 0, 0.192, 0.18, 0.375]
    flag, offset = my_car.lane_det_location_vert(
        0.135, tar, side=1, dis_out=1.2
    )  # 巡航找食材
    try:
        img_side = my_car.cap_side.read()
        dets_ret = my_car.task_det(img_side)
        print("检测目标是", dets_ret)

        for det_ret in dets_ret:
            if det_ret[0] == index:
                y = det_ret[5]
        print("-----------------y值是----------------------", y)
    except:
        y = -1

    if y < 0 or y == 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.1, 0.095)  # 手臂上抬
        my_car.task.arm.set_hand_angle(-80)  # 掌心向前
        my_car.task.arm.set(0.27, 0.12)  # 向前抓
        time.sleep(0.5)
        my_car.task.arm.set(0, 0.12)  # 往后缩
        # time.sleep(0.5)
        # my_car.task.arm.set_hand_angle(90)#掌心向下
        # time.sleep(0.5)

        # my_car.task.arm.set(0,0.03)#手臂向下放食材
        # my_car.task.arm.grap(0)#松手

    if y > 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.1, 0.015)  # 手臂下降
        my_car.task.arm.set_hand_angle(-80)  # 掌心向前
        my_car.task.arm.set(0.08, 0.1)
        my_car.task.arm.set(0.18, 0.01)  # 向前抓
        my_car.task.arm.set(0.26, 0.01)
        time.sleep(0.5)
        my_car.task.arm.set(0.15, 0.1)
        my_car.task.arm.set(0, 0.04)  # 往后缩
        # time.sleep(0.5)
        # my_car.task.arm.set_hand_angle(90)#掌心向下
        # time.sleep(0.5)
        # my_car.task.arm.set(0,0.03)#手臂向下放食材
        # my_car.task.arm.grap(0)#松
    if flag == False:
        my_car.set_vel_time(-0.3, 0, 0, 1.2)
    my_car.set_pose_offset([0, -0.03, 0], 0.2)
    return [the_food1, the_food2]
    
    
    
def answer2():
    my_car.lane_dis_offset(0.3, 1.3)  ######my_car.lane_dis_offset(0.3, 0.4)
    my_car.set_pose_offset([0, -0.055, 0], 1)  ######my_car.lane_dis_offset(0.3, 0.4)
    my_car.lane_sensor(0.30, value_h=0.3, sides=1)  #####(0.2, value_h=0.3, sides=1)
    my_car.set_pose_offset([0, 0, math.pi / 30], 0.5)
    my_car.set_pose_offset([0.235, 0, 0], 1)
    my_car.task.arm.grap(1)
    time.sleep(0.5)
    my_car.task.arm.set_hand_angle(-80)
    time.sleep(0.5)
    my_car.task.arm.set(0.1, 0.07)
    time.sleep(0.5)
    
    my_car.set_pose_offset([0, 0.015, 0], 0.5)
    
    my_car.task.arm.set(0.25, 0.07)  # 推的距离调整
    my_car.task.arm.set(0.1, 0.07)
    my_car.set_pose_offset([0, 0.02, 0], 0.5)
    
    
    
def put_food(the_food1, the_food2):
    my_car.task.arm.grap(1)

    # the_food1 = 'egg'
    # the_food2 = 'tomato'
    # my_car.lane_dis_offset(0.3, 1)
    # 巡航找文字
    my_car.task.arm.switch_side(-1)

    my_car.task.arm.set(0.15, 0.01)  # 摄像头看得到两个文字的高度

    tar = [
        16,
        0,
        "text_det",
        0.9494228959083557,
        0.075,
        -0.2520833333333333,
        0.385,
        0.4875,
    ]# w = 0.385

    my_car.lane_det_location_v4(
        0.2, tar, side=-1, dis_out=0.5
    )  # 巡航到第一列有文字的地方
    time.sleep(0.5)
    try:
        text1 = my_car.get_ocr_list_plus()  # 获取文字
        print(text1)
    except Exception as e:
        print("/n", e, "/n")
        text1 = [
            "表面裹着红亮的酱汁呈深褐色，表面略带光泽，酸甜味突出外酥里嫩，酱汁浓郁，口感层次丰富。",
            "金黄色与鲜红色混合食材软烂出汁，整体呈现红黄相间的色彩。",
        ]

    # 识别文字，这个先不写
    my_car.set_vel_time(0.2, 0, 0, 2)
    # my_car.set_pose_offset([0.4, 0, 0])  # 向前走点,不要看到第一列文字

    my_car.lane_det_location_v4(
        0.2, tar, side=-1, dis_out=0.5
    )  # 巡航到第二列有文字的地方
    time.sleep(1)

    try:

        text2 = my_car.get_ocr_list_plus()  # 获取文字
        print(text2)
    except Exception as e:
        print("/n", e, "/n")
        text2 = [
            "表面油亮，整体呈现鲜亮的绿色，口感清脆，带有淡淡的蔬菜清甜，味道清淡爽口。",
            "深绿色的切片与浅棕色的肉块混合，肉片略带焦香，带有明显的辛辣味，咸香嫩滑整体口感鲜辣开胃间的色彩。",
        ]

    try:
        text_all = (
            the_food1
            + ","
            + the_food2
            + ",1."
            + text1[0]
            + ",2."
            + text1[1]
            + ",3."
            + text2[0]
            + ",4."
            + text2[1]
        )
        answer_dish = answer.ask2(text_all)
        print(f"\n{answer_dish}\n")
        answer_dish = answer_dish["answer"]
        answer_dish = int(answer_dish)
    except Exception as e:
        print("/n", e, "/n")
        answer_dish = 3
        print(f"\n{answer_dish}\n")
    my_car.set_pose_offset([0.02,0,0])

    if answer_dish == 1 or answer_dish == 3:
        if answer_dish == 1:
            back = -0.24
        if answer_dish == 3:
            back = -0.115
        # 放上栏 第一个
        # my_car.set_pose_offset([back, 0, 0])
        my_car.set_vel_time(back / 0.515, 0, 0, 0.5)
        my_car.task.arm.grap(1)  # 吸
        time.sleep(1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前
        my_car.task.arm.set(0.1, 0.135)  # 抬起手臂
        my_car.task.arm.set(0.0, 0.135)  # 伸手去放物品
        my_car.set_vel_time(0.1, 0, 0, 0.1)  # 向前挪一点点
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松手
        time.sleep(0.5)
        my_car.task.arm.set(0.19, 0.132)  # 缩手

        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        # my_car.task.arm.set_arm_angle(-120)  # 手臂里弯
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.26, 0)  # 向下拿
        my_car.task.arm.set(0.26, 0.132)
        my_car.task.arm.switch_side(-1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前

        my_car.set_vel_time(-0.1, 0, 0, 0.55)
        my_car.task.arm.set(0, 0.132)  # 伸手去放物品
        my_car.set_vel_time(-0.1, 0, 0, 0.1)
        my_car.set_pose_offset([-0.02,0,0])
        my_car.task.arm.grap(0)  # 松手
        time.sleep(0.5)
        my_car.task.arm.set(0.266, 0.132)

    if answer_dish == 2 or answer_dish == 4:
        if answer_dish == 2:
            back = -0.24
        if answer_dish == 4:
            back = -0.115
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
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松手
        time.sleep(0.5)
        my_car.task.arm.set(0.03, 0.06)
        my_car.task.arm.set(0.19, 0.06)  # 缩手

        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        # my_car.task.arm.set_arm_angle(-120)  # 手臂里弯
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.26, 0)  # 向下拿
        my_car.task.arm.set(0.26, 0.05)
        my_car.task.arm.switch_side(-1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前

        my_car.set_vel_time(-0.1, 0, 0, 0.55)
        my_car.task.arm.set(0.1, 0.03)  # 压低手臂
        my_car.task.arm.set(0, 0.025)  # 伸手去放物品
        my_car.set_vel_time(-0.1, 0, 0, 0.1)
        my_car.task.arm.grap(0)  # 松手
        time.sleep(0.5)
        my_car.task.arm.set(0.266, 0.03)  # 缩j/

def eject1():
    # finetune_car_position=my_car.get_odometry()
    # print(finetune_car_position)
    # print(finetune_car_position[2]-start_car_position[2])
    # my_car.set_pose_offset([0, 0, (finetune_car_position[2]-start_car_position[2]) ] , 0.2)
    '''
    my_car.lane_dis_offset(0.3, 1.5)


    my_car.lane_dis_offset(0.3, 0.3)
    my_car.set_pose_offset([0, 0, math.pi / 35], 1)

    my_car.task.eject(1)
    time.sleep(0.5)
    # my_car.set_pose_offset([0.7, 0, 0],0.5)
    my_car.lane_dis_offset(0.3, 0.7)

    print("finish camp task!!!!!")
    '''
    my_car.lane_dis_offset(0.3, 1.18)
    my_car.set_pose_offset([0, 0, math.pi / 30], 1)
    my_car.set_pose_offset([0, 0, math.pi / 25], 1)
    my_car.task.eject(1)
    time.sleep(0.5)
    my_car.set_pose_offset([0, 0, -math.pi / 20], 1)
    my_car.set_pose_offset([0,-0.05 , 0], 0.5)
    time.sleep(2)
    
def plants(side):

    sys.stdout.flush()

    # 1. 获取目标列表
    tar_list = my_car.task.planting(side, arm_set=True)
    
    # 2. 定位到最近的目标
    
    pose_dict = my_car.lane_det_location_plant(speed=0.10, targets=tar_list, side=side, dis_out=1.8)
    
    # 3. 根据定位结果确定圆柱体类型并执行操作
    if pose_dict is not False and len(pose_dict) > 0:
        # 获取找到的目标ID
        found_id = list(pose_dict.keys())[0]
        
        # 根据ID确定圆柱体类型
        if found_id == 100:  # cylinder3
            radius = 2
            print("检测到大圆柱体(cylinder3) - 执行浇水操作")
        elif found_id == 80:  # cylinder2
            radius = 1
            print("检测到中圆柱体(cylinder2) - 执行蜂鸣示意")
        elif found_id == 60:  # cylinder1
            radius = 0
            print("检测到小圆柱体(cylinder1) - 执行补光操作")
        else:
            print(f"未知的目标ID: {found_id}")
            radius = None
        
        # 4. 执行对应的操作
        if radius is not None:
            my_car.task.planting(side, radius=radius)
            time.sleep(1)
    
    sys.stdout.flush()


def plant_action3():
    side = 1
    my_car.task.arm.set(0.07, 0)
    my_car.set_pose_offset([0, 0, -math.pi / 30], 0.5)
    my_car.lane_sensor(0.25, value_h=0.2, sides=1)
    
    ##my_car.task.arm.set(0.07, 0)    #从这段开始运行记得加手臂初始化动作
    ##my_car.task.arm.switch_side(1)
    
    my_car.set_pose_offset([0.17, 0, 0], 1)
   
    # side = 1
    plants(side)

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False

    '''
    start_hri()
   # my_car.task.eject(1)
    #my_car.lane_sensor(0.2, value_h=0.2, sides=1)
    
    
    my_car.task.arm.grap(1)
    time.sleep(3)
    my_car.task.arm.grap(0)
    '''
    '''
    my_car.task.arm.set(0.10, 0.02)
    tar = [19, 0, 'egg',0.8597215414047241,  0.028125, 0.47291666666666665, 0.225, 0.2625]  #   0.7483181953430176, 0, 0.2, 0.27, 0.7
    print(f"tar:{tar}")
    # tar=[index, 0, answer, 0.5542138814926147, 0, 0.192, 0.18, 0.375]
    flag, offset = my_car.lane_det_location_vert(
        0.135, tar, side=1, dis_out=1
    )  # 巡航找食材
    
    '''
    '''
    my_car.lane_dis_offset(0.3, 0.1)
    my_car.lane_sensor(0.27, value_h=0.6, sides=-1)
    my_car.set_pose_offset([0, 0, -math.pi / 25], 1)
    my_car.set_pose_offset([-0.05, 0, 0], 1)
    # my_car.set_pose_offset([0, 0, -math.pi / 30], 1)
    
    my_car.task.arm.grap(1)
    '''
    get_food()
    answer2()
    put_food()
    
    a = "tomato"
    b = "egg"
    put_food(a,b)
    
    
    #my_car.task.arm.switch_side(-1)
    #my_car.task.arm.set(0.20, 0.1)
    #plant_action3()
    #eject1()
    
    # get_food()
    
    # my_car.lane_dis_offset(0.3, 1.1)
    #my_car.task.eject(1)
    
    #my_car.task.arm.set_offset(0, -0.1)