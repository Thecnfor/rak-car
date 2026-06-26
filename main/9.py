import time
import threading
import os
import numpy as np
from log_info import logger
from car_wrap import MyCar
from tools import CountRecord
import math
import sys, os


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
    tar = [index, 0, the_food1, 0.9151089787483215, 0, 0.2, 0.33, 0.7]
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
        my_car.task.arm.set(0.135, 0.099)  # 手臂上抬
        my_car.task.arm.set_hand_angle(-80)  # 掌心向前
        my_car.task.arm.set(0, 0.099)  # 向前抓
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
    tar = [index, 0, the_food2, 0.7483181953430176, 0, 0.2, 0.27, 0.7]
    print(f"tar:{tar}")
    # tar=[index, 0, answer, 0.5542138814926147, 0, 0.192, 0.18, 0.375]
    flag, offset = my_car.lane_det_location_vert(
        0.135, tar, side=1, dis_out=1
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

    if y < 0.3 or y == 0.3:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.1, 0.095)  # 手臂上抬
        my_car.task.arm.set_hand_angle(-80)  # 掌心向前
        my_car.task.arm.set(0.27, 0.1)  # 向前抓
        time.sleep(0.5)
        my_car.task.arm.set(0, 0.1)  # 往后缩
        # time.sleep(0.5)
        # my_car.task.arm.set_hand_angle(90)#掌心向下
        # time.sleep(0.5)

        # my_car.task.arm.set(0,0.03)#手臂向下放食材
        # my_car.task.arm.grap(0)#松手

    if y > 0.3:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.1, 0.015)  # 手臂下降
        my_car.task.arm.set_hand_angle(-80)  # 掌心向前
        my_car.task.arm.set(0.08, 0.1)
        my_car.task.arm.set(0.18, 0.01)  # 向前抓
        my_car.task.arm.set(0.27, 0.01)
        time.sleep(0.5)
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
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    get_food()