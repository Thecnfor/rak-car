# -*- coding: utf-8 -*-
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
import sys, os
from ernie_bot.base import answer

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


# ??????????
def get_key_by_value(d, value):
    for k, v in d.items():
        if v == value:
            return k
    return None

my_car = MyCar()
my_car.STOP_PARAM = False

my_car.set_pose_offset([0.7, 0, 0], 3)
my_car.task.arm.set_hand_angle(90)
# my_car.task.arm.set(0.1, 0)
# time.sleep(0.2)
'''
side = my_car.get_card_side()
print(side)

if side == -1:
    my_car.task.arm.set(0.07, 0)
    my_car.task.arm.switch_side(1)
    # my_car.task.arm.set_arm_angle(-96)
    my_car.set_pose_offset([0, 0, math.pi / 4 * side], 1)
    time.sleep(0.3)

    my_car.lane_dis_offset(0.2, 0.58)
    my_car.set_pose_offset([0, -0.035, 0])
    my_car.set_pose_offset([0, 0, -math.pi / 35 * side])
    time.sleep(0.3)

    my_car.set_pose_offset([0.28, 0, 0])

else:
    my_car.task.arm.set(0.18, 0)
    my_car.task.arm.switch_side(-1)
    # my_car.task.arm.set_arm_angle(90)
    my_car.set_pose_offset([0, 0, math.pi / 4 * side], 1)
    time.sleep(0.3)

    my_car.lane_dis_offset(0.2, 0.58)
    my_car.set_pose_offset([0, 0.035, 0])
    my_car.set_pose_offset([0, 0, -math.pi / 12 * side])
    time.sleep(0.3)
    my_car.set_pose_offset([0.28, 0, 0])

    # my_car.set_pos_offset([0, 0, math.pi / 5.2 * -1], 1)

# my_car.task.arm.set(0.07, 0)
# my_car.task.arm.switch_side(1)
# my_car.task.arm.switch_side(1)
# my_car.task.arm.set_hand_angle(90)
# time.sleep(1)
# side=-1
cylinder_id = 1
pts = my_car.task.pick_up_cylinder(cylinder_id, side, True)

pos_start = np.array(my_car.get_odometry())
pos_zone = pos_start.copy()
pos_zone[0] += 0.36

pose_dict = my_car.lane_det_location_v8_multi(0.1, pts, side=side * -1, dis_out=2)
print(pose_dict)

for i in range(3):
    if i == 0:
        run_dis = my_car.calculation_dis(np.array(pose_dict[100]), np.array(pos_zone))
        my_car.set_pose(pose_dict[100])
    if i == 1:
        run_dis = my_car.calculation_dis(np.array(pose_dict[80]), np.array(pos_zone))
        my_car.set_pose(pose_dict[80])
    if i == 2:
        run_dis = my_car.calculation_dis(np.array(pose_dict[60]), np.array(pos_zone))
        my_car.set_pose(pose_dict[60])

    my_car.task.pick_up_cylinder(i, side)
    my_car.set_pose_offset([run_dis, 0, 0])
    my_car.task.put_down_cylinder(i, side)
    my_car.set_pose_offset([-0.1, 0, 0])

my_car.lane_sensor(0.2, value_h=0.3, sides=1, stop=True)
my_car.task.arm.set(0.1, 0.18)
my_car.task.arm.switch_side(1)
my_car.task.arm.set(0.1, 0)
'''

my_car.task.arm.switch_side(-1)  # 手臂向右
my_car.task.arm.set(0.265, 0)  # 手臂向后退到能看到文字的位置
tar = [9, 0, 'text_det', 0, 0.0390625, 0.7916666666666666, 0.334375, 0.39166666666666666]
# time.sleep(1)
my_car.set_velocity(0.3, 0, 0)
time.sleep(0.5)
my_car.lane_det_location_v4(0.2, tar, side=-1, dis_out=2)  # 巡航找文字
try:
    text = my_car.get_ocr_list()[0]  # 获取文字
    foods = answer.ask1(text)
    the_food1 = foods['answer']
    print(the_food1)
except Exception as e:
    print(e)
    the_food1 = 'egg'
my_car.set_pose_offset([-0.25, 0, 0])  # 后退一步
my_car.task.arm.set(0.135, 0)  # 调整手臂定位食材
index = get_key_by_value(index_form, the_food1)
tar = [index, 0, the_food1, 0.9151089787483215, 0, 0.2, 0.32, 0.7]
my_car.lane_det_location_vert(0.15, tar, side=-1, dis_out=2)
img_side = my_car.cap_side.read()
dets_ret = my_car.task_det(img_side)
print("object is", dets_ret)

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


    # my_car.task.arm.set_hand_angle(90)#掌心向下
    # time.sleep(0.5)
    # my_car.task.arm.set(0.265,0.03)#手臂向下放食材
    # my_car.task.arm.grap(0)#松

my_car.set_pose_offset([-0.5, 0, 0])  # 后退取第二个食材




my_car.task.arm.switch_side(1)  # 手臂向左
my_car.task.arm.grap(0)
my_car.task.arm.set(0, 0)  # 达到摄像头能看到文字的位置
tar = [9, 0, 'text_det', 0.9307849407196045, -0.0546875, 0.7645833333333333, 0.33, 0.30416666666666664]
time.sleep(1)
my_car.lane_det_location_v4(0.2, tar, side=1, dis_out=2)  # 巡航找文本
try:
    text = my_car.get_ocr_list()[0]  # 获取文字
    foods = answer.ask1(text)
    the_food2 = foods['answer']
    print(the_food2)
except Exception as e:
    print(e)
    the_food2 = 'tomato'

my_car.set_pose_offset([-0.25, 0, 0])  # 后退
my_car.task.arm.set(0.15, 0)  # 调整手臂定位食材

index = get_key_by_value(index_form, the_food2)
tar = [index, 0, the_food2, 0.7483181953430176, 0, 0.2, 0.35, 0.7]
print(f"tar:{tar}")
# tar=[index, 0, answer, 0.5542138814926147, 0, 0.192, 0.18, 0.375]
my_car.lane_det_location_vert(0.135, tar, side=1, dis_out=2)
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
            
            
            
            
            
            
            
            
            
    
    