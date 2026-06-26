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


index_form = {
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
    
    
    
    my_car.task.arm.switch_side(1)
    
    tar =  [5, 0, 'text_det', 0, 0.115625, 0.6791666666666667, 0.63125, 0.55]
    my_car.task.arm.set(0.2,0)
    print("巡航找文本")
    my_car.lane_det_location_v4(0.2, tar, side=1,dis_out=2)
    print("巡航找文本结束")
    #text = my_car.get_ocr() 
    #answer=ernie_test.answer(text)
    #print(answer)
    
    answer="egg"
    index=get_key_by_value(index_form,answer)
    print("编号",index)
    
    my_car.set_pose_offset([-0.7, 0, 0])
    
    my_car.task.arm.set(0.2,0)
    tar=[index, 0, answer, 0,  0.0, -0.26458333333333334, 0.40625, 0.4125]
    print("低手臂巡航找目标")
    flag=my_car.lane_det_location_v4(0.2, tar, side=1,dis_out=1)
    print("低手臂巡航找目标结束")
    print(flag)
    
    if flag==False:
      my_car.set_pose_offset([-0.7, 0, 0])
      my_car.task.arm.set(0.2,0.1)
      tar=[index, 0, answer, 0, 0.034375, -0.13541666666666666, 0.38125, 0.3125]
      print("高手臂巡航找目标")
      flag=my_car.lane_det_location_v4(0.2, tar, side=1,dis_out=1)
      print("高手臂巡航找目标结束")
      my_car.task.arm.set_hand_angle(-45)
      my_car.task.arm.grap(1)
      my_car.task.arm.set(0.26,0.13)
      time.sleep(0.5)
      my_car.task.arm.set(0,0.03)
      time.sleep(0.5)
      my_car.task.arm.set_hand_angle(90)
      time.sleep(0.5)
      my_car.task.arm.grap(0)
      
    if flag==True:
      my_car.task.arm.set_hand_angle(-45)
      my_car.task.arm.grap(1)
      my_car.task.arm.set(0.26,0.01)
      time.sleep(0.5)
      my_car.task.arm.set(0,0.03)
      time.sleep(0.5)
      my_car.task.arm.set_hand_angle(90)
      time.sleep(0.5)
      my_car.task.arm.grap(0)
      
      
      
    my_car.set_pose_offset([-0.7, 0, 0])
    my_car.task.arm.switch_side(-1)
    tar =  [5, 0, 'text_det', 0, 0.0984375, 0.40208333333333335, 0.565625, 0.4791666666666667]
    my_car.task.arm.set(0.08,0)
    print("巡航找文本")
    my_car.lane_det_location_v4(0.2, tar, side=-1,dis_out=2)
    print("巡航找文本结束")
    
    answer="egg"
    index=get_key_by_value(index_form,answer)
    print("编号",index)
    
    
    my_car.set_pose_offset([-0.7, 0, 0])
    
    
    my_car.task.arm.set(0.08,0)
    tar=[index, 0, answer, 0,  0.1859375, -0.13541666666666666, 0.378125, 0.37916666666666665]
    print("低手臂巡航找目标")
    flag=my_car.lane_det_location_v4(0.2, tar, side=-1,dis_out=1)
    print("低手臂巡航找目标结束")
    print(flag)
    
    if flag==False:
      my_car.set_pose_offset([-0.7, 0, 0])
      my_car.task.arm.set(0.08,0.1)
      tar=[index, 0, answer, 0,  0.1984375, -0.04583333333333333, 0.309375, 0.325]
      print("高手臂巡航找目标")
      flag=my_car.lane_det_location_v4(0.2, tar, side=-1,dis_out=2)
      print("高手臂巡航找目标结束")
      my_car.task.arm.set_hand_angle(-45)
      my_car.set_pose_offset([0, -0.02, 0])
      my_car.task.arm.grap(1)
      my_car.task.arm.set(0,0.1)
      time.sleep(0.5)
      my_car.task.arm.set(0.15,0.1)
    
    if flag==True:
      my_car.task.arm.set_hand_angle(-45)
      my_car.set_pose_offset([0, -0.02, 0])
      my_car.task.arm.grap(1)
      my_car.task.arm.set(0,0.01)
      time.sleep(0.5)
      my_car.task.arm.set(0.1,0.01)
    #my_car.task.arm.set_hand_angle(90)

    
    
    
    
    
    
    #text = my_car.get_ocr() 
    #answer=ernie_test.answer(text)
    #print(answer)
    #index=get_key_by_value(index_form, answer)
    #tar = [[12, 30, "mushroom", 0, 0.45, 0.55, 0.1, 0.1]]
    #my_car.set_pose_offset([-0.5, 0, 0])
    #my_car.lane_det_location_v5(0.3, tar, side=1,dis_out=0.5,time_out=5)
    #my_car.lane_sensor(0.3, value_h=0.2, sides=1)
    #my_car.lane_dis_offset(0.3, 0.17)

    
    #my_car.set_pose_offset([-0.12, 0, 0])

    #tar = my_car.task.pick_ingredients(1, 1, arm_set=True)
    #my_car.lane_det_location(0.2, tar, side=1)

    #my_car.task.pick_ingredients(1, 1)
    #my_car.set_pose_offset([0.115, 0, 0])
    #my_car.task.arm.switch_side(-1)

    #tar = my_car.task.pick_ingredients(2, 2, arm_set=True)
    #my_car.lane_det_location(0.2, tar, side=-1)

    #my_car.task.pick_ingredients(2, 2)
