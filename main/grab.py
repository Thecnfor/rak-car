# -*- coding: utf-8 -*-
import sys
sys.path.append("/home/jetson/workspace/vehicle_wbt/")

import random
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
from ernie_bot.base import answer_wenxin
from ernie_bot.base import answer
from ernie_bot import Weather

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
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

# 反向查找函数
def get_key_by_value(d, value):
    for k, v in d.items():
        if v == value:
            return k
    return None
    
def ocr():
  while True:
    text = my_car.get_ocr_list_plus()
    try:
      text = text[0]
      print(f'ocr content: {text}')
      foods = answer.ask1(text)
      print("\n食物是:", foods)
    except:
    	print("未识别到文字")

def weather():
    weather_api = Weather()
    city = "三水"  
    
    try:
        weather = weather_api.get_weather_by_city(city)
        answer = weather_api.get_weather_num(weather['weather'])    
        my_car.task.weather_set(answer)
        print(f"当前天气: {weather['weather']}，温度: {weather['temperature']}℃")
    except Exception as e:
        print(f"获取天气信息失败: {e}")
        
        



if __name__ == "__main__":
  my_car = MyCar()
  # weather()
  
  # ocr()
  # my_car.task.eject(1)
    
############################################################################################
  
  my_car.task.arm.switch_side(1)
  my_car.task.arm.set(0, 0)  # 初始化手臂
  
  my_car.lane_sensor(0.2, value_h=0.3, sides=-1)  
  my_car.task.arm.switch_side(-1)
  my_car.task.arm.grap(1)
  my_car.task.arm.set(0, 0.13)
  
  my_car.set_pose_offset([0.22, 0, 0], 1.2)
  my_car.set_pose_offset([0, -0.02, 0], 0.2)
  
  my_car.task.arm.set(0, 0.08)
  my_car.task.arm.set(0, 0.13)
  
  my_car.task.arm.grap(1)
  my_car.task.arm.switch_side(-1)
  my_car.task.arm.set(0.26, 0.13)
  my_car.task.arm.set(0.26, 0)
  my_car.task.arm.grap(0)
  my_car.task.arm.set(0.26, 0.13)
  
  #my_car.set_pose_offset([0, 0.15, 0], 1)
  
  # my_car.task.arm.switch_side(1)
  # my_car.task.arm.set(0.07, 0)    #0.095

  '''
  while True:
    img_side = my_car.cap_side.read()  
    dets_ret = my_car.task_det(img_side) 
    
    #image = my_car.cap_front.read()
    #dets_ret = my_car.front_det(image)
    
    dets_ret.sort(key=lambda x: (x[4])**2 + (x[5])**2) 
    print (dets_ret)    
    time.sleep(0.1)
  '''
    
