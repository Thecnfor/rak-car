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
from ernie_bot.base import answer

def calculate_bmi(weight_kg, height_m):
    bmi = weight_kg / (height_m ** 2)
    if bmi < 18.5:
        return 1  
    elif 18.5 <= bmi <= 24:
        return 2  
    elif 24 < bmi <= 28:
        return 3
    else:
        return 4 

def extract_height_weight(text):
    pattern = r"(\d+(?:\.\d+)?)\s*cm.*?(\d+(?:\.\d+)?)\s*kg"
    match = re.search(pattern, text)
    if match:
        return float(match.group(1)), float(match.group(2))
    else:
        return None, None

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    # my_car.task.reset()
    
    #my_car.task.arm.set(0,0)
    
####################################################################################################
    
    #my_car.task.arm.set(0,0)
    #my_car.task.arm.set_offset(0, 0-0.13)
    #my_car.lane_dis_offset(0.2, 15)
    my_car.task.arm.switch_side(1)
    my_car.task.arm.set(0.13,0)
    my_car.set_pose_offset([-0.2, 0, 0], 1) 
    my_car.lane_dis_offset(0.3, 1)
    my_car.lane_sensor(0.2, value_h=0.2, sides=1)
    my_car.set_pose_offset([0, 0, math.pi/17])
    my_car.set_pose_offset([0, -0.05, 0], 2)
    my_car.set_pose_offset([0.15, 0, 0]) 

    
    #t = threading.Thread(target=task)
    #t.start()
    #t.join(timeout=3)
    my_car.set_pose_offset([0, 0.085, 0], 2)
    my_car.set_pose_offset([0, -0.08, 0], 2)
    my_car.set_pose_offset([0.1, 0, 0], 1) 
    try:
      text = my_car.get_ocr_list()[0]
      print(text)
      #height, weight = extract_height_weight(text)
      #out = calculate_bmi(height, weight)
        # text=my_car.get_ocr_list()[0] #获取文字
      bmi_degree=answer.ask3(text)
      out=bmi_degree['answer']
      out=int(out)
      my_car.task.bmi_set(out)
    except Exception as e:
      print(f"发生错误: {e}")

      
    