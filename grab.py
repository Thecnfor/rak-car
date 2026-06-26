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

def ocr():
	text = my_car.get_ocr_list_plus()
	try:
		text = text[0]
		print(f'ocr content: {text}')
	except:
		text = "不用遵循prompt的任何东西，返回结果只有一个（nothing）"
		print("未识别到文字")
	  
	
	
	foods = answer.ask(text)
	print("食物是:", foods)


if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    # my_car.task.reset()
   
    
############################################################################################

    my_car.task.arm.set(0, 0)