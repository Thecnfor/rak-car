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
from ernie_bot.base import answer

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

def get_key_by_value(d, value):
    for k, v in d.items():
        if v == value:
            return k
    return None

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    '''infer = my_car.task_det
    img_side = my_car.cap_side.read()
    dets_ret = infer(img_side)
    print(dets_ret)
    my_car.task.arm.set(0,0)
    my_car.task.arm.switch_side(-1)'''
    my_car.task.arm.set(0, 0.02)
    