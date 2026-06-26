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
import qqq


if __name__ == "__main__":
    # 每次调用 front() 都是新的实例，运行完就销毁
    my_car = MyCar()
    my_car.STOP_PARAM = False
    #hanoi()
    #bmi()
    #camp()
    qqq.eject1()
    a,b=qqq.get_food()
    qqq.answer1()
    qqq.eject2()
    qqq.put_food(a,b)
    qqq.old_people()
    #my_car.set_vel_time(-0.3, 0 , 0, 1.4)
