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


if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    my_car.task.arm.set(0.15,0)
    #text=my_car.get_ocr()
    
    text=my_car.get_ocr_list()
    
    #img = my_car.cap_side.read()
    #text = my_car.ocr_rec(img)
    
    
    print(text)