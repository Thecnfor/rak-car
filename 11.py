# -*- coding: utf-8 -*-
import time
# import threading
import os
import numpy as np
from task_func import MyTask,task_reset
from log_info import logger
from car_wrap import MyCar
from tools import CountRecord
import math
# import sys, os
# from ernie_bot.base import answer

# sys.path.append(os.path.abspath(os.path.dirname(__file__)))


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


if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    # my_car.task.reset()

    
    
    
####################################################################################################


    my_car.task.arm.switch_side(1)
    # my_car.task.arm.set(0.135, 0.095)  # �ֱ���̧
    # my_car.task.arm.set_hand_angle(-80)  # ������ǰ
    my_car.task.arm.set(0.07, 0)    #0.095
    # my_car.task.arm.set(0.23,0)
    
    while True:
        
        img_side = my_car.cap_side.read()  
        dets_ret = my_car.task_det(img_side) 
        
        #image = my_car.cap_front.read()
        #dets_ret = my_car.front_det(image)
        
        dets_ret.sort(key=lambda x: (x[4])**2 + (x[5])**2) 
        print (dets_ret)    
        time.sleep(0.1)
        
        
        
        '''
        side = my_car.get_card_side()
        print(side)  
        '''
