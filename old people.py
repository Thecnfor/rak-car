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

sys.path.append(os.path.abspath(os.path.dirname(__file__)))

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    # my_car.task.reset()
    
    #my_car.task.arm.set(0,0)
    
############################################################################################

    my_car.lane_sensor(0.2, value_h=0.3, sides=1)
    
    time.sleep(0.5)
    my_car.set_pose_offset([0, 0.3, 0], 0.5)
    time.sleep(0.5)
    my_car.set_pose_offset([0, -0.2, 0], 2)
    time.sleep(0.5)
    my_car.lane_dis_offset(0.3, 0.7)