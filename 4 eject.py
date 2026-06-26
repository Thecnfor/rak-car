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

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    # my_car.task.reset()

####################################################################################################


    #my_car.lane_dis_offset(0.3, 2.1)
    #my_car.set_pose_offset([0, 0, 0.15])
    my_car.task.eject(1)