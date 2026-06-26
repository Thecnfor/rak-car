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
    
####################################################################################################
    """my_car.task.arm.set(0.05,0.05)
    tar = my_car.task.pick_ingredients(2, 1, arm_set=True)
    print (tar)
    my_car.lane_det_location(0.2, tar, side=1)
    my_car.task.pick_ingredients(2, 1)"""
    
    """img_side = my_car.cap_side.read()
    dets_ret = my_car.task_det(img_side)  
    print (dets_ret)"""

    '''my_car.task.arm.set(my_car.task.arm.horiz_mid, 0)
    

    tar = [10, 0, 'cauliflower', 0.9247666001319885, 0.115625, 0.25, 0.20625, 0.57]
    success = my_car.lane_det_location_v5(0.2, tar, side=1, dis_out=0.5)
    
    if not success:
        logger.warning("First attempt failed, backing up and retrying")
        

        my_car.set_pose_offset([-0.5, 0, 0]) 
        
  
        my_car.task.arm.set(my_car.task.arm.horiz_mid, 0.1)
        
     
        success = my_car.lane_det_location_v9(0.2, tar, side=1, dis_out=0.5)
        
        if not success:
            logger.error("Second attempt also failed")
    
        else:
            logger.info("Second attempt succeeded")
    else:
        logger.info("First attempt succeeded")'''

    my_car.task.arm.set(0.135, 0)
