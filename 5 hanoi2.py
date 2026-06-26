
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
    
    my_car.task.arm.set(0,0)
    
####################################################################################################

#hanoi###################################################################3
   
    my_car.set_pose_offset([0.7, 0, 0], 3)  
    
    time.sleep(0.2)

    side = my_car.get_card_side()
    print(side)    
    
 
    if side == -1:
      #my_car.task.arm.switch_side(1)
      my_car.task.arm.set_arm_angle(-96)
      my_car.set_pose_offset([0, 0, math.pi / 4 * side], 1)
      time.sleep(0.5)

      my_car.lane_dis_offset(0.2, 0.58) 
      my_car.set_pose_offset([0, -0.035, 0], 1) 

      my_car.set_pose_offset([0, 0, -math.pi / 11 * side], 1)
      time.sleep(0.3)
      
    else:
      my_car.task.arm.set_arm_angle(90)

      my_car.set_pose_offset([0, 0, math.pi / 3 * side], 1)
      time.sleep(0.5)
      my_car.lane_dis_offset(0.2, 0.58) 
      my_car.set_pose_offset([0, 0.025, 0], 1)  
  
      my_car.set_pose_offset([0, 0, -math.pi / 11 * side], 1)  
      time.sleep(0.3)
      #my_car.set_pos_offset([0, 0, math.pi / 5.2 * -1], 1) """

    
    my_car.set_pose_offset([0.65, 0, 0], 3)
    
    my_car.task.arm.set(0.1,0.07)
    

        
        
