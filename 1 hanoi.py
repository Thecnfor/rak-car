
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
    side = -1
    my_car.task.arm.set(0.13, 0.01)
    # my_car.set_pose_offset([0.37, 0, 0], 3)
    #pose_dict = {}
    #pose_last = None
    # my_car.set_pose_offset([-0.37, 0, 0], 3)
    cylinder_id = 1
    pts = my_car.task.pick_up_cylinder(cylinder_id, True)
    for i in range(3):
        index = my_car.lane_det_location_v6(0.2, pts[i], side=side * -1, dis_out=2)
        #if index is not 
        my_car.beep()
        pose_dict[index] = my_car.get_odometry().copy()
        if i == 2:
            pose_last = my_car.get_odometry().copy()
        # print(index)
        # pose_list.append([index, my_car.get_odometry().copy()])'''
    
        if i < 2:
            my_car.set_pose_offset([0.08, 0, 0])
            # my_car.beep()
    '''angle_det = my_car.get_odometry()[2]

    pose_end = [0, 0, angle_det]
    pose_end[0] = pose_last[0] + 0.12 * math.cos(angle_det)
    pose_end[1] = pose_last[1] + 0.12 * math.sin(angle_det)
    for i in range(3):
    det = pose_dict[i]
    det[2] = angle_det
    my_car.set_pose(det)
    # my_car.lane_det_location(0.2, pts, side=side*-1)
    my_car.task.pick_up_cylinder(i)
    my_car.set_pose(pose_end)
    my_car.task.put_down_cylinder(i)'''
    