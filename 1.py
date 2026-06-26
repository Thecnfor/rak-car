import time
import threading
import os
import numpy as np
from task_func import MyTask, task_reset
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

    # hanoi###################################################################3
    #my_car.task.arm.set_hand_angle(90)
    
    side =-1
    my_car.task.arm.set(0.10, 0)
    
    pos_start = np.array(my_car.get_odometry())
    my_car.set_pose_offset([-0.35, 0, 0])
    # my_car.set_pose_offset([0.37, 0, 0], 3)
    pose_dict = {}
    pose_last = None
    # my_car.set_pose_offset([-0.37, 0, 0], 3)
    cylinder_id = 1
    pts = my_car.task.pick_up_cylinder(cylinder_id,side,True)
    for i in range(3):
        my_car.task.arm.set(0.10, 0)
        index = my_car.lane_det_location_v8(0.2, pts[i], side=side * -1, dis_out=2)
        my_car.beep()
        run_dis = my_car.calculation_dis(pos_start, np.array(my_car.get_odometry()))
        if index== 100:
            my_car.task.pick_up_cylinder(0,side)
            my_car.set_pose_offset([run_dis, 0, 0], 1)
            my_car.task.put_down_cylinder(0,side)
        if index== 80:
            my_car.task.pick_up_cylinder(1,side)
            my_car.set_pose_offset([run_dis, 0, 0], 1)
            my_car.task.put_down_cylinder(1,side)
        if index== 60:
            my_car.task.pick_up_cylinder(2,side)
            my_car.set_pose_offset([run_dis, 0, 0], 1)
            my_car.task.put_down_cylinder(2,side)
        if i < 2:
            my_car.set_pose_offset([-0.35, 0, 0])

