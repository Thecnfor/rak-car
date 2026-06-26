#!/usr/bin/python
# -*- coding: utf-8 -*-
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
# 添加上本文件对应目录
sys.path.append(os.path.abspath(os.path.dirname(__file__))) 

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    # my_car.task.reset()
    

    def hanoi_tower_func():
        my_car.lane_dis_offset(0.3, 0.5)
        # print(my_car.get_odometry())
        my_car.set_pose_offset([0.3,0,0], 1)
        # print(my_car.get_odometry())
        det_side = my_car.lane_det_dis2pt(0.2, 0.19)
        side = my_car.get_card_side()
        # print(side)
        # 调整检测方向
        my_car.task.arm.switch_side(side*-1)
        
        # 调整车子朝向
        my_car.set_pose_offset([0, 0, math.pi/4*side], 1)
        
        # 第一个要抓取的圆柱
        cylinder_id = 1
        # 调整抓手位置，获取要抓取的圆柱信息
        pts = my_car.task.pick_up_cylinder(cylinder_id, True)
        # 走一段距离
        my_car.lane_dis_offset(0.3,0.66)
        
        # 第二次感应到侧面位置
        # my_car.lane_sensor(0.2, value_h=0.3, sides=side*-1)
        my_car.lane_sensor(0.2, value_h=0.3, sides=side*-1, stop=True)
        # return
        # 记录此时的位置
        pose_dict = {}
        pose_last = None
        for i in range(3):
            # 根据给定信息定位目标
            index = my_car.lane_det_location(0.2, pts, side=side*-1)
            my_car.beep()
            pose_dict[index] = my_car.get_odometry().copy()
            if i == 2:
                pose_last = my_car.get_odometry().copy()
            # print(index)
            # pose_list.append([index, my_car.get_odometry().copy()])
            
            if i < 2:
                my_car.set_pose_offset([0.08, 0, 0])
                my_car.beep()
            
        # print(pose_dict)
        # 根据识别到的位置调整方向位置
        # angle = math.atan((pose_dict[2][1] - pose_dict[0][1]) / (pose_dict[2][0] - pose_dict[0][0]))
        # print(angle)
        # my_car.set_pose_offset([0, 0, -angle])
        # 重新定位最后一个圆柱
        # my_car.lane_det_location(0.2, pts, side=side*-1)
        angle_det = my_car.get_odometry()[2]
        # 计算目的地终点坐标
        pose_end = [0, 0, angle_det]
        pose_end[0] = pose_last[0] + 0.12*math.cos(angle_det)
        pose_end[1] = pose_last[1] + 0.12*math.sin(angle_det)
        # print(det)
        # 调整到目的地
        # my_car.set_pose(det)
        for i in range(3):
            det = pose_dict[i]
            det[2] = angle_det
            my_car.set_pose(det)
            # my_car.lane_det_location(0.2, pts, side=side*-1)
            my_car.task.pick_up_cylinder(i)
            my_car.set_pose(pose_end)
            my_car.task.put_down_cylinder(i)
        # return

    def bmi_cal():
        # my_car.lane_dis_offset(0.3, 0.8)
        # 准备手臂位置
        pts = my_car.task.bmi_set(arm_set=True)
        # 巡航到bmi识别附件
        my_car.lane_sensor(0.3, value_h=0.30, sides=1)
        # 推开bmi识别标签
        # my_car.lane_dis_offset(0.3, 0.5)
        my_car.set_pose_offset([0.03, -0.03, 0])
        my_car.set_pose_offset([0.0, 0.13, 0])
        my_car.set_pose_offset([0.0, -0.08, 0])
        my_car.set_pose_offset([0.13, 0, 0])
        # 调整bmi识别位置
        my_car.lane_det_location(0.2, pts, side=1)
        # 识别相关文字
        # text = my_car.get_ocr()
        time.sleep(0.3)
        out = 2
        my_car.task.bmi_set(out)
        time.sleep(0.5)
        # 调整位置准备放置球
        # my_car.lane_dis_offset(0.21, 0.19)
        # my_car.set_pose_offset([0, 0.05, 0], 0.7)
        # my_car.task.put_down_ball()
    

    def camp_fun():
        angle_offset = -math.pi/2*0.82
        # dis_angle = -math.pi/2*0.3
        # dis = 1.
        dis_x = 1.36
        dis_y = -0.87
        # print(dis_x, dis_y)
        angle_now = my_car.get_odometry()[2]
        x_offset = dis_x*math.cos(angle_now) - dis_y*math.sin(angle_now)
        y_offset = dis_y*math.cos(angle_now) + dis_x*math.sin(angle_now)
        angle_tar = angle_now - math.pi*2 + angle_offset
        pose = my_car.get_odometry().copy()
        pose[0] = pose[0] + x_offset
        pose[1] = pose[1] + y_offset
        pose[2] = angle_tar
        # print(pose)
        # return
    
        my_car.lane_sensor(0.3, value_h=1, sides=-1)
        # time.sleep(25)
        # my_car.lane_sensor()
        my_car.lane_dis_offset(0.3, 0.5)
        my_car.set_vel_time(0.3, 0, -0.5, 1.8)
        my_car.lane_dis_offset(0.3, 2.95)
        
        my_car.set_pose(pose, vel=[0.2, 0.2, math.pi/3])

        # my_car.
        # my_car.set_vel_time(0.3, 0, -0.1, 1)
        # my_car.move_advance([0.3, 0, 0], value_l=1, sides=-1)
        # my_car.lane_time(0, 1)
        # my_car.move_advance([0.3, 0, -0.2], value_l=1, sides=-1)
        # my_car.move_distance([0.3, 0, 0], 0.25)
        # my_car.move_advance([0.3, 0, 0], value_l=1, sides=-1)
        # my_car.move_advance([0.3, 0, 0], value_l=0.5, sides=-1)


    def send_fun():
        # my_car.move_advance([0.3, 0, 0], value_l=1, sides=-1)
        # my_car.move_distance([0.3, 0, -0.1], 0.25)
        # my_car.lane_sensor(0.3, value_l=1.1, sides=-1)
        my_car.lane_dis_offset(0.3, 1.54)
        my_car.task.eject(1)
        

    # 获取食材
    def task_ingredients():
        tar = my_car.task.get_ingredients(side=1,ocr_mode=True, arm_set=True)
        my_car.lane_sensor(0.3, value_h=0.2, sides=1)
        my_car.lane_dis_offset(0.3, 0.17)
        my_car.lane_det_location(0.2, tar, side=1)
        my_car.set_pose_offset([-0.12, 0, 0])
        tar = my_car.task.pick_ingredients(1, 1, arm_set=True)
        my_car.lane_det_location(0.2, tar, side=1)
        my_car.task.pick_ingredients(1, 1)
        
        my_car.set_pose_offset([0.115, 0, 0])
        my_car.task.arm.switch_side(-1)
        tar = my_car.task.pick_ingredients(2, 2, arm_set=True)
        my_car.lane_det_location(0.2, tar, side=-1)
        my_car.task.pick_ingredients(2, 2)


    def task_answer():
        my_car.lane_sensor(0.3, value_h=0.3, sides=1)
        my_car.task.arm.switch_side(1)
        my_car.move_distance([0.3, 0, 0], 0.22)
        tar = my_car.task.get_answer(arm_set=True)
        my_car.lane_det_location(0.2, tar, side=1)
        text = my_car.get_ocr()
        print(text)
        out = 0
        pose_tar_offset = [0.08*out-0.12, 0, 0]
        my_car.set_pose_offset(pose_tar_offset)
        my_car.task.get_answer()
        # my_car.move_distance([0.3, 0, 0], 0.24)

    def task_fun2():
        # 巡航到投掷任务点2
        my_car.lane_sensor(0.3, value_h=0.5, sides=-1)
        # 调整方向
        my_car.lane_time(0, 1)
        my_car.set_pose_offset([-0.05, 0, 0])
        my_car.task.eject(2)
        # my_car.task.arm.switch_side(-1)
        # my_car.move_distance([0.3, 0, 0], 0.24)
        # tar = my_car.task.get_answer(arm_set=True)

    def task_food():
        # my_car.lane_sensor(0.3, value_h=0.5, sides=-1)
        
        tar = my_car.task.set_food(arm_set=True)
        my_car.task.arm.switch_side(-1)
        my_car.set_pose_offset([0.18, 0, 0])
        my_car.lane_time(0, 1)
        my_car.lane_det_location(0.2, tar, side=-1) 
        my_car.set_pose_offset([0.075, 0, 0])
        my_car.task.set_food(1, row=2)
        my_car.set_pose_offset([0.045, 0, 0])
        my_car.task.set_food(2, row=2)
        # my_car.lane_dis_offset(0.3, 0.17)
        # my_car.lane_det_location(0.2, tar, side=1)
        # my_car.task.pick_ingredients(1, 1)

    def task_help():
        my_car.lane_sensor(0.3, value_h=0.5, sides=1)
        my_car.task.help_peo(arm_set=True)
        my_car.set_pose_offset([0.07, 0.14, 0])
        my_car.set_pose_offset([-0.1, 0.15, 0])
        my_car.set_pose_offset([0, -0.3, 0])
        my_car.set_pose_offset([0.7, 0, 0], vel=[0.3, 0.3, 0])
        # my_car.set_pose_offset([0.1, -0.1, 0])

        # my_car.move_advance([0.2, 0, 0], value_h=0.5, sides=1, dis_out=0.05)
        # my_car.task.help_peo()

    def go_start():
        my_car.lane_sensor(0.3, value_l=0.4, sides=-1)
        my_car.set_pose_offset([0.85, 0, 0], 2.8)
        my_car.set_pose_offset([0.45, -0.09, -0.6], 2.5)
        # 前移
        # my_car.set_pose_offset([0.3, 0, 0], 2.5)
        # my_car.set_pose_offset([0.45, -0.09, -0.6], 2.5)
        # 离开道路到修整营地
        # my_car.set_pose_offset([0.15, -0.4, 0], 2)
        # 做任务
        # my_car.do_action_list(actions_map)
    def car_move():
        my_car.set_pose([0.20, 0,0], 1)

    my_car.beep()
    time.sleep(0.2)
    functions = [hanoi_tower_func, bmi_cal, camp_fun, send_fun, task_ingredients, task_answer, task_fun2, task_food, task_help]
    my_car.manage(functions, 9)

