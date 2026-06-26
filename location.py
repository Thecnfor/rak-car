#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import threading
import os
import platform
import signal
from camera import Camera
import numpy as np
from vehicle import ArmBase, ScreenShow, Key4Btn, Infrared, LedLight,CarBase, Beep
from simple_pid import PID
import difflib
import cv2, math
from task_func import MyTask
from infer_cs import ClintInterface, Bbox
from ernie_bot import ErnieBotWrap, ActionPrompt, HumAttrPrompt
from tools import CountRecord, get_yaml, IndexWrap
import sys, os

# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__))) 
from log_info import logger
def lane_det_location_multi_object(self, speed, pts_tar=[[0, 70, 'text_det',  0, 0, 0, 0.70, 0.70]], dis_out=0.05, side=1, time_out=2, det='task'):
        end_time = time.time() + time_out
        infer = self.task_det
        loc_pid = get_yaml(self.yaml_path)["location_pid"]
        pid_x = PID(**loc_pid["pid_x"])
        pid_x.output_limits = (-speed, speed)
        pid_y = PID(**loc_pid["pid_y"])
        pid_y.output_limits = (-0.15, 0.15)
        # pid_w = PID(1.0, 0, 0.00, setpoint=0, output_limits=(-0.15, 0.15))

        # 用于相同记录结果的计数类
        x_count = CountRecord(5)
        dis_count = CountRecord(5)
        
        out_x = speed
        out_y = 0
        
        # 此时设置相对初始位置
        # self.set_pos_relative()
        # self.dis_tra_st = self.get_dis_traveled()
        # 获取当前位置
        x_st, y_st, _ = self.get_odometry()
        find_tar = False
        tar = []
        for pt_tar in pts_tar:
            # id, 物体宽度，置信度, 归一化bbox[x_c, y_c, w, h]
            tar_id, tar_width, tar_label, tar_score, tar_bbox = pt_tar[0], pt_tar[1], pt_tar[2], pt_tar[3], pt_tar[4:]
            tar_width *= 0.001
            tar_x, tar_y, tar_dis = self.det2pose(tar_bbox, tar_width)
            tar.append([tar_id, tar_width, tar_x, tar_y, tar_dis])
        # logger.info("tar x:{} dis:{}".format(tar_x, tar_dis))
        tar_id, tar_width, tar_x, tar_y, tar_dis = tar[0]
        pid_x.setpoint = tar_x
        pid_y.setpoint = tar_dis
        tar_index = 0
        flag_location = False
        while True:
            if self._stop_flag:
                return
            if time.time() > end_time:
                logger.info("time out")
                self.set_velocity(0, 0, 0)
                return False
            _pos_x, _pos_y, _pos_omage = self.get_odometry() # 用来计算距离

            if abs(_pos_x-x_st) > dis_out or abs(_pos_y-y_st) > dis_out:
                if not find_tar:
                    logger.info("task location dis out")
                    self.set_velocity(0, 0, 0)
                    return False
            img_side = self.cap_side.read()
            dets_ret = infer(img_side)
            
            # dets_ret = self.mot_hum(img_side)
            # cv2.imshow("side", img_side)
            # cv2.waitKey(1)
            
            # 进行排序，此处排列按照自中心由近及远的顺序
            dets_ret.sort(key=lambda x: (x[4])**2 + (x[5])**2)
            # print(dets_ret)
            # # 找到最近对应的类别，类别存在第一个位置
            # det = self.get_list_by_val(dets_ret, 2, tar_label)
            
            # 如果没有，就重新获取
            if len(dets_ret) > 0:
                det = dets_ret[0]
                # 结果分解
                det_id, obj_id , det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                # if find_tar is False:
                    # tar_index = 0
                    # for tar_pt in tar:
                for index, tar_pt in enumerate(tar):
                    if det_id == tar_pt[0]:
                        tar_index = index
                        tar_id, tar_width, tar_x, tar_y, tar_dis = tar_pt
                        pid_x.setpoint = tar_x
                        pid_y.setpoint = tar_dis
                        find_tar = True
                        # print("find tar", tar_id)
                        break
                        
                if det_id == tar_id:
                    _x, _y, _dis = self.det2pose(det_bbox, tar_width)
                    out_x = pid_x(_x) * side
                    out_y = pid_y(_dis) * side
                    # out_y = pid_y(_dis)
                    # out_y = pid_w(bbox_error[2])
                    # 检测偏差值连续小于阈值时，跳出循环
                    # print(bbox_error)
                    # print("err x:{:.2}, dis:{:.2}, tar x:{:.2}, tar dis:{:.2}".format(_x, _dis, tar_x, tar_dis))
                    flag_x = x_count(abs(_x - tar_x) < 0.01)
                    flag_dis = dis_count(abs(_dis - tar_dis) < 0.01)
                    if flag_x:
                        out_x = 0
                    if flag_dis:
                        out_y = 0
                    if flag_x and flag_dis:
                        logger.info("location{} ok".format(tar_id))
                        # flag_location = True
                        # 停止
                        self.set_velocity(0, 0, 0)
                        return tar_index
                
                # print("error_x:{:.2}, error_y:{:.2}, out_x:{:.2}, out_y:{:2}".format(bbox_error[0], bbox_error[2], out_x, out_y))
            else:
                x_count(False)
                dis_count(False)
            self.set_velocity(out_x, out_y, 0)



def lane_det_location_single_object(self, speed, pt_tar=[0, 70, 'text_det', 0, 0, 0, 0.70, 0.70], dis_out=0.05, side=1, time_out=2, det='task'):
    end_time = time.time() + time_out
    infer = self.task_det
    loc_pid = get_yaml(self.yaml_path)["location_pid"]
    pid_x = PID(**loc_pid["pid_x"])
    pid_x.output_limits = (-speed, speed)
    pid_y = PID(**loc_pid["pid_y"])
    pid_y.output_limits = (-0.15, 0.15)

    # 用于相同记录结果的计数类
    x_count = CountRecord(5)
    dis_count = CountRecord(5)
    
    out_x = speed
    out_y = 0
    
    # 获取当前位置
    x_st, y_st, _ = self.get_odometry()
    tar_id, tar_width, tar_label, tar_score, *tar_bbox = pt_tar
    tar_width *= 0.001
    tar_x, tar_y, tar_dis = self.det2pose(tar_bbox, tar_width)
    pid_x.setpoint = tar_x
    pid_y.setpoint = tar_dis

    while True:
        # time out 和 dis out的检测 不用管
        if self._stop_flag:
            return
        if time.time() > end_time:
            logger.info("time out")
            self.set_velocity(0, 0, 0)
            return False
        _pos_x, _pos_y, _pos_omage = self.get_odometry() 
        if abs(_pos_x - x_st) > dis_out or abs(_pos_y - y_st) > dis_out:
            logger.info("task location dis out")
            self.set_velocity(0, 0, 0)
            return False
        
        img_side = self.cap_side.read()
        dets_ret = infer(img_side)
        
        # 如果检测到有东西了
        if len(dets_ret) > 0:
            # 由近及远排序
            dets_ret.sort(key=lambda x: (x[4])**2 + (x[5])**2)
            det_id, obj_id, det_label, det_score, det_bbox = dets_ret[0][0], dets_ret[0][1], dets_ret[0][2], dets_ret[0][3], dets_ret[0][4:]
            
            if det_id == tar_id:
                # 将图像中的目标转换为机器人坐标系下的位置
                _x, _y, _dis = self.det2pose(det_bbox, tar_width)
                out_x = pid_x(_x) * side
                out_y = pid_y(_dis) * side
                
                flag_x = x_count(abs(_x - tar_x) < 0.01)
                flag_dis = dis_count(abs(_dis - tar_dis) < 0.01)
                
                if flag_x:
                    out_x = 0
                if flag_dis:
                    out_y = 0
                    
                if flag_x and flag_dis:
                    logger.info("location{} ok".format(tar_id))
                    self.set_velocity(0, 0, 0)
                    return True
        else:
            x_count(False)
            dis_count(False)
        self.set_velocity(out_x, out_y, 0)
    
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
    #tar = my_car.task.get_ingredients(side=1, ocr_mode=True, arm_set=True)
    #my_car.lane_sensor(0.3, value_h=0.2, sides=1)
    #my_car.lane_dis_offset(0.3, 0.17)
    #my_car.lane_det_location(0.2, tar, side=1)
    #my_car.set_pose_offset([-0.12, 0, 0])
    
    #tar = my_car.task.pick_ingredients(1, 1, arm_set=True)
    #my_car.lane_det_location(0.2, tar, side=1)

    #my_car.task.pick_ingredients(1, 1)
    #my_car.set_pose_offset([0.115, 0, 0])
    # my_car.task.arm.set(0.5, 0.4)
    # my_car.lane_det_location_single_object(0.2,[12, 0, 'mushroom', 0.5104383230209351, -0.553125, 0.48125, 0.70625, 0.4875],dis_out=5,side=-1)
    
    #my_car.lane_det_location_v1(0.2,[[16, 0, 'cylinder3', 0.7833783626556396, -0.6890625, -0.275, 0.621875, 0.6416666666666667]],dis_out=5,side=1,time_out=10)
    my_car.lane_det_location_v5(0.2,[12, 0, 'mushroom', 0.5104383230209351, -0.553125, 0.48125, 0.70625, 0.4875],dis_out=5,side=-1)