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
    
    
    #巡航找文字
    my_car.task.arm.switch_side(-1)
    
    my_car.task.arm.set(0.15,0.01) #摄像头看得到两个文字的高度
    
    tar=[9, 0, 'text_det', 0.9494228959083557, 0.075, -0.2520833333333333, 0.4375, 0.4875]
    
    my_car.lane_det_location_v4(0.2, tar, side=-1,dis_out=1)#巡航到第一列有文字的地方
    
    #识别文字，这个先不写
    
    my_car.set_pose_offset([0.4,0,0]) #向前走点,不要看到第一列文字
    
    my_car.lane_det_location_v4(0.2, tar, side=-1,dis_out=1)#巡航到第二列有文字的地方
    
    #my_car.set_pose_offset([-0.1,0,0])#向后退一点
    answer=3
    if answer==1 or answer==3:
    
        if answer==1:
            back=-0.1
        
        if answer==3:
            back=-0.27
        #放上栏 第一个
        my_car.set_pose_offset([back,0,0])
        
        my_car.task.arm.grap(1) #吸
        
        time.sleep(3)
        
        my_car.task.arm.set_hand_angle(-45)#手心向前
        
        my_car.task.arm.set(0.1,0.132) #抬起手臂
        
        
        my_car.task.arm.set(0.02,0.132) #伸手去放物品
        
        my_car.task.arm.grap(0) #松手
        
        my_car.task.arm.set(0.266,0.132) #缩手
        
        
        '''#放上栏 第二个
        my_car.set_pose_offset([-0.06,0,0])
        
        my_car.task.arm.switch_side(-1)#手臂向右
        
        my_car.task.arm.set_hand_angle(90)#手心向下
        
        my_car.task.arm.set(0.255,0.11)#初始位置高一点比较好
    
        
        my_car.task.arm.grap(1) #吸
        
        my_car.task.arm.set(0.255,0) #下去拿物品
        time.sleep(1)
        
        my_car.task.arm.set(0.255,0.05)#轻微抬起
        
        my_car.task.arm.set_hand_angle(-45)#手心向前
        
        my_car.task.arm.set(0.15,0.132) #抬起手臂
        
        
        my_car.task.arm.set(0.01,0.132) #伸手去放物品
        
        my_car.task.arm.grap(0) #松手
        
        my_car.task.arm.set(0.266,0.132) #缩手'''
        
        
        
    if answer==2 or answer==4:    
        
        if answer==2:
            back=-0.1
        
        if answer==4:
            back=-0.27
        
        #放下栏 第一个
        my_car.set_pose_offset([back,0,0])
        
        my_car.task.arm.set(0.15,0.03)
        
        my_car.task.arm.grap(1) #吸
        time.sleep(3)
        
        my_car.task.arm.set_hand_angle(-45)#手心向前
        
        my_car.task.arm.set(0.1,0.03) #压低手臂
        
        
        my_car.task.arm.set(0,0.03) #伸手去放物品
        
        my_car.task.arm.grap(0) #松手
        
        my_car.task.arm.set(0.266,0.03) #缩手
        
        
        
        '''#放下栏 第二个
        my_car.set_pose_offset([-0.06,0,0])
        
        my_car.task.arm.switch_side(-1)#手臂向右
        
        my_car.task.arm.set_hand_angle(90)#手心向下
        
        my_car.task.arm.set(0.255,0.05)#初始位置高一点比较好
        #my_car.task.arm.set_hand_angle(90)
        
        my_car.task.arm.grap(1) #吸
        
        my_car.task.arm.set(0.255,0) #下去拿物品
        
        time.sleep(2)
        
        my_car.task.arm.set(0.255,0.05)#轻微抬起
        
        my_car.task.arm.set_hand_angle(-45)#手心向前
        
        my_car.task.arm.set(0.15,0.03) #压低手臂
        
        
        my_car.task.arm.set(0.01,0.03) #伸手去放物品
        
        my_car.task.arm.grap(0) #松手
        
        my_car.task.arm.set(0.266,0.03) #缩手'''
        
        
        
        
        
    
        
    
