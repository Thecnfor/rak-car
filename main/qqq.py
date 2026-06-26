# -*- coding: utf-8 -*-
import sys
import logging

sys.path.append("/home/jetson/workspace/vehicle_wbt/")

import time
import threading
import os
import numpy as np
from task_func import MyTask
from log_info import logger
from car_wrap import MyCar
from tools import CountRecord
import math
import sys, os ,subprocess
from ernie_bot.base import answer
from ernie_bot import Weather

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
index_form = {
    0: "cauliflower",
    1: "greens",
    2: "cylinder3",
    3: "turn_right",
    4: "chili",
    5: "tofu",
    6: "turn_left",
    7: "eggplant",
    8: "wax_gourd",
    9: "cylinder2",
    10: "cylinder1",
    11: "potato",
    12: "tomato",
    13: "cabbage",
    14: "meat",
    15: "sparerib",
    16: "text_det",
    17: "chicken",
    18: "celery",
    19: "egg",
    20: "mushroom",
    21: "green_beans",
}


# ??????????
def get_key_by_value(d, value):
    for k, v in d.items():
        if v == value:
            return k
    return None

def plants(side):

    sys.stdout.flush()

    # 1. 获取目标列表
    tar_list = my_car.task.planting(side, arm_set=True)
    
    # 2. 定位到最近的目标
    
    pose_dict = my_car.lane_det_location_plant(speed=0.10, targets=tar_list, side=side, dis_out=1.8)
    
    # 3. 根据定位结果确定圆柱体类型并执行操作
    if pose_dict is not False and len(pose_dict) > 0:
        # 获取找到的目标ID
        found_id = list(pose_dict.keys())[0]
        
        # 根据ID确定圆柱体类型
        if found_id == 100:  # cylinder3
            radius = 2
            print("检测到大圆柱体(cylinder3) - 执行浇水操作")
        elif found_id == 80:  # cylinder2
            radius = 1
            print("检测到中圆柱体(cylinder2) - 执行蜂鸣示意")
        elif found_id == 60:  # cylinder1
            radius = 0
            print("检测到小圆柱体(cylinder1) - 执行补光操作")
        else:
            print(f"未知的目标ID: {found_id}")
            radius = None
        
        # 4. 执行对应的操作
        if radius is not None:
            my_car.task.planting(side, radius=radius)
            time.sleep(1)
    
    sys.stdout.flush()
    
def plant_action1(side):    
    my_car.set_pose_offset([0.29, 0, 0], 1.8)
    plants(side)
    
    

def weather_action():
    my_car.task.arm.set_hand_angle(60)      #初始化
    my_car.set_pose_offset([0.76, 0, 0], 3)    #走到方向牌

    side = my_car.get_card_side()
    print(side)     #识别方向
    
    
    
    # 不同方向微调到转弯后的道路起始位置
    if side == -1:

        my_car.task.arm.set(0.07, 0)
        my_car.task.arm.switch_side(1)
        # my_car.task.arm.set_arm_angle(-96)
        my_car.set_pose_offset([0, 0, -math.pi / 4], 1)
        time.sleep(0.3)
        my_car.set_pose_offset([0.18, 0, 0], 1)
        my_car.set_pose_offset([0, 0, math.pi / 10], 0.5)
        my_car.set_pose_offset([0.12, 0, 0], 1)
        my_car.set_pose_offset([0, -0.02, 0], 0.5) 
        my_car.set_pose_offset([0.3, 0, 0], 1.5)
        my_car.set_pose_offset([0, 0, math.pi / 6.8], 0.6)
        
        #my_car.set_pose_offset([0, 0.05, 0], 0.5)   ###11.7
        
        # my_car.set_pose_offset([0, -0.03, 0], 0.1)
        my_car.set_pose_offset([0.31, 0, 0], 1.5)    ###[0.23, 0, 0]
        my_car.set_pose_offset([0, -0.01, 0], 0.2)

    else:
        
        
        my_car.task.arm.set(0.25, 0)
        my_car.task.arm.switch_side(-1)
        
        # my_car.task.arm.set_arm_angle(90)
        my_car.set_pose_offset([0, 0, math.pi / 4 * side], 1)
        time.sleep(0.3)
        my_car.lane_dis_offset(0.2, 0.58)
        my_car.set_pose_offset([0, 0.035, 0], 0.2)
        # my_car.set_pose_offset([0, 0, -math.pi / 25 * side], 0.5)  ###
        time.sleep(0.3)
        my_car.set_pose_offset([0.28, 0, 0])

        
    side *= -1
    
    weather_api = Weather()
    city = "东莞市"  
    
    try:
        weather = weather_api.get_weather_by_city(city)
        answer = weather_api.get_weather_num(weather['weather'])    
        my_car.task.weather_set(answer)
        print(f"当前天气: {weather['weather']}，温度: {weather['temperature']}℃")
    except Exception as e:
        print(f"获取天气信息失败: {e}")
    time.sleep(8)
    return side 


def hanoi():
    # my_car.task.arm.set(0, 0)
    my_car.task.arm.set_hand_angle(60)
    start_car_position = my_car.get_odometry()
    my_car.set_pose_offset([0.7, 0, 0], 3)
    # my_car.task.arm.set_hand_angle(60)
    # my_car.task.arm.set(0.1, 0)
    # time.sleep(0.2)

    side = my_car.get_card_side()
    print(side)

    if side == -1:
        my_car.task.arm.set(0.04, 0)
        my_car.task.arm.switch_side(1)
        # my_car.task.arm.set_arm_angle(-96)
        my_car.set_pose_offset([0, 0, -math.pi / 4], 1)
        time.sleep(0.3)
        my_car.set_pose_offset([0.18, 0, 0], 1)
        my_car.set_pose_offset([0, 0, math.pi / 10], 0.5)
        my_car.set_pose_offset([0.12, 0, 0], 1)
        my_car.set_pose_offset([0, -0.02, 0], 0.5)
        my_car.set_pose_offset([0.3, 0, 0], 1.5)
        my_car.set_pose_offset([0, 0, math.pi / 6.8], 0.6)
        # my_car.set_pose_offset([0, -0.03, 0], 0.1)
        my_car.set_pose_offset([0.23, 0, 0], 1.5)
        my_car.set_pose_offset([0, -0.01, 0], 0.2)
        length = 0.415

    else:

        my_car.task.arm.set(0.21, 0)
        my_car.task.arm.switch_side(-1)

        # my_car.task.arm.set_arm_angle(90)
        my_car.set_pose_offset([0, 0, math.pi / 4 * side], 1)
        time.sleep(0.3)
        """
        my_car.set_pose_offset([0.18, 0, 0], 1)
        my_car.set_pose_offset([0, 0, -math.pi / 10], 0.5)
        my_car.set_pose_offset([0.12, 0, 0], 1)
        my_car.set_pose_offset([0, 0.04, 0], 0.5)
        my_car.set_pose_offset([0.3, 0, 0],1.5)
        my_car.set_pose_offset([0, 0, -math.pi /6.8], 0.6)
        #my_car.set_pose_offset([0, -0.03, 0], 0.1)
        my_car.set_pose_offset([0.23, 0, 0], 1.5)
        my_car.set_pose_offset([0, 0.03, 0], 0.5)    """

        my_car.lane_dis_offset(0.2, 0.58)
        my_car.set_pose_offset([0, 0.035, 0], 0.2)
        my_car.set_pose_offset([0, 0, -math.pi / 20 * side], 0.5)  ###
        time.sleep(0.3)
        my_car.set_pose_offset([0.28, 0, 0])

        length = 0.41

        # my_car.set_pos_offset([0, 0, math.pi / 5.2 * -1], 1)

    # my_car.task.arm.set(0.07, 0)
    # my_car.task.arm.switch_side(1)
    # my_car.task.arm.switch_side(1)
    # my_car.task.arm.set_hand_angle(90)
    # time.sleep(1)
    # side=-1
    # my_car.beep()
    # print("11111111111111111")
    sys.stdout.flush()
    cylinder_id = 1
    pts = my_car.task.pick_up_cylinder(cylinder_id, side, True)
    pos_start = np.array(my_car.get_odometry())
    pos_zone = pos_start.copy()
    pos_zone[0] += length
    print("prepare for location")
    
    my_car.set_pose_offset([0.1, 0, 0])  #2025.10.25
    
    pose_dict = my_car.lane_det_location_v8_multi(
        0.15, pts, side=side * -1, dis_out=1.9
    )  ###dis_out=2  dis_out=1.8
    print(pose_dict)
    if pose_dict is not False and len(pose_dict) > 0:
        for i in range(2):
            if i == 0:
                run_dis = my_car.calculation_dis(
                    np.array(pose_dict[100]), np.array(pos_zone)
                )
                my_car.set_pose(pose_dict[100], 1.5)
            if i == 1:
                run_dis = my_car.calculation_dis(
                    np.array(pose_dict[80]), np.array(pos_zone)
                )
                my_car.set_pose(pose_dict[80], 1.5)
            if i == 2:
                run_dis = my_car.calculation_dis(
                    np.array(pose_dict[60]), np.array(pos_zone)
                )
                my_car.set_pose(pose_dict[60], 1.5)

            my_car.task.pick_up_cylinder(i, side)
            my_car.set_pose_offset([run_dis, 0, 0], 1.8)
            my_car.task.put_down_cylinder(i, side)
    # my_car.set_pose_offset([-0.2, 0, 0])
    # my_car.set_pose_offset([-0.1, 0, 0])
    sys.stdout.flush()


def bmi():
    # my_car.task.arm.set(0,0)
    # my_car.task.arm.set_offset(0, 0-0.13)
    # my_car.lane_dis_offset(0.2, 15)
    """
    my_car.task.arm.switch_side(1)  # 臂转到左边准备识别
    my_car.task.arm.set(0.13, 0)  # 臂升到摄像头能看到文字的高度
    my_car.set_pose_offset([-0.2, 0, 0])  # 倒退一点以便巡航
    my_car.lane_dis_offset(0.3, 2)  # 先巡航一段避免红外意外识别到汉诺塔的柱子

    my_car.set_pose_offset([0, -0.04, 0])

    my_car.lane_sensor(0.2, value_h=0.2, sides=1)  # 红外识别bmi位置
    #my_car.set_pose_offset([0, 0, math.pi / 25])  # 将车挪正
    my_car.set_pose_offset([0, -0.02, 0], 0.5)  # 往右平移让车不要太贴线
    my_car.set_pose_offset([0.15, 0, 0])  # 直走到推杆位置

    # t = threading.Thread(target=task)
    # t.start()
    # t.join(timeout=3)
    my_car.set_pose_offset([0, 0.075, 0], 2)  # 推杆
    my_car.set_pose_offset([0, -0.08, 0], 2)  # 回位
    my_car.set_pose_offset([0.1, 0, 0], 1)  # 直走到摄像头识别文字位置
    try:
        text = my_car.get_ocr_list()[0]
        print(text)
        # height, weight = extract_height_weight(text)
        # out = calculate_bmi(height, weight)
        # text=my_car.get_ocr_list()[0] #获取文字
        bmi_degree = answer.ask3(text)
        print(bmi_degree)
        out = bmi_degree['answer']
        out = int(out)
        # my_car.task.bmi_set(out)
    except Exception as e:
        print(f"error: {e}")
    """
    my_car.task.arm.switch_side(1)  # 臂转到左边准备识别
    my_car.task.arm.set(0.13, 0)  # 臂升到摄像头能看到文字的高度
    # my_car.set_pose_offset([-0.2, 0, 0])  # 倒退一点以便巡航
    my_car.lane_dis_offset(0.3, 1.95)  # 先巡航一段避免红外意外识别到汉诺塔的柱子

    my_car.set_pose_offset([0, -0.07, 0])

    my_car.lane_sensor(0.2, value_h=0.2, sides=1)  # 红外识别bmi位置

    my_car.set_pose_offset([0, 0, math.pi / 38])  # 将车挪正

    # 0my_car.set_pose_offset([0, -0.02, 0], 0.5)  # 往右平移让车不要太贴线

    my_car.set_pose_offset([0.10, 0, 0])  # 直走到推杆位置

    # t = threading.Thread(target=task)
    # t.start()
    # t.join(timeout=3)

    my_car.set_pose_offset([0, 0.12, 0], 2)  # 推杆

    my_car.set_pose_offset([0, -0.09, 0], 2)  # 回位
    my_car.set_pose_offset([0.1, 0, 0], 1)  # 直走到摄像头识别文字位置
    try:
        text = my_car.get_ocr_list()[0]
        print(text)
        # height, weight = extract_height_weight(text)
        # out = calculate_bmi(height, weight)
        # text=my_car.get_ocr_list()[0] #获取文字
        bmi_degree = answer.ask3(text)
        print(bmi_degree)
        out = bmi_degree["answer"]
        out = int(out)
        my_car.task.bmi_set(out)
    except Exception as e:
        print(f"error: {e}")


def bmi():
    # my_car.task.arm.set(0,0)
    # my_car.task.arm.set_offset(0, 0-0.13)
    # my_car.lane_dis_offset(0.2, 15)

    my_car.task.arm.switch_side(1)  # 臂转到左边准备识别
    my_car.task.arm.set(0.13, 0)  # 臂升到摄像头能看到文字的高度
    my_car.set_pose_offset([-0.2, 0, 0])  # 倒退一点以便巡航
    my_car.lane_dis_offset(0.3, 2.1)  # 先巡航一段避免红外意外识别到汉诺塔的柱子

    my_car.set_pose_offset([0, -0.04, 0])

    my_car.lane_sensor(0.2, value_h=0.2, sides=1)  # 红外识别bmi位置
    # my_car.set_pose_offset([0, 0, math.pi / 25])  # 将车挪正
    my_car.set_pose_offset([0, -0.02, 0], 0.5)  # 往右平移让车不要太贴线
    my_car.set_pose_offset([0, 0, math.pi / 25], 0.5)

    my_car.set_pose_offset([0.15, 0, 0])  # 直走到推杆位置

    # t = threading.Thread(target=task)
    # t.start()
    # t.join(timeout=3)
    my_car.set_pose_offset([0, 0.075, 0], 2)  # 推杆
    my_car.set_pose_offset([0, -0.07, 0], 2)  # 回位
    # my_car.set_pose_offset([0.1, 0, 0], 1)  # 直走到摄像头识别文字位置
    my_car.set_vel_time(0.25, 0, 0, 0.4)
    time.sleep(2)


def plant_action2():

    my_car.set_pose_offset([0, 0.04, 0]) 
        
    my_car.set_pose_offset([-0.2, 0, 0])    # 倒退

    
    my_car.lane_dis_offset(0.3, 2)    # 巡航

    my_car.set_pose_offset([0, -0.04, 0])

    my_car.lane_sensor(0.2, value_h=0.2, sides=1)  # 红外
    # my_car.set_pose_offset([0, 0, math.pi / 25])  # 修正
    
    my_car.task.arm.set(0.07, 0)
    my_car.task.arm.switch_side(1)    #恢复手臂初始位置
    
    my_car.set_pose_offset([0.2, 0, 0],0.7)    #准备开始识别
    time.sleep(0.5)
    
    side = 1
    plants(side)


    # my_car.task.arm.set(0,0)
    # my_car.task.arm.grap(1)
    # my_car.task.arm.set_hand_angle(80)
    """
        angle_offset = -math.pi / 2 * 0.82
        # dis_angle = -math.pi/2*0.3
        # dis = 1.
        dis_x = 1.36
        dis_y = -0.87
        # print(dis_x, dis_y)
        angle_now = my_car.get_odometry()[2]
        x_offset = dis_x * math.cos(angle_now) - dis_y * math.sin(angle_now)
        y_offset = dis_y * math.cos(angle_now) + dis_x * math.sin(angle_now)
        angle_tar = angle_now - math.pi * 2 + angle_offset
        pose = my_car.get_odometry().copy()
        pose[0] = pose[0] + x_offset
        pose[1] = pose[1] + y_offset
        pose[2] = angle_tar
        # print(pose)
        # return

        my_car.lane_sensor(0.3, value_h=1, sides=-1)  # 先红外到圆环处
        # time.sleep(25)
        # my_car.lane_sensor()

        my_car.task.arm.set(0.265, 0)

        # my_car.lane_sensor(0.3, value_h=1, sides=-1)

        # time.sleep(25)
        # my_car.lane_sensor()

        my_car.task.arm.set(0.265, 0)  ############

        my_car.lane_dis_offset(0.3, 0.5)

        ## my_car.set_vel_time(0.3, -0.01, -0.53, 6)
        ## my_car.set_vel_time(0.3, -0.02, -0.52, 3)
        ## my_car.lane_dis_offset(0.3, 3.0)

        my_car.set_vel_time(0.3, -0.02, -0.53, 3)

        my_car.lane_dis_offset(0.3, 1)

        my_car.set_vel_time(0.3, -0.005, -0.53, 3)
        my_car.lane_dis_offset(0.3, 2.7)

        # my_car.lane_dis_offset(0.3, 4.58)
        my_car.set_pose_offset([0, -0.03, 0], 0.2)
        my_car.set_pose_offset([-0.03,0,0],0.2)
        my_car.set_pose_offset([0, 0, math.pi / 30], 1)

    def eject1():
        # finetune_car_position=my_car.get_odometry()
        # print(finetune_car_position)
        # print(finetune_car_position[2]-start_car_position[2])
        # my_car.set_pose_offset([0, 0, (finetune_car_position[2]-start_car_position[2]) ] , 0.2)

        my_car.task.eject(1)
        time.sleep(0.5)
        # my_car.set_pose_offset([0.7, 0, 0],0.5)
        my_car.lane_dis_offset(0.3, 0.7)

        print("finish camp task!!!!!")
    """
def camp():
    angle_offset = -math.pi / 2 * 0.82
    # dis_angle = -math.pi/2*0.3
    # dis = 1.
    dis_x = 1.36
    dis_y = -0.87
    # print(dis_x, dis_y)
    angle_now = my_car.get_odometry()[2]
    x_offset = dis_x * math.cos(angle_now) - dis_y * math.sin(angle_now)
    y_offset = dis_y * math.cos(angle_now) + dis_x * math.sin(angle_now)
    angle_tar = angle_now - math.pi * 2 + angle_offset
    pose = my_car.get_odometry().copy()
    pose[0] = pose[0] + x_offset
    pose[1] = pose[1] + y_offset
    pose[2] = angle_tar
    # print(pose)
    # return

    my_car.lane_sensor(0.3, value_h=1, sides=-1)  #

    ############ 更改：删除手臂动作

    my_car.lane_dis_offset(0.3, 0.5)

    ## my_car.set_vel_time(0.3, -0.01, -0.53, 6)
    ## my_car.set_vel_time(0.3, -0.02, -0.52, 3)
    ## my_car.lane_dis_offset(0.3, 3.0)

    my_car.set_vel_time(0.3, -0.02, -0.53, 3)

    my_car.lane_dis_offset(0.3, 1)

    my_car.set_vel_time(0.3, -0.005, -0.53, 3)

    # my_car.lane_dis_offset(0.3, 2.7)
    my_car.lane_dis_offset(0.3, 1.1)
    

    # my_car.lane_dis_offset(0.3, 4.58)

    # my_car.set_pose_offset([0, -0.03, 0], 0.2)
    # my_car.set_pose_offset([-0.03,0,0],0.2)
    # my_car.set_pose_offset([0, 0, math.pi / 30], 1)
    
    
def plant_action3():
    side = 1
    my_car.task.arm.set(0.07, 0)
    
    #my_car.set_pose_offset([0, 0, math.pi / 25], 0.5)
    
    my_car.lane_sensor(0.25, value_h=0.2, sides=1)
    
    ##my_car.task.arm.set(0.07, 0)    #从这段开始运行记得加手臂初始化动作
    ##my_car.task.arm.switch_side(1)
    
    my_car.set_pose_offset([0.14, 0, 0], 1)
   
    # side = 1
    plants(side)



def magic():
    # my_car.task.arm.set(0.17, 0.15)
    # my_car.task.arm.switch_side(1)
    my_car.set_pose_offset([0, 0, -math.pi / 30], 0.5)

    my_car.lane_sensor(0.25, value_h=0.2, sides=1)
    # my_car.set_pose_offset([0.24, 0, 0],1.2)
    my_car.set_vel_time(0.235, 0, 0, 0.85)

    my_car.task.arm.switch_side(1)
    my_car.task.arm.set_hand_angle(60)

    my_car.task.arm.set(0.265, 0.1)
    my_car.task.arm.grap(1)
    my_car.task.arm.set(0.265, 0.07)
    time.sleep(1)
    my_car.task.arm.set(0.265, 0.1)
    my_car.task.arm.switch_side(-1)
    my_car.task.arm.set(0.17, 0.1)
    my_car.task.arm.set_arm_angle(-115)
    my_car.task.arm.set(0.17, 0)
    my_car.task.arm.grap(0)

    my_car.task.arm.set(0.17, 0.1)


def eject1():
    # finetune_car_position=my_car.get_odometry()
    # print(finetune_car_position)
    # print(finetune_car_position[2]-start_car_position[2])
    # my_car.set_pose_offset([0, 0, (finetune_car_position[2]-start_car_position[2]) ] , 0.2)
    '''
    my_car.lane_dis_offset(0.3, 1.5)


    my_car.lane_dis_offset(0.3, 0.3)
    my_car.set_pose_offset([0, 0, math.pi / 35], 1)

    my_car.task.eject(1)
    time.sleep(0.5)
    # my_car.set_pose_offset([0.7, 0, 0],0.5)
    my_car.lane_dis_offset(0.3, 0.7)

    print("finish camp task!!!!!")
    '''
    my_car.lane_dis_offset(0.3, 1.18)
    my_car.set_pose_offset([0, 0, math.pi / 28], 1)
    my_car.set_pose_offset([0, 0, math.pi / 25], 1)
    my_car.task.eject(1)
    time.sleep(0.5)
    my_car.set_pose_offset([0, 0, -math.pi / 20], 1)
    my_car.set_pose_offset([0,-0.05 , 0], 0.5)
    time.sleep(1)
    


def get_food():
    my_car.task.arm.switch_side(-1)  # 手臂向右
    my_car.task.arm.set(0.265, 0)  # 手臂向后退到能看到文字的位置
    tar = [
        16,
        0,
        "text_det",
        0,
        0.0390625,
        0.7916666666666666,
        0.37,
        0.39166666666666666,
    ]
    # time.sleep(1)
    # my_car.set_velocity(0.3, 0, 0)
    time.sleep(0.5)
    flag, offset = my_car.lane_det_location_v4(
        0.2, tar, side=-1, dis_out=1
    )  # 巡航找文字
    time.sleep(0.7)
    try:
        text = my_car.get_ocr_list_plus()
        # 获取文字
        print(text)
        text = text[0]
        foods = answer.ask1(text)
        the_food1 = foods["answer"]
        print("\n食物是\n", the_food1)
    except Exception as e:
        print(e)
        the_food1 = "egg"
    # my_car.set_vel_time(-0.5,0,0,0.5)
    v = -0.5
    t = -offset / v
    v = -0.48
    print("时间是", t)
    my_car.set_vel_time(v, 0, 0, t)  # 后退
    my_car.set_vel_time(0.3, 0, 0, 0.6)

    # my_car.set_pose_offset([-0.25, 0, 0])  # 后退一步
    my_car.task.arm.set(0.135, 0)  # 调整手臂定位食材
    index = get_key_by_value(index_form, the_food1)
    if index == None:
        index = 12
    #tar = [index, 0, the_food1, 0.9151089787483215, 0, 0.2, 0.33, 0.7]
    tar = [index, 0, the_food1, 0.9151089787483215, 0, 0.2, 0.342, 0.7]
    
    flag, offset = my_car.lane_det_location_vert(0.15, tar, side=-1, dis_out=1.2)

    try:
        img_side = my_car.cap_side.read()
        dets_ret = my_car.task_det(img_side)
        print("object is", dets_ret)
        for det_ret in dets_ret:
            if det_ret[0] == index:
                y = det_ret[5]
    except:
        y = 0
    if flag == False:
        y = -1

    if y < 0 or y == 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.135, 0.095)  # 手臂上抬
        my_car.task.arm.set_hand_angle(-80)  # 掌心向前
        my_car.task.arm.set(0, 0.115)  # 向前抓
        time.sleep(0.5)

        my_car.task.arm.set(0.268, 0.1)  # 往后缩
        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        my_car.task.arm.set_arm_angle(-95)
        my_car.task.arm.set(0.27, 0)
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松
        my_car.task.arm.set(0.27, 0.05)
        time.sleep(1)
        """my_car.task.arm.set(0.18, 0.1)  # 往后缩
        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        
        my_car.task.arm.set_arm_angle(-121)  # 手臂里弯
        my_car.task.arm.set(0.18, 0.02)  # 手臂下放
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松手
        time.sleep(1)"""

    if y > 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.135, 0)  # 手臂下降
        my_car.task.arm.set_hand_angle(-80)  # 掌心向前
        my_car.task.arm.set(0, 0.012)  # 向前抓
        time.sleep(0.5)

        my_car.task.arm.set(0.268, 0.05)  # 往后缩
        my_car.task.arm.set_hand_angle(60)
        my_car.task.arm.set_arm_angle(-95)
        my_car.task.arm.set(0.268, 0)
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松
        my_car.task.arm.set(0.268, 0.05)
        time.sleep(1)
        """my_car.task.arm.set(0.18, 0.05)  # 往后缩
        my_car.task.arm.set_hand_angle(60)  # 掌心向下
        my_car.task.arm.set_arm_angle(-115)  # 手臂里弯
        my_car.task.arm.set(0.18, 0.02)  # 手臂下放
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松
        time.sleep(1)"""

    v = -0.5
    t = -offset / v
    v = -0.5
    print("时间是", t)
    my_car.set_vel_time(v, 0, 0, t)  # 后退
    my_car.task.arm.switch_side(1)  # 手臂向左
    my_car.task.arm.grap(0)
    my_car.task.arm.set(0.268, 0.1)
    my_car.task.arm.set(0, 0.06)
    my_car.task.arm.set_hand_angle(-80)
    my_car.task.arm.set(0, 0.02)  # 达到摄像头能看到文字的位置
    my_car.task.arm.set_hand_angle(-80)
    tar = [
        16,
        0,
        "text_det",
        0.9307849407196045,
        -0.0546875,
        0.7645833333333333,
        0.34,
        0.30416666666666664,
    ]
    time.sleep(1)
    flag, offset = my_car.lane_det_location_v4(
        0.2, tar, side=1, dis_out=1
    )  # 巡航找文本
    time.sleep(0.7)
    try:
        text = my_car.get_ocr_list_plus()[0]  # 获取文字
        print(text)
        foods = answer.ask1(text)
        the_food2 = foods["answer"]
        print("\n第二个食物是\n", the_food2)
    except Exception as e:
        print(e)
        the_food2 = "tomato"
    if flag == False:
        y = -1

    v = -0.5
    t = -offset / v
    v = -0.48
    print("时间是", t)
    my_car.set_vel_time(v, 0, 0, t)  # 后退

    my_car.task.arm.set(0.10, 0.02)  # 调整手臂定位食材

    index = get_key_by_value(index_form, the_food2)
    if index == None:
        index = 19
    tar = [index, 0, the_food2,0.8597215414047241, 0.028125, 0.37291666666666665, 0.40, 0.2625]  #   0.7483181953430176, 0, 0.2, 0.27, 0.7
    print(f"tar:{tar}")
    # tar=[index, 0, answer, 0.5542138814926147, 0, 0.192, 0.18, 0.375]
    flag, offset = my_car.lane_det_location_vert(
        0.135, tar, side=1, dis_out=1.2
    )  # 巡航找食材
    try:
        img_side = my_car.cap_side.read()
        dets_ret = my_car.task_det(img_side)
        print("检测目标是", dets_ret)

        for det_ret in dets_ret:
            if det_ret[0] == index:
                y = det_ret[5]
        print("-----------------y值是----------------------", y)
    except:
        y = -1

    if y < 0 or y == 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.1, 0.095)  # 手臂上抬
        my_car.task.arm.set_hand_angle(-80)  # 掌心向前
        my_car.task.arm.set(0.27, 0.12)  # 向前抓
        time.sleep(0.5)
        my_car.task.arm.set(0, 0.12)  # 往后缩
        # time.sleep(0.5)
        # my_car.task.arm.set_hand_angle(90)#掌心向下
        # time.sleep(0.5)

        # my_car.task.arm.set(0,0.03)#手臂向下放食材
        # my_car.task.arm.grap(0)#松手

    if y > 0:
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.1, 0.015)  # 手臂下降
        my_car.task.arm.set_hand_angle(-80)  # 掌心向前
        my_car.task.arm.set(0.08, 0.1)
        my_car.task.arm.set(0.18, 0.01)  # 向前抓
        my_car.task.arm.set(0.26, 0.01)
        time.sleep(0.5)
        my_car.task.arm.set(0.15, 0.1)
        my_car.task.arm.set(0, 0.04)  # 往后缩
        # time.sleep(0.5)
        # my_car.task.arm.set_hand_angle(90)#掌心向下
        # time.sleep(0.5)
        # my_car.task.arm.set(0,0.03)#手臂向下放食材
        # my_car.task.arm.grap(0)#松
    if flag == False:
        my_car.set_vel_time(-0.3, 0, 0, 1.2)
    my_car.set_pose_offset([0, -0.03, 0], 0.2)
    return [the_food1, the_food2]

def answer1():
    my_car.lane_dis_offset(0.3, 1.3)  ######my_car.lane_dis_offset(0.3, 0.4)
    my_car.set_pose_offset([0, -0.055, 0], 1)
    my_car.lane_sensor(0.30, value_h=0.3, sides=1)  #####(0.2, value_h=0.3, sides=1)
    my_car.set_pose_offset([0, 0, math.pi / 30], 0.5)
    my_car.set_pose_offset([0.15, 0, 0], 1)  ###
    my_car.task.arm.grap(1)
    time.sleep(0.5)
    my_car.task.arm.set_hand_angle(-80)
    time.sleep(0.5)
    my_car.task.arm.set(0.1, 0.07)
    my_car.task.arm.set(0.1, 0.06)
    time.sleep(0.5)
    my_car.task.arm.set(0.26, 0.07)
    my_car.task.arm.set(0.1, 0.07)
    my_car.set_pose_offset([0, 0.02, 0], 0.5)
    """
    #第四个答案
    my_car.lane_sensor(0.25, value_h=0.3, sides=1)
    my_car.set_pose_offset([0.38, 0, 0], 1)
    my_car.task.arm.grap(1)
    time.sleep(0.5)
    my_car.task.arm.set_hand_angle(-45)
    time.sleep(0.5)
    my_car.task.arm.set(0.1,0.08)
    time.sleep(0.5)
    my_car.task.arm.set(0.24,0.08)
    my_car.task.arm.set(0.1,0.08)
    """


def answer2():
    my_car.lane_dis_offset(0.3, 1.3)  ######my_car.lane_dis_offset(0.3, 0.4)
    my_car.set_pose_offset([0, -0.055, 0], 1)  ######my_car.lane_dis_offset(0.3, 0.4)
    my_car.lane_sensor(0.30, value_h=0.3, sides=1)  #####(0.2, value_h=0.3, sides=1)
    my_car.set_pose_offset([0, 0, math.pi / 30], 0.5)
    my_car.set_pose_offset([0.235, 0, 0], 1)
    my_car.task.arm.grap(1)
    time.sleep(0.5)
    my_car.task.arm.set_hand_angle(-80)
    time.sleep(0.5)
    my_car.task.arm.set(0.1, 0.07)
    time.sleep(0.5)
    
    my_car.set_pose_offset([0, 0.015, 0], 0.5)
    
    my_car.task.arm.set(0.25, 0.07)  # 推的距离调整
    my_car.task.arm.set(0.1, 0.07)
    my_car.set_pose_offset([0, 0.02, 0], 0.5)


def answer3():
    my_car.lane_dis_offset(0.3, 1.3)  ######my_car.lane_dis_offset(0.3, 0.4)
    my_car.set_pose_offset([0, -0.055, 0], 1)
    my_car.lane_sensor(0.30, value_h=0.3, sides=1)  #####(0.2, value_h=0.3, sides=1)
    my_car.set_pose_offset([0, 0, math.pi / 30], 0.5)
    my_car.set_pose_offset([0.32, 0, 0], 1)
    my_car.task.arm.grap(1)
    time.sleep(0.5)
    my_car.task.arm.set_hand_angle(-80)
    time.sleep(0.5)
    my_car.task.arm.set(0.1, 0.07)
    time.sleep(0.5)
    my_car.task.arm.set(0.26, 0.07)
    my_car.task.arm.set(0.1, 0.07)
    my_car.set_pose_offset([0, 0.02, 0], 0.5)


def answer4():
    my_car.lane_dis_offset(0.3, 1.3)  ######my_car.lane_dis_offset(0.3, 0.4)
    my_car.set_pose_offset([0, -0.055, 0], 1)
    my_car.lane_sensor(0.30, value_h=0.3, sides=1)  #####(0.2, value_h=0.3, sides=1)
    my_car.set_pose_offset([0, 0, math.pi / 30], 0.5)
    my_car.set_pose_offset([0.43, 0, 0], 1)
    my_car.task.arm.grap(1)
    time.sleep(0.5)
    my_car.task.arm.set_hand_angle(-80)
    time.sleep(0.5)
    my_car.task.arm.set(0.1, 0.07)
    time.sleep(0.5)
    # my_car.set_pose_offset([0,0.03,0],0.5)
    my_car.task.arm.set(0.265, 0.07)

    # my_car.set_pose_offset([0,0,math.pi/25],0.5)

    my_car.task.arm.set(0.1, 0.07)
    my_car.set_pose_offset([0, 0.02, 0], 0.5)

    ########


def answer_real():
    my_car.lane_dis_offset(0.3, 0.4)
    my_car.task.arm.switch_side(1)
    my_car.task.arm.set(0, 0.01)
    # my_car.lane_dis_offset(0.3, 0.4)
    my_car.lane_sensor(0.25, value_h=0.3, sides=1)
    time.sleep(0.5)
    my_car.set_pose_offset([0.30, 0, 0], 1)
    try:
        text = my_car.get_ocr_list()
        question = (
            text[0]
            + "选项答案有A."
            + text[1]
            + "B."
            + text[2]
            + "C."
            + text[3]
            + "D."
            + text[4]
        )
        answer_edu = answer.ask4(question)
        answer_edu["answer"]
    except Exception as e:
        print("/n", e, "/n")
        answer_edu = "A"
    if answer_edu == "A":
        adjust = -0.2
    if answer_edu == "B":
        adjust = -0.05
    if answer_edu == "C":
        adjust = -0.01
    if answer_edu == "D":
        adjust = 0.08
    my_car.set_pose_offset([adjust, 0, 0], 1)
    my_car.task.arm.grap(1)

    time.sleep(0.2)
    my_car.task.arm.set_hand_angle(-80)
    time.sleep(0.2)
    my_car.task.arm.set(0.1, 0.06)
    time.sleep(0.2)
    my_car.task.arm.set(0.24, 0.06)
    my_car.task.arm.set(0.1, 0.06)
    my_car.set_pose_offset([0, 0.035, 0], 0.5)


def eject2():
    my_car.lane_dis_offset(0.3, 0.1)
    my_car.lane_sensor(0.27, value_h=0.6, sides=-1)
    my_car.set_pose_offset([0, 0, math.pi / 25], 1)
    my_car.set_pose_offset([-0.05, 0, 0], 1)  ###([-0.1, 0, 0], 1)
    my_car.task.eject(2)
    my_car.set_pose_offset([0, 0, -math.pi / 30], 1)

    #######


def put_food(the_food1, the_food2):

    my_car.lane_dis_offset(0.3, 0.1)
    my_car.lane_sensor(0.27, value_h=0.6, sides=-1)
    my_car.set_pose_offset([0, 0, -math.pi / 25], 1)
    my_car.set_pose_offset([-0.05, 0, 0], 1)
    
    my_car.task.arm.grap(1)

    # the_food1 = 'egg'
    # the_food2 = 'tomato'
    # my_car.lane_dis_offset(0.3, 1)
    # 巡航找文字
    my_car.task.arm.switch_side(-1)

    my_car.task.arm.set(0.15, 0.01)  # 摄像头看得到两个文字的高度

    tar = [
        16,
        0,
        "text_det",
        0.9494228959083557,
        0.075,
        -0.2520833333333333,
        0.385,
        0.4875,
    ]# w = 0.385

    my_car.lane_det_location_v4(
        0.2, tar, side=-1, dis_out=0.5
    )  # 巡航到第一列有文字的地方
    time.sleep(0.5)
    try:
        text1 = my_car.get_ocr_list_plus()  # 获取文字
        print(text1)
    except Exception as e:
        print("/n", e, "/n")
        text1 = [
            "表面裹着红亮的酱汁呈深褐色，表面略带光泽，酸甜味突出外酥里嫩，酱汁浓郁，口感层次丰富。",
            "金黄色与鲜红色混合食材软烂出汁，整体呈现红黄相间的色彩。",
        ]

    # 识别文字，这个先不写
    my_car.set_vel_time(0.2, 0, 0, 2)
    # my_car.set_pose_offset([0.4, 0, 0])  # 向前走点,不要看到第一列文字

    my_car.lane_det_location_v4(
        0.2, tar, side=-1, dis_out=0.5
    )  # 巡航到第二列有文字的地方
    time.sleep(1)

    try:

        text2 = my_car.get_ocr_list_plus()  # 获取文字
        print(text2)
    except Exception as e:
        print("/n", e, "/n")
        text2 = [
            "表面油亮，整体呈现鲜亮的绿色，口感清脆，带有淡淡的蔬菜清甜，味道清淡爽口。",
            "深绿色的切片与浅棕色的肉块混合，肉片略带焦香，带有明显的辛辣味，咸香嫩滑整体口感鲜辣开胃间的色彩。",
        ]

    try:
        text_all = (
            the_food1
            + ","
            + the_food2
            + ",1."
            + text1[0]
            + ",2."
            + text1[1]
            + ",3."
            + text2[0]
            + ",4."
            + text2[1]
        )
        answer_dish = answer.ask2(text_all)
        print(f"\n{answer_dish}\n")
        answer_dish = answer_dish["answer"]
        answer_dish = int(answer_dish)
    except Exception as e:
        print("/n", e, "/n")
        answer_dish = 3
        print(f"\n{answer_dish}\n")
    my_car.set_pose_offset([0.02,0,0])

    if answer_dish == 1 or answer_dish == 3:
        if answer_dish == 1:
            back = -0.24
        if answer_dish == 3:
            back = -0.115
        # 放上栏 第一个
        # my_car.set_pose_offset([back, 0, 0])
        my_car.set_vel_time(back / 0.515, 0, 0, 0.5)
        my_car.task.arm.grap(1)  # 吸
        time.sleep(1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前
        my_car.task.arm.set(0.1, 0.135)  # 抬起手臂
        my_car.task.arm.set(0.0, 0.135)  # 伸手去放物品
        my_car.set_vel_time(0.1, 0, 0, 0.1)  # 向前挪一点点
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松手
        time.sleep(0.5)
        my_car.task.arm.set(0.19, 0.132)  # 缩手

        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        # my_car.task.arm.set_arm_angle(-120)  # 手臂里弯
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.26, 0)  # 向下拿
        my_car.task.arm.set(0.26, 0.132)
        my_car.task.arm.switch_side(-1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前

        my_car.set_vel_time(-0.1, 0, 0, 0.55)
        my_car.task.arm.set(0, 0.132)  # 伸手去放物品
        my_car.set_vel_time(-0.1, 0, 0, 0.1)
        my_car.set_pose_offset([-0.02,0,0])
        my_car.task.arm.grap(0)  # 松手
        time.sleep(0.5)
        my_car.task.arm.set(0.266, 0.132)

    if answer_dish == 2 or answer_dish == 4:
        if answer_dish == 2:
            back = -0.24
        if answer_dish == 4:
            back = -0.115
        # 放下栏 第一个
        # my_car.set_pose_offset([back, 0, 0])
        my_car.set_vel_time(back / 0.5, 0, 0, 0.5)
        my_car.task.arm.set(0.15, 0.03)
        my_car.task.arm.grap(1)  # 吸
        time.sleep(1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前
        my_car.task.arm.set(0.1, 0.03)  # 压低手臂
        my_car.task.arm.set(0, 0.02)  # 伸手去放物品
        my_car.set_vel_time(0.1, 0, 0, 0.1)
        time.sleep(0.5)
        my_car.task.arm.grap(0)  # 松手
        time.sleep(0.5)
        my_car.task.arm.set(0.03, 0.06)
        my_car.task.arm.set(0.19, 0.06)  # 缩手

        my_car.task.arm.set_hand_angle(60)  # 手臂向下
        # my_car.task.arm.set_arm_angle(-120)  # 手臂里弯
        my_car.task.arm.grap(1)  # 吸
        my_car.task.arm.set(0.26, 0)  # 向下拿
        my_car.task.arm.set(0.26, 0.05)
        my_car.task.arm.switch_side(-1)
        my_car.task.arm.set_hand_angle(-80)  # 手心向前

        my_car.set_vel_time(-0.1, 0, 0, 0.55)
        my_car.task.arm.set(0.1, 0.03)  # 压低手臂
        my_car.task.arm.set(0, 0.025)  # 伸手去放物品
        my_car.set_vel_time(-0.1, 0, 0, 0.1)
        my_car.task.arm.grap(0)  # 松手
        time.sleep(0.5)
        my_car.task.arm.set(0.266, 0.03)  # 缩j/

    """  
def old_people():
   # my_car.set_pose_offset([-0.2, 0, 0], 0.1)
    my_car.lane_dis_offset(0.3, 2)
    my_car.task.arm.set(0.1, 0.1) 
    my_car.lane_sensor(0.2, value_h=0.3, sides=1)
    my_car.task.arm.switch_side(1)
    my_car.set_pose_offset([0.25, 0, 0],0.5)
    #my_car.task.arm.switch_side(1)
    my_car.task.arm.set(0.25, 0.06) 
    my_car.set_pose_offset([0, 0.15, 0],0.5)
    my_car.set_pose_offset([-0.1,0,0],0.5)
    my_car.task.arm.set(0.25, 0.06) 
    my_car.set_pose_offset([0, -0.15, 0])
    my_car.set_pose_offset([-0.2, 0, 0],0.5)
    my_car.set_pose_offset([0, 0.3, 0],0.5)

    my_car.set_pose_offset([0, -0.3, 0],0.5)

    my_car.set_pose_offset([1, 0, 0],0.3)
    """


"""def turn(answer_dish):
    if answer_dish == 3 or answer_dish == 4:
        my_car.lane_dis_offset(0.3, 0.42)
        my_car.set_pose_offset([0, 0, -math.pi / 5], 1)
        my_car.lane_dis_offset(0.3, 1.1)
        my_car.set_pose_offset([0, 0, math.pi / 5], 1)
        my_car.lane_dis_offset(0.3, 0.6)
    else:
        my_car.lane_dis_offset(0.3, 0.5)
        my_car.set_pose_offset([0, 0, -math.pi / 5], 1)
        my_car.lane_dis_offset(0.3, 1.1)
        my_car.set_pose_offset([0, 0, math.pi / 5], 1)
        my_car.lane_dis_offset(0.3, 0.6)"""


def medicine():
    my_car.set_pose_offset([0, 0.11, 0], 1)
    my_car.task.arm.set(0.1,0.1)
    my_car.task.arm.switch_side(1)
    my_car.set_pose_offset([0, 0, -math.pi/1.5], 2)    # [0, 0, -math.pi/1.55]
    
    my_car.set_pose_offset([0, -0.06, 0], 0.6)
    my_car.set_pose_offset([0, 0, -math.pi/30], 1.5)
    my_car.set_pose_offset([0, 0.03, 0], 0.3)
    my_car.set_pose_offset([0, 0.03, 0], 0.7)
    my_car.set_pose_offset([0, 0, math.pi/30], 1.5)
    
    my_car.task.eject(3)
    
    my_car.set_pose_offset([0, 0, math.pi/1.5], 2)
    my_car.set_pose_offset([0, -0.11, 0], 1)





def old_people():
    my_car.lane_dis_offset(0.3, 2)
    my_car.task.arm.set(0.1, 0.1)
    my_car.lane_sensor(0.2, value_h=0.3, sides=1)
    my_car.task.arm.switch_side(1)
    time.sleep(0.5)
    my_car.set_vel_time(0.25, 0, 0, 0.85)
    time.sleep(0.5)
    my_car.task.arm.set(0.25, 0.06)
    time.sleep(0.5)
    my_car.set_vel_time(0, 0.2, 0, 0.5)
    time.sleep(0.5)
    my_car.set_vel_time(-0.2, 0, 0, 0.7)
    time.sleep(0.5)
    my_car.set_vel_time(0, -0.2, 0, 0.5)
    time.sleep(0.5)
    my_car.set_vel_time(-0.2, 0, 0, 0.7)
    time.sleep(0.5)
    my_car.set_vel_time(0, +0.2, 0, 1.5)
    time.sleep(0.5)
    my_car.set_vel_time(0, -0.2, 0, 1.5)  #(0, -0.2, 0, 1.5)
    time.sleep(0.5)
    # my_car.set_vel_time(0.4, 0, 0, 2)
    my_car.lane_dis_offset(0.38, 1)


"""
if __name__ == "__main__":
    # 每次调用 front() 都是新的实例，运行完就销毁
    my_car = MyCar()
    my_car.STOP_PARAM = False
    '''
    while True:
        side = my_car.get_card_side()
        print(side)
    '''
    #hanoi()
    #bmi()
    #camp()
    #eject1()
    #a,b=get_food()
    answer4()
    #eject2()
    #c=put_food("egg","tomato")
    #turn(c)
    #old_people()
    #my_car.set_vel_time(-0.3, 0 , 0, 1.4)


"""


def push_A():
    # hanoi()
    weather()
    camp()
    # magic()
    eject1()
    
    try:
        a, b = get_food()
    except:
        a = "tomato"
        b = "egg"
    answer1()
    # answer_real()
    eject2()
    put_food(a, b)  #####c = put_food(a, b)
    # turn(c)
    old_people()
    

def push_b():
    # side = weather_action()
    #plant_action1(side)
    #plant_action2()
    #camp()
    #magic()
    plant_action3()
    eject1()
    
    try:
        a, b = get_food()
    except:
        a = "tomato"
        b = "egg"
    answer2()
    # eject2()
    c = put_food(a, b)
    # turn(c)
    medicine()
    old_people()


def push_c():

    hanoi()
    bmi()
    camp()
    magic()
    eject1()

    try:
        a, b = get_food()
    except:
        a = "tomato"
        b = "egg"
    answer3()
    eject2()
    c = put_food(a, b)
    # turn(c)
    old_people()


def push_D():
    hanoi()
    bmi()
    camp()
    magic()
    eject1()
    try:
        a, b = get_food()
    except:
        a = "tomato"
        b = "egg"
    answer4()
    eject2()
    c = put_food("egg", "tomato")
    # turn(c)
    old_people()


import random


def all_all_all():
    hanoi()
    bmi()
    camp()
    eject1()
    a, b = get_food()
    # 将函数放入列表中
    function_list = [answer1, answer2, answer3, answer4]

    # 随机选择一个函数并执行
    random_function = random.choice(function_list)
    random_function()
    answer4()
    eject2()
    # c = put_food("egg", "tomato")
    # turn(c)
    old_people()

    

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    # my_car.task.reset()
    my_car.beep()
    time.sleep(0.2)
    # push_D()
    # functions = [push_A, push_b, push_c, push_D, all_all_all]
    sys.stdout.flush()
    # my_car.manage(functions, 5)

    push_b()
    