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

    sys.stdout.flush()
    
    
    
    
def weather():    
    weather_api = Weather()
    city = "三水区"  
    

    try:
        weather = weather_api.get_weather_by_city(city)
        answer = weather_api.get_weather_num(weather['weather'])    
        my_car.task.weather_set(answer)
        print(f"当前天气: {weather['weather']}，温度: {weather['temperature']}℃")
        return weather
    except Exception as e:
        print(f"获取天气信息失败: {e}")
        return 0


if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    # my_car.task.reset()
    
    #my_car.task.arm.set(0,0)
    
####################################################################################################
    #my_car.task.arm.set_hand_angle(0)
    
    '''
    targets = [[2, 100, 'cylinder3', 0.927634596824646, 0.1109375, 0.6270833333333333, 0.609375, 0.7375], 
                   [9, 80, 'cylinder2', 0.9523223042488098, 0.040625, 0.6416666666666667, 0.5125, 0.7083333333333334], 
                   [10, 60, 'cylinder1', 0.9164451360702515, 0.065625, 0.625, 0.4125, 0.6833333333333333]]
                   
    my_car.lane_det_location_plant(0.15, targets, side=-1, dis_out=1.8)
    '''
    
    '''
    my_car.task.arm.set_hand_angle(-90)
    time.sleep(1)
    my_car.task.arm.set_hand_angle(70)
    '''
    
        
    '''
    tar_list = planting(my_car, radius, side,arm_set=True)

    my_car.lane_det_location_plant(0.15, tar_list, side=side * -1, dis_out=1.8)
    my_car.task.planting(my_car, radius, side)
    '''
    
    # my_car.task.arm.set(0.16, 0)
    # my_car.task.arm.set(0.07, 0)
    # my_car.task.arm.switch_side(1)
    # side = 1
    
    

    ######################################################################################
    
    '''     
    # 浇花1
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
        my_car.set_pose_offset([0, 0, -math.pi / 20 * side], 0.5)  ###
        time.sleep(0.3)
        my_car.set_pose_offset([0.28, 0, 0])
 
 
    side *= -1
    plants(side)
    '''
    
    '''
    #天气预报
    side=-1
    
    # my_car.set_pose_offset([0.15, 0, 0], 0.6)
    my_car.set_pose_offset([0.33, 0, 0], 1.8)
    weather()
    '''
    
    
    
    '''
    #浇花2
    my_car.set_pose_offset([-0.2, 0, 0])    # 倒退
    my_car.lane_dis_offset(0.3, 2)    # 巡航

    my_car.set_pose_offset([0, -0.04, 0])

    my_car.lane_sensor(0.2, value_h=0.2, sides=1)  # 红外
    my_car.set_pose_offset([0, 0, math.pi / 25])  # 修正
    
    my_car.task.arm.set(0.07, 0)
    my_car.task.arm.switch_side(1)    #恢复手臂初始位置
    
    my_car.set_pose_offset([0.2, 0, 0],0.7)    #准备开始识别
    
    side = 1
    plants(side)
    '''
    
    
    '''
    #camp
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
    '''
    
    
    '''
    #浇花3
    my_car.set_pose_offset([0, 0, -math.pi / 30], 0.5)
    my_car.lane_sensor(0.25, value_h=0.2, sides=1)
    
    ##my_car.task.arm.set(0.07, 0)    #从这段开始运行记得加手臂初始化动作
    ##my_car.task.arm.switch_side(1)
    
    my_car.set_pose_offset([0.17, 0, 0], 1)
   
    side = 1
    plants(side)
    '''
    
    
    '''
    #eject1
    my_car.lane_dis_offset(0.3, 1.2)
    my_car.set_pose_offset([0, 0, math.pi / 25], 1)
    my_car.task.eject(1)
    time.sleep(0.5)
    '''
    
    
    
    #送药
   # my_car.set_pose_offset([0, 0.15, 0], 1)
    # my_car.set_pose_offset([0, 0, -math.pi/1.55], 2)
    # my_car.task.eject(2)
    
    
    
    #送药衔接
    '''
    #eject
    my_car.lane_dis_offset(0.3, 0.1)
    my_car.lane_sensor(0.27, value_h=0.6, sides=-1)
    my_car.set_pose_offset([0, 0, -math.pi / 30], 1)
    my_car.set_pose_offset([-0.05, 0, 0], 1)
    
    
    
    
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
    ]

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
        print(answer_dish)
        answer_dish = answer_dish["answer"]
        answer_dish = int(answer_dish)
    except Exception as e:
        print("/n", e, "/n")
        answer_dish = 3
    # my_car.set_pose_offset([-0.1,0,0])#向后退一点

    if answer_dish == 1 or answer_dish == 3:
        if answer_dish == 1:
            back = -0.24
        if answer_dish == 3:
            back = -0.115
        # 放上栏 第一个
        # my_car.set_pose_offset([back, 0, 0])
        my_car.set_vel_time(back / 0.5, 0, 0, 0.5)
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
        my_car.task.arm.set(0.266, 0.03)
    '''
    '''
    my_car.set_pose_offset([0, 0.11, 0], 1)
    my_car.set_pose_offset([0, 0, -math.pi/1.55], 2)
    my_car.task.eject(3)
    
    my_car.set_pose_offset([0, 0, math.pi/1.55], 2)
    my_car.set_pose_offset([0, -0.11, 0], 1)
    '''
    '''
    my_car.lane_dis_offset(0.3, 0.1)
    my_car.lane_sensor(0.27, value_h=0.6, sides=-1)
    my_car.set_pose_offset([0, 0, -math.pi / 30], 1)
    '''
    
    my_car.task.arm.switch_side(-1)
    
    
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
    
    