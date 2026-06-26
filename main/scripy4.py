# -*- coding: utf-8 -*-
import sys
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
from ernie_bot.base import answer


index_form = {
    0: 'turn_right',
    1: 'tofu',
    2: 'chili',
    3: 'turn_left',
    4: 'cylinder1',
    5: 'tomato',
    6: 'potato',
    7: 'cylinder3',
    8: 'cylinder2',
    9: 'text_det',
    10: 'chicken',
    11: 'meat',
    12: 'celery',
    13: 'egg',
    14: 'mushroom',
    15: 'green_beans',
    16: 'cauliflower',
    17: 'greens'
}



# ??????????
def get_key_by_value(d, value):
    for k, v in d.items():
        if v == value:
            return k
    return None

class MyCarTask:
    def __init__(self):
        self.my_car = MyCar()
        self.my_car.STOP_PARAM = False
        
    class MyCarTask:
    def __init__(self):
        self.my_car = MyCar()
        self.my_car.STOP_PARAM = False

    def front(self):
        #self.my_car.task.arm.set(0, 0)
        start_car_position=self.my_car.get_odometry()
        print(start_car_position)
        self.my_car.set_pose_offset([0.7, 0, 0], 3)
        self.my_car.task.arm.set_hand_angle(90)
        # my_car.task.arm.set(0.1, 0)
        # time.sleep(0.2)

        side = self.my_car.get_card_side()
        print(side)

        if side == -1:
            '''
            self.my_car.task.arm.set(0.07, 0)
            self.my_car.task.arm.switch_side(1)
            # my_car.task.arm.set_arm_angle(-96)
            self.my_car.set_pose_offset([0, 0, math.pi / 4 * side], 1)
            time.sleep(0.3)
            


            #self.my_car.lane_dis_offset(0.2, 0.58)
            ##self.my_car.set_pose_offset([0.1, 0, 0])
            
            #self.my_car.set_pose_offset([0, -0.035, 0])
            #self.my_car.set_pose_offset([0, 0, -math.pi / 28 * side])
            #time.sleep(0.3)
            #self.my_car.set_pose_offset([0.28, 0, 0])
            my_car.set_pose_offset([0.7, 0, 0], 3)
            '''
            self.my_car.task.arm.set_hand_angle(90)
            # my_car.task.arm.set(0.1, 0)
            # time.sleep(0.2)
            self.my_car.task.arm.set(0.04, 0)
            self.my_car.task.arm.switch_side(1)
            # my_car.task.arm.set_arm_angle(-96)
            self.my_car.set_pose_offset([0, 0, -math.pi / 4 ], 1)
            time.sleep(0.3)
            self.my_car.set_pose_offset([0.18, 0, 0], 1)
            self.my_car.set_pose_offset([0, 0, math.pi / 10], 0.5)
            self.my_car.set_pose_offset([0.12, 0, 0], 1)
            self.my_car.set_pose_offset([0, -0.02, 0], 0.5)
            self.my_car.set_pose_offset([0.3, 0, 0],1.5)
            self.my_car.set_pose_offset([0, 0, math.pi /6.5], 0.6)
            #self.my_car.set_pose_offset([0, -0.03, 0], 0.1)
            self.my_car.set_pose_offset([0.23, 0, 0], 1.5)

            length=0.40
            
        else:
            
            self.my_car.task.arm.set(0.21, 0)
            self.my_car.task.arm.switch_side(-1)
            
            #self.my_car.task.arm.set_arm_angle(90)
            self.my_car.set_pose_offset([0, 0, math.pi / 4 * side], 1)
            time.sleep(0.3)
            '''
            self.my_car.set_pose_offset([0.18, 0, 0], 1)
            self.my_car.set_pose_offset([0, 0, -math.pi / 10], 0.5)
            self.my_car.set_pose_offset([0.12, 0, 0], 1)
            self.my_car.set_pose_offset([0, 0.04, 0], 0.5)
            self.my_car.set_pose_offset([0.3, 0, 0],1.5)
            self.my_car.set_pose_offset([0, 0, -math.pi /6.8], 0.6)
            #self.my_car.set_pose_offset([0, -0.03, 0], 0.1)
            self.my_car.set_pose_offset([0.23, 0, 0], 1.5)
            self.my_car.set_pose_offset([0, 0.03, 0], 0.5)    '''

            self.my_car.lane_dis_offset(0.2, 0.58)
            self.my_car.set_pose_offset([0, 0.035, 0],0.5)
            self.my_car.set_pose_offset([0, 0, -math.pi / 12.7 * side],0.5)
            time.sleep(0.3)
            self.my_car.set_pose_offset([0.28, 0, 0])
            
            length=0.39
            # my_car.set_pos_offset([0, 0, math.pi / 5.2 * -1], 1)

        # my_car.task.arm.set(0.07, 0)
        # my_car.task.arm.switch_side(1)
        # my_car.task.arm.switch_side(1)
        # my_car.task.arm.set_hand_angle(90)
        # time.sleep(1)
        # side=-1
        cylinder_id = 1
        print("before pts")
        pts = self.my_car.task.pick_up_cylinder(cylinder_id, side, True)
        print("after pts")
        pos_start = np.array(self.my_car.get_odometry())
        pos_zone = pos_start.copy()
        pos_zone[0] += length
        print("prepare for location")
        

        pose_dict = self.my_car.lane_det_location_v8_multi(0.2, pts, side=side * -1, dis_out=2)
        print(pose_dict)

        for i in range(3):
            if i == 0:
                run_dis = self.my_car.calculation_dis(np.array(pose_dict[100]), np.array(pos_zone))
                self.my_car.set_pose(pose_dict[100],1)
            if i == 1:
                run_dis = self.my_car.calculation_dis(np.array(pose_dict[80]), np.array(pos_zone))
                self.my_car.set_pose(pose_dict[80],1)
            if i == 2:
                run_dis = self.my_car.calculation_dis(np.array(pose_dict[60]), np.array(pos_zone))
                self.my_car.set_pose(pose_dict[60],1)

            self.my_car.task.pick_up_cylinder(i, side)
            self.my_car.set_pose_offset([run_dis, 0, 0])
            self.my_car.task.put_down_cylinder(i, side)
        self.my_car.set_pose_offset([-0.1, 0, 0])

    

        #########bmi
        # my_car.task.arm.set(0,0)
        # my_car.task.arm.set_offset(0, 0-0.13)
        # my_car.lane_dis_offset(0.2, 15)
        self.my_car.task.arm.switch_side(1)
        self.my_car.task.arm.set(0.13, 0)
        self.my_car.set_pose_offset([-0.2, 0, 0])
        self.my_car.lane_dis_offset(0.3, 2)
        self.my_car.lane_sensor(0.2, value_h=0.2, sides=1)
        self.my_car.set_pose_offset([0, 0, math.pi/17])
        self.my_car.set_pose_offset([0, -0.03, 0], 0.5)   ###[0, -0.05, 0]
        self.my_car.set_pose_offset([0.15, 0, 0]) 

        # t = threading.Thread(target=task)
        # t.start()
        # t.join(timeout=3)
        self.my_car.set_pose_offset([0, 0.095, 0], 2)
        self.my_car.set_pose_offset([0, -0.08, 0], 2)
        self.my_car.set_pose_offset([0.1, 0, 0], 1)
        try:
            text = self.my_car.get_ocr_list()[0]
            print(text)
            # height, weight = extract_height_weight(text)
            # out = calculate_bmi(height, weight)
            # text=my_car.get_ocr_list()[0] #获取文字
            bmi_degree = answer.ask3(text)
            print(bmi_degree)
            out = bmi_degree['answer']
            out = int(out)
            self.my_car.task.bmi_set(out)
        except Exception as e:
            print(f"error: {e}")

        ######## camp
        # my_car.task.arm.set(0,0)
        # my_car.task.arm.grap(1)
        # my_car.task.arm.set_hand_angle(80)
        angle_offset = -math.pi / 2 * 0.82
        # dis_angle = -math.pi/2*0.3
        # dis = 1.
        dis_x = 1.36
        dis_y = -0.87
        # print(dis_x, dis_y)
        angle_now = self.my_car.get_odometry()[2]
        x_offset = dis_x * math.cos(angle_now) - dis_y * math.sin(angle_now)
        y_offset = dis_y * math.cos(angle_now) + dis_x * math.sin(angle_now)
        angle_tar = angle_now - math.pi * 2 + angle_offset
        pose = self.my_car.get_odometry().copy()
        pose[0] = pose[0] + x_offset
        pose[1] = pose[1] + y_offset
        pose[2] = angle_tar
        # print(pose)
        # return

        self.my_car.lane_sensor(0.3, value_h=1, sides=-1)
        # time.sleep(25)
        # my_car.lane_sensor()
        
        self.my_car.task.arm.set(0.265, 0)       ############
        
        self.my_car.lane_dis_offset(0.3, 0.5)

        self.my_car.set_vel_time(0.3, 0, -0.5, 4)
        self.my_car.lane_dis_offset(0.3, 4.5)
        #my_car.lane_dis_offset(0.3, 4.58)
        self.my_car.set_pose_offset([0, 0, -math.pi /14], 1)
        self.my_car.lane_dis_offset(0.3, 0.10)
        #self.my_car.set_pose_offset([-0.05, 0, 0])


        ######## eject1
        #finetune_car_position=self.my_car.get_odometry()
        #print(finetune_car_position)
        #print(finetune_car_position[2]-start_car_position[2])
        #self.my_car.set_pose_offset([0, 0, (finetune_car_position[2]-start_car_position[2]) ] , 0.2)
        
        self.my_car.task.eject(1)
        time.sleep(0.5)
        #self.my_car.set_pose_offset([0.7, 0, 0],0.5)
        self.my_car.lane_dis_offset(0.3, 0.7)   

        print("finish camp task!!!!!")

    def end(self):
        self.my_car.task.arm.switch_side(-1)  # 手臂向右
        self.my_car.task.arm.set(0.265, 0)  # 手臂向后退到能看到文字的位置
        tar = [9, 0, 'text_det', 0, 0.0390625, 0.7916666666666666, 0.37, 0.39166666666666666]
        # time.sleep(1)
        #self.my_car.set_velocity(0.3, 0, 0)
        time.sleep(0.5)
        self.my_car.lane_det_location_v4(0.2, tar, side=-1, dis_out=4)  # 巡航找文字
        self.my_car.lane_det_location_v4(0.2, tar, side=-1, dis_out=4) 
        
        try:
            text = self.my_car.get_ocr_list_plus()[0]  # 获取文字
            foods = answer.ask1(text)
            the_food1 = foods['answer']
            print(the_food1)
        except Exception as e:
            print(e)
            the_food1 = 'egg'
        self.my_car.set_pose_offset([-0.25, 0, 0])  # 后退一步
        self.my_car.task.arm.set(0.135, 0)  # 调整手臂定位食材
        index = get_key_by_value(index_form, the_food1)
        tar = [index, 0, the_food1, 0.9151089787483215, 0, 0.2, 0.32, 0.7]
        self.my_car.lane_det_location_vert(0.15, tar, side=-1, dis_out=2)
        img_side = self.my_car.cap_side.read()
        dets_ret = self.my_car.task_det(img_side)
        print("object is", dets_ret)

        for det_ret in dets_ret:
            if det_ret[0] == index:
                y = det_ret[5]
        if y < 0 or y == 0:
            self.my_car.task.arm.grap(1)  # 吸
            self.my_car.task.arm.set(0.135, 0.1)  # 手臂上抬
            self.my_car.task.arm.set_hand_angle(-45)  # 掌心向前
            self.my_car.task.arm.set(0, 0.1)  # 向前抓
            time.sleep(0.5)
            self.my_car.task.arm.set(0.265, 0.1)  # 往后缩

            # time.sleep(0.5)
            # my_car.task.arm.set(0.265,0.03)#手臂向下放食材
            # my_car.task.arm.grap(0)#松手

        if y > 0:
            self.my_car.task.arm.grap(1)  # 吸
            self.my_car.task.arm.set(0.135, 0.01)  # 手臂下降
            self.my_car.task.arm.set_hand_angle(-45)  # 掌心向前
            self.my_car.task.arm.set(0, 0.03)  # 向前抓
            time.sleep(0.5)
            self.my_car.task.arm.set(0.265, 0.05)  # 往后缩


            # my_car.task.arm.set_hand_angle(90)#掌心向下
            # time.sleep(0.5)
            # my_car.task.arm.set(0.265,0.03)#手臂向下放食材
            # my_car.task.arm.grap(0)#松

        self.my_car.set_pose_offset([-0.5, 0, 0])  # 后退取第二个食材
        
        
        

        self.my_car.task.arm.switch_side(1)  # 手臂向左
        self.my_car.task.arm.grap(0)
        self.my_car.task.arm.set(0, 0)  # 达到摄像头能看到文字的位置
        tar = [9, 0, 'text_det', 0.9307849407196045, -0.0546875, 0.7645833333333333, 0.37, 0.30416666666666664]
        time.sleep(1)
        self.my_car.lane_det_location_v4(0.2, tar, side=1, dis_out=2)  # 巡航找文本
        try:
            text = self.my_car.get_ocr_list_plus()[0]  # 获取文字
            foods = answer.ask1(text)
            the_food2 = foods['answer']
            print(the_food2)
        except Exception as e:
            print(e)
            the_food2 = 'tomato'

        self.my_car.set_pose_offset([-0.25, 0, 0])  # 后退
        self.my_car.task.arm.set(0.135, 0)  # 调整手臂定位食材

        index = get_key_by_value(index_form, the_food2)
        tar = [index, 0, the_food2, 0.7483181953430176, 0, 0.2, 0.34, 0.7]
        print(f"tar:{tar}")
        # tar=[index, 0, answer, 0.5542138814926147, 0, 0.192, 0.18, 0.375]
        self.my_car.lane_det_location_vert(0.135, tar, side=1, dis_out=2)
        img_side = self.my_car.cap_side.read()
        dets_ret = self.my_car.task_det(img_side)
        print("检测目标是", dets_ret)

        for det_ret in dets_ret:
            if det_ret[0] == index:
                y = det_ret[5]
        if y < 0 or y == 0:
            self.my_car.task.arm.grap(1)  # 吸
            self.my_car.task.arm.set(0.135, 0.1)  # 手臂上抬
            self.my_car.task.arm.set_hand_angle(-45)  # 掌心向前
            self.my_car.task.arm.set(0.27, 0.1)  # 向前抓
            time.sleep(0.5)
            self.my_car.task.arm.set(0, 0.1)  # 往后缩
            # time.sleep(0.5)
            # my_car.task.arm.set_hand_angle(90)#掌心向下
            # time.sleep(0.5)

            # my_car.task.arm.set(0,0.03)#手臂向下放食材
            # my_car.task.arm.grap(0)#松手

        if y > 0:
            self.my_car.task.arm.grap(1)  # 吸
            self.my_car.task.arm.set(0.135, 0.01)  # 手臂下降
            self.my_car.task.arm.set_hand_angle(-45)  # 掌心向前
            self.my_car.task.arm.set(0.27, 0.01)  # 向前抓
            time.sleep(0.5)
            self.my_car.task.arm.set(0, 0.01)  # 往后缩
            # time.sleep(0.5)
            # my_car.task.arm.set_hand_angle(90)#掌心向下
            # time.sleep(0.5)
            # my_car.task.arm.set(0,0.03)#手臂向下放食材
            # my_car.task.arm.grap(0)#松
            
            
        self.my_car.set_pose_offset([0, -0.05, 0], 0.2)       
            
            
            
            
            
            
            
            
            
        
        ########
        #第三个答案
        self.my_car.lane_sensor(0.25, value_h=0.3, sides=1)
        self.my_car.set_pose_offset([0.29, 0, 0], 1)
        self.my_car.task.arm.grap(1)
        time.sleep(0.5)
        self.my_car.task.arm.set_hand_angle(-45)
        time.sleep(0.5)
        self.my_car.task.arm.set(0.1,0.08)
        time.sleep(0.5)
        self.my_car.task.arm.set(0.24,0.08)
        self.my_car.task.arm.set(0.1,0.08)
        
        
        ########
        self.my_car.lane_dis_offset(0.3, 0.1)
          
            
        self.my_car.lane_sensor(0.27, value_h=0.3, sides=-1)
        self.my_car.set_pose_offset([-0.15, 0, 0], 1)     ###([-0.1, 0, 0], 1) 
           
        self.my_car.task.eject(2)
        
        ########
        self.my_car.task.arm.grap(1)
        
        #the_food1 = 'egg'
        #the_food2 = 'tomato'
        # my_car.lane_dis_offset(0.3, 1)
        # 巡航找文字
        self.my_car.task.arm.switch_side(-1)
        
        self.my_car.task.arm.set(0.15, 0.01)  # 摄像头看得到两个文字的高度
        
        tar = [9, 0, 'text_det', 0.9494228959083557, 0.075, -0.2520833333333333, 0.4375, 0.4875]
        
        self.my_car.lane_det_location_v4(0.2, tar, side=-1, dis_out=2)  # 巡航到第一列有文字的地方
        time.sleep(0.5)
        try:
        
            text1 = self.my_car.get_ocr_list_plus()  # 获取文字
            print(text1)
        except Exception as e:
            print(e)
            text1 = ['表面裹着红亮的酱汁呈深褐色，表面略带光泽，酸甜味突出外酥里嫩，酱汁浓郁，口感层次丰富。',
                    '金黄色与鲜红色混合食材软烂出汁，整体呈现红黄相间的色彩。']
        
        # 识别文字，这个先不写
        
        self.my_car.set_pose_offset([0.4, 0, 0])  # 向前走点,不要看到第一列文字
        
        self.my_car.lane_det_location_v4(0.2, tar, side=-1, dis_out=2.5)  # 巡航到第二列有文字的地方
        time.sleep(1)
        
        try:
        
            text2 = self.my_car.get_ocr_list_plus()  # 获取文字
            print(text2)
        except Exception as e:
            print(e)
            text2 = ['表面油亮，整体呈现鲜亮的绿色，口感清脆，带有淡淡的蔬菜清甜，味道清淡爽口。',
                    '深绿色的切片与浅棕色的肉块混合，肉片略带焦香，带有明显的辛辣味，咸香嫩滑整体口感鲜辣开胃间的色彩。']
        
        text_all = the_food1 + ',' + the_food2 + ',1.' + text1[0] + ',2.' + text1[1] + ',3.' + text2[0] + ',4.' + text2[1]
        
        try:
            answer_dish = answer.ask2(text_all)
            print(answer_dish)
            answer_dish = answer_dish['answer']
            int(answer_dish)
        except Exception as e:
            print(e)
            answer_dish = 3
        # my_car.set_pose_offset([-0.1,0,0])#向后退一点
        
        
        if answer_dish == 1 or answer_dish == 3:
            if answer_dish == 1:
                back = -0.29
            if answer_dish == 3:
                back = -0.1
            # 放上栏 第一个
            self.my_car.set_pose_offset([back, 0, 0])
            self.my_car.task.arm.grap(1)  # 吸
            time.sleep(3)
            self.my_car.task.arm.set_hand_angle(-45)  # 手心向前
            self.my_car.task.arm.set(0.1, 0.132)  # 抬起手臂
            self.my_car.task.arm.set(0, 0.132)  # 伸手去放物品
            self.my_car.task.arm.grap(0)  # 松手
            self.my_car.task.arm.set(0.266, 0.132)  # 缩手
        if answer_dish == 2 or answer_dish == 4:
            if answer_dish == 2:
                back = -0.29
            if answer_dish == 4:
                back = -0.1
            # 放下栏 第一个
            self.my_car.set_pose_offset([back, 0, 0])
            self.my_car.task.arm.set(0.15, 0.03)
            self.my_car.task.arm.grap(1)  # 吸
            time.sleep(3)
            self.my_car.task.arm.set_hand_angle(-45)  # 手心向前
            self.my_car.task.arm.set(0.1, 0.03)  # 压低手臂
            self.my_car.task.arm.set(0, 0.03)  # 伸手去放物品
            self.my_car.task.arm.grap(0)  # 松手
            self.my_car.task.arm.set(0.266, 0.03)  # 缩手
        
        ##############
        self.my_car.set_pose_offset([-0.2, 0, 0], 2)
        self.my_car.lane_dis_offset(0.3, 2)
        self.my_car.lane_sensor(0.2, value_h=0.3, sides=1)
        self.my_car.set_pose_offset([0.05, 0, 0], 1)
        self.my_car.set_pose_offset([0, 0.03, 0], 2)
        time.sleep(0.5)
        self.my_car.set_pose_offset([-0.02, 0.33, 0], 0.5)   ####([-0.02, 0.3, 0], 0.5)
        time.sleep(0.5)
        self.my_car.set_pose_offset([0, -0.23, 0], 2)
        time.sleep(0.5)
        self.my_car.lane_dis_offset(0.3, 0.7)

if __name__ == "__main__":
    a=MyCarTask()
    a.front()
    a.end()