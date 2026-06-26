from vehicle import ArmBase, ScreenShow, Key4Btn, ServoBus, ServoPwm, MotorWrap, StepperWrap, PoutD,Beep
import cv2
import time
import numpy as np
import yaml, os, math


class Ejection():
    def __init__(self, portm=5, portd=4, port_step=1) -> None:
        self.motor = MotorWrap(portm, -1, type="motor_280", perimeter=0.06/15*8)
        self.pout = PoutD(portd)
        self.step1 = StepperWrap(port_step)
        self.step_rad_st = self.step1.get_rad()
        self.step1_rad_cnt = 0

    def reset(self, vel=0.05):
        rad_last = self.motor.get_rad()
        
        while True:
            self.motor.set_linear(vel)
            time.sleep(0.02)
            rad_now = self.motor.get_rad()
            if abs(rad_now - rad_last) < 0.02:
                break
            rad_last = rad_now
        
        self.motor.set_linear(0)
        
    def eject(self, x=0.5, vel=0.05):  #####
        self.reset()
        self.pout.set(1)
        self.motor.reset()
        self.motor.set_linear(0-abs(vel))
        length = 0.105   #####0.08       ####0.105
        while True:
            self.motor.set_linear(0-abs(vel))
            if abs(self.motor.get_dis()) > length:
                break
        self.motor.set_linear(0)
        self.step1_rad_cnt += 1
        self.step1.set_rad(math.pi/5*4*self.step1_rad_cnt + self.step_rad_st)
        self.pout.set(1)
        time.sleep(1)
        while True:
            self.motor.set_linear(abs(vel))
            if abs(self.motor.get_dis()) < x:
                break
        self.motor.set_linear(0)
        self.pout.set(0)


    
class MyTask:
    def __init__(self):
    
        # 旋转舵机
        # self.servo_bmi = ServoBus(2)
        self.servo_weather = ServoBus(2)
        # self.servo_weather.set_speed(0)
        # self.servo_rotate.set_angle(90, 0)
        
        #蜂鸣器
        self.ring=Beep()
        
        # 发射装置
        self.ejection = Ejection()
        time.sleep(3)   #####

        # 机械臂
        self.arm = ArmBase()
        
        self.servo_weather.set_angle(0)

    def reset(self):
        self.arm.reset()
        
        
    def beep(self):
        self.ring.rings()
        time.sleep(0.2)
        
        

    # 抓圆柱，选则大小
    '''def pick_up_cylinder(self, radius, arm_set=False):
        # 定位目标的参数 label_id, obj_width, label, prob, err_x, err_y, width, height
        tar_list =  [[13, 100, "cylinder1", 0,  0, 0.28, 0.75, 0.97], [14, 80, "cylinder2", 0, 0, 0.3, 0.61, 0.9], 
                     [15, 60, "cylinder3", 0, 0, 0.2, 0.45, 0.7]]
        # pt_tar = tar_list[radius-1]
        #height_list = [0.08, 0.08, 0.15]
        tar_height =0.02
        tar_horiz = self.arm.horiz_mid
        # 手臂方向向下
        self.arm.set_hand_angle(90)
        if arm_set:
            tar_height = 0.045
            # 到达目标位置
            self.arm.set(tar_horiz, tar_height)
            return tar_list
        # 抓取圆柱
        self.arm.grap(1)
        # 到圆柱的位置
        #horiz_offset = 0.06 * self.arm.side#中间位置
        if radius == 0:
            # self.arm.set(tar_horiz, tar_height+0.04)
            self.arm.set_offset(0, 0.04)
        #tar_horiz = self.arm.horiz_mid + horiz_offset
        self.arm.set(0.25, 0.11)
        # 往下放,抓住
        self.arm.set_offset(0, -0.015)
        time.sleep(0.5)
        # 抬起一定高度
        # height_offset = 0.07
        height_offset = 0.07
        if radius == 2:
            height_offset += 0.03
        self.arm.set_offset(0, height_offset)
        # self.arm.set_offset(0, 0.08, 1.3)'''
        
    

    
    
    
    def planting(self, side, radius=None, arm_set=False):
        # cylinder3: 大圆柱体   缺乏水分植物    浇水模拟    radius=2
        # cylinder2: 中圆柱体   健康植物    蜂鸣示意    radius=1
        # cylinder1: 小圆柱体   缺乏日照植物    转盘补光模拟    radius=0
    
        if side==-1:
            targets =[[2, 100, 'cylinder3', 0.9106670022010803, 0.0046875, 0.5895833333333333, 0.584375, 0.7208333333333333],
                        [9, 80, 'cylinder2', 0.9157900810241699, 0.01875, 0.5604166666666667, 0.475, 0.6541666666666667],
                        [10, 60, 'cylinder1', 0.9004784822463989, -0.0140625, 0.55625, 0.340625, 0.5958333333333333]]
    
        else:
            targets =[[2, 100, 'cylinder3', 0.927634596824646, 0.1109375, 0.6270833333333333, 0.609375, 0.7375], 
                       [9, 80, 'cylinder2', 0.9523223042488098, 0.040625, 0.6416666666666667, 0.5125, 0.7083333333333334], 
                       [10, 60, 'cylinder1', 0.9164451360702515, 0.065625, 0.625, 0.4125, 0.6833333333333333]]
    
        if arm_set:
            return targets  
        
    
    
        if radius is not None:
            if radius==0:      #转盘补光
                '''
                self.servo_weather.set_speed(50)
                time.sleep(5)
                self.servo_weather.set_speed(0)  
                '''
                servo_bus =  ServoBus(2)  
                servo_bus.set_angle(180)
                time.sleep(1)
                # servo_bus.set_angle(100)
                # time.sleep(0.5)
                servo_bus.set_angle(0)
                
                                              
            if radius==1:    #蜂鸣示意
                self.beep()    
                time.sleep(0.5)
                self.beep()
            if radius ==2:    #浇水                    
                if side==-1:             
                    self.arm.set(0.12,0.16)
                    self.arm.set(0.02,0.16)
                    self.arm.set_hand_angle(-90)        
                    time.sleep(1)
                    self.arm.set_hand_angle(70)
                if side == 1:
                    self.arm.set(0.15,0.16)
                    self.arm.set(0.24,0.16)
                    self.arm.set_hand_angle(-90)
                    time.sleep(1)
                    self.arm.set_hand_angle(70)
        
        
        
        
    
    def pick_up_cylinder(self, radius, side,arm_set=False):
        if side==-1:
            tar_list =[[2, 100, 'cylinder3', 0.9106670022010803, 0.0046875, 0.5895833333333333, 0.584375, 0.7208333333333333]
, [9, 80, 'cylinder2', 0.9157900810241699, 0.01875, 0.5604166666666667, 0.475, 0.6541666666666667],[10, 60, 'cylinder1', 0.9004784822463989, -0.0140625, 0.55625, 0.340625, 0.5958333333333333]]

        if side==1:
            tar_list =[[2, 100, 'cylinder3', 0.927634596824646, 0.1109375, 0.6270833333333333, 0.609375, 0.7375], [9, 80, 'cylinder2', 0.9523223042488098, 0.040625, 0.6416666666666667, 0.5125, 0.7083333333333334], [10, 60, 'cylinder1', 0.9164451360702515, 0.065625, 0.625, 0.4125, 0.6833333333333333]]           
        if arm_set:
            return tar_list   
        if side==-1: 
            if radius==0:             
                self.arm.set(0.07,0.1)
            self.arm.set(0.255,0.11)
            self.arm.grap(1)  
            self.arm.set(0.255, 0.055)  ###
            #self.arm.set_offset(0, -0.06)
            time.sleep(0.3)  
            
            if radius==0:
                self.arm.set(0.255,0.16)
            if radius==1:
                # self.arm.set(0.255,0.152)     #########arm.set(0.27,0.18)
                self.arm.set(0.255,0.175)
                time.sleep(0.5)
            if radius ==2:
                self.arm.set(0.255,0.215)
                self.arm.set(0.255,0.215)
        if side==1:
            if radius==0:
                self.arm.set(0.16,0.1)
            self.arm.set(0,0.11)
            #if radius==2:
                #my_car.set_pose_offset([0, 0, 0], )
            self.arm.grap(1)
            #self.arm.set_offset(0, -0.03)
            self.arm.set(0, 0.055)
            time.sleep(0.3) 
                
            if radius==0:
                self.arm.set(0,0.16)       #########arm.set(0,0.18)
            if radius==1:
                #self.arm.set(0,0.152)
                self.arm.set(0,0.175)
                time.sleep(0.5)         
            if radius ==2:
                self.arm.set(0,0.215)
                self.arm.set(0,0.215)

    def put_down_cylinder(self, radius,side):
        if side==-1:
            if radius==0:
                self.arm.set(0.255,0.085)  
                self.arm.grap(0)
                self.arm.set(0.255,0.11)       #####
                time.sleep(0.5)
            if radius==1:
                #self.arm.set(0.255,0.21)    ########delete                
                ##self.arm.set(0.255,0.13)
                self.arm.set(0.255,0.14)
                self.arm.grap(0)
                time.sleep(0.5)
                self.arm.set(0.255,0.18)
            if radius==2:
                self.arm.set(0.255,0.195)
                self.arm.set(0.255,0.195)
                self.arm.grap(0)
                time.sleep(0.5)
                self.arm.set(0.255,0.215)
        if side==1:
            if radius==0:
                self.arm.set(0,0.085)
                self.arm.grap(0)
                self.arm.set(0,0.11)         #####
                time.sleep(0.5)
            if radius==1:
                #self.arm.set(0,0.21) 
                ##self.arm.set(0,0.13)
                self.arm.set(0,0.14)
                self.arm.grap(0)
                time.sleep(0.5)
                self.arm.set(0,0.18)
            if radius==2:
                self.arm.set(0,0.195)
                self.arm.set(0,0.195)
                self.arm.grap(0)
                time.sleep(0.5)
                self.arm.set(0,0.215)


        '''def pick_up_cylinder_v2(self, radius, arm_set=False):
            # 定位目标的参数 label_id, obj_width, label, prob, err_x, err_y, width, height
            tar_list =  [[16, 100, "cylinder3", 0, -0.0094, 0.1292, 0.7312, 0.9167], [17, 80, "cylinder2", 0, 0.0344, 0.1437, 0.6375, 0.8458], 
                         [15, 60, "cylinder1", 0,  0.0297, 0.0917, 0.4594, 0.775]]
                         
            if arm_set:
                return tar_list   
                          
            my_car.task.arm.set(0.1,0.1)
            my_car.task.arm.set(0.27,0.1)
            my_car.task.arm.grap(1)
            my_car.task.arm.set_offset(0, -0.032)
            time.sleep(0.3)  
                
            if radius==0:
                my_car.task.arm.set(0.27,0.18)
            if radius==1:
                my_car.task.arm.set(0.27,0.18)
            if radius ==2:
                my_car.task.arm.set(0.27,0.205)


        def put_down_cylinder(self, radius):
        
            if radius==0:
                my_car.task.arm.set(0.27,0.1)
                my_car.task.arm.grap(0)
                time.sleep(0.5)
            if radius==1:
                my_car.task.arm.set(0.27,0.12)
                my_car.task.arm.grap(0)
                time.sleep(0.5)
                my_car.task.arm.set(0.27,0.18)
            if radius==2:
                my_car.task.arm.set(0.27,0.18)
                my_car.task.arm.grap(0)
                time.sleep(0.5)
                my_car.task.arm.set(0.27,0.205)'''
                
            
    '''def put_down_cylinder(self, radius):
        # tar_height = 0.02
        height_offset = 0.005
        if radius==0:
            height_offset = 0.11
        # 下放放开物块
        self.arm.set_offset(0, 0-height_offset)
        # time.sleep(0.2)
        self.arm.grap(0)
        time.sleep(0.5)
        # 抬起
        self.arm.set_offset(0, 0.02)
        # horiz_offset = 0.1 * self.arm.side * -1
        # self.arm.set_offset(horiz_offset, 0)
    def put_down_cylinder(self, radius):
        # tar_height = 0.02
        height_offset = 0.02
        if radius==0:
            height_offset = 0.08
        # 下放放开物块
        self.arm.set_offset(0, 0-height_offset)
        # time.sleep(0.2)
        self.arm.grap(0)
        time.sleep(0.5)
        # 抬起
        self.arm.set_offset(0, 0.02)
        # horiz_offset = 0.1 * self.arm.side * -1
        # self.arm.set_offset(horiz_offset, 0)'''
        
        
    def weather_set(self, num=0, arm_set=False):
        tar = [[0, 70, 'text_det', 0, 0, -0.31, 0.85, 1.0]]
        weather_status = {0:0, 1:-45, 2: -135, 3:45, 4:135}
        tar_height = 0.045
        tar_horiz = 0.15
        if arm_set:
            self.arm.set_hand_angle(48)
            self.arm.set(tar_horiz, tar_height)
            return tar
        self.servo_weather.set_angle(weather_status[num])
    
    def bmi_set(self, num=0, arm_set=False):
        tar = [[0, 70, 'text_det', 0, 0, -0.31, 0.85, 1.0]]
        bmi = {0:0, 1:-45, 2: -135, 3:45, 4:135}
        tar_height = 0.045
        tar_horiz = 0.15
        if arm_set:
            self.arm.set_hand_angle(48)
            self.arm.set(tar_horiz, tar_height)
            return tar
        self.servo_bmi.set_angle(bmi[num])
        # self.servo_bmi.set_angle(0)

    def get_ingredients(self, side=1, ocr_mode=False, arm_set=False):
        tar =  [5, 0, 'text_det', 0.8996267914772034, 0.0390625, 0.22083333333333333, 0.496875, 0.4583333333333333]

        if ocr_mode:
            tar_height = 0.0
        else:
            tar_height = 0.07

        tar_horiz = self.arm.horiz_mid
        self.arm.set_hand_angle(48)
        self.arm.switch_side(side)
        self.arm.set(tar_horiz, tar_height)

        if arm_set:
            return tar
        # self.arm.switch_side(side)
        # self.arm.set_offset(0.1, 0)

    def pick_ingredients(self, num=1, row=1, arm_set=False):
        """tar = [
            [1, 30, 'tofu', 0, 0, 0.03, 0.25, 0.24],
            [2, 30, 'tomato', 0, 0, 0.03, 0.25, 0.24],
            [3, 30, 'chili', 0, 0, 0.03, 0.25, 0.24],
            [4, 30, 'chicken', 0, 0, 0.03, 0.25, 0.24],
            [5, 30, 'meat', 0, 0, 0.03, 0.25, 0.24],
            [6, 30, 'celery', 0, 0, 0.03, 0.25, 0.24],
            [7, 30, 'egg', 0, 0, 0.03, 0.25, 0.24],
            [8, 30, 'mushroom', 0, 0, 0.03, 0.25, 0.24],
            [9, 30, 'green_beans', 0, 0, 0.03, 0.25, 0.24],
            [10, 30, 'potato', 0, 0, 0.03, 0.25, 0.24],
            [11, 30, 'cauliflower', 0, 0, 0.03, 0.25, 0.24],
            [12, 30, 'greens', 0, 0, 0.03, 0.25, 0.24]
        ]"""
        tar = [[11, 30, 'chicken', 0, 0, 0.03, 0.25, 0.24], [8, 30, 'tomato', 0, 0, 0.03, 0.25, 0.24], [13, 30, 'egg', 0, 0, -0.15, 0.22, 0.22] ]
        # 计算高度，手臂根据高度设置位置
        tar_height = 0.04 + (row-1)*0.09
        horiz_offset = 0 * self.arm.side
        tar_horiz = self.arm.horiz_mid + horiz_offset
        
        self.arm.set(tar_horiz+0.02, tar_height)
        # 准备抓取
        self.arm.grap(1)
        time.sleep(0.5)
        # 如果是进行识别，这里手向下
        if arm_set:
            self.arm.set_hand_angle(45)
            return tar
        # 手水平
        self.arm.set_hand_angle(-45)
        time.sleep(0.5)
        # 手臂向外伸，去抓取物块
        horiz_offset = 0.13*self.arm.side
        self.arm.set_offset(horiz_offset, 0)
        
        # self.arm.set(0.26, 0.10)
        if num > 1:
            # 第二块保持住不动
            self.arm.set_offset(-0.18*self.arm.side, 0.02, speed=[0.12, 0.04])
            return tar
        # 收回手臂
        # self.arm.set_offset(-0.14*self.arm.side, 0.03, speed=[0.12, 0.04])
        self.arm.set(tar_horiz, 0.05)
        # 手向下
        self.arm.set_hand_angle(80)
        # 放下物块
        self.arm.set_offset(-0.13*self.arm.side, 0, speed=[0.12, 0.04])
        # self.arm.set(0.14-self.arm.side*0.14, 0.04, speed=[0.08, 0.04])
        
        self.arm.set_offset(0, -0.045, speed=[0.12, 0.04])
        # time.sleep(0.5)
        self.arm.grap(0)
        time.sleep(0.5)
        self.arm.set_offset(0, 0.045, speed=[0.12, 0.04])

    def get_answer(self, arm_set=False):
        tar = [[0, 70, 'text_det', 0, 0, 0.32, 0.23, 0.24]]
        self.arm.grap(1)
        self.arm.switch_side(1)
        self.arm.set_hand_angle(48)
        tar_height = 0.1
        tar_horiz = self.arm.horiz_mid
        
        self.arm.set(tar_horiz, tar_height)
        if arm_set:
            return tar
        # 竖着向下为-45
        self.arm.set_hand_angle(-45)
        self.arm.set_offset(0.09, 0)
        self.arm.set_offset(-0.09, 0)


    def set_food(self, num=1, row=1, arm_set=False):
        # 定位目标的参数 label_id, obj_id, label, prob, err_x, err_y, width, height
        tar = [[0, 70, 'text_det', 0, 0, -0.14, 0.46, 0.53]]
        # 气泵吸气并关闭阀门，调整手臂方向向右

        self.arm.grap(1)
        self.arm.switch_side(-1)

        if arm_set:
            # 准备识别的位置，手朝向下
            self.arm.set_hand_angle(45)
            tar_height = 0.12
            tar_horiz = self.arm.horiz_mid
            # 到达准备位置
            self.arm.set(tar_horiz, tar_height)
            return tar
        

        if num > 1:
            # 如果放的不是第一个，需要先抓取，手朝向下
            self.arm.set_hand_angle(45)
            # 到达抓取位置，准备抓取
            self.arm.set(self.arm.horiz_mid+0.14, 0.04, speed=[0.12, 0.04])
            # 向下移动抓取
            self.arm.grap(1)
            self.arm.set_offset(0, -0.04, speed=[0.12, 0.04])
            time.sleep(0.5)
            # 向上移动
            self.arm.set_offset(0, 0.05, speed=[0.12, 0.04])
            # 手臂指向方向调整水平
            self.arm.set_hand_angle(-45)
        self.arm.set_hand_angle(-45)
        # 根据目标位置调整手臂位置
        tar_height = 0.02 + (row-1)*0.1
        horiz_offset = 0 * self.arm.side
        #  准备放食材到指定位置
        tar_horiz = self.arm.horiz_mid + horiz_offset
        self.arm.set(tar_horiz, tar_height)
        # 手臂向前伸运动0.14m
        self.arm.set_offset(0.13*self.arm.side, 0, speed=[0.12, 0.04])
        # self.arm.set()
        # self.arm.set_hand_angle(-45)
        # self.arm.set_offset(-0.09, 0)
        self.arm.grap(0)
        time.sleep(0.5)
        self.arm.set_offset(-0.1*self.arm.side, 0, speed=[0.12, 0.04])

    def eject(self, area=1):
        dis_list = {1:0.080, 2:0.0535,3:0.050}   ####{1:0.080, 2:0.0535,3:0.050}
        self.ejection.eject(dis_list[area])
   
    def help_peo(self, arm_set=False):
        # 调整方向向左
        self.arm.switch_side(1)
        # 调整手水平
        self.arm.set_hand_angle(-45)
        tar_height = 0.08
        tar_horiz = self.arm.horiz_mid
        self.arm.set(tar_horiz, tar_height)
        if arm_set:
            return
        # 伸长手臂
        self.arm.set_offset(0.1, 0)
        self.arm.set_offset(-0.1, 0)



def task_reset():
    task = MyTask()
    task.reset()
    time.sleep(0.1)

def bmi_test():
    task = MyTask()
    task.bmi_set(0)

def cylinder_test():
    task = MyTask()
    key = Key4Btn(1)
    # task.arm.reset()
    i = 0
    tar = task.pick_up_cylinder(i, arm_set=True)
    while True:
        if key.get_key()!=0:
            
            time.sleep(1)
            task.pick_up_cylinder(i)
            time.sleep(1)
            task.put_down_cylinder(i)
            time.sleep(1)
            i = i+1
    # for i in range(3):
    #     tar = task.pick_up_cylinder(i+1, arm_set=True)
    #     time.sleep(0.8)
    #     task.pick_up_cylinder(i+1)
    #     time.sleep(0.5)
    #     task.put_down_cylinder(i+1)
    #     time.sleep(0.5)

# 定义一个函数highball_test
def ingredients_test():
    task = MyTask()
    task.get_ingredients(1, arm_set=True)

def pick_ingredients_test():
    task = MyTask()
    task.get_ingredients(1, ocr_mode=True, arm_set=True)
    task.arm.switch_side(1)
    task.pick_ingredients(1)
    task.arm.switch_side(-1)
    task.pick_ingredients(2, 2)

def answer_test():
    task = MyTask()
    task.get_answer(arm_set=True)
    # time.sleep(1)
    # task.get_answer()

def food_test():
    task = MyTask()
    task.set_food(arm_set=True)
    time.sleep(1)
    task.set_food()
    task.set_food(2)
    
def eject_test():
    task = MyTask()
    task.eject(2)

if __name__ == "__main__":
    
    import argparse
    args = argparse.ArgumentParser()
    args.add_argument('--op', type=str, default="none")
    args = args.parse_args()
    print(args)
    if args.op == "reset":
        task_reset()

    # eject_test()
    # cylinder_test()
    bmi_test()
    # ingredients_test()
    # pick_ingredients_test()
    # answer_test()
    # food_test()
