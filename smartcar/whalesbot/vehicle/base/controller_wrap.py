#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 硬件 wrapper 层 —— MC602 专用（2026-08-03：MC601 / 无线路径整体删除）。
# 全进程唯一通信方式：MC602 USB 有线（serial_wrap 单例）。
# 历史背景：原来每个 wrapper 类持 *_1（mc601）与 *_2（mc602）双实例,按模块级
# ctl_id 分派 funcs[ctl_id]()。现在只剩 mc602 一条路,直接构造 *_2 实例调用,
# 双实例 + ctl_id 分派表全部移除。公开类名/方法签名保持不变（vehicle/__init__.py
# 与 smartcar/__init__.py 的导出面不受影响）。

import os, math
import time
import numpy as np
import sys
# 添加上本地目录
dir_this = os.path.abspath(os.path.dirname(__file__))
sys.path.append(dir_this)
dir_root = os.path.abspath(os.path.join(dir_this, "..", ".."))
sys.path.append(dir_root)
# mc602 协议设备层（Motor_2 / EncoderMotor_2 / ServoBus_2 / ...）
from .mc602_ctl2 import *
from .serial_wrap import serial_wrap
from ...tools import PID, CountRecord

# 导入自定义log模块
from ...tools import logger

def limit_val(val, min_val, max_val):
    return max(min(val, max_val), min_val)

class MotorConvert:
    def __init__(self, perimeter=None) -> None:
        # 编码器一圈12栅格编码值48 , 减速比(28/11)^4=41.98183184208729，输出一圈2015.12792842019
        self.encoder_resolution = 2015.12792842019
        # 编码速度转换值
        self.speed_rate = 100
        if perimeter is None:
            perimeter = 0.06*math.pi
        self.dis_resolution = perimeter / self.encoder_resolution
    
    def set_perimeter(self, perimeter):
        self.dis_resolution = perimeter / self.encoder_resolution
    
    def set_diameter(self, diameter):
        self.dis_resolution = diameter * math.pi / self.encoder_resolution
    
    def sp2virtual(self, speed:np.any):
        # 速度转为encoder输出
        speed_encoder = speed / self.dis_resolution
        # encoder转为控制器设置值
        speed_out = speed_encoder / self.speed_rate
        speed_out = np.clip(speed_out, -100, 100).astype(np.int8)
        return speed_out
    
    def dis2true(self, encoder_dis):
        dis_out = encoder_dis * self.dis_resolution
        return dis_out
    
    def sp2true(self, speed):
        # 控制器速度转为encoder输出
        speed_encoder = int(speed * self.speed_rate)
        speed_out = speed_encoder * self.dis_resolution
        return speed_out
    
    def encoder2dis(self, encoder_dis):
        dis_out = encoder_dis * self.dis_resolution
        return dis_out
    
    def dis2encoder(self, dis):
        encoder_out = dis / self.dis_resolution
        return encoder_out

class Beep():
    def __init__(self) -> None:
        self.beep = Buzzer_2()

    def rings(self, freq=200, duration=0.2):
        return self.beep.rings(freq, duration)

class AnalogInput():
    def __init__(self, port_id=None) -> None:
        self.sensor = AnalogInput_2(port_id)

    def read(self):
        return float(self.sensor.no_act())

class AnalogInput2():
    def __init__(self, port_id=None) -> None:
        self.sensor = Sensor_Analog2_2(port_id)

    def read(self):
        return float(self.sensor.no_act())

# 红外传感器
class Infrared():
    def __init__(self, port_id=None) -> None:
        self.infrared = Infrared_2(port_id)

    def read(self):
        # 模拟量的结果转为浮点数单位 m
        return self.infrared.no_act() / 1000

class NixieTube():
    def __init__(self, port_id=None) -> None:
        self.nixie_tube = NixieTube_2(port_id)

    def set_number(self, number):
        return self.nixie_tube.set_number(number)

class BluetoothPad():
    def __init__(self) -> None:
        self.blue_pad = BluetoothPad_2()

    def read(self):
        '''
        获取蓝牙手柄的值：
        - return: [ 左摇杆x, 左摇杆y, 右摇杆x, 右摇杆y, 按键值 ]
        - 按键值: 
            - sum = 2^key[0] +2^key[1] +...+ 2^key[15]，
            - `key[n]` 为第n个按键值，按下为n，未按下为0
        ---
        ```
        .╭────╮                            ╭────╮.    
        .| 10 |                            | 11 |.
        ╭╰════╯────────────────────────────╰════╯╮
        │  ╭────╮       WhalesBot        ╭────╮  │
        │  │ 12 │                        │ 13 │  │ 
        │  ╰────╯  ╭──╮            ╭──╮  ╰────╯  │ 
        │          │14│            │15│          │ 
        │  ╭───────╰══╯╮          ╭╰══╯───────╮  │ 
        │  │     0     │          │     4     │  │ 
        │  │ 1 < 8 > 3 │          │ 7 < 9 > 5 │  │ 
        │  │     2     │          │     6     │  │ 
        │  ╰───────────╯          ╰───────────╯  │ 
        ╰────────────────────────────────────────╯ 
        ```'''
        return self.blue_pad.get_stick()

class BoardKey():
    def __init__(self) -> None:
        self.board_key = BoardKey_2()

    def read(self):
        return self.board_key.no_act()

class LedLight():
    def __init__(self, port_id=None) -> None:
        self.led = LedLight_2(port_id)

    def set_light(self, led_id, r, g, b):
        return self.led.set_light(led_id, r, g, b)

class Key4Btn():
    def __init__(self, port_id=None) -> None:
        self.key4btn = Key4Btn_2(port_id)

    def read(self):
        return self.key4btn.get_btn()

    def get_key(self):
        return self.key4btn.get_btn()

class Motor4():
    def __init__(self) -> None:
        self.motor4 = Motor4_2()
        self.encoders = EncoderMotors4_2()
        self.encoders.reset()

    def set_speed(self, speeds):
        return self.motor4.set_speed(speeds)

    def get_encoder(self):
        return self.encoders.get()

    def reset(self):
        return self.encoders.reset()

class EncoderMotor():
    def __init__(self, port_id) -> None:
        self.encoder = EncoderMotor_2(port_id)

    def get_encoder(self):
        return self.encoder.get()

    def reset(self):
        return self.encoder.reset()

class Motor():
    # 编码器一圈12栅格编码值48 , 减速比(28/11)^4=41.98183184208729，输出一圈2015.12792842019
    motor_resolutions = {"motor_280": 48*(28/11)**4, "motor_280_0": 48*46}

    def __init__(self, port_id, reverse=1, type="motor_280") -> None:
        self.motor = Motor_2(port_id, reverse=reverse)
        self.encoder = EncoderMotor_2(port_id, reverse=reverse)
        self.encoder.reset()

        encoder_resolution = self.motor_resolutions[type]
        encoder2sp = 100
        # 弧度到编码器的比例
        self.rad2encoder = encoder_resolution / math.pi / 2
        self.encoder2rad = 1 / self.rad2encoder
        # 弧度到电机的虚拟速度的比例
        self.rad2virtual = self.rad2encoder / encoder2sp
        self.virtual2rad = 1 / self.rad2virtual

    def set_sp(self, speed):
        speed = limit_val(speed, -100, 100)
        return self.motor.set_speed(speed)

    def set_angular(self, angular):
        return self.set_sp(self.rad2virtual * angular)

    def get_encoder(self):
        return self.encoder.get_encoder()

    def get_rad(self):
        return self.get_encoder()*self.encoder2rad

    def reset(self):
        self.motor.reset()
        return self.encoder.reset()

class Motors():
    """四轮批量电机（麦轮底盘用）：批量写走 motor4 帧,批量读走 encoder4 帧。"""
    # 编码器一圈12栅格编码值48 , 减速比(28/11)^4=41.98183184208729，输出一圈2015.12792842019
    motor_resolutions = {"motor_280": 48*(28/11)**4, "motor_280_0": 48*46}

    def __init__(self, port_list=None, reverse=False, type="motor_280") -> None:
        encoder_resolution = self.motor_resolutions[type]
        encoder2sp = 100
        # 弧度到编码器的比例
        self.rad2encoder = encoder_resolution / math.pi / 2
        self.encoder2rad = 1 / self.rad2encoder
        # 弧度到电机的虚拟速度的比例
        self.rad2virtual = self.rad2encoder / encoder2sp
        self.virtual2rad = 1 / self.rad2virtual

        self.motors = Motors_2(port_list, reverse)

    def set_speed(self, speeds):
        return self.motors.set_speed(speeds)

    def set_angular(self, angular):
        sp_virtual = np.array(angular) * self.rad2virtual
        sp_virtual = np.clip(sp_virtual, -100, 100).astype(np.int8)
        return self.set_speed(sp_virtual)

    def get_encoder(self):
        return self.motors.get_encoder()

    # 获取弧度值
    def get_rad(self):
        encoder_last = np.array(self.get_encoder())
        return encoder_last * self.encoder2rad

    def reset(self):
        return self.motors.reset_encoder()

class WheelWrap():
    def __init__(self, port_list=None,raduis=0.03, motor_type="motor_280", reverse=False) -> None:
        self.motors = Motors(port_list, reverse, motor_type)
        self.raduis = raduis
        self.linear2rad = 1 / self.raduis
    
    def set_linear(self, vel_linear):
        # 线速度转角速度
        angular = np.array(vel_linear) * self.linear2rad
        return self.motors.set_angular(angular)
    
    def set_angular(self, angular):
        return self.motors.set_angular(angular)
    
    def get_rad(self):
        return self.motors.get_rad()

    def get_linear(self):
        d_linear = self.motors.get_rad() * self.raduis
        return d_linear
    
    def reset(self):
        return self.motors.reset()

class MotorWrap():
    """单电机 + 编码器封装（机械臂 x 轴用）：线速度/位移 <-> 角速度/弧度。"""
    def __init__(self, id=1, reverse=1, type="motor_280",perimeter=0.06*math.pi) -> None:
        self.motor = Motor(id, reverse, type)
        self.motor.reset()
        self.rad2dis = perimeter / math.pi / 2
        self.dis2rad = 1 / self.rad2dis
        self.count_flag = CountRecord(10)
    
    def set_linear(self, vel_linear):
        # 线速度转角速度
        angular = vel_linear * self.dis2rad
        return self.motor.set_angular(angular)
    
    def set_angular(self, angular):
        return self.motor.set_angular(angular)
    
    def get_rad(self):
        return self.motor.get_rad()

    def get_dis(self):
        return self.motor.get_rad() * self.rad2dis
    
    def reset(self):
        return self.motor.reset()

class ServoPwm():
    def __init__(self, port_id=None, mode=180, raw: bool = False) -> None:
        """PWM 舵机 wrapper。

        Args:
            port_id: 物理端口号。
            mode: 默认 180。raw=False 时生效,公式:protocol = int(angle/mode*180+90)。
            raw: True 时**绕过 +90 公式**,angle 直传 mc602 协议字段;
                同时把 mc602 的 angle 字节切到 signed(servo_pwm format: "bbBb"),
                支持负协议值(范围 [-128, 127])。**仅供 storage servo 这类需要
                真正直传舵机内部协议值的场景使用**,不要给 hand_servo 等仍走
                业务角度语义的下游用。
        """
        self.mode = mode
        self.raw = raw
        self.servo = ServoPwm_2(port_id)
        if raw:
            # 切到 signed byte: dev_id(b) + mode(b) + port_id(B) + speed(B) + angle(b)
            # 原 "bbBB" 第四字节 unsigned → 改 "bbBb" 第四字节 signed
            self.servo.data_struct.set_format("bbBb")

    def set_angle(self, angle, speed=100):
        if not self.raw:
            angle = int(angle / self.mode * 180 + 90)
        return self.servo.set_angle(angle, speed)

class ServoBus():
    def __init__(self,port_id=None) -> None:
        self.servo_bus = ServoBus_2(port_id)
        logger.info(f"总线电机初始化完成，ID:{port_id}")

    def set_angle(self, angle, speed=100):
        return self.servo_bus.set_angle(angle, speed)

    def set_speed(self, speed):
        return self.servo_bus.set_speed(speed)

    def read_angle(self, port_id=None):
        """读取总线舵机当前角度（mc602 ServoBus_2 协议）。"""
        return self.servo_bus.read_angle(port_id)

class PoutD():
    def __init__(self, port):
        self.pout = PoutD_2(port)

    def set(self, val):
        return self.pout.set(val)

class ScreenShow():
    def __init__(self) -> None:
        self.screen = ScreenShow_2()

    def show(self, args):
        return self.screen.show(args)

class Battry():
    def __init__(self) -> None:
        self.battry = Battry_2()

    def read(self):
        return self.battry.read()

class StepperWrap():
    def __init__(self, id, reverse=1, perimeter=0.008) -> None:
        self.reverse = reverse
        self.stepper = Stepper_2(id)
        
        # 系数
        # gradient = 9
        # 步进值1.8度 8细分, 2相位
        self.stepper2rad = math.pi / 180 * 1.8 / 16
        self.rad2pwm = 16 * 180 / 1.8 / math.pi
        
        # 计算半径，即弧度转弧长的系数
        self.rad2dis = perimeter / math.pi / 2
        self.dis2rad = 1/self.rad2dis
    
    def get_rad(self):
        return self.stepper.get_step() * self.stepper2rad * self.reverse

    def set_rad(self, rad, time=0.5):
        pid = PID(5,0,0)
        pid.setpoint = rad
        if time < 0.1:
            time = 0.1
        rad_vel = abs(self.get_rad() - rad) / time
        pid.output_limits = (-rad_vel, rad_vel)
        cnt = 0
        while True:
            if abs(self.get_rad()-rad) < 0.1:
                cnt += 1
                if cnt > 10:
                    break
            else:
                cnt = 0
            self.set_angular(pid(self.get_rad()))
        self.set_angular(0)

    def set_angular(self, angular):
        return self.stepper.set(int(angular * self.rad2pwm * self.reverse))

    def set_velocity(self, velocity):
        return self.set_angular(velocity* self.dis2rad)
    
    def get_dis(self):
        return self.get_rad() * self.rad2dis

    def reset(self):
        return self.stepper.reset()

def stepper_test():
    step1 = StepperWrap(1,reverse=1, perimeter=0.008)
    step2 = StepperWrap(2,reverse=1, perimeter=0.008)

    while True:
        step1.set_rad(math.pi/5*2)
        step2.set_rad(math.pi/5*2)
        time.sleep(1)
        step1.set_rad(0)
        step2.set_rad(0)
        time.sleep(1)

def servo_test():
    servo4 = ServoBus(4)
    while True:
        servo4.set_angle(90)
        time.sleep(1)
        servo4.set_angle(0)
        time.sleep(1)

if __name__ == "__main__":
    beep = Beep()
    beep.rings(200, 0.2)
    battry = Battry()
    motor5_wrap = MotorWrap(6, reverse=-1, type="motor_280", perimeter=0.06/12*8)
    sp = -0.1
    motor5_wrap.set_linear(sp)
    for i in range(100):
        time.sleep(0.01)
        motor5_wrap.set_linear(sp)
    motor5_wrap.set_linear(0)
