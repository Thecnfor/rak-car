#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 开始编码格式和运行环境选择

import os, math
import time
import numpy as np
import sys
# 添加上本地目录
dir_this = os.path.abspath(os.path.dirname(__file__))
sys.path.append(dir_this) 
# 添加上两层目录
dir_root = os.path.abspath(os.path.join(dir_this, "..", ".."))
sys.path.append(dir_root) 
# mc601
from log_info import logger
from vehicle import *

def beep_test():
    beep = Beep()
    for i in range(10):
        beep.rings()
        time.sleep(1)

def motor_test():
    motor = Motors(2)
    motor.reset()
    for i in range(10):
        start = time.time()
        while True:
            motor.set_speed(-20)
            if time.time() - start > 1:
                break
        # motor.set_speed(20)
        # time.sleep(1)
        motor.set_speed(0)
        print(motor.get_encoder())
        time.sleep(1)

def motor4_test():
    motors = Motor4()
    for i in range(10):
        start = time.time()
        while True:
            motors.set_speed([40, 40, 40, 40])
            if time.time() - start > 1:
                break
        motors.set_speed([0, 0, 0, 0])
        print(motors.get_encoder())
        time.sleep(1)
        start = time.time()
        while True:
            motors.set_speed([-40, -40, -40, -40])
            if time.time() - start > 1:
                break
        motors.set_speed([0, 0, 0, 0])
        print(motors.get_encoder())
        time.sleep(1)

def battary_test():
    battary = Battry()
    while True:
        print(battary.read())
        time.sleep(1)

def sensor_infred_test():
    sensor1 = AnalogInput(1)
    while(1):
        print(sensor1.read())
        time.sleep(1)

def board_key_test():
    key = BoardKey()
    while True:
        res = key.read()
        print(res)
        time.sleep(0.1)

def show_test():
    show = ScreenShow()
    show.show("my_test\n\nok")

def servo_bus_test():
    servo_bus = ServoBus(1)
    while(1):
        servo_bus.set_angle(80, 80)
        time.sleep(1)
        servo_bus.set_angle(0, 80)
        time.sleep(1)

def servo_speed_test():
    servo_bus = ServoBus(2)
    while(1):
        servo_bus.set_speed(50)
        time.sleep(1)
        servo_bus.set_speed(0)
        time.sleep(1)
        servo_bus.set_speed(-50)
        time.sleep(1)
        servo_bus.set_speed(0)
        time.sleep(1)

def servo_pwm_test():
    servo_pwm = ServoPwm(2)
    while(1):
        servo_pwm.set_angle(70, 50)
        time.sleep(1)
        servo_pwm.set_angle(50, 50)
        time.sleep(1)

def key_test():
    flag = False
    key = Key4Btn(1)
    show = NixieTube(3)
    last_time = time.time()
    index = 1
    index_all = 10
    while True:
        val_key = key.read()
        now = time.time()
        try:
            fps = (1 / (now - last_time))
        except ZeroDivisionError:
            fps = 100
        last_time = now
        # print("fps:", fps)
        if val_key != 0:
            logger.info("key:{}".format(val_key))
        if val_key == 2:
            index = (index + 1) % index_all
            show.set_number(index)
        elif val_key == 4:
            index = (index - 1 + index_all) % index_all
            show.set_number(index)

def light_test():
    led = LedLight(6)
    while(1):
        led.set_light(1, 255, 0, 0)
        time.sleep(1)
        led.set_light(1, 0, 255, 0)
        time.sleep(1)
        led.set_light(1, 0, 0, 255)
        time.sleep(1)
        led.set_light(1, 0, 0, 0)
        time.sleep(1)

def nixie_test():
    nixie = NixieTube(3)
    while(1):
        nixie.set_number(1234)
        time.sleep(1)
        nixie.set_number(0)
        time.sleep(1)

if __name__ == "__main__":
    pass
    # serial_wrap.ping_port()
    # beep_test()
    # motor_test()
    # motor4_test()
    # sensor_infred_test()
    # sensor_tof_test()
    # board_key_test()
    # key_test()
    # show_test()
    # key_test()
    # servo_bus_test()

    servo_speed_test()
    # battary_test()
    # light_test()
    # nixie_test()
    # servo_pwm_test()

