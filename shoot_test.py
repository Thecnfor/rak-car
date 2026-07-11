#!/usr/bin/python
# -*- coding: utf-8 -*-
"""最小化子弹发射测试脚本 v2 - 收起仓门+多种电平组合"""
import time
from smartcar.whalesbot.vehicle import (
    ArmController,
    MecanumDriver,
    ServoPwm,
)
from smartcar.whalesbot.vehicle.base.controller_wrap import PoutD

if __name__ == "__main__":
    car = MecanumDriver()
    arm = ArmController()
    servo_1 = ServoPwm(1, 180)
    shoot = PoutD(4)

    print("[init] 收起存储仓")
    servo_1.set_angle(165)
    time.sleep(1.0)
    print("[init] done")

    print("[A] 拉高 1.5s -> 释放")
    shoot.set(1)
    time.sleep(1.5)
    shoot.set(0)
    time.sleep(2.0)

    print("[B] 拉低 1.5s -> 拉高 1.5s")
    shoot.set(0)
    time.sleep(1.5)
    shoot.set(1)
    time.sleep(1.5)
    shoot.set(0)
    time.sleep(1.0)

    print("[C] 拉高 3.0s -> 释放")
    shoot.set(1)
    time.sleep(3.0)
    shoot.set(0)
    time.sleep(2.0)

    print("done")