"""vehicle_wbt_smartcar_hw — MC602 下位机协议层。

Ground truth: baidu_smartcar_2026 SDK。字节格式 1:1 对齐,
错误风格遵循 SDK(失败返回 None)。

模块布局:
    serial      - MC602 串口包装(SDK serial_wrap.py 简化版)
    mc602_ctl2  - 23 个设备类 + DevCmdInterface + DevListWrap(SDK 1:1)
                 P 口:  AnalogInput, Infrared, Touch, Ultrasonic, Ambient, Key4Btn
                 M 口:  Motor, Motor4, Motors(批量), EncoderMotor, EncoderMotors4, Stepper
                 舵机:  ServoPwm, ServoBus
                 数字:  PoutD
                 其他:  Buzzer, Battry, BluetoothPad, LedLight, NixieTube, ScreenShow, BoardKey
    odometry    - Odometry + MecanumChassis 纯计算(SDK 简化版)
    arm         - ArmController 设备容器(简化版,无 PID 闭环)
"""
from __future__ import annotations

from .serial import MC602, serial_mc602
from .mc602_ctl2 import (
    Ambient_2,
    AnalogInput_2,
    Battry_2,
    BluetoothPad_2,
    BoardKey_2,
    Buzzer_2,
    DevCmdInterface,
    DevListWrap,
    EncoderMotor_2,
    EncoderMotors4_2,
    Infrared_2,
    Key4Btn_2,
    LedLight_2,
    Motor_2,
    Motor4_2,
    Motors_2,
    NixieTube_2,
    PoutD_2,
    ScreenShow_2,
    Sensor_Analog2_2,
    ServoBus_2,
    ServoPwm_2,
    Stepper_2,
    Touch_2,
    Ultrasonic_2,
)
from .odometry import MecanumChassis, Odometry
from .arm import ArmController

__all__ = [
    'MC602',
    'serial_mc602',
    'Ambient_2',
    'AnalogInput_2',
    'Battry_2',
    'BluetoothPad_2',
    'BoardKey_2',
    'Buzzer_2',
    'DevCmdInterface',
    'DevListWrap',
    'EncoderMotor_2',
    'EncoderMotors4_2',
    'Infrared_2',
    'Key4Btn_2',
    'LedLight_2',
    'Motor_2',
    'Motor4_2',
    'Motors_2',
    'NixieTube_2',
    'PoutD_2',
    'ScreenShow_2',
    'Sensor_Analog2_2',
    'ServoBus_2',
    'ServoPwm_2',
    'Stepper_2',
    'Touch_2',
    'Ultrasonic_2',
    'MecanumChassis',
    'Odometry',
    'ArmController',
]
