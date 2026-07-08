"""机械臂控制类(简化版,Phase 1 不含 PID 闭环)。

完整 PID 闭环逻辑留给上层机械臂同事实现。本类只提供:
- 设备引用
- set_arm_angle(角度或方向字符串)
- set_hand_angle(角度或方向字符串)
- grasp(bool)(控制泵+阀)
- reset_position()(简单粗暴 reset)

源码对齐:baidu_smartcar_2026/smartcar/whalesbot/vehicle/arm/arm_base.py
"""
from __future__ import annotations

import os
import time

import yaml

from .mc602_ctl2 import (
    AnalogInput_2,
    Motor_2,
    PoutD_2,
    ServoBus_2,
    ServoPwm_2,
    Stepper_2,
)


class ArmController:
    """机械臂设备容器。Phase 1 简化版,只暴露直接调用,无 PID 闭环。"""

    def __init__(self, config_path: str | None = None) -> None:
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), 'arm_cfg.yaml')
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.arm_length: float = cfg['arm_length']
        self.config = cfg

        # 垂直轴 (Y)
        vc = cfg['vert_cfg']
        self.motor_y = Stepper_2(port_id=vc['motor']['port_id'])
        self.y_limit_sensor = AnalogInput_2(port_id=vc['limit_port'])
        self.y_threshold = vc['threshold']

        # 水平轴 (X)
        hc = cfg['horiz_cfg']
        self.motor_x = Motor_2(port_id=hc['motor']['port_id'])
        self.x_threshold = hc['threshold']

        # 手部
        hc_hand = cfg['hand_cfg']
        self.hand_servo = ServoPwm_2(port_id=hc_hand['hand2']['port'])
        self.hand_angle_list = hc_hand['hand2']['angle_list']
        self.arm_servo = ServoBus_2(port_id=hc_hand['hand']['port'])
        self.arm_angle_list = hc_hand['hand']['angle_list']

        # 泵 + 阀
        gc = hc_hand['grap']
        self.pump = PoutD_2(port_id=gc['port_pump'])
        self.valve = PoutD_2(port_id=gc['port_valve'])

        # 当前位置(由上层通过 set_x_mm / set_y_mm 更新;Phase 1 不做编码器回读)
        self.x_pose_now = 0.0
        self.y_pose_now = 0.0

    def set_arm_angle(self, angle, speed: int = 80):
        """设置腕部总线舵机角度。angle 可为字符串 'LEFT'/'MID'/'RIGHT' 或数字。"""
        if isinstance(angle, str):
            assert angle in ('LEFT', 'MID', 'RIGHT'), f"bad angle: {angle}"
            angle = self.arm_angle_list[angle]
        self.arm_servo.set_angle(int(angle), int(speed))

    def set_hand_angle(self, angle, speed: int = 80):
        """设置手部 PWM 舵机角度。angle 可为字符串 'UP'/'MID'/'DOWN' 或数字。"""
        if isinstance(angle, str):
            assert angle in ('UP', 'MID', 'DOWN'), f"bad angle: {angle}"
            angle = self.hand_angle_list[angle]
        self.hand_servo.set_angle(int(angle), int(speed))

    def grasp(self, value: bool):
        """抓取/释放。True = 抓(泵开,阀关);False = 放(泵关,阀开)。"""
        self.pump.set(1 if value else 0)
        self.valve.set(0 if value else 1)

    def set_x_velocity(self, v: float):
        """设置水平轴电机速度(直接调,不闭环)。"""
        self.motor_x.set_speed(int(v))

    def set_y_velocity(self, v: float):
        """设置垂直轴步进电机速度(直接调,不闭环)。"""
        self.motor_y.set_pwm(int(v))

    def stop(self):
        """紧急停止所有电机。"""
        self.set_x_velocity(0)
        self.set_y_velocity(0)

    def reset_position(self):
        """简单 reset:手朝上 + 臂朝右 + 释放抓取。完整 PID reset 留给上层。"""
        self.set_hand_angle('UP')
        time.sleep(0.5)
        self.set_arm_angle('RIGHT')
        time.sleep(0.5)
        self.grasp(False)