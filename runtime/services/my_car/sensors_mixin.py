#!/usr/bin/python
# -*- coding: utf-8 -*-
"""传感器 / 外设 Mixin。

从 my_car.py 拆出：sensor_init、存储仓舵机、射击继电器、灯光、数码管、
蓝牙手柄、电池、IR 距离。依赖 MyCar 在 __init__ 里构造各 SDK 实例。
"""
import time

from smartcar.whalesbot.vehicle import (
    BluetoothPad,
    Infrared,
    LedLight,
    ServoPwm,
)
from smartcar.whalesbot.vehicle.base.controller_wrap import Battry, PoutD


class SensorsMixin:
    """MyCar 的传感器 / 外设行为。"""

    def sensor_init(self, cfg):
        """
        初始化传感器

        根据配置初始化按键、灯光和红外传感器。

        参数:
            cfg: 配置字典，包含传感器的配置信息

        """
        cfg_sensor = cfg["io"]
        # print(cfg_sensor)
        self.light = LedLight(cfg_sensor["light"])
        self.left_sensor = Infrared(cfg_sensor["left_sensor"])
        self.right_sensor = Infrared(cfg_sensor["right_sensor"])
        # 存储仓舵机：**raw 直传**(2026-07-17 用户原话"这个存储仓舵机不要任何软限制")。
        #   raw=True → 绕过 ServoPwm wrapper 的 +90 公式,angle 直传 mc602 协议字段;
        #   mc602 servo_pwm format 同时切到 "bbBb"(angle signed byte,范围 [-128, 127])。
        # LEFT/RIGHT 角度常量 → 直接就是协议值,不再是 +90 偏移后的值。
        #   LEFT  = -42  (原 -42° 业务角 + 90 = 48 协议值,raw 后变 -42 协议值)
        #   RIGHT = 165  (原 165° 业务角 + 90 = 255 协议值,raw 后变 165 协议值,**超 signed byte 127 上限
        #                  会回绕 → 业务层禁止再走 RIGHT 走这条路径,需用 set_storage_angle 直接传目标协议值)
        # ⚠️ raw 模式下 LEFT/RIGHT 历史角度常量**已失效**,舵机物理位置由 caller 重新标定。
        # 业务层 main/arm/api.py: set_storage / set_storage_angle 已取消 y 安全门。
        self.servo_1_angle_list = [-42, 165]
        self.servo_1_flag = 0
        self.servo_1 = ServoPwm(1, 180, raw=True)
        # 默认不主动写舵机：保留用户上一次 set_storage 留下的物理位置。
        # 需要"每次启动回到 LEFT"再把下面这行 set_angle(...) 注释打开。
        # self.servo_1.set_angle(self.servo_1_angle_list[self.servo_1_flag])
        self.blue_pad = BluetoothPad()
        self.shoot = PoutD(4)
        self.battery = Battry()

    def set_storage(self, state=False):
        """
        设置储存仓的位置

        根据状态参数控制储存仓的开关。

        ⚠️ 2026-07-17 协议层改成 raw 直传（servo_1 以 `ServoPwm(1, 180, raw=True)` 构造）：
          - angle 直传 mc602 协议字段,**不再 +90 偏移**
          - mc602 servo_pwm format 切到 "bbBb",angle 是 signed byte (合法区间 [-128, 127])
          - RIGHT=165 **超 signed byte 上限,业务层禁止走这条路径**
            要"开仓更开"用 set_storage_angle(目标协议值) 直接标定。
        物理碰撞由 ArmClient 取消 y 安全门,撞车风险 caller 自负。

        参数:
            state (bool): 储存仓状态。False 表示放下（LEFT），True 表示收起（RIGHT）。默认为 False。

        返回:
            dict: {"side": "LEFT"/"RIGHT", "flag": 0/1, "angle": int, "state": bool}
        """
        flag = 1 if state else 0
        angle = self.servo_1_angle_list[flag]
        # raw 直传模式下 LEFT/RIGHT 角度常量就是 raw 协议值(servo_1_angle_list)。
        # RIGHT=165 在 signed byte 上越界,会 wrap 成负值 —— 业务层若需要 165 协议值必须改用 set_storage_angle。
        # ⚠️ 跑比赛前现场重新标定舵机物理位置:用 set_storage_angle 试探,把新角度写回 servo_1_angle_list。
        self.servo_1.set_angle(angle)
        self.servo_1_flag = flag
        return {
            "side": "RIGHT" if flag == 1 else "LEFT",
            "flag": int(flag),
            "angle": int(angle),
            "state": bool(state),
        }

    def shooting(self):
        # 继电器触发型枪口：单次触发必须保证固定高电平脉冲，并可靠拉低收尾。
        self.shoot.set(0)
        time.sleep(0.05)
        try:
            self.shoot.set(1)
            time.sleep(0.25)
        finally:
            self.shoot.set(0)
        time.sleep(0.2)

    def set_shoot_state(self, value):
        self.shoot.set(1 if value else 0)
        return bool(value)

    def set_storage_angle(self, angle, speed=100):
        """
        直传存储仓舵机 raw 协议值（2026-07-17 起,+90 偏移已去掉）。

        ⚠️ raw 直传 + signed byte 协议：
          - angle 直接写入 mc602 servo_pwm 协议字段,**不再 +90**
          - 合法区间 [-128, 127]（signed byte），超出 struct.error
          - 与 set_storage 的 LEFT/RIGHT 自动路径不同,**调用方完全控制舵机**
        物理位置由 main 层用本接口现场标定。

        参数:
            angle: raw 协议值（int）,目标舵机内部位置;不再代表"业务角度"。
            speed: 舵机速度,默认 100。
        """
        self.servo_1_flag = None
        self.servo_1.set_angle(angle, speed)
        return angle

    def set_pwm_servo_angle(self, port, angle, mode=180, speed=100):
        servo = ServoPwm(port, mode)
        servo.set_angle(angle, speed)
        return {"port": port, "angle": angle, "mode": mode, "speed": speed}

    def set_digital_output(self, port, value):
        PoutD(port).set(1 if value else 0)
        return {"port": port, "value": bool(value)}

    def set_light_color(self, led_id, r, g, b):
        self.light.set_light(led_id, r, g, b)
        return {"led_id": led_id, "r": r, "g": g, "b": b}

    def show_text(self, text):
        content = str(text)
        self.display.show(content)
        return content

    def get_bluetooth_pad(self):
        return self.blue_pad.read()

    def get_battery_voltage(self):
        return self.battery.read()

    def get_ir_distance(self, side="left"):
        sensor = self.left_sensor
        if str(side).lower() in {"right", "r", "1"}:
            sensor = self.right_sensor
        return sensor.read()

    def get_all_ir_distance(self):
        return {
            "left": self.left_sensor.read(),
            "right": self.right_sensor.read(),
        }
