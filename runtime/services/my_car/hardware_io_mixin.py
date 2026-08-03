#!/usr/bin/python
# -*- coding: utf-8 -*-
"""实时硬件控制 Mixin。

从 my_car.py 拆出：底盘速度 / 4 轮速 / 编码器 / 单电机 / 步进 / 总线舵机 /
模拟量。全部走 car_lock 同步路径、绕过 job_queue，50Hz 友好。
"""


class HardwareIOMixin:
    """MyCar 的实时硬件控制行为。"""

    # === 实时硬件控制 wrapper（car_lock 同步路径，绕过 job_queue，50Hz 友好） ===
    # 内部 SDK 实例按 (kind, port) 懒构造、绑定 self._realtime_instances，
    # 重建 car 时随旧实例自动失效，无需手动清理。

    def set_chassis_velocity(self, x=0.0, y=0.0, z=0.0, duration=None):
        x = float(x)
        y = float(y)
        z = float(z)
        if duration is None:
            self.set_velocity(x, y, z)
        else:
            self.set_velocity_for_duration(x, y, z, float(duration))
        return {"x": x, "y": y, "z": z, "duration": duration}

    def _get_realtime_instance(self, kind, port, **ctor_kwargs):
        """懒构造并缓存 SDK 实例，避免反复 new。"""
        cache = getattr(self, "_realtime_instances", None)
        if cache is None:
            self._realtime_instances = {}
            cache = self._realtime_instances
        key = (kind, int(port))
        if key not in cache:
            from smartcar.whalesbot.vehicle.base import controller_wrap as _cw
            cls_map = {
                "stepper": _cw.StepperWrap,
                "motor": _cw.Motor_2,
                "encoder": _cw.EncoderMotor_2,
                "bus_servo": _cw.ServoBus,
                "analog": _cw.AnalogInput,
                "analog2": _cw.AnalogInput2,
            }
            cls = cls_map[kind]
            if kind in ("stepper", "motor", "encoder"):
                cache[key] = cls(port_id=int(port), **ctor_kwargs)
            else:
                cache[key] = cls(port_id=int(port))
        return cache[key]

    def set_wheel_speeds(self, speeds):
        """直接下发 4 轮线速度，绕过 set_chassis_velocity 的里程计耦合路径。"""
        speeds = list(speeds)
        if len(speeds) != 4:
            raise ValueError("speeds 必须是长度为 4 的数组 [v1, v2, v3, v4]")
        speeds_f = [float(s) for s in speeds]
        self.wheels_chassis.set_linear(speeds_f)
        return {"speeds": speeds_f}

    def get_wheel_encoders(self):
        """读取 4 轮编码器弧度累计值。"""
        rad = self.wheels_chassis.get_rad()
        try:
            return [float(x) for x in rad]
        except TypeError:
            return [float(rad)]

    def set_single_motor(self, port, speed, reverse=1):
        """单电机原始速度控制。"""
        motor = self._get_realtime_instance("motor", port, reverse=int(reverse))
        motor.set_speed(speed)
        return {"port": int(port), "speed": float(speed), "reverse": int(reverse)}

    def get_encoder(self, port, reverse=1):
        """读取单电机编码器原始累计值。"""
        enc = self._get_realtime_instance("encoder", port, reverse=int(reverse))
        return int(enc.get_encoder())

    def set_stepper_rad(self, port, rad, time=0.5, reverse=1, perimeter=0.008):
        """底盘步进电机弧度定位。注意：port 不要跟 arm_cfg.yaml 里配置的机械臂 y 轴端口冲突。"""
        stepper = self._get_realtime_instance(
            "stepper", port, reverse=int(reverse), perimeter=float(perimeter)
        )
        stepper.set_rad(float(rad), time=float(time))
        return {"port": int(port), "rad": float(rad), "time": float(time)}

    def set_bus_servo(self, port, angle, speed=100):
        """总线舵机角度下发（mc602 ServoBus_2 协议）。"""
        servo = self._get_realtime_instance("bus_servo", port)
        servo.set_angle(float(angle), int(speed))
        return {"port": int(port), "angle": float(angle), "speed": int(speed)}

    def read_bus_servo(self, port):
        """读取总线舵机当前角度（mc602 ServoBus_2 协议）。"""
        servo = self._get_realtime_instance("bus_servo", port)
        return int(servo.read_angle())

    def read_analog(self, port):
        """读单路模拟量（mc602 走 Sensor_Analog2_2，dev_id=0x08）。"""
        ai = self._get_realtime_instance("analog2", port)
        return float(ai.read())

    def read_analog2(self, port):
        """读第二路模拟量（mc602 走 AnalogInput，dev_id=0x07 mode=0）。"""
        ai = self._get_realtime_instance("analog", port)
        return float(ai.read())
