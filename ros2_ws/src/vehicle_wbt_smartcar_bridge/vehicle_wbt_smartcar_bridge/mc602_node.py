"""MC602 通用 ROS2 节点。

唯一串口所有者。12 service + 2 topic,把 ROS2 请求转发到底层 SDK 设备类。
路径前缀:`/vehicle_wbt/v1/mc602/*`(对齐项目顶层 `/vehicle_wbt/v1/` 命名空间)。

源码对齐:baidu_smartcar_2026 SDK (mc602_ctl2.py + serial_wrap.py)。
错误风格:SDK 失败返回 None → service 响应 success: false。
"""
from __future__ import annotations

import os
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Header

from vehicle_wbt_smartcar_hw import (
    ArmController,
    Battry_2,
    Buzzer_2,
    EncoderMotors4_2,
    Infrared_2,
    Motor_2,
    Motors_2,
    PoutD_2,
    ServoBus_2,
    ServoPwm_2,
    Stepper_2,
    serial_mc602,
)
from vehicle_wbt_smartcar_msgs.msg import RawState
from vehicle_wbt_smartcar_msgs.srv import (
    Buzzer,
    ReadAnalog,
    ReadBattery,
    ReadEncoders,
    ReadIR,
    ResetEncoders,
    SetDcMotor,
    SetDout,
    SetServoBus,
    SetServoPwm,
    SetStepper,
    SetWheels,
)

DEFAULT_PORTS_YAML = Path(__file__).parent.parent / 'config' / 'mc602_ports.yaml'

NODE_NAME = 'mc602_io'
SERVICE_PREFIX = '/vehicle_wbt/v1/mc602'


class MC602Node(Node):
    """MC602 通用 IO 节点,独占 /dev/ttyUSB*。"""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        # 参数
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 1_000_000)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('sensor_rate_hz', 20.0)
        self.declare_parameter('ports_config', str(DEFAULT_PORTS_YAML))
        self.declare_parameter('auto_ping', False)

        # 加载端口路由表
        cfg_path = self.get_parameter('ports_config').value
        with open(cfg_path, 'r') as f:
            self.ports_cfg = yaml.safe_load(f)

        # 打开串口
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud').value
        serial_mc602.port = port
        serial_mc602.baudrate = baud
        if not serial_mc602.open():
            raise RuntimeError(f'Cannot open {port}')
        self.get_logger().info(f'opened {port} @ {baud}')

        # 实例化所有设备类
        chassis_cfg = self.ports_cfg['chassis']
        self._motors = Motors_2(chassis_cfg['motors'])
        self._encoder4 = EncoderMotors4_2()

        arm_cfg = self.ports_cfg['arm']
        self._stepper_y = Stepper_2(port_id=arm_cfg['stepper_y']['port'])
        self._dc_motor_x = Motor_2(port_id=arm_cfg['dc_motor_x']['port'])
        self._servo_pwm_hand = ServoPwm_2(port_id=arm_cfg['servo_pwm_hand']['port'])
        self._servo_bus_wrist = ServoBus_2(port_id=arm_cfg['servo_bus_wrist']['port'])
        self._pump = PoutD_2(port_id=arm_cfg['pump']['port'])
        self._valve = PoutD_2(port_id=arm_cfg['valve']['port'])

        shooter_cfg = self.ports_cfg['shooter']
        self._shooter = PoutD_2(port_id=shooter_cfg['barrel']['port'])

        ir_cfg = self.ports_cfg['ir']
        self._ir_left = Infrared_2(port_id=ir_cfg['left'])
        self._ir_right = Infrared_2(port_id=ir_cfg['right'])

        self._battery = Battry_2()
        self._buzzer = Buzzer_2()

        # 状态缓存
        self._last_encoders = [0, 0, 0, 0]
        self._last_ir_left = 0.0
        self._last_ir_right = 0.0
        self._last_battery = 0.0
        self._pump_state = False
        self._valve_state = False

        # 12 service server,路径前缀 /vehicle_wbt/v1/mc602/*
        self.create_service(SetWheels, f'{SERVICE_PREFIX}/set_wheels', self._on_set_wheels)
        self.create_service(ReadEncoders, f'{SERVICE_PREFIX}/read_encoders', self._on_read_encoders)
        self.create_service(ResetEncoders, f'{SERVICE_PREFIX}/reset_encoders', self._on_reset_encoders)
        self.create_service(SetServoPwm, f'{SERVICE_PREFIX}/set_servo_pwm', self._on_set_servo_pwm)
        self.create_service(SetServoBus, f'{SERVICE_PREFIX}/set_servo_bus', self._on_set_servo_bus)
        self.create_service(SetStepper, f'{SERVICE_PREFIX}/set_stepper', self._on_set_stepper)
        self.create_service(SetDcMotor, f'{SERVICE_PREFIX}/set_dc_motor', self._on_set_dc_motor)
        self.create_service(SetDout, f'{SERVICE_PREFIX}/set_pout', self._on_set_pout)
        self.create_service(ReadIR, f'{SERVICE_PREFIX}/read_ir', self._on_read_ir)
        self.create_service(ReadBattery, f'{SERVICE_PREFIX}/read_battery', self._on_read_battery)
        self.create_service(ReadAnalog, f'{SERVICE_PREFIX}/read_analog', self._on_read_analog)
        self.create_service(Buzzer, f'{SERVICE_PREFIX}/buzzer', self._on_buzzer)

        # 2 topic publisher
        self._raw_pub = self.create_publisher(RawState, f'{SERVICE_PREFIX}/state/raw', 10)
        self._heartbeat_pub = self.create_publisher(Header, f'{SERVICE_PREFIX}/heartbeat', 10)

        # 3 timer
        sensor_dt = 1.0 / self.get_parameter('sensor_rate_hz').value
        self.create_timer(sensor_dt, self._tick_sensor)
        control_dt = 1.0 / self.get_parameter('control_rate_hz').value
        self.create_timer(control_dt, self._tick_control)
        self.create_timer(1.0, self._tick_heartbeat)

        self.get_logger().info('MC602 node started')

    def _on_set_wheels(self, req, resp):
        try:
            self._motors.set_speed([int(req.v0), int(req.v1), int(req.v2), int(req.v3)])
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_wheels: {e}')
            resp.success = False
        return resp

    def _on_read_encoders(self, req, resp):
        try:
            encs = self._encoder4.get()
            if isinstance(encs, int):
                encs = [encs, 0, 0, 0]
            elif encs is None:
                encs = [0, 0, 0, 0]
            else:
                encs = list(encs)[:4] + [0] * (4 - min(4, len(encs)))
            resp.v = encs[:4]
            self._last_encoders = list(resp.v)
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_encoders: {e}')
            resp.v = [0, 0, 0, 0]
            resp.success = False
        return resp

    def _on_reset_encoders(self, req, resp):
        try:
            self._motors.reset_encoder()
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'reset_encoders: {e}')
            resp.success = False
        return resp

    def _on_set_servo_pwm(self, req, resp):
        try:
            self._servo_pwm_hand.set_angle(int(req.angle))
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_servo_pwm: {e}')
            resp.success = False
        return resp

    def _on_set_servo_bus(self, req, resp):
        try:
            self._servo_bus_wrist.set_angle(int(req.angle), int(req.speed))
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_servo_bus: {e}')
            resp.success = False
        return resp

    def _on_set_stepper(self, req, resp):
        try:
            self._stepper_y.set_pwm(int(req.freq))
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_stepper: {e}')
            resp.success = False
        return resp

    def _on_set_dc_motor(self, req, resp):
        try:
            self._dc_motor_x.set_speed(int(req.speed))
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_dc_motor: {e}')
            resp.success = False
        return resp

    def _on_set_pout(self, req, resp):
        try:
            port = int(req.port)
            state = bool(req.state)
            if port == self.ports_cfg['arm']['pump']['port']:
                self._pump.set(1 if state else 0)
                self._pump_state = state
            elif port == self.ports_cfg['arm']['valve']['port']:
                self._valve.set(1 if state else 0)
                self._valve_state = state
            elif port == self.ports_cfg['shooter']['barrel']['port']:
                self._shooter.set(1 if state else 0)
            else:
                self.get_logger().warn(f'unknown pout port: {port}')
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_pout: {e}')
            resp.success = False
        return resp

    def _on_read_ir(self, req, resp):
        try:
            port = int(req.port)
            if port == self.ports_cfg['ir']['left']:
                val = self._ir_left.no_act()
            elif port == self.ports_cfg['ir']['right']:
                val = self._ir_right.no_act()
            else:
                val = None
            if val is None or not isinstance(val, list) or not val:
                raw = 0
            else:
                raw = val[0]
            resp.distance_m = float(raw) / 1000.0
            if port == self.ports_cfg['ir']['left']:
                self._last_ir_left = resp.distance_m
            else:
                self._last_ir_right = resp.distance_m
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_ir: {e}')
            resp.distance_m = 0.0
            resp.success = False
        return resp

    def _on_read_battery(self, req, resp):
        try:
            v = self._battery.read()
            if v is None:
                resp.voltage_v = 0.0
                resp.success = False
            else:
                # Battry_2.read() 内部已经 /1000 转伏(mc602_ctl2.py:509),
                # 返回值已经是伏,不要再除
                resp.voltage_v = float(v)
                self._last_battery = resp.voltage_v
                resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_battery: {e}')
            resp.voltage_v = 0.0
            resp.success = False
        return resp

    def _on_read_analog(self, req, resp):
        try:
            from vehicle_wbt_smartcar_hw import AnalogInput_2
            sensor = AnalogInput_2(port_id=int(req.port))
            val = sensor.no_act()
            if val is None or not isinstance(val, list) or not val:
                resp.value = 0
            else:
                resp.value = int(val[0])
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_analog: {e}')
            resp.value = 0
            resp.success = False
        return resp

    def _on_buzzer(self, req, resp):
        try:
            self._buzzer.rings(int(req.freq_hz), int(req.duration_ms) / 1000.0)
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'buzzer: {e}')
            resp.success = False
        return resp

    def _tick_sensor(self):
        try:
            encs = self._encoder4.get()
            if isinstance(encs, list) and len(encs) >= 4:
                self._last_encoders = list(encs[:4])
            v = self._battery.read()
            if v is not None:
                # Battry_2.read() 内部已经 /1000 转伏,不要再除
                self._last_battery = float(v)
            ir_l = self._ir_left.no_act()
            if ir_l and isinstance(ir_l, list) and ir_l:
                self._last_ir_left = float(ir_l[0]) / 1000.0
            ir_r = self._ir_right.no_act()
            if ir_r and isinstance(ir_r, list) and ir_r:
                self._last_ir_right = float(ir_r[0]) / 1000.0
        except Exception as e:
            self.get_logger().warn(f'sensor tick: {e}')

        msg = RawState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.encoders = self._last_encoders
        msg.ir_left_m = self._last_ir_left
        msg.ir_right_m = self._last_ir_right
        msg.battery_v = self._last_battery
        msg.arm_y_pos = 0
        msg.arm_x_pos = 0
        msg.pump_on = self._pump_state
        msg.valve_on = self._valve_state
        self._raw_pub.publish(msg)

    def _tick_control(self):
        pass

    def _tick_heartbeat(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        self._heartbeat_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MC602Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
