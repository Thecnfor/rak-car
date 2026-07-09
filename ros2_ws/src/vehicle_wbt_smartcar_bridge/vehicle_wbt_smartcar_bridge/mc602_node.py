"""MC602 通用 ROS2 节点。

唯一串口所有者。12 service + 2 topic,把 ROS2 请求转发到底层 SDK 设备类。
路径前缀:`/vehicle_wbt/v1/mc602/*`(对齐项目顶层 `/vehicle_wbt/v1/` 命名空间)。

源码对齐:baidu_smartcar_2026 SDK (mc602_ctl2.py + serial_wrap.py)。
错误风格:SDK 失败返回 None → service 响应 success: false。
"""
from __future__ import annotations

import os
import time
from pathlib import Path
from threading import Lock, Thread

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Header

from vehicle_wbt_smartcar_hw import (
    Ambient_2,
    ArmController,
    Battry_2,
    BluetoothPad_2,
    BoardKey_2,
    Buzzer_2,
    EncoderMotors4_2,
    Infrared_2,
    Key4Btn_2,
    LedLight_2,
    Motor_2,
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
    serial_mc602,
)
from vehicle_wbt_smartcar_msgs.msg import ButtonEvent, Melody, MelodyNote, RawState
from vehicle_wbt_smartcar_msgs.srv import (
    Buzzer,
    PlayPredefined,
    ReadAmbient,
    ReadAnalog,
    ReadBattery,
    ReadBluetooth,
    ReadEncoders,
    ReadIR,
    ReadKey4,
    ReadTouch,
    ReadUltrasonic,
    ResetEncoders,
    SetDcMotor,
    SetDout,
    SetLed,
    SetNixie,
    SetServoBus,
    SetServoPwm,
    SetStepper,
    SetWheels,
    ShowScreen,
)

# Default path: ament-managed share dir (where data_files installs the YAML).
# Override via the `ports_config` parameter.
DEFAULT_PORTS_YAML = os.path.join(
    get_package_share_directory('vehicle_wbt_smartcar_bridge'),
    'config', 'mc602_ports.yaml',
)

NODE_NAME = 'mc602_io'
SERVICE_PREFIX = '/vehicle_wbt/v1/mc602'

# 预置旋律(freq_hz, duration_ms)。用 (note, dur) 元组方便编辑。
PREDEFINED_MELODIES: dict[str, list[tuple[int, int]]] = {
    # C C G G A A G*  F F E E D D C*  (14 notes, ~7s)
    'twinkle': [
        (262, 400), (262, 400), (392, 400), (392, 400),
        (440, 400), (440, 400), (392, 600),
        (349, 400), (349, 400), (330, 400), (330, 400),
        (294, 400), (294, 400), (262, 800),
    ],
    # Mary Had a Little Lamb (24 notes, ~12s)
    'mary': [
        (330, 500), (294, 500), (262, 500), (294, 500), (330, 500), (330, 500), (330, 800),
        (294, 500), (294, 500), (294, 800),
        (330, 500), (392, 500), (392, 800),
        (330, 500), (294, 500), (262, 500), (294, 500),
        (330, 500), (330, 500), (330, 500), (330, 500),
        (294, 500), (294, 500), (330, 500), (294, 500), (262, 1000),
    ],
    # Happy Birthday (22 notes, ~10s)
    'birthday': [
        (262, 300), (262, 300), (294, 800),
        (262, 300), (349, 300), (330, 800),
        (262, 300), (262, 300), (294, 800),
        (262, 300), (392, 300), (349, 800),
        (262, 300), (440, 300), (392, 300), (349, 300), (330, 800),
        (466, 300), (466, 300), (440, 300), (392, 300), (349, 800),
    ],
}


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
        self._board_key = BoardKey_2()
        # 新增设备:port-id keyed caches(避免每个 call 都新建实例)
        self._touch_cache: dict[int, Touch_2] = {}
        self._ultrasonic_cache: dict[int, Ultrasonic_2] = {}
        self._ambient_cache: dict[int, Ambient_2] = {}
        self._analog2_cache: dict[int, Sensor_Analog2_2] = {}
        self._key4_cache: dict[int, Key4Btn_2] = {}
        self._nixie_cache: dict[int, NixieTube_2] = {}
        self._led_cache: dict[int, LedLight_2] = {}
        # 全局单例类(无 port)
        self._bluetooth = BluetoothPad_2()
        self._screen = ScreenShow_2()

        # 状态缓存
        self._last_encoders = [0, 0, 0, 0]
        self._last_ir_left = 0.0
        self._last_ir_right = 0.0
        self._last_battery = 0.0
        self._pump_state = False
        self._valve_state = False
        self._last_board_key: bool | None = None  # None = 未初始化,等第一次读

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
        # 新增 8 个 service(P 口 3 类 + 全局类 3 + 数字输出 1 + 屏幕 1)
        self.create_service(ReadTouch, f'{SERVICE_PREFIX}/read_touch', self._on_read_touch)
        self.create_service(ReadUltrasonic, f'{SERVICE_PREFIX}/read_ultrasonic', self._on_read_ultrasonic)
        self.create_service(ReadAmbient, f'{SERVICE_PREFIX}/read_ambient', self._on_read_ambient)
        self.create_service(ReadBluetooth, f'{SERVICE_PREFIX}/read_bluetooth', self._on_read_bluetooth)
        self.create_service(ReadKey4, f'{SERVICE_PREFIX}/read_key4', self._on_read_key4)
        self.create_service(SetLed, f'{SERVICE_PREFIX}/set_led', self._on_set_led)
        self.create_service(SetNixie, f'{SERVICE_PREFIX}/set_nixie', self._on_set_nixie)
        self.create_service(ShowScreen, f'{SERVICE_PREFIX}/show_screen', self._on_show_screen)

        # 2 topic publisher
        self._raw_pub = self.create_publisher(RawState, f'{SERVICE_PREFIX}/state/raw', 10)
        self._heartbeat_pub = self.create_publisher(Header, f'{SERVICE_PREFIX}/heartbeat', 10)
        self._button_event_pub = self.create_publisher(
            ButtonEvent, f'{SERVICE_PREFIX}/board/button_events', 10)

        # 旋律播放(topic + service):不在 callback 线程里阻塞
        self._play_lock = Lock()
        self._play_thread: Thread | None = None
        self.create_subscription(
            Melody, f'{SERVICE_PREFIX}/play_melody', self._on_play_melody, 10)
        self.create_service(
            PlayPredefined, f'{SERVICE_PREFIX}/play_predefined', self._on_play_predefined)

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
            # SDK no_act() for bbH format returns the unpacked value (int mm)
            raw_mm = int(val) if val is not None else 0
            resp.distance_m = float(raw_mm) / 1000.0
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
            # SDK no_act() for bbH format returns the unpacked value (int ADC 0-4095)
            resp.value = int(val) if val is not None else 0
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
            # SDK no_act() for bbH format returns int (mm). Convert to meters.
            self._last_ir_left = float(int(ir_l) if ir_l is not None else 0) / 1000.0
            ir_r = self._ir_right.no_act()
            self._last_ir_right = float(int(ir_r) if ir_r is not None else 0) / 1000.0
            # 板载按钮:SDK BoardKey_2.no_act() 返回 [mode, value] 2-tuple
            # no-press value=0,按下 value>0
            k = self._board_key.no_act()
            if k is not None and isinstance(k, (list, tuple)) and len(k) >= 2:
                current_key = bool(k[1])  # value byte, 0/1
            else:
                current_key = False
            if self._last_board_key is not None and current_key != self._last_board_key:
                ev = ButtonEvent()
                ev.pressed = current_key
                self._button_event_pub.publish(ev)
                self.get_logger().info(f'board key edge: pressed={current_key}')
            self._last_board_key = current_key
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
        msg.board_key = bool(self._last_board_key) if self._last_board_key is not None else False
        self._raw_pub.publish(msg)

    def _tick_control(self):
        pass

    def _tick_heartbeat(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        self._heartbeat_pub.publish(msg)

    # ---- 新增 8 个 service handler ----

    def _get_or_create(self, cache: dict, cls, port: int):
        """per-port device cache(避免每个 service call 新建实例)"""
        if port not in cache:
            cache[port] = cls(port_id=port)
        return cache[port]

    def _on_read_touch(self, req, resp):
        try:
            dev = self._get_or_create(self._touch_cache, Touch_2, int(req.port))
            val = dev.no_act()
            # SDK no_act() returns int (0/1)
            resp.pressed = bool(int(val)) if val is not None else False
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_touch: {e}')
            resp.pressed = False
            resp.success = False
        return resp

    def _on_read_ultrasonic(self, req, resp):
        try:
            dev = self._get_or_create(self._ultrasonic_cache, Ultrasonic_2, int(req.port))
            val = dev.no_act()
            # SDK no_act() returns int (mm)
            raw_mm = int(val) if val is not None else 0
            resp.distance_m = float(raw_mm) / 1000.0
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_ultrasonic: {e}')
            resp.distance_m = 0.0
            resp.success = False
        return resp

    def _on_read_ambient(self, req, resp):
        try:
            dev = self._get_or_create(self._ambient_cache, Ambient_2, int(req.port))
            val = dev.no_act()
            # SDK no_act() returns int (ADC 0-4095)
            resp.value = int(val) if val is not None else 0
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_ambient: {e}')
            resp.value = 0
            resp.success = False
        return resp

    def _on_read_bluetooth(self, req, resp):
        try:
            data = self._bluetooth.get_stick()
            if data is None or len(data) < 5:
                resp.lx = resp.ly = resp.rx = resp.ry = 0.0
                resp.btn = 0
                resp.success = False
            else:
                resp.lx, resp.ly, resp.rx, resp.ry, resp.btn = data[0], data[1], data[2], data[3], data[4]
                resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_bluetooth: {e}')
            resp.lx = resp.ly = resp.rx = resp.ry = 0.0
            resp.btn = 0
            resp.success = False
        return resp

    def _on_read_key4(self, req, resp):
        try:
            dev = self._get_or_create(self._key4_cache, Key4Btn_2, int(req.port))
            key = dev.read()
            resp.key = int(key) if key is not None else 0
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_key4: {e}')
            resp.key = 0
            resp.success = False
        return resp

    def _on_set_led(self, req, resp):
        try:
            dev = self._get_or_create(self._led_cache, LedLight_2, int(req.port))
            dev.set_light(int(req.led_id), int(req.r), int(req.g), int(req.b))
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_led: {e}')
            resp.success = False
        return resp

    def _on_set_nixie(self, req, resp):
        try:
            dev = self._get_or_create(self._nixie_cache, NixieTube_2, int(req.port))
            dev.set_number(int(req.num))
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_nixie: {e}')
            resp.success = False
        return resp

    def _on_show_screen(self, req, resp):
        try:
            self._screen.show(req.text)
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'show_screen: {e}')
            resp.success = False
        return resp

    # ---- 旋律播放(topic + service) ----

    def _on_play_melody(self, msg: Melody) -> None:
        """Topic callback: 收到 Melody 后 spawn 一个 Thread 顺序播放。
        不阻塞 rclpy executor 线程。
        """
        notes = [(int(n.freq_hz), int(n.duration_ms)) for n in msg.notes]
        if not notes:
            return
        if self._play_thread and self._play_thread.is_alive():
            self.get_logger().warn('melody already playing, dropping new request')
            return
        self._play_thread = Thread(
            target=self._play_notes_thread, args=(notes, 'topic'),
            daemon=True)
        self._play_thread.start()

    def _on_play_predefined(self, req, resp) -> PlayPredefined.Response:
        """Service: 收到旋律名 → 查表 → spawn 线程播。返回 success 表示已启动(不等播完)。"""
        name = req.name.strip().lower()
        if name not in PREDEFINED_MELODIES:
            resp.success = False
            resp.message = f'unknown melody "{name}". available: {sorted(PREDEFINED_MELODIES.keys())}'
            return resp
        if self._play_thread and self._play_thread.is_alive():
            resp.success = False
            resp.message = 'melody already playing'
            return resp
        notes = PREDEFINED_MELODIES[name]
        self._play_thread = Thread(
            target=self._play_notes_thread, args=(notes, name),
            daemon=True)
        self._play_thread.start()
        resp.success = True
        resp.message = f'playing "{name}" ({len(notes)} notes, ~{sum(d for _, d in notes)/1000:.1f}s)'
        return resp

    def _play_notes_thread(self, notes: list[tuple[int, int]], source: str) -> None:
        """实际播放(在线程里跑):每个音 SDK rings() + time.sleep(duration/1000)。"""
        with self._play_lock:
            self.get_logger().info(f'[{source}] playing {len(notes)} notes')
            for i, (freq, dur_ms) in enumerate(notes, 1):
                try:
                    self._buzzer.rings(freq, dur_ms / 1000.0)
                except Exception as e:
                    self.get_logger().warn(f'[{source}] note {i} {freq}Hz fail: {e}')
                time.sleep(dur_ms / 1000.0)  # 等当前音播完再发下一个
            self.get_logger().info(f'[{source}] done')


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
