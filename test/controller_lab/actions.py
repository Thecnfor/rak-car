"""最小动作语义层。"""

from __future__ import annotations

from .devices import DeviceCommand


def beep(serial_obj, freq: int = 262, duration: float = 0.2):
    return DeviceCommand("beep").set(serial_obj, int(freq / 2), int(duration * 20))


def set_motor4(serial_obj, speeds):
    if len(speeds) != 4:
        raise ValueError("motor4 需要 4 路速度")
    return DeviceCommand("motor4").set(serial_obj, *[int(speed) for speed in speeds])


def stop_chassis(serial_obj):
    return set_motor4(serial_obj, [0, 0, 0, 0])


def read_encoder4(serial_obj):
    return DeviceCommand("encoder4").get(serial_obj)


def set_motor(serial_obj, port: int, speed: int):
    return DeviceCommand("motor", port_id=port).set(serial_obj, int(speed))


def read_encoder(serial_obj, port: int):
    return DeviceCommand("encoder", port_id=port).get(serial_obj)


def set_servo_pwm(serial_obj, port: int, angle: int, speed: int = 80):
    return DeviceCommand("servo_pwm", port_id=port).set(serial_obj, int(angle), int(speed))


def set_servo_bus(serial_obj, port: int, angle: int, speed: int = 80):
    return DeviceCommand("servo_bus", port_id=port).set(serial_obj, int(angle), int(speed))


def set_dout(serial_obj, port: int, value: int):
    return DeviceCommand("dout", port_id=port).set(serial_obj, int(value))


def set_stepper(serial_obj, port: int, velocity: int, position: int):
    return DeviceCommand("stepper", port_id=port).set(serial_obj, int(velocity), int(position))


def read_infrared(serial_obj, port: int):
    return DeviceCommand("sensor_infrared", port_id=port).get(serial_obj)


def read_board_key(serial_obj):
    return DeviceCommand("board_key").get(serial_obj)


def read_bluetooth_pad(serial_obj):
    return DeviceCommand("bluetooth").get(serial_obj)
