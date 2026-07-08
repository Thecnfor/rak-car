"""MyCar — dev-box SDK entry point.

Mirrors car_wrap_2026.MyCar 1:1. Task scripts port with near-zero
changes. The class is constructed once per task run (matches the
official init() pattern in car_task_function.py).

Usage:
  import rclpy
  rclpy.init()
  from vehicle_wbt_smartcar_sdk import MyCar
  my_car = MyCar()
  my_car.beep()
  my_car.lane_dis_offset(speed=0.3, dis_hold=0.85)
  rclpy.shutdown()

Perception methods (lane_det_location, get_ocr, get_detection_results,
animal_image_analysis, etc.) are NOT in the SDK because they require
inference backends that live on the dev box. Dev box code subscribes
directly to the camera topics under /vehicle_wbt/v1/sensors/camera/*
and runs its own inference.
"""
from __future__ import annotations

from typing import Sequence

import rclpy
from rclpy.node import Node

from .arm import ArmClient
from .chassis import ChassisClient
from .peripheral import PeripheralClient
from .utils import delay, calculation_dis  # re-export


class MyCar:
    """Thin ROS2 SDK facade mirroring the official MyCar class."""

    # Class var mirrors MyCar.STOP_PARAM (gate for emergency-stop checks).
    STOP_PARAM: bool = True

    def __init__(self, node: Node = None) -> None:
        self._owns_node = node is None
        if self._owns_node:
            rclpy.init()
            self.node = rclpy.create_node('smartcar_sdk_my_car')
        else:
            self.node = node

        self.chassis = ChassisClient(self.node)
        self.arm = ArmClient(self.node)
        self.peripheral = PeripheralClient(self.node)

        self._stop_flag = False
        # Match official init() pattern: SDK doesn't have key thread; dev box
        # is responsible for keyboard/CLI interrupt.
        self.beep()
        self.STOP_PARAM = False

    # ------------------------------------------------------------------
    # Lifecycle / peripheral
    # ------------------------------------------------------------------

    def beep(self):
        self.peripheral.beep()
        delay(0.2)

    def set_storage(self, state: bool = False):
        self.peripheral.set_storage(state)

    def shooting(self):
        self.peripheral.shooting()

    def delay(self, time_hold: float):
        delay(time_hold)

    # ------------------------------------------------------------------
    # Motion — delegate to ChassisClient
    # ------------------------------------------------------------------

    def move_for(self, vec: Sequence[float]):
        """Mirrors MyCar.move_for([dx, dy, dz])."""
        return self.chassis.move_for(vec)

    def move_to_position(self, coords: Sequence[float]):
        return self.chassis.move_to_position(coords)

    def move_time(self, sp: Sequence[float], dur_time: float = 1.0, stop=None):
        return self.chassis.move_time(sp, dur_time)

    def move_distance(self, sp: float, dis: float = 0.1, stop=None):
        return self.chassis.move_distance(sp, dis)

    def move_base(self, sp, end_function, stop=None):
        """No return — runs until end_function() returns True. The official
        uses an end_function callback. Here we approximate by calling
        lane_base + a polling spin loop. For most competition use cases,
        prefer lane_dis_offset (the more specific API).
        """
        self.chassis.lane_base(sp[0] if hasattr(sp, '__getitem__') else float(sp))
        if end_function is not None:
            import time
            while not end_function():
                rclpy.spin_once(self.node, timeout_sec=0.05)
                time.sleep(0.02)

    def reset_position(self):
        return self.chassis.reset_position()

    def get_odometry(self, reset: bool = False):
        """Return (x, y, z) from /vehicle_wbt/v1/state/chassis/odometry."""
        from vehicle_wbt_smartcar_msgs.msg import ChassisState
        latest = {'msg': None}

        def cb(msg: ChassisState):
            latest['msg'] = msg

        sub = self.node.create_subscription(
            ChassisState, '/vehicle_wbt/v1/state/chassis/odometry', cb, 10)
        deadline = __import__('time').time() + 1.0
        while __import__('time').time() < deadline and latest['msg'] is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(sub)
        if latest['msg'] is None:
            raise TimeoutError('chassis odometry not received')
        m = latest['msg']
        return (m.x, m.y, m.z)

    def get_distance(self, reset: bool = False) -> float:
        """Mirrors MyCar.get_distance. Returns the integrated distance since
        the last reset. We approximate via (x, y) magnitude from odom.
        """
        x, y, _ = self.get_odometry(reset=reset)
        return float((x ** 2 + y ** 2) ** 0.5)

    def calculation_dis(self, pos_dst, pos_src):
        return calculation_dis(pos_dst, pos_src)

    # ------------------------------------------------------------------
    # Lane-follow — delegate to ChassisClient
    # ------------------------------------------------------------------

    def lane_base(self, speed: float, end_function=None, stop=None):
        self.chassis.lane_base(speed)
        if end_function is not None:
            import time
            while not end_function():
                rclpy.spin_once(self.node, timeout_sec=0.05)
                time.sleep(0.02)

    def lane_time(self, speed: float, time_dur: float, stop=None):
        return self.chassis.lane_time(speed, time_dur)

    def lane_dis(self, speed: float, dis_end: float, stop=None):
        return self.chassis.lane_dis(speed, dis_end)

    def lane_dis_offset(self, speed: float, dis_hold: float, stop=None):
        return self.chassis.lane_dis_offset(speed, dis_hold)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self):
        """Mirrors MyCar.close()."""
        if self._owns_node:
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()