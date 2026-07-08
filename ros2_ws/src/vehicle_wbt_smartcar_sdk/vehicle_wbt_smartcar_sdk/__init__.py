"""vehicle_wbt_smartcar_sdk — dev-box-side Python SDK.

Mirrors the official Baidu SmartCar 2026 MyCar API surface 1:1. Task
scripts that worked with car_wrap_2026.MyCar port with near-zero
changes — just `from vehicle_wbt_smartcar_sdk import MyCar` instead of
`from car_wrap_2026 import MyCar`.

Usage:
  from vehicle_wbt_smartcar_sdk import MyCar
  my_car = MyCar()
  my_car.beep()
  my_car.move_for([0.1, 0, 0])
  my_car.arm.grasp(True)

The SDK is a thin ROS2 client. It does NOT touch hardware directly —
all operations route through vehicle_wbt_smartcar_bridge services on
the Jetson over the LAN (ROS_DOMAIN_ID=42).
"""
from .my_car import MyCar, ArmClient

__all__ = ['MyCar', 'ArmClient']
__version__ = '0.1.0'