"""vehicle_wbt_smartcar_bridge — ROS2 service bridge mirroring Baidu SmartCar 2026 MyCar API.

Lazy imports: don't pull smartcar_bridge_node (legacy) at package import
time, since it transitively depends on peripheral_service which may not
be present. Entry points (mc602_node / chassis_kinematics_node / etc.)
import their own modules directly.
"""
__version__ = '0.1.0'