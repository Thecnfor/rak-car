"""smartcar_bridge_node — main entry point.

This is the Jetson-side ROS2 node that exposes the official Baidu
SmartCar 2026 MyCar API (car_wrap_2026.py) as ROS2 services + state
topics. It wraps existing rclcpp nodes (mecanum_chassis_node,
arm_node) — does NOT duplicate hardware control.

Launch with:
  ros2 launch vehicle_wbt_smartcar_bridge smartcar_bridge.launch.py
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from vehicle_wbt_smartcar_bridge import config as cfg
from vehicle_wbt_smartcar_bridge.chassis_service import ChassisService
from vehicle_wbt_smartcar_bridge.arm_service import ArmService
# peripheral_service.py was removed 2026-07-09 (Phase 1 covers buzzer/storage/shooter).
# Keep optional import so smartcar_bridge_node still loads if someone re-adds it.
try:
    from vehicle_wbt_smartcar_bridge.peripheral_service import PeripheralService
    _HAS_PERIPHERAL = True
except ImportError:
    _HAS_PERIPHERAL = False
from vehicle_wbt_smartcar_bridge.state_publisher import StatePublisher


class SmartcarBridgeNode(Node):
    """Composite node that hosts the bridge services + publishers."""

    def __init__(self) -> None:
        super().__init__('smartcar_bridge_node')
        domain_id = __import__('os').environ.get('ROS_DOMAIN_ID', '?')
        self.get_logger().info(
            f'smartcar_bridge_node starting on Jetson (ROS_DOMAIN_ID={domain_id})')

        # Component services. All share this node for executor simplicity.
        self.chassis = ChassisService(self)
        self.arm = ArmService(self)
        self.peripheral = PeripheralService(self)
        self.state = StatePublisher(self)

        self.get_logger().info(
            'smartcar_bridge_node ready: '
            'chassis=9 srv, arm=8 srv, peripheral=3 srv')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmartcarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()