"""Launch file for vehicle_wbt_smartcar_bridge.

Brings up:
- smartcar_bridge_node: service bridge to MyCar API (legacy)

Note: mc602_peripheral_node was removed in 2026-07-09 — Phase 1's
mc602_node now owns buzzer / storage / shooter IO via
/vehicle_wbt/v1/mc602/buzzer, /set_pout, and /play_melody topic.

Run:
  ros2 launch vehicle_wbt_smartcar_bridge smartcar_bridge.launch.py
"""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # Force ROS_DOMAIN_ID so dev box can discover this bridge over LAN.
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),

        Node(
            package='vehicle_wbt_smartcar_bridge',
            executable='smartcar_bridge_node',
            name='smartcar_bridge_node',
            output='screen',
            parameters=[{
                'chassis_rate_hz': 50.0,
                'arm_rate_hz': 50.0,
                'status_rate_hz': 1.0,
            }],
        ),
    ])