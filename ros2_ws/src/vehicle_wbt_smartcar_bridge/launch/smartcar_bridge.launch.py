"""Launch file for vehicle_wbt_smartcar_bridge.

Brings up the bridge node that exposes the official Baidu SmartCar 2026
MyCar API surface as ROS2 services + state topics on top of the
existing rclcpp nodes (mecanum_chassis_node, arm_node).

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
                # Default rates can be overridden at launch time.
                'chassis_rate_hz': 50.0,
                'arm_rate_hz': 50.0,
                'status_rate_hz': 1.0,
            }],
        ),
    ])