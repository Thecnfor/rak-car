"""Launch file for vehicle_wbt_smartcar_bridge.

Brings up:
- smartcar_bridge_node: service bridge to MyCar API
- mc602_peripheral_node: real MC602 hardware driver (buzzer / storage / shooter)

Run:
  ros2 launch vehicle_wbt_smartcar_bridge smartcar_bridge.launch.py
  # Override serial port if not /dev/ttyUSB0:
  ros2 launch vehicle_wbt_smartcar_bridge smartcar_bridge.launch.py \
      serial_port:=/dev/ttyUSB1 baud:=1000000
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # Force ROS_DOMAIN_ID so dev box can discover this bridge over LAN.
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),

        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='MC602 controller serial device'),
        DeclareLaunchArgument(
            'baud', default_value='1000000',
            description='MC602 baud rate'),

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

        Node(
            package='vehicle_wbt_smartcar_bridge',
            executable='mc602_peripheral_node',
            name='mc602_peripheral_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud': LaunchConfiguration('baud'),
            }],
        ),
    ])