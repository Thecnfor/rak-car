# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""
Launch file: full sidecar system on Jetson Orin (real hardware).

Brings up:
- 2 cameras (front + side)
- 2 IR sensors (left + right)
- mecanum chassis node (subscribes /cmd/vel_safe, publishes /state/odom)
- arm node (subscribes /cmd/arm/trajectory, publishes /state/actuators)
- safety gate (4-layer; gates /cmd/vel_raw -> /cmd/vel_safe)

Usage:
  ros2 launch vehicle_wbt_platform_cpp full_system.launch.py

Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §生命周期
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB0",
            description="MC602 controller serial device"),
        DeclareLaunchArgument(
            "baud", default_value="1000000",
            description="MC602 baud rate"),

        # Front camera
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="camera_node",
            name="camera_front",
            output="screen",
            parameters=[{
                "camera_id": "front",
                "device": "/dev/cam2",
                "image_width": 640,
                "image_height": 480,
                "rate_hz": 10.0,
            }],
        ),
        # Side camera
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="camera_node",
            name="camera_side",
            output="screen",
            parameters=[{
                "camera_id": "side",
                "device": "/dev/cam1",
                "image_width": 640,
                "image_height": 480,
                "rate_hz": 10.0,
            }],
        ),

        # Left + right IR
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="infrared_node",
            name="ir_left",
            output="screen",
            parameters=[{
                "ir_id": "left",
                "mc602_port": 8,  # P8
                "min_range_m": 0.02,
                "max_range_m": 0.3,
                "rate_hz": 20.0,
            }],
        ),
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="infrared_node",
            name="ir_right",
            output="screen",
            parameters=[{
                "ir_id": "right",
                "mc602_port": 7,  # P7
                "min_range_m": 0.02,
                "max_range_m": 0.3,
                "rate_hz": 20.0,
            }],
        ),

        # Mecanum chassis
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="mecanum_chassis_node",
            name="mecanum_chassis",
            output="screen",
            parameters=[{
                "chassis_Lx": 0.15,
                "chassis_Ly": 0.10,
                "wheel_radius": 0.03,
                "publish_rate_hz": 50.0,
            }],
        ),

        # Arm
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="arm_node",
            name="arm_main",
            output="screen",
            parameters=[{
                "arm_id": "main",
                "publish_rate_hz": 50.0,
            }],
        ),

        # 4-layer safety gate
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="safety_gate_node",
            name="safety_gate",
            output="screen",
            parameters=[{
                "max_linear_velocity": 0.3,
                "max_angular_velocity": 0.5,
                "deadman_ms": 500,
            }],
        ),
    ])
