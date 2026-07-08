# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""
Launch file: dev/desktop sidecar system with mock hardware.

Same node set as full_system.launch.py but with all hardware parameters
overridden to mock /dev/null-style devices so it runs on a dev machine
without Jetson. Used for:
- Local dev iteration (no ssh to Jetson — runs locally)
- CI integration tests
- New team member onboarding

Usage:
  ros2 launch vehicle_wbt_platform_cpp mock_system.launch.py

Spec: docs/development/test-matrix.md
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="camera_node", name="camera_front",
            output="screen",
            parameters=[{
                "camera_id": "front", "device": "/dev/null",
                "image_width": 320, "image_height": 240, "rate_hz": 5.0,
            }],
        ),
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="infrared_node", name="ir_left",
            output="screen",
            parameters=[{
                "ir_id": "left", "mc602_port": 8,
                "min_range_m": 0.02, "max_range_m": 0.3, "rate_hz": 10.0,
            }],
        ),
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="mecanum_chassis_node", name="mecanum_chassis",
            output="screen",
            parameters=[{
                "chassis_Lx": 0.15, "chassis_Ly": 0.10,
                "wheel_radius": 0.03, "publish_rate_hz": 20.0,
            }],
        ),
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="arm_node", name="arm_main",
            output="screen",
            parameters=[{"arm_id": "main", "publish_rate_hz": 10.0}],
        ),
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="safety_gate_node", name="safety_gate",
            output="screen",
            parameters=[{
                "max_linear_velocity": 0.3,
                "max_angular_velocity": 0.5,
                "deadman_ms": 1000,  # relaxed for dev iteration
            }],
        ),
    ])
