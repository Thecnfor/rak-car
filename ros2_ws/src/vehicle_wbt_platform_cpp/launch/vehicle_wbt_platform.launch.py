# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""
Launch file: bring up the C++ core of vehicle_wbt Platform.

Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §生命周期

Phase 1.5 stub: only the MecanumChassis kinematics node. Real nodes
(camera, IR, motor command, ros2_control spawner) land in Plan B.

The Python orchestrator in vehicle_wbt_platform spawns this via
launch.actions.IncludeLaunchDescription, so users don't need to remember
this filename.

Usage:
  ros2 launch vehicle_wbt_platform_cpp vehicle_wbt_platform.launch.py \
      serial_port:=/dev/ttyUSB0 baud:=1000000
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial device for MC602 controller (e.g. /dev/ttyUSB0)",
    )
    baud_arg = DeclareLaunchArgument(
        "baud",
        default_value="1000000",
        description="MC602 baud rate: 380400 (MC601), 1000000 (MC602 USB), 115200 (MC602 wireless)",
    )

    mecanum_node = Node(
        package="vehicle_wbt_platform_cpp",
        executable="mecanum_chassis_node",  # TODO: implement in Plan B
        name="mecanum_chassis",
        output="screen",
        parameters=[{
            "serial_port": LaunchConfiguration("serial_port"),
            "baud": LaunchConfiguration("baud"),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        baud_arg,
        mecanum_node,
    ])
