# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""
Launch file: full sidecar system on Jetson Orin (real hardware).

Brings up:
- 2 cameras (front + arm) — each publishing 5 streams: image_raw,
  image_compressed, camera_info, camera_status, camera_meta
- 2 IR sensors (left + right)
- mecanum chassis node (subscribes /cmd/vel_safe, publishes /state/odom)
- arm node (subscribes /cmd/arm/trajectory, publishes /state/actuators)
- safety gate (4-layer; gates /cmd/vel_raw -> /cmd/vel_safe)

Usage:
  ros2 launch vehicle_wbt_platform_cpp full_system.launch.py

Note on camera device paths: this launch file uses the device symlinks
produced by /etc/udev/rules.d/99-usbvideo.rules. The mapping (devpath ->
symlink) is hardware-specific; if you move the physical cables, remap via
the udev rules before relaunching. See docs/hardware-port-mapping.md.

Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §生命周期
"""
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # ---- Environment ----
        # ROS_DOMAIN_ID=42 is the convention this project uses for DDS
        # discovery so dev machines on the same LAN auto-see Jetson topics
        # (and vice versa). See docs/development/README.md.
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),

        # ---- Launch args (all camera / robot params overridable) ----
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB0",
            description="MC602 controller serial device"),
        DeclareLaunchArgument(
            "baud", default_value="1000000",
            description="MC602 baud rate"),
        DeclareLaunchArgument(
            "front_device", default_value="/dev/cam2",
            description="Front camera device path. Competition target: /dev/cam2. "
            "Override per machine, e.g. on this dev box the front cam is at /dev/cam4."),
        DeclareLaunchArgument(
            "arm_device", default_value="/dev/cam1",
            description="Arm camera device path. Competition target: /dev/cam1. "
            "Override per machine, e.g. on this dev box the arm cam is at /dev/cam3."),
        DeclareLaunchArgument(
            "image_rate_hz", default_value="10.0",
            description="Camera frame rate (Hz)"),
        DeclareLaunchArgument(
            "jpeg_quality", default_value="85",
            description="JPEG quality for image_compressed (0..100)"),
        DeclareLaunchArgument(
            "front_calibration_url", default_value="",
            description="URL/path for camera_front camera_info_manager load. "
            "Empty = no calibration, camera_info NOT published. "
            "Default paths after install: "
            "'file:///opt/ros/.../share/.../params/camera_front.yaml' "
            "or 'package://vehicle_wbt_platform_cpp/params/camera_front.yaml'."),
        DeclareLaunchArgument(
            "arm_calibration_url", default_value="",
            description="URL/path for camera_arm camera_info_manager load. "
            "See front_calibration_url."),

        # ---- Cameras: one node per physical camera. Each publishes 5 streams
        # under /vehicle_wbt/v1/sensors/camera/<camera_id>/{image_raw,
        # image_compressed, camera_info, camera_status, camera_meta}. ----

        # Front camera — device overridable via front_device launch arg.
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="camera_node",
            name="camera_front",
            output="screen",
            parameters=[{
                "camera_id": "front",
                "device": LaunchConfiguration("front_device"),
                "image_width": 640,
                "image_height": 480,
                "rate_hz": LaunchConfiguration("image_rate_hz"),
                "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                "calibration_url": LaunchConfiguration("front_calibration_url"),
            }],
        ),

        # Arm (mechanical-arm wrist) camera — device overridable via arm_device.
        Node(
            package="vehicle_wbt_platform_cpp",
            executable="camera_node",
            name="camera_arm",
            output="screen",
            parameters=[{
                "camera_id": "arm",
                "device": LaunchConfiguration("arm_device"),
                "image_width": 640,
                "image_height": 480,
                "rate_hz": LaunchConfiguration("image_rate_hz"),
                "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                "calibration_url": LaunchConfiguration("arm_calibration_url"),
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
