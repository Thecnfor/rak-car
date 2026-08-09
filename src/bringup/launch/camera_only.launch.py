# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""
Launch the two real cameras without any MC602-dependent nodes.

This is the safe hardware bringup while the lower controller is unavailable.
It starts only camera_node instances and publishes the normal camera contract:
image_raw, image_compressed, camera_status, camera_meta, and camera_info when
real calibration is supplied.

Usage:
  ros2 launch bringup camera_only.launch.py
  ros2 launch bringup camera_only.launch.py front_device:=/dev/cam4 arm_device:=/dev/cam3

The default device mapping is the Orin competition mapping. Override both
paths when udev aliases are unavailable or the physical USB ports changed.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # Keep the LAN DDS contract identical to every other bringup entrypoint.
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),

        DeclareLaunchArgument(
            "front_device",
            default_value="/dev/cam4",
            description="Front camera V4L2 device path."),
        DeclareLaunchArgument(
            "arm_device",
            default_value="/dev/cam3",
            description="Arm camera V4L2 device path."),
        DeclareLaunchArgument(
            "image_rate_hz",
            default_value="30.0",
            description="Requested frame rate for both cameras."),
        DeclareLaunchArgument(
            "image_width",
            default_value="640",
            description="Requested camera width; driver may negotiate a supported mode."),
        DeclareLaunchArgument(
            "image_height",
            default_value="480",
            description="Requested camera height; driver may negotiate a supported mode."),
        DeclareLaunchArgument(
            "pixel_format",
            default_value="YUYV",
            description="Requested V4L2 fourcc pixel format."),
        DeclareLaunchArgument(
            "jpeg_quality",
            default_value="85",
            description="JPEG quality for image_compressed (0..100)."),
        DeclareLaunchArgument(
            "front_calibration_url",
            default_value="",
            description="Front camera calibration URL; empty disables camera_info."),
        DeclareLaunchArgument(
            "arm_calibration_url",
            default_value="",
            description="Arm camera calibration URL; empty disables camera_info."),

        Node(
            package="hardware",
            executable="camera_node",
            name="camera_front",
            output="screen",
            parameters=[{
                "camera_id": "front",
                "device": LaunchConfiguration("front_device"),
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "pixel_format": LaunchConfiguration("pixel_format"),
                "rate_hz": LaunchConfiguration("image_rate_hz"),
                "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                "calibration_url": LaunchConfiguration("front_calibration_url"),
            }],
        ),
        Node(
            package="hardware",
            executable="camera_node",
            name="camera_arm",
            output="screen",
            parameters=[{
                "camera_id": "arm",
                "device": LaunchConfiguration("arm_device"),
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "pixel_format": LaunchConfiguration("pixel_format"),
                "rate_hz": LaunchConfiguration("image_rate_hz"),
                "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                "calibration_url": LaunchConfiguration("arm_calibration_url"),
            }],
        ),
    ])
