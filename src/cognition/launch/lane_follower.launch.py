# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""
Launch lane_follower (循线感知) 单独节点,模型路径按 Orin 实际情况覆盖。

用法:
  ros2 launch cognition lane_follower.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/rak/sensors/camera/front/image_compressed"),
        DeclareLaunchArgument(
            "correction_weights",
            default_value="/home/xrak/models/lane/correction_cnn/correction_cnn.pdparams"),
        DeclareLaunchArgument(
            "cnn_lane_dir",
            default_value="/home/xrak/models/lane/cnn_lane"),
        DeclareLaunchArgument(
            "angle_source", default_value="correction",
            description="correction | lane | blend"),
        DeclareLaunchArgument(
            "image_transport", default_value="compressed"),
        Node(
            package="cognition",
            executable="lane-follower",
            name="lane_follower",
            output="screen",
            parameters=[{
                "camera_topic": LaunchConfiguration("camera_topic"),
                "image_transport": LaunchConfiguration("image_transport"),
                "correction_weights": LaunchConfiguration("correction_weights"),
                "cnn_lane_dir": LaunchConfiguration("cnn_lane_dir"),
                "angle_source": LaunchConfiguration("angle_source"),
                "publish_rate_hz": 20.0,
                "steer_scale": 1.0,
                "blend_weight": 0.5,
            }],
        ),
    ])
