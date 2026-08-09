# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""Start RViz2 only; observe the remote full-vision stack over DDS."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("bringup"), "config", "vision_overlay.rviz")
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="vision_rviz",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
