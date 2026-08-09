# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""
Full vision stack launch (Orin side).

Brings up:
  - camera_front + camera_side (real hardware, /dev/cam4 + /dev/cam3 by default)
  - lane_follower (correction_cnn + cnn_lane TensorRT engines, 30Hz)
  - detector_node (PP-YOLOE-Plus TensorRT engine)
  - vision_overlay (joins images + perception into RViz-ready overlay topics)

Does NOT start MC602 / ros2_control / chassis / arm / peripheral. This is
the safe visual bringup while the lower controller is unavailable.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    camera_front_yaml = os.path.join(
        bringup_share, "params", "camera_front.yaml")
    camera_side_yaml = os.path.join(
        bringup_share, "params", "camera_arm.yaml")

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),

        DeclareLaunchArgument("front_device", default_value="/dev/cam4"),
        DeclareLaunchArgument("side_device", default_value="/dev/cam3"),
        DeclareLaunchArgument("image_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("lane_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="480"),
        DeclareLaunchArgument("pixel_format", default_value="YUYV"),
        DeclareLaunchArgument("detector_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("overlay_rate_hz", default_value="10.0"),
        DeclareLaunchArgument(
            "correction_engine",
            default_value="/home/xrak/models/lane/correction_cnn/correction_cnn_fp16.engine"),
        DeclareLaunchArgument(
            "cnn_lane_engine",
            default_value="/home/xrak/models/lane/cnn_lane/cnn_lane_fp16.engine"),
        DeclareLaunchArgument(
            "detector_engine",
            default_value="/home/xrak/models/ppyoloe_plus_crn_s_80e_coco/model_fp16.engine"),
        DeclareLaunchArgument(
            "detector_labels",
            default_value="/home/xrak/models/ppyoloe_plus_crn_s_80e_coco/labels.txt"),
        DeclareLaunchArgument("score_threshold", default_value="0.5"),

        Node(
            package="hardware", executable="camera_node",
            name="camera_front", output="screen",
            parameters=[{
                "camera_id": "front",
                "device": LaunchConfiguration("front_device"),
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "pixel_format": LaunchConfiguration("pixel_format"),
                "rate_hz": LaunchConfiguration("image_rate_hz"),
                "calibration_url": "package://bringup/params/camera_front.yaml",
            }],
        ),
        Node(
            package="hardware", executable="camera_node",
            name="camera_side", output="screen",
            parameters=[{
                "camera_id": "side",
                "device": LaunchConfiguration("side_device"),
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "pixel_format": LaunchConfiguration("pixel_format"),
                "rate_hz": LaunchConfiguration("image_rate_hz"),
                "calibration_url": "package://bringup/params/camera_arm.yaml",
            }],
        ),

        Node(
            package="cognition", executable="lane-follower",
            name="lane_follower", output="screen",
            parameters=[{
                "camera_topic": "/rak/sensors/camera/front/image_compressed",
                "image_transport": "compressed",
                "correction_engine": LaunchConfiguration("correction_engine"),
                "cnn_lane_engine": LaunchConfiguration("cnn_lane_engine"),
                "publish_rate_hz": LaunchConfiguration("lane_rate_hz"),
                "angle_source": "correction",
            }],
        ),

        Node(
            package="cognition", executable="detector-node",
            name="detector_node", output="screen",
            parameters=[{
                "camera_topic": "/rak/sensors/camera/side/image_compressed",
                "image_transport": "compressed",
                "engine_path": LaunchConfiguration("detector_engine"),
                "labels_file": LaunchConfiguration("detector_labels"),
                "model_id": "task",
                "publish_rate_hz": LaunchConfiguration("detector_rate_hz"),
                "score_threshold": LaunchConfiguration("score_threshold"),
            }],
        ),

        Node(
            package="cognition", executable="vision-overlay",
            name="front_vision_overlay", output="screen",
            parameters=[{
                "camera_topic": "/rak/sensors/camera/front/image_raw",
                "result_topic": "/rak/perception/lane",
                "output_topic": "/rak/visualization/front_overlay",
                "overlay_type": "lane",
                "publish_rate_hz": LaunchConfiguration("overlay_rate_hz"),
            }],
        ),
        Node(
            package="cognition", executable="vision-overlay",
            name="side_vision_overlay", output="screen",
            parameters=[{
                "camera_topic": "/rak/sensors/camera/side/image_raw",
                "result_topic": "/rak/perception/detections/task",
                "output_topic": "/rak/visualization/side_overlay",
                "overlay_type": "detection",
                "publish_rate_hz": LaunchConfiguration("overlay_rate_hz"),
            }],
        ),
    ])