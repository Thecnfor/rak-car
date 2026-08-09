# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""
双相机 AI 分工 launch:
  front 相机 → 只跑循线(lane_follower: correction_cnn + cnn_lane)
  side  相机 → 只跑 task 检测(detector_node: PP-YOLOE engine)

不混着用:每个模型节点钉死各自相机 topic(见参数)。
设备号默认 front=/dev/video0, side=/dev/video2,按实际摆放用 launch 参数覆盖。

用法:
  ros2 launch cognition dual_camera_ai.launch.py
  # 换相机:  front_device:=/dev/video2 side_device:=/dev/video0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("front_device", default_value="/dev/cam4"),
        DeclareLaunchArgument("side_device", default_value="/dev/cam3"),
        DeclareLaunchArgument("image_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="480"),
        DeclareLaunchArgument("pixel_format", default_value="YUYV"),

        # ---- front 相机(循线用)----
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
            }],
        ),
        # ---- side 相机(task 检测用)----
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
            }],
        ),
        # ---- front → 循线(两 lane 模型)----
        Node(
            package="cognition", executable="lane-follower",
            name="lane_follower", output="screen",
            parameters=[{
                "camera_topic": "/rak/sensors/camera/front/image_compressed",
                "image_transport": "compressed",
                "correction_engine": "/home/xrak/models/lane/correction_cnn/correction_cnn_fp16.engine",
                "cnn_lane_engine": "/home/xrak/models/lane/cnn_lane/cnn_lane_fp16.engine",
                "publish_rate_hz": 30.0,
            }],
        ),
        # ---- side → task 检测(PP-YOLOE engine)----
        Node(
            package="cognition", executable="detector-node",
            name="detector_node", output="screen",
            parameters=[{
                "camera_topic": "/rak/sensors/camera/side/image_compressed",
                "image_transport": "compressed",
                "engine_path": "/home/xrak/models/ppyoloe_plus_crn_s_80e_coco/model_fp16.engine",
                "labels_file": "/home/xrak/models/ppyoloe_plus_crn_s_80e_coco/labels.txt",
                "model_id": "task",
                "publish_rate_hz": 20.0,
                "score_threshold": 0.5,
            }],
        ),

        # ---- 输出 overlay，供 RViz2 直接显示识别结果 ----
        Node(
            package="cognition", executable="vision-overlay",
            name="front_vision_overlay", output="screen",
            parameters=[{
                "camera_topic": "/rak/sensors/camera/front/image_raw",
                "result_topic": "/rak/perception/lane",
                "output_topic": "/rak/visualization/front_overlay",
                "overlay_type": "lane",
                "publish_rate_hz": 10.0,
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
                "publish_rate_hz": 10.0,
            }],
        ),
    ])
