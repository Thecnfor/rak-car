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
        DeclareLaunchArgument("front_device", default_value="/dev/video0"),
        DeclareLaunchArgument("side_device", default_value="/dev/video2"),
        DeclareLaunchArgument("image_rate_hz", default_value="20.0"),

        # ---- front 相机(循线用)----
        Node(
            package="hardware", executable="camera_node",
            name="camera_front", output="screen",
            parameters=[{
                "camera_id": "front",
                "device": LaunchConfiguration("front_device"),
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
                "publish_rate_hz": 20.0,
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
                "score_threshold": 0.3,
            }],
        ),
    ])
