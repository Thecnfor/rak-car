# visp_servo.launch.py — ViSP 机械臂视觉伺服节点 (2026-08-10).
#
# 链路:
#   camera image + camera_info + DetectionArray
#     → visp_servo_node (ViSP vpServo 图像居中, 相机速度 → base)
#     → /rak/control/arm/servo_twist (TwistStamped, base_link 系)
#     → moveit_servo (move_group.launch.py) → joint_trajectory_controller
#
# 前置 (必须按顺序就绪):
#   1. middleware.launch.py   — ros2_control + joint_trajectory_controller
#   2. move_group.launch.py   — moveit_servo 收 /rak/control/arm/servo_twist
#   3. full_vision.launch.py (或等价的) — 在 arm 相机流上跑 detector,
#      发布 DetectionArray 到 target_topic
#   4. arm 相机已标定 (camera_info 话题在发, K 非零)
#
# 安全: enabled 默认 false —— 视觉伺服必须由任务层显式使能
#   (ros2 service call /visp_servo_node/set_enabled std_srvs/srv/SetBool "{data: true}")
#   保证机械臂驱动器 (move_group / arm_cartesian_move_node / 本节点) 同一时刻
#   只有一个在掌控。
#
# 用法 (Orin):
#   ros2 launch bringup visp_servo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/rak/sensors/camera/arm/image_compressed',
            description='arm 相机压缩图 (新鲜度心跳)'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/rak/sensors/camera/arm/camera_info',
            description='arm 相机标定 (K 全零 = 未标定 → 不发指令)'),
        DeclareLaunchArgument(
            'target_topic',
            default_value='/rak/perception/detections/arm',
            description='arm 相机流上的 DetectionArray'),
        DeclareLaunchArgument(
            'servo_topic',
            default_value='/rak/control/arm/servo_twist',
            description='moveit_servo 笛卡尔输入话题 (TwistStamped)'),
        DeclareLaunchArgument(
            'target_class', default_value='',
            description='目标类别名 (空 = 取最高分检测)'),
        DeclareLaunchArgument(
            'enabled', default_value='false',
            description='默认关闭; 任务层用 set_enabled 服务显式使能'),

        Node(
            package='hardware', executable='visp_servo_node',
            name='visp_servo_node', output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'target_topic': LaunchConfiguration('target_topic'),
                'servo_topic': LaunchConfiguration('servo_topic'),
                'target_class': LaunchConfiguration('target_class'),
                'enabled': LaunchConfiguration('enabled'),
                'score_threshold': 0.5,
                'publish_rate_hz': 30.0,
                'feature_timeout_sec': 0.5,
                'linear_gain': 0.5,
                'max_linear_velocity': 0.05,
                'max_angular_velocity': 0.2,
                'deadband_px': 5.0,
            }],
        ),
    ])
