# middleware.launch.py — 中间层主控制平面 (2026-08-09).
#
# Spec: docs/superpowers/specs/2026-08-09-midlayer-ros2control-cartesian-design.md §9
#
# 拓扑:
#   mc602_bridge_node (唯一串口 owner)
#     → ros2_control (controller_manager + MC602HardwareInterface, 走 bridge)
#         ├── mecanum_drive_controller
#         ├── joint_trajectory_controller
#         └── joint_state_broadcaster
#     → action servers (chassis_navigate / arm_cartesian_move)
#     → peripheral_node
#
# 前置: Orin 需要 ros2_control 全家 (见 config/middleware_controllers.yaml 注释)。

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    urdf_path = os.path.join(bringup_share, "urdf", "rak.urdf.xacro")
    controllers_yaml = os.path.join(bringup_share, "config", "middleware_controllers.yaml")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]), value_type=str)

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),

        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud", default_value="1000000"),

        # ---- 1. MC602 单总线 owner ----
        Node(
            package="hardware",
            executable="mc602_bridge_node",
            name="mc602_bridge",
            output="screen",
            parameters=[{
                "mc602_serial_port": LaunchConfiguration("serial_port"),
                "mc602_baud": LaunchConfiguration("baud"),
            }],
        ),

        # ---- 2. robot_state_publisher (发布 /robot_description + TF) ----
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # ---- 3. ros2_control 主控制平面 ----
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                controllers_yaml,
            ],
            remappings=[("/controller_manager/robot_description", "/robot_description")],
        ),

        # ---- 4. controllers (延迟等 controller_manager 就绪) ----
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "control", "load_controller", "--set-state", "active",
                         "joint_state_broadcaster"],
                    output="screen",
                ),
            ],
        ),
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "control", "load_controller", "--set-state", "active",
                         "mecanum_drive_controller"],
                    output="screen",
                ),
            ],
        ),
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "control", "load_controller", "--set-state", "active",
                         "joint_trajectory_controller"],
                    output="screen",
                ),
            ],
        ),

        # ---- 5. 中间层 action servers + 外设 ----
        Node(
            package="hardware",
            executable="chassis_navigate_node",
            name="chassis_navigate",
            output="screen",
            parameters=[{
                "odom_topic": "/odom",
                "cmd_topic": "/rak/cmd/vel_raw",
            }],
        ),
        Node(
            package="hardware",
            executable="arm_cartesian_move_node",
            name="arm_cartesian_move",
            output="screen",
            parameters=[{
                "traj_topic": "/joint_trajectory_controller/joint_trajectory",
                "joint_state_topic": "/joint_states",
            }],
        ),
        Node(
            package="hardware",
            executable="peripheral_node",
            name="peripheral",
            output="screen",
            parameters=[{
                "mc602_serial_port": LaunchConfiguration("serial_port"),
                "mc602_baud": LaunchConfiguration("baud"),
                "mc602_transport": "bridge",
            }],
        ),
    ])
