# move_group.launch.py — rak 机械臂 moveit2 规划 + moveit_servo (2026-08-10).
#
# 提供:
#   - move_group      : OMPL 规划 + FollowJointTrajectory 执行 (走
#                       joint_trajectory_controller, 与 middleware 共用)
#   - servo_server    : moveit_servo 连续笛卡尔伺服 (视觉追踪用), 输入
#                       /rak/control/arm/servo_twist (geometry_msgs/TwistStamped)
#                       → 关节轨迹 → 同一 controller
#
# 前置: 先起 middleware.launch.py (ros2_control + joint_state_broadcaster
#       + joint_trajectory_controller), move_group 需要 /joint_states 反馈。
#
# 用法 (Orin):
#   ros2 launch bringup move_group.launch.py
# 提供 /move_group (moveit_msgs/action/MoveGroup) + /servo_server.
#
# 参数命名空间说明: moveit_servo 的 ServoParameters 在 "moveit_servo." 前缀下
# 读参数 (Humble 源码 servo_parameters.h, ns 默认 "moveit_servo"), 所以这里
# 用 {"moveit_servo": servo_yaml} 包一层, 与官方 demo 完全一致。

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    moveit_dir = os.path.join(bringup_share, 'config', 'moveit')
    urdf_path = os.path.join(bringup_share, 'urdf', 'rak.urdf.xacro')
    srdf_path = os.path.join(moveit_dir, 'rak.srdf')

    with open(srdf_path, 'r') as f:
        srdf_text = f.read()

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)
    robot_description_semantic = ParameterValue(srdf_text, value_type=str)

    # moveit 的 OMPL 规划配置挂在 move_group 的 'ompl' 命名空间下。
    ompl_cfg = yaml.safe_load(open(os.path.join(moveit_dir, 'ompl_planning.yaml')))
    ompl_ns = {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': ['DefaultPlanningRequestAdapter'],
        'start_state_max_bounds_error': 0.1,
        **ompl_cfg,
    }

    # 关节限位: moveit 从 robot_description_planning.joint_limits 读覆盖值。
    joint_limits = yaml.safe_load(open(os.path.join(moveit_dir, 'joint_limits.yaml')))

    # 控制器: moveit_simple_controller_manager 从 'moveit_controllers' 读。
    moveit_controllers = yaml.safe_load(open(os.path.join(moveit_dir, 'controllers.yaml')))

    move_group_params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_planning': joint_limits,
        'use_sim_time': False,
        'planning_pipelines': ['ompl'],
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'default_planning_request_adapters': ['DefaultPlanningRequestAdapter'],
        'max_planning_attempts': 10,
        'ompl': ompl_ns,
        'moveit_controllers': moveit_controllers,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'move_group_capabilities': {'capabilities': []},
    }

    # moveit_servo 参数: yaml 顶层是参数本体 (无命名空间), 包一层 moveit_servo.
    servo_yaml = yaml.safe_load(open(os.path.join(moveit_dir, 'servo_parameters.yaml')))
    servo_params = {'moveit_servo': servo_yaml}

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_joint_state_topic',
            default_value='/joint_states',
            description='joint_states 反馈话题 (middleware 的 joint_state_broadcaster)'),

        DeclareLaunchArgument(
            'servo_twist_topic',
            default_value='/rak/control/arm/servo_twist',
            description='moveit_servo 笛卡尔输入话题 (TwistStamped, 视觉追踪/连续路径用)'),

        Node(
            package='moveit_ros_move_group', executable='move_group',
            name='move_group', output='screen',
            parameters=[move_group_params],
        ),

        Node(
            package='moveit_servo', executable='servo_node_main',
            name='servo_server', output='screen',
            parameters=[
                servo_params,
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                {'use_sim_time': False},
            ],
            remappings=[
                ('/joint_states', LaunchConfiguration('arm_joint_state_topic')),
            ],
        ),
    ])
