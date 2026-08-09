# nav2_system.launch.py — rak 静态场地导航栈 (2026-08-10).
#
# 不做 SLAM/AMCL (无激光, ADR-003 决策): 假定位 = 静态 map→odom 恒等变换,
# 适用于"场地固定起点"的竞赛场景。要换真实定位, 把 fake_localization 换成
# amcl / slam_toolbox 并把 map 换成真实场地图即可。
#
# 关键接线 (安全门永远在环内):
#   nav2 controller 输出 /cmd_vel ──remap──> /rak/cmd/vel_raw
#       → safety_gate → /rak/cmd/vel_safe → mecanum_chassis
#   nav2 读 /rak/state/odom (odom_topic 参数, 已在 nav2_params.yaml 配好)
#
# 行为树 XML 路径: 不写死 /opt/ros/humble/... 具体路径 (跨发行版规则),
# 启动时用 get_package_share_directory('nav2_bt_navigator') 解析并注入。
#
# 用法 (Orin):
#   ros2 launch bringup nav2_system.launch.py
# 提供 /follow_waypoints (nav2_msgs/action/FollowWaypoints),
# BehaviorClient 在 nav2 在线时自动切到该后端。

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# nav2 默认话题 → rak 契约。cmd_vel 必须走 vel_raw (过 safety_gate)。
nav2_remaps = [
    ('/cmd_vel', '/rak/cmd/vel_raw'),
    ('/odom', '/rak/state/odom'),
]

nav2_core_nodes = [
    ('nav2_planner', 'planner_server'),
    ('nav2_controller', 'controller_server'),
    ('nav2_smoother', 'smoother_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_waypoint_follower', 'waypoint_follower'),
]


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    params_file = os.path.join(bringup_share, 'params', 'nav2_params.yaml')
    map_yaml = os.path.join(bringup_share, 'maps', 'arena.yaml')

    # 加载统一 params, 注入行为树 XML 路径 (按 nav2_bt_navigator 实际安装位置).
    params = yaml.safe_load(open(params_file, 'r'))
    bt_dir = os.path.join(
        get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')
    bt_ns = params['bt_navigator']['ros__parameters']
    bt_ns['navigate_to_pose_bt_xml'] = os.path.join(
        bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml')
    bt_ns['navigate_through_poses_bt_xml'] = os.path.join(
        bt_dir, 'navigate_through_poses_w_replanning_and_recovery.xml')
    bt_ns['follow_waypoints_bt_xml'] = os.path.join(
        bt_dir, 'follow_waypoints.xml')

    nodes = []

    # 1. 静态地图 (map_server) + 其生命周期
    nodes.append(Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[{'yaml_filename': map_yaml}],
    ))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server']}],
    ))

    # 2. 假定位: 静态 map→odom 恒等 (场地固定起点; 无 AMCL/激光)
    nodes.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='fake_localization', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    ))

    # 3. nav2 核心栈 (同一 dict, 各节点读自己的命名空间)
    for pkg, exe in nav2_core_nodes:
        nodes.append(Node(
            package=pkg, executable=exe, output='screen',
            parameters=[params], remappings=nav2_remaps,
        ))

    # 4. 核心栈生命周期管理 (统一 configure/activate)
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [n[1] for n in nav2_core_nodes],
        }],
    ))

    return LaunchDescription(nodes)
