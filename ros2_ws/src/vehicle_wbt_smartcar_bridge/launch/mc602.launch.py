"""MC602 节点启动文件。

启动 mc602_node — Jetson 端通用 MC602 IO 节点,12 service + 2 topic,独占 /dev/ttyUSB*。
dev box 同事通过 LAN DDS 直接调 /vehicle_wbt/v1/mc602/* service,无需 SSH Jetson。

与 smartcar_bridge.launch.py 并存(后者跑老 smartcar_bridge_node,服务于
vehicle_wbt_smartcar_sdk 老用户)。两个 launch 可同时跑,不同 node name 不冲突。

Run:
  ros2 launch vehicle_wbt_smartcar_bridge mc602.launch.py
  # Override serial port:
  ros2 launch vehicle_wbt_smartcar_bridge mc602.launch.py \
      serial_port:=/dev/ttyUSB1 baud:=1000000
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 强制 ROS_DOMAIN_ID 让 dev box 通过 LAN DDS 自动发现本节点
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        # 强制 CycloneDDS(对齐 Jetson env file);如果 RMW_OVERRIDE 已设(Jetson 端
        # 装了 Fast DDS / Connext 等),跳过强制,让本机 dev 也能跑 launch
        # Set env var only if RMW_OVERRIDE is unset
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp',
            condition=IfCondition(
                PythonExpression(["'", EnvironmentVariable('RMW_OVERRIDE', default_value=''), "' == ''"])
            ),
        ),

        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB1',
            description='MC602 串口设备路径'),
        DeclareLaunchArgument(
            'baud', default_value='1000000',
            description='MC602 波特率(默认 1Mbps = SDK MC602 默认)'),
        DeclareLaunchArgument(
            'control_rate_hz', default_value='50.0',
            description='控制循环频率(Hz)'),
        DeclareLaunchArgument(
            'sensor_rate_hz', default_value='20.0',
            description='传感器发布频率(Hz)'),
        DeclareLaunchArgument(
            'enable_chassis_kinematics', default_value='true',
            description='是否启动 chassis_kinematics_node(/cmd_vel /odom /tf)'),
        DeclareLaunchArgument(
            'chassis_track', default_value='0.30',
            description='麦纳姆轮距 (m)'),
        DeclareLaunchArgument(
            'chassis_wheel_base', default_value='0.28',
            description='麦纳姆轴距 (m)'),
        DeclareLaunchArgument(
            'chassis_wheel_radius', default_value='0.03',
            description='麦纳姆轮半径 (m)'),

        Node(
            package='vehicle_wbt_smartcar_bridge',
            executable='mc602_node',
            name='mc602_io',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud': LaunchConfiguration('baud'),
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                'sensor_rate_hz': LaunchConfiguration('sensor_rate_hz'),
            }],
        ),

        # 麦纳姆运动学子节点:Twist → 4 轮 + encoder → /odom + /tf
        # 默认开启;不需要时传 enable_chassis_kinematics:=false
        Node(
            condition=IfCondition(LaunchConfiguration('enable_chassis_kinematics')),
            package='vehicle_wbt_smartcar_bridge',
            executable='chassis_kinematics_node',
            name='chassis_kinematics',
            output='screen',
            parameters=[{
                'track': LaunchConfiguration('chassis_track'),
                'wheel_base': LaunchConfiguration('chassis_wheel_base'),
                'wheel_radius': LaunchConfiguration('chassis_wheel_radius'),
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            }],
        ),
    ])
