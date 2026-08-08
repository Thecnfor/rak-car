# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""
One-click dev launch — start robot_state_publisher (URDF) + the 5
dev-sidecar stub nodes (camera/IR/chassis/arm/safety_gate) +
mission_runner_node in a single process.

This is the "no-hardware development" entry point. Run it from any
machine that has ROS2 Humble or Jazzy and our colcon workspace built,
and you'll see the full system run with stub data on the topics.

After this is up:
  - ros2 topic list                          (see all active topics)
  - ros2 topic echo /rak/state/odom  (see odom)
  - rviz2 -d urdf/rak.rviz          (3D visualization)
  - ros2 topic echo /rak/state/mission_progress  (task status)

Usage:
  ros2 launch bringup dev_all.launch.py
  ros2 launch bringup dev_all.launch.py \\
      with_mission:='[seeding]'                    # also run a mission
  ros2 launch bringup dev_all.launch.py \\
      with_rviz:=true                              # try to open RViz

Note: the included stub nodes are dev-sidecar only — they have NO production
trigger path. See docs/development/no-hw-dev.md and CLAUDE.md "no mocks in
production code" rule for the distinction between dev stubs and production mocks.

Spec: docs/development/no-hw-dev.md
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import (
    LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ----- Args -----
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("bringup"),
            "urdf", "rak.urdf.xacro",
        ]),
        description="Absolute path to the URDF xacro file",
    )
    with_rviz_arg = DeclareLaunchArgument(
        "with_rviz", default_value="false",
        description="If true, also launch RViz2 in the background (requires DISPLAY)",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([
            FindPackageShare("bringup"),
            "urdf", "rak.rviz",
        ]),
        description="RViz2 config file to load",
    )
    with_mission_arg = DeclareLaunchArgument(
        "with_mission", default_value="",
        description="Task list for MissionRunnerNode (e.g. '[seeding]'). "
                    "Only used when run_mission:=true.",
    )
    run_mission_arg = DeclareLaunchArgument(
        "run_mission", default_value="false",
        description="If true, start MissionRunnerNode with the with_mission task list",
    )
    mission_timeout_arg = DeclareLaunchArgument(
        "mission_timeout_sec", default_value="30.0",
        description="Per-task timeout in seconds",
    )

    # ----- robot_state_publisher: loads URDF, publishes /tf -----
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", LaunchConfiguration("urdf_path")]),
                value_type=str,
            ),
        }],
    )

    # ----- Include the 5 dev-sidecar stub nodes (camera/IR/chassis/arm/safety_gate).
    #       Variable name kept as mock_system for parity with the included
    #       launch file (mock_system.launch.py); the included file is dev-only
    #       and has no production trigger path. -----
    mock_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("bringup"),
            "/launch/mock_system.launch.py",
        ]),
    )

    # ----- Optional: MissionRunnerNode (drives a task list) -----
    mission_runner = Node(
        package="hardware",
        executable="mission_runner_node",
        name="mission_runner_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_mission")),
        parameters=[{
            "task_list": LaunchConfiguration("with_mission"),
            "task_timeout_sec": LaunchConfiguration("mission_timeout_sec"),
        }],
    )

    # ----- Optional: RViz2 in background (needs DISPLAY) -----
    rviz2 = ExecuteProcess(
        cmd=["rviz2", "-d", LaunchConfiguration("rviz_config")],
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("with_rviz")),
    )

    return LaunchDescription([
        urdf_path_arg,
        with_rviz_arg,
        rviz_config_arg,
        with_mission_arg,
        run_mission_arg,
        mission_timeout_arg,
        robot_state_publisher,
        mock_system,
        mission_runner,
        rviz2,
    ])
