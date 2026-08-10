cmake_minimum_required(VERSION 3.16)
project(hardware LANGUAGES C CXX)

# -----------------------------------------------------------------------------
# C++ standard + warnings
# -----------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Installed executables and sibling shared libs (seeding_task, task_registry)
# are co-located in lib/${PROJECT_NAME}, so a bare $ORIGIN rpath resolves
# them from the install tree (ros2 run works, not just build/).
set(CMAKE_INSTALL_RPATH "$ORIGIN")

# -----------------------------------------------------------------------------
# Dependencies
# ros2_control is OPTIONAL: if not installed, the hardware interface plugin
# is skipped. Everything else (rclcpp nodes, gtest) still builds.
#
# We use find_package + target_link_libraries instead of ament_target_dependencies
# because the latter is missing from ament_cmake 2.8.7 on Lyrical
# (and ament_target_dependencies has been deprecated upstream in favor of
# plain modern CMake). This pattern works on Humble + Iron + Jazzy + Lyrical.
#
# Custom interfaces (CameraMeta, LaneResult, DetectionArray, ActuatorState)
# live in the separate `msgs` package (src/msgs) — this package consumes them,
# it does not generate them.
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)           # visp_servo_node 的 set_enabled 服务
find_package(msgs REQUIRED)              # rak custom interfaces
find_package(OpenCV REQUIRED)
find_package(camera_info_manager QUIET)   # camera_node loads YAML calibration (Humble-only)
find_package(hardware_interface QUIET)
find_package(pluginlib QUIET)
find_package(rcpputils REQUIRED)
find_package(nav2_msgs QUIET)             # optional: nav2 FollowWaypoints behavior backend
find_package(moveit_msgs QUIET)           # optional: moveit MoveGroup arm backend
# ViSP visual-servo node (Orin-only). 注意必须大写 find_package(VISP):
# 上游导出的是 VISPConfig.cmake (大写), 小写 find_package(visp) 在区分大小写
# 的文件系统上找不到 (visp_DIR-NOTFOUND)。
# COMPONENTS 只取本节点用到的纯数学模块 (vs/visual_features/core):
# 链接全套 VISP_LIBRARIES 会把 visp_sensor 拉进来, 而它 INTERFACE 依赖
# 未安装的 libdc1394 → "No rule to make target" 链接失败。
find_package(VISP COMPONENTS vs visual_features core QUIET)

# Track whether camera_info_manager is available.
set(CAMERA_INFO_MANAGER_FOUND ${camera_info_manager_FOUND})

# -----------------------------------------------------------------------------
# rclcpp nodes — each includes its needed core sources directly.
# We avoid a shared library to keep the CMake name space simple.
# -----------------------------------------------------------------------------
# camera_node — publishes 5 streams per camera: image_raw, image_compressed,
# camera_info, camera_status, camera_meta. Live V4L2 capture via OpenCV
# (no synthetic fallback). camera_info loaded from YAML via
# camera_info_manager. Publishes /tf_static for camera optical frame.
# OPTIONAL: skipped on distros without camera_info_manager (Lyrical).
#
# Node entry points (main) live in src/nodes/; core logic (adapters, chassis,
# state machines) stays in src/. See CLAUDE.md layout conventions.
if(CAMERA_INFO_MANAGER_FOUND)
add_executable(camera_node
  src/nodes/camera_node.cpp
  src/camera_capture_config.cpp
)
target_include_directories(camera_node PUBLIC include)
target_link_libraries(camera_node PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  diagnostic_msgs::diagnostic_msgs
  tf2_ros::tf2_ros
  tf2::tf2
  geometry_msgs::geometry_msgs
  camera_info_manager::camera_info_manager
  msgs::msgs
  opencv_core
  opencv_imgcodecs
  opencv_imgproc
  opencv_videoio
)
endif()

# infrared_node — uses MC602Adapter::read_infrared for real IR sensor reads
add_executable(infrared_node
  src/nodes/infrared_node.cpp
  src/mc602_adapter.cpp
  src/mc602_protocol.cpp
  src/serial_transport.cpp
  src/bridge_transport.cpp
  src/serial_port.cpp
)
target_include_directories(infrared_node PUBLIC include)
target_link_libraries(infrared_node PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  msgs::msgs
)

# mc602_bridge_node — single owner of the MC602 serial bus (SerialScheduler)
add_executable(mc602_bridge_node
  src/nodes/mc602_bridge_node.cpp
  src/serial_scheduler.cpp
  src/mc602_bootloader.cpp
  src/serial_port.cpp
)
target_include_directories(mc602_bridge_node PUBLIC include)
target_link_libraries(mc602_bridge_node PUBLIC
  rclcpp::rclcpp
  std_msgs::std_msgs
  msgs::msgs
)

# mecanum_chassis_node — also pulls in mecanum_chassis.cpp + mc602_adapter.cpp
add_executable(mecanum_chassis_node
  src/nodes/mecanum_chassis_node.cpp
  src/mecanum_chassis.cpp
  src/mc602_adapter.cpp
  src/mc602_protocol.cpp
  src/serial_transport.cpp
  src/bridge_transport.cpp
  src/serial_port.cpp
)
target_include_directories(mecanum_chassis_node PUBLIC include)
target_link_libraries(mecanum_chassis_node PUBLIC
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  sensor_msgs::sensor_msgs
  tf2_ros::tf2_ros
  tf2::tf2
  std_msgs::std_msgs
  msgs::msgs
)

# arm_node — uses MC602Adapter for joint command dispatch (burst-packed)
add_executable(arm_node
  src/nodes/arm_node.cpp
  src/mc602_adapter.cpp
  src/mc602_protocol.cpp
  src/serial_transport.cpp
  src/bridge_transport.cpp
  src/serial_port.cpp
)
target_include_directories(arm_node PUBLIC include)
target_link_libraries(arm_node PUBLIC
  rclcpp::rclcpp
  trajectory_msgs::trajectory_msgs
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  msgs::msgs
)

# safety_gate_node
add_executable(safety_gate_node src/nodes/safety_gate_node.cpp)
target_include_directories(safety_gate_node PUBLIC include)
target_link_libraries(safety_gate_node PUBLIC
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  std_msgs::std_msgs
)

# mission_runner_node — thin ROS2 shell; the mission state machine lives in
# the rclcpp-free MissionRunner class (src/mission_runner.cpp).
add_executable(mission_runner_node
  src/nodes/mission_runner_node.cpp
  src/mission_runner.cpp
)
target_include_directories(mission_runner_node PUBLIC include)
target_link_libraries(mission_runner_node PUBLIC
  rclcpp::rclcpp
  std_msgs::std_msgs
)

# arm_cartesian_move_node — ArmCartesianMove action server (middle layer).
# Thin shell over ArmCartesianPlanner; publishes a JointTrajectory to
# joint_trajectory_controller and monitors /joint_states for feedback.
add_executable(arm_cartesian_move_node
  src/nodes/arm_cartesian_move_node.cpp
  src/arm_cartesian_planner.cpp
  src/arm_kinematic_model.cpp
)
target_include_directories(arm_cartesian_move_node PUBLIC include)
target_link_libraries(arm_cartesian_move_node PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  sensor_msgs::sensor_msgs
  trajectory_msgs::trajectory_msgs
  msgs::msgs
)

# peripheral_node — beep / led / nixie / dout services over the MC602 bridge.
add_executable(peripheral_node
  src/nodes/peripheral_node.cpp
  src/mc602_adapter.cpp
  src/mc602_protocol.cpp
  src/serial_transport.cpp
  src/bridge_transport.cpp
  src/serial_port.cpp
)
target_include_directories(peripheral_node PUBLIC include)
target_link_libraries(peripheral_node PUBLIC
  rclcpp::rclcpp
  msgs::msgs
)

# chassis_navigate_node — ChassisNavigate action server (pose → cmd_vel).
add_executable(chassis_navigate_node
  src/nodes/chassis_navigate_node.cpp
)
target_include_directories(chassis_navigate_node PUBLIC include)
target_link_libraries(chassis_navigate_node PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  nav_msgs::nav_msgs
  geometry_msgs::geometry_msgs
  msgs::msgs
)

# visp_servo_node — ViSP 视觉伺服 (eye-to-hand 图像居中) → moveit_servo.
# OPTIONAL: 仅 Orin (Humble) 装了 ViSP 才编译; dev (Lyrical) 无 visp 包跳过,
# 与 nav2_msgs/moveit_msgs 同一可选依赖模式。package.xml 不声明 visp,
# 避免 dev 端 colcon 依赖解析失败。
if(VISP_FOUND)
  add_executable(visp_servo_node src/nodes/visp_servo_node.cpp)
  target_include_directories(visp_servo_node PUBLIC
    include
    ${VISP_INCLUDE_DIRS}   # visp3/... headers + 依赖的 opencv/eigen 头
  )
  target_link_libraries(visp_servo_node PUBLIC
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    geometry_msgs::geometry_msgs
    diagnostic_msgs::diagnostic_msgs
    std_srvs::std_srvs
    tf2_ros::tf2_ros
    tf2::tf2
    msgs::msgs
    ${VISP_LIBRARIES}
  )
  install(TARGETS visp_servo_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Optional ros2_control hardware interface plugin
if(hardware_interface_FOUND AND pluginlib_FOUND)
  # bridge_transport.cpp must be compiled INTO the plugin — a shared library
  # tolerates undefined symbols at link time, but controller_manager would
  # fail to dlopen the plugin at runtime without them.
  add_library(hardware SHARED
    src/mc602_hardware_interface.cpp
    src/bridge_transport.cpp
  )
  target_include_directories(hardware PUBLIC include)
  target_link_libraries(hardware PUBLIC
    rclcpp::rclcpp
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    msgs::msgs
  )
  pluginlib_export_plugin_description_file(hardware_interface plugin.xml)
  install(TARGETS hardware
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    find_package(ament_cmake_gmock REQUIRED)
    ament_add_gmock(test_mc602_hardware_interface
      test/test_mc602_hardware_interface.cpp
      src/mc602_hardware_interface.cpp
      src/mc602_adapter.cpp
      src/mc602_protocol.cpp
      src/serial_transport.cpp
      src/serial_port.cpp
      src/bridge_transport.cpp
    )
    target_include_directories(test_mc602_hardware_interface PUBLIC include)
    target_link_libraries(test_mc602_hardware_interface
      rclcpp::rclcpp
      hardware_interface::hardware_interface
      msgs::msgs
    )
  endif()
endif()

if(CAMERA_INFO_MANAGER_FOUND)
  install(TARGETS
    camera_node infrared_node mc602_bridge_node mecanum_chassis_node arm_node
    safety_gate_node mission_runner_node arm_cartesian_move_node
    peripheral_node chassis_navigate_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
else()
  install(TARGETS
    infrared_node mc602_bridge_node mecanum_chassis_node arm_node safety_gate_node
    mission_runner_node arm_cartesian_move_node peripheral_node chassis_navigate_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# SeedingTask is a library linked into the mission_runner_node — it has
# a REGISTER_TASK static initializer that registers itself at startup.
# With --as-needed (default), the linker drops libraries the executable
# references no symbol from, so the initializer would never run. Force the
# library to load with --no-as-needed.
add_library(seeding_task SHARED src/seeding_task.cpp)
target_include_directories(seeding_task PUBLIC include)
target_link_libraries(seeding_task PUBLIC
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  std_msgs::std_msgs
  behavior_client
)
target_link_libraries(mission_runner_node PUBLIC
  "-Wl,--no-as-needed"
  seeding_task
  "-Wl,--as-needed"
)
install(TARGETS seeding_task
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# TaskRegistry singleton — used by mission_runner_node to look up tasks by name.
add_library(task_registry SHARED src/task_registry.cpp)
target_include_directories(task_registry PUBLIC include)
target_link_libraries(task_registry PUBLIC
  rclcpp::rclcpp
)
target_link_libraries(mission_runner_node PUBLIC task_registry)
install(TARGETS task_registry
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# BehaviorClient — task-facing adapters over the rak action contract.
# nav2_msgs is optional: when present the FollowWaypoints backend is compiled
# in (RAK_HAVE_NAV2); otherwise chassis falls back to the local ChassisNavigate
# P-loop. Same optional-dependency pattern as ros2_control above.
add_library(behavior_client SHARED src/behavior_client.cpp
  src/arm_cartesian_planner.cpp
  src/arm_kinematic_model.cpp
)
target_include_directories(behavior_client PUBLIC include)
target_link_libraries(behavior_client PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  std_msgs::std_msgs
  msgs::msgs
)
if(nav2_msgs_FOUND)
  target_compile_definitions(behavior_client PUBLIC RAK_HAVE_NAV2)
  target_link_libraries(behavior_client PUBLIC nav2_msgs::nav2_msgs)
endif()
if(moveit_msgs_FOUND)
  target_compile_definitions(behavior_client PUBLIC RAK_HAVE_MOVEIT)
  target_link_libraries(behavior_client PUBLIC moveit_msgs::moveit_msgs)
endif()
install(TARGETS behavior_client
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Launch files, URDF, calibration params, and DDS config ship in the
# `bringup` package (src/bringup), not here.

# -----------------------------------------------------------------------------
# Testing
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_camera_capture_config
    test/test_camera_capture_config.cpp
    src/camera_capture_config.cpp
  )
  target_include_directories(test_camera_capture_config PUBLIC include)
  target_link_libraries(test_camera_capture_config opencv_videoio)

  ament_add_gmock(test_mc602_adapter
    test/test_mc602_adapter.cpp
    src/mc602_adapter.cpp
    src/mc602_protocol.cpp
    src/serial_transport.cpp
    src/serial_port.cpp
  )
  target_include_directories(test_mc602_adapter PUBLIC include)

  # Protocol-layer tests — frame structure, parameter encoding, port validation
  ament_add_gmock(test_mc602_protocol
    test/test_mc602_protocol.cpp
    src/mc602_adapter.cpp
    src/mc602_protocol.cpp
    src/serial_transport.cpp
    src/serial_port.cpp
  )
  target_include_directories(test_mc602_protocol PUBLIC include)
  target_link_libraries(test_mc602_protocol rclcpp::rclcpp pluginlib::pluginlib)

  # ament_add_gmock uses plain target_link_libraries internally, so we
  # must use the plain (un-keyworded) signature here for consistency.
  target_link_libraries(test_mc602_adapter rclcpp::rclcpp pluginlib::pluginlib)

  # SerialScheduler — priority ordering, single-owner serialization, timeouts
  ament_add_gmock(test_serial_scheduler
    test/test_serial_scheduler.cpp
    src/serial_scheduler.cpp
  )
  target_include_directories(test_serial_scheduler PUBLIC include)

  # SerialPort unit tests — injection seam, frame pass-through, error handling
  ament_add_gmock(test_serial_port
    test/test_serial_port.cpp
    src/serial_port.cpp
  )
  target_include_directories(test_serial_port PUBLIC include)

  # BridgeTransport — executor-safety regression (P0, spec §7.1)
  ament_add_gmock(test_bridge_transport
    test/test_bridge_transport.cpp
    src/bridge_transport.cpp
  )
  target_include_directories(test_bridge_transport PUBLIC include)
  # plain signature (ament_add_gmock uses plain target_link_libraries internally)
  target_link_libraries(test_bridge_transport rclcpp::rclcpp msgs::msgs)

  ament_add_gmock(test_mecanum_chassis
    test/test_mecanum_chassis.cpp
    src/mecanum_chassis.cpp
  )
  target_include_directories(test_mecanum_chassis PUBLIC include)

  # ArmKinematicModel — pure serial-chain FK + analytic task-space IK
  ament_add_gmock(test_arm_kinematic_model
    test/test_arm_kinematic_model.cpp
    src/arm_kinematic_model.cpp
  )
  target_include_directories(test_arm_kinematic_model PUBLIC include)

  # ArmCartesianPlanner — goal → joint-space plan + rejection semantics
  ament_add_gmock(test_arm_cartesian_planner
    test/test_arm_cartesian_planner.cpp
    src/arm_cartesian_planner.cpp
    src/arm_kinematic_model.cpp
  )
  target_include_directories(test_arm_cartesian_planner PUBLIC include)

  # Pure-logic safety gate tests — no rclcpp needed, just gtest
  ament_add_gmock(test_safety_gate_logic
    test/test_safety_gate_logic.cpp
  )
  target_include_directories(test_safety_gate_logic PUBLIC include)

  # Pure task framework tests — no rclcpp needed (uses nullptr sentinel)
  ament_add_gmock(test_base_task
    test/test_base_task.cpp
  )
  target_include_directories(test_base_task PUBLIC include)
  target_link_libraries(test_base_task task_registry)

  # MissionRunner state machine — pure class, fake tasks + synthetic clocks
  ament_add_gmock(test_mission_runner
    test/test_mission_runner.cpp
    src/mission_runner.cpp
  )
  target_include_directories(test_mission_runner PUBLIC include)

  # BehaviorClient — end-to-end vs an in-process ChassisNavigate action server
  ament_add_gmock(test_behavior_client
    test/test_behavior_client.cpp
    src/behavior_client.cpp
    src/arm_cartesian_planner.cpp
    src/arm_kinematic_model.cpp
  )
  target_include_directories(test_behavior_client PUBLIC include)
  target_link_libraries(test_behavior_client rclcpp::rclcpp rclcpp_action::rclcpp_action msgs::msgs)
  if(nav2_msgs_FOUND)
    target_compile_definitions(test_behavior_client PUBLIC RAK_HAVE_NAV2)
    target_link_libraries(test_behavior_client nav2_msgs::nav2_msgs)
  endif()
  if(moveit_msgs_FOUND)
    target_compile_definitions(test_behavior_client PUBLIC RAK_HAVE_MOVEIT)
    target_link_libraries(test_behavior_client moveit_msgs::moveit_msgs)
  endif()
endif()

ament_package()
