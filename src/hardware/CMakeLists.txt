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
find_package(msgs REQUIRED)              # rak custom interfaces
find_package(OpenCV REQUIRED)
find_package(camera_info_manager QUIET)   # camera_node loads YAML calibration (Humble-only)
find_package(hardware_interface QUIET)
find_package(pluginlib QUIET)
find_package(rcpputils REQUIRED)

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
add_executable(camera_node src/nodes/camera_node.cpp)
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

# Optional ros2_control hardware interface plugin
if(hardware_interface_FOUND AND pluginlib_FOUND)
  add_library(_hardware_plugin SHARED src/mc602_hardware_interface.cpp)
  target_include_directories(_hardware_plugin PUBLIC include)
  target_link_libraries(_hardware_plugin PUBLIC
    rclcpp::rclcpp
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    msgs::msgs
  )
  pluginlib_export_plugin_description_file(hardware_interface plugin.xml)
  install(TARGETS _hardware_plugin
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(CAMERA_INFO_MANAGER_FOUND)
  install(TARGETS
    camera_node infrared_node mc602_bridge_node mecanum_chassis_node arm_node
    safety_gate_node mission_runner_node arm_cartesian_move_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
else()
  install(TARGETS
    infrared_node mc602_bridge_node mecanum_chassis_node arm_node safety_gate_node
    mission_runner_node arm_cartesian_move_node
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

  ament_add_gmock(test_base_controller_iface
    test/test_base_controller_iface.cpp
  )
  target_include_directories(test_base_controller_iface PUBLIC include)

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
endif()

ament_package()
