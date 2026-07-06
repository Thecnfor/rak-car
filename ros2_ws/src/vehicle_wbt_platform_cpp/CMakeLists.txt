cmake_minimum_required(VERSION 3.16)
project(vehicle_wbt_platform_cpp LANGUAGES CXX)

# -----------------------------------------------------------------------------
# C++ standard + warnings
# -----------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# Dependencies
# ros2_control is OPTIONAL: if not installed, the hardware interface plugin
# is skipped. Everything else (5 rclcpp nodes, gtest, msgs) still builds.
#
# We use find_package + target_link_libraries instead of ament_target_dependencies
# because the latter is missing from ament_cmake 2.8.7 on Lyrical
# (and ament_target_dependencies has been deprecated upstream in favor of
# plain modern CMake). This pattern works on Humble + Iron + Jazzy + Lyrical.
#
# Custom .msg files (msg/LaneResult.msg etc.) are documented in the spec
# but not generated here — rosidl_generate_interfaces has a bug in
# Lyrical 2.8.7 that fails C and Python typesupport generation. They will
# be re-enabled once ament is fixed or when running on Humble. Until then,
# the C++ nodes use standard ROS2 messages (sensor_msgs/JointState etc.).
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(hardware_interface QUIET)
find_package(pluginlib QUIET)
find_package(rcpputils REQUIRED)

# -----------------------------------------------------------------------------
# rclcpp nodes — each includes its needed core sources directly.
# We avoid a shared library to keep the CMake name space simple.
# -----------------------------------------------------------------------------
# camera_node
add_executable(camera_node src/camera_node.cpp)
target_include_directories(camera_node PUBLIC include)
target_link_libraries(camera_node PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
)

# infrared_node
add_executable(infrared_node src/infrared_node.cpp)
target_include_directories(infrared_node PUBLIC include)
target_link_libraries(infrared_node PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
)

# mecanum_chassis_node — also pulls in mecanum_chassis.cpp
add_executable(mecanum_chassis_node
  src/mecanum_chassis_node.cpp
  src/mecanum_chassis.cpp
)
target_include_directories(mecanum_chassis_node PUBLIC include)
target_link_libraries(mecanum_chassis_node PUBLIC
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  tf2_ros::tf2_ros
  tf2::tf2
  std_msgs::std_msgs
)

# arm_node — uses sensor_msgs/JointState (standard) instead of custom ActuatorState.msg
add_executable(arm_node src/arm_node.cpp)
target_include_directories(arm_node PUBLIC include)
target_link_libraries(arm_node PUBLIC
  rclcpp::rclcpp
  trajectory_msgs::trajectory_msgs
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
)

# safety_gate_node
add_executable(safety_gate_node src/safety_gate_node.cpp)
target_include_directories(safety_gate_node PUBLIC include)
target_link_libraries(safety_gate_node PUBLIC
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  std_msgs::std_msgs
)

# Optional ros2_control hardware interface plugin
if(hardware_interface_FOUND AND pluginlib_FOUND)
  add_library(_vehicle_wbt_platform_cpp_plugin SHARED src/mc602_hardware_interface.cpp)
  target_include_directories(_vehicle_wbt_platform_cpp_plugin PUBLIC include)
  target_link_libraries(_vehicle_wbt_platform_cpp_plugin PUBLIC
    rclcpp::rclcpp
    hardware_interface::hardware_interface
    pluginlib::pluginlib
  )
  pluginlib_export_plugin_description_file(hardware_interface plugin.xml)
  install(TARGETS _vehicle_wbt_platform_cpp_plugin
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS
  camera_node infrared_node mecanum_chassis_node arm_node safety_gate_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch config params urdf
  DESTINATION share/${PROJECT_NAME}
)

# Note: .msg files are installed as documentation (not generated)
install(DIRECTORY msg/ DESTINATION share/${PROJECT_NAME}/msg)

# -----------------------------------------------------------------------------
# Testing
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_mc602_adapter
    test/test_mc602_adapter.cpp
    src/mc602_adapter.cpp
  )
  target_include_directories(test_mc602_adapter PUBLIC include)
  # ament_add_gmock uses plain target_link_libraries internally, so we
  # must use the plain (un-keyworded) signature here for consistency.
  target_link_libraries(test_mc602_adapter rclcpp::rclcpp pluginlib::pluginlib)

  ament_add_gmock(test_mecanum_chassis
    test/test_mecanum_chassis.cpp
    src/mecanum_chassis.cpp
  )
  target_include_directories(test_mecanum_chassis PUBLIC include)

  ament_add_gmock(test_base_controller_iface
    test/test_base_controller_iface.cpp
  )
  target_include_directories(test_base_controller_iface PUBLIC include)
endif()

ament_package()
