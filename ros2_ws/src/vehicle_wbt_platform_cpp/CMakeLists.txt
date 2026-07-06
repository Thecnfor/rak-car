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
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)

# Custom message types — generate C++ headers
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneResult.msg"
  "msg/DetectionArray.msg"
  "msg/ActuatorState.msg"
  DEPENDENCIES std_msgs
)

# -----------------------------------------------------------------------------
# Library: vehicle_wbt_platform_cpp (headers + impls)
# -----------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/mc602_adapter.cpp
  src/mecanum_chassis.cpp
  src/mc602_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/vehicle_wbt_platform_cpp>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  hardware_interface
  pluginlib
  rcpputils
  "rosidl_typesupport_cpp"
)

# -----------------------------------------------------------------------------
# Executable rclcpp nodes
# -----------------------------------------------------------------------------
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs)

add_executable(infrared_node src/infrared_node.cpp)
ament_target_dependencies(infrared_node rclcpp sensor_msgs)

add_executable(mecanum_chassis_node src/mecanum_chassis_node.cpp)
ament_target_dependencies(mecanum_chassis_node
  rclcpp geometry_msgs nav_msgs tf2_ros
)
target_link_libraries(mecanum_chassis_node ${PROJECT_NAME})

add_executable(arm_node src/arm_node.cpp)
ament_target_dependencies(arm_node
  rclcpp trajectory_msgs "rosidl_typesupport_cpp"
)
target_link_libraries(arm_node ${PROJECT_NAME})

add_executable(safety_gate_node src/safety_gate_node.cpp)
ament_target_dependencies(safety_gate_node
  rclcpp geometry_msgs std_msgs
)

install(TARGETS
  camera_node infrared_node mecanum_chassis_node arm_node safety_gate_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# ros2_control hardware interface plugin (pluginlib export)
# -----------------------------------------------------------------------------
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

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

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Install custom messages runtime (msg/_*.hpp / .so)
ament_export_dependencies(rosidl_default_runtime)

# -----------------------------------------------------------------------------
# Testing
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_mc602_adapter test/test_mc602_adapter.cpp)
  target_link_libraries(test_mc602_adapter ${PROJECT_NAME})
  ament_target_dependencies(test_mc602_adapter
    rclcpp hardware_interface pluginlib
  )

  ament_add_gmock(test_mecanum_chassis test/test_mecanum_chassis.cpp)
  target_link_libraries(test_mecanum_chassis ${PROJECT_NAME})

  ament_add_gmock(test_base_controller_iface test/test_base_controller_iface.cpp)
  target_link_libraries(test_base_controller_iface ${PROJECT_NAME})
endif()

# -----------------------------------------------------------------------------
# ament_package
# -----------------------------------------------------------------------------
ament_package()
