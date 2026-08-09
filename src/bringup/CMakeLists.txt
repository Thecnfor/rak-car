cmake_minimum_required(VERSION 3.16)
project(bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# rak assembly package: launch files, URDF, calibration params, static
# nav2 maps, and the CycloneDDS config. Contains no executable code — it
# ships the files that compose the hardware + cognition packages into a
# runnable system (full_system / mock_system / dev_all / nav2_system launches).
install(
  DIRECTORY launch config params urdf maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
