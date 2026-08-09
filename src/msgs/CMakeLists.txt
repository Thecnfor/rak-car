cmake_minimum_required(VERSION 3.16)
project(msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# rak custom interface definitions. Keeping these in a dedicated
# package lets the Python cognition layer depend on msgs alone (not on the
# whole hardware package), and lets any node reuse them.
rosidl_generate_interfaces(msgs
  "action/ChassisNavigate.action"
  "action/ArmCartesianMove.action"
  "action/ArmExecuteTrajectory.action"
  "msg/ActuatorState.msg"
  "msg/CameraMeta.msg"
  "msg/DetectionArray.msg"
  "msg/Frame.msg"
  "msg/LaneResult.msg"
  "msg/Pose2D.msg"
  "msg/ArmKinematicState.msg"
  "msg/ActuatorDiagnostics.msg"
  "srv/Mc602Transaction.srv"
  "srv/SetBeep.srv"
  "srv/SetRgbLed.srv"
  "srv/LedShow.srv"
  "srv/SetNixie.srv"
  "srv/SetDigitalOut.srv"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs sensor_msgs std_msgs trajectory_msgs
)

ament_package()
