cmake_minimum_required(VERSION 3.16)
project(vehicle_wbt_smartcar_msgs LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(SRV_FILES
  "srv/MoveFor.srv"
  "srv/MoveToPosition.srv"
  "srv/MoveTime.srv"
  "srv/MoveDistance.srv"
  "srv/ResetOdometry.srv"
  "srv/LaneBase.srv"
  "srv/LaneTime.srv"
  "srv/LaneDis.srv"
  "srv/LaneDisOffset.srv"
  "srv/ArmReset.srv"
  "srv/ArmSetPose.srv"
  "srv/ArmGrasp.srv"
  "srv/ArmMoveX.srv"
  "srv/ArmMoveY.srv"
  "srv/ArmSetArmAngle.srv"
  "srv/ArmSetHandAngle.srv"
  "srv/Beep.srv"
  "srv/SetStorage.srv"
  "srv/Shoot.srv"
  # Phase 1 additive (DO NOT REMOVE existing entries above)
  "srv/SetWheels.srv"
  "srv/ReadEncoders.srv"
  "srv/ResetEncoders.srv"
  "srv/SetServoPwm.srv"
  "srv/SetServoBus.srv"
  "srv/SetStepper.srv"
  "srv/SetDcMotor.srv"
  "srv/SetDout.srv"
  "srv/ReadIR.srv"
  "srv/ReadBattery.srv"
  "srv/ReadAnalog.srv"
  "srv/Buzzer.srv"
  "srv/PlayPredefined.srv"
  "srv/ReadTouch.srv"
  "srv/ReadUltrasonic.srv"
  "srv/ReadAmbient.srv"
  "srv/ReadBluetooth.srv"
  "srv/ReadKey4.srv"
  "srv/SetLed.srv"
  "srv/SetNixie.srv"
  "srv/ShowScreen.srv"
)

set(MSG_FILES
  "msg/ArmPose.msg"
  "msg/ChassisState.msg"
  "msg/BridgeStatus.msg"
  # Phase 1 additive
  "msg/RawState.msg"
  "msg/MelodyNote.msg"
  "msg/Melody.msg"
  "msg/ButtonEvent.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_runtime_cpp)

ament_package()