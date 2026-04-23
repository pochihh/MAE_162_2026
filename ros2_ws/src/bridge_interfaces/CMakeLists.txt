cmake_minimum_required(VERSION 3.8)
project(bridge_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  # Global GPS / field localizer
  "msg/TagDetection.msg"
  "msg/TagDetectionArray.msg"
  # Raw bridge telemetry
  "msg/SystemState.msg"
  "msg/SystemPower.msg"
  "msg/SystemInfo.msg"
  "msg/SystemConfig.msg"
  "msg/SystemDiag.msg"
  "msg/SysOdomParamReq.msg"
  "msg/SysOdomParamRsp.msg"
  "msg/DCPid.msg"
  "msg/DCMotorState.msg"
  "msg/DCStateAll.msg"
  "msg/StepConfig.msg"
  "msg/StepperState.msg"
  "msg/StepStateAll.msg"
  "msg/ServoChannelState.msg"
  "msg/ServoStateAll.msg"
  "msg/SensorImu.msg"
  "msg/SensorKinematics.msg"
  "msg/SensorMagCalStatus.msg"
  "msg/IOInputState.msg"
  "msg/IOOutputState.msg"
  # Raw bridge commands
  "msg/SysConfigSet.msg"
  "msg/SysOdomParamSet.msg"
  "msg/SysOdomReset.msg"
  "msg/DCEnable.msg"
  "msg/DCSetPosition.msg"
  "msg/DCSetVelocity.msg"
  "msg/DCSetPwm.msg"
  "msg/DCResetPosition.msg"
  "msg/DCHome.msg"
  "msg/DCPidReq.msg"
  "msg/DCPidSet.msg"
  "msg/StepEnable.msg"
  "msg/StepMove.msg"
  "msg/StepHome.msg"
  "msg/StepConfigReq.msg"
  "msg/StepConfigSet.msg"
  "msg/ServoEnable.msg"
  "msg/ServoSet.msg"
  "msg/SensorMagCalCmd.msg"
  "msg/IOSetLed.msg"
  "msg/IOSetNeopixel.msg"
  "msg/SysCommand.msg"
)

set(srv_files
  "srv/SetFirmwareState.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
