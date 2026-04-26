cmake_minimum_required(VERSION 3.8)
project(drone_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TeleopCommand.msg"
  "msg/TeleopAction.msg"
  "msg/DroneState.msg"
  "msg/DroneInfo.msg"
  "msg/GcsHeartbeat.msg"
  "msg/Toggle.msg"
  "msg/FrameData.msg"
  "msg/AltitudeControllerInput.msg"
  "msg/McuVerticalEstimate.msg"
  "srv/SetPidGains.srv"
  "srv/SetTargetHeight.srv"
  DEPENDENCIES
    builtin_interfaces
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
