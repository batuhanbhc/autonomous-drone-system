cmake_minimum_required(VERSION 3.8)
project(drone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TeleopCommand.msg"
  "msg/TeleopAction.msg"
  "msg/DroneState.msg"
  "msg/DroneInfo.msg"
  "msg/GcsHeartbeat.msg"
  "msg/Toggle.msg"
  "msg/FrameData.msg"
  DEPENDENCIES 
    builtin_interfaces
    sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
