cmake_minimum_required(VERSION 3.8)
project(camera_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_node
  src/camera_node.cpp
)

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  yaml-cpp
)

target_link_libraries(camera_node
  ${OpenCV_LIBS}
  yaml-cpp
)

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
