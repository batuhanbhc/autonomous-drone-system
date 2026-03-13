cmake_minimum_required(VERSION 3.8)
project(drone_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# camera_capture node
add_executable(camera_capture_node
  src/camera_capture_node.cpp
)

target_include_directories(camera_capture_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_capture_node
  yaml-cpp
  ${OpenCV_LIBS}
)

ament_target_dependencies(camera_capture_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

install(TARGETS camera_capture_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()