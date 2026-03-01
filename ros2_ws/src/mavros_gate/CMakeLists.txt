cmake_minimum_required(VERSION 3.8)
project(mavros_gate)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(teleop_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(command_gate src/command_gate.cpp)

ament_target_dependencies(command_gate
  rclcpp
  mavros_msgs
  teleop_msgs
  ament_index_cpp
)

target_link_libraries(command_gate yaml-cpp)

install(TARGETS
  command_gate
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()