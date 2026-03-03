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

add_executable(control_gate
  src/control_gate.cpp
  src/utils.cpp
  src/subscriptions.cpp
  src/commands.cpp
)

target_include_directories(control_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(control_gate
  rclcpp
  mavros_msgs
  teleop_msgs
  ament_index_cpp
)

target_link_libraries(control_gate yaml-cpp)

install(TARGETS control_gate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()