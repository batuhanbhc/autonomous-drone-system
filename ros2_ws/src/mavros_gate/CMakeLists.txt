cmake_minimum_required(VERSION 3.8)
project(mavros_gate)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── control_gate ─────────────────────────────────────────────────────────────
add_executable(control_gate
  src/control_gate.cpp
  src/utils.cpp
  src/subscriptions.cpp
  src/commands.cpp
  src/publishers.cpp
)

target_include_directories(control_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(control_gate
  rclcpp
  mavros_msgs
  drone_msgs
  geometry_msgs
  std_msgs
  ament_index_cpp
)

target_link_libraries(control_gate yaml-cpp)

# ── mcu_bridge ───────────────────────────────────────────────────────────────
add_executable(mcu_bridge
  src/mcu_bridge.cpp
)

target_include_directories(mcu_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mcu_bridge
  rclcpp
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(mcu_bridge yaml-cpp)

# ── altitude_controller ───────────────────────────────────────────────────────
#
# Soft-PID controller node.
# Subscribes:  <ns>/altitude_controller/input  (drone_msgs/AltitudeControllerInput)
# Publishes:   <ns>/altitude_controller/output (std_msgs/Float32)
# Service:     <ns>/altitude_controller/set_pid_gains (drone_msgs/srv/SetPidGains)
#
add_executable(altitude_controller
  src/altitude_controller.cpp
)

target_include_directories(altitude_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(altitude_controller
  rclcpp
  drone_msgs
  std_msgs
  ament_index_cpp
)

target_link_libraries(altitude_controller yaml-cpp)

# ── install ───────────────────────────────────────────────────────────────────
install(TARGETS
  control_gate
  mcu_bridge
  altitude_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()