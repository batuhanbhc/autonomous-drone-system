cmake_minimum_required(VERSION 3.8)
project(drone_pipeline)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# ── Dependencies ──────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
# ── camera_capture node ───────────────────────────────────────
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
# ── flight_logger node ────────────────────────────────────────
add_executable(flight_logger_node
src/flight_logger.cpp
)
target_include_directories(flight_logger_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
target_link_libraries(flight_logger_node
yaml-cpp
)
ament_target_dependencies(flight_logger_node
rclcpp
nav_msgs
mavros_msgs
)
# ── save_video node ───────────────────────────────────────────
add_executable(save_video_node
src/save_video.cpp
)
target_include_directories(save_video_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(save_video_node
yaml-cpp
${OpenCV_LIBS}
)
ament_target_dependencies(save_video_node
rclcpp
sensor_msgs
cv_bridge
nav_msgs
mavros_msgs
drone_msgs
)
# ── Install ───────────────────────────────────────────────────
install(TARGETS
camera_capture_node
flight_logger_node
save_video_node
DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)
ament_package()