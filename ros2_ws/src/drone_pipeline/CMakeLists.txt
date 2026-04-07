cmake_minimum_required(VERSION 3.8)
project(drone_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavcodec libavformat libavutil libswscale)

find_path(TURBOJPEG_INCLUDE_DIR turbojpeg.h)
find_library(TURBOJPEG_LIBRARY turbojpeg)
if(NOT TURBOJPEG_INCLUDE_DIR OR NOT TURBOJPEG_LIBRARY)
  message(FATAL_ERROR "libturbojpeg not found — install libturbojpeg0-dev")
endif()

# ── HailoRT ───────────────────────────────────────────────────────────────────
find_library(HAILORT_LIBRARY hailort
  HINTS /usr/lib /usr/local/lib
)
find_path(HAILORT_INCLUDE_DIR hailo/hailort.hpp
  HINTS /usr/include /usr/local/include
)
if(NOT HAILORT_LIBRARY OR NOT HAILORT_INCLUDE_DIR)
  message(FATAL_ERROR "HailoRT not found — install hailort dev package")
endif()
message(STATUS "HailoRT: ${HAILORT_LIBRARY}  includes: ${HAILORT_INCLUDE_DIR}")

# ── Shared library (components) ───────────────────────────────────────────────
# NOTE: record_video.cpp has been retired — its functionality (H264/MP4 recording,
#       detection CSV, record toggle topics) is now part of vision_pipeline.cpp.
#       The retired sources are kept in src/record_video_RETIRED.cpp for reference.
add_library(drone_pipeline_components SHARED
  src/camera_capture_node.cpp
  src/mjpeg_writer.cpp
  src/h264_encoder.cpp
  src/vision_pipeline.cpp
)

target_include_directories(drone_pipeline_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${TURBOJPEG_INCLUDE_DIR}
  ${HAILORT_INCLUDE_DIR}
)

target_link_libraries(drone_pipeline_components
  yaml-cpp
  ${OpenCV_LIBS}
  PkgConfig::LIBAV
  ${TURBOJPEG_LIBRARY}
  ${HAILORT_LIBRARY}
)

ament_target_dependencies(drone_pipeline_components
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  cv_bridge
  nav_msgs
  mavros_msgs
  drone_msgs
  ffmpeg_image_transport_msgs
  tf2
)

rclcpp_components_register_node(drone_pipeline_components
  PLUGIN "drone_pipeline::CameraCapture"
  EXECUTABLE camera_capture_node
)
# NOTE: drone_pipeline::RecordVideo has been retired; no longer registered.
rclcpp_components_register_node(drone_pipeline_components
  PLUGIN "drone_pipeline::VisionPipeline"
  EXECUTABLE vision_pipeline_node
)

add_executable(flight_logger_node
  src/flight_logger.cpp
)
target_include_directories(flight_logger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(flight_logger_node yaml-cpp)
ament_target_dependencies(flight_logger_node
  rclcpp
  nav_msgs
  mavros_msgs
)

install(TARGETS
  drone_pipeline_components
  flight_logger_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()