#ifndef DRONE_PIPELINE__CAMERA_CAPTURE_HPP_
#define DRONE_PIPELINE__CAMERA_CAPTURE_HPP_

#include <atomic>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace drone_pipeline
{

// pixel_format removed — MJPEG is hardcoded throughout
struct CameraConfig
{
  int         width;
  int         height;
  int         fps;
  std::string device_path;
  uint8_t     drone_id;
  std::string raw_images_topic;
};

class CameraCapture : public rclcpp::Node
{
public:
  explicit CameraCapture(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraCapture();

private:
  CameraConfig loadConfig();
  void         openDevice();   // open + configure V4L2 fd
  void         startStreaming();
  void         stopStreaming();
  void         captureThread();

  CameraConfig config_;

  // UniquePtr publisher — required for intra-process zero-copy.
  // When both publisher and subscription are created with
  // rclcpp::NodeOptions().use_intra_process_comms(true) in the same
  // process, rclcpp will hand the unique_ptr directly to the subscriber
  // without any copy or serialisation.
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;

  int                fd_{-1};          // V4L2 file descriptor
  std::thread        capture_thread_;
  std::atomic<bool>  running_{false};
};

}  // namespace drone_pipeline

#endif  // DRONE_PIPELINE__CAMERA_CAPTURE_HPP_