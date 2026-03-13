#ifndef DRONE_CAMERA__CAMERA_CAPTURE_HPP_
#define DRONE_CAMERA__CAMERA_CAPTURE_HPP_

#include <atomic>
#include <string>
#include <thread>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/videoio.hpp>

namespace drone_camera
{

enum class PixelFormat { YUY2, MJPEG };

struct CameraConfig
{
  int width;
  int height;
  int fps;
  std::string device_path;
  std::string pixel_format;   // "YUY2" or "MJPEG" from yaml
  uint8_t drone_id;
  std::string raw_images_topic;
};

class CameraCapture : public rclcpp::Node
{
public:
  explicit CameraCapture(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraCapture();

private:
  CameraConfig loadConfig();
  void forceV4L2Fps(const std::string & device, int fps);
  void captureThread();

  CameraConfig config_;
  image_transport::Publisher image_pub_;
  cv::VideoCapture cap_;

  std::thread capture_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace drone_camera

#endif