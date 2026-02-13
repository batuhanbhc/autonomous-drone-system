#ifndef CAMERA_NODE_HPP_
#define CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode();

private:
  void load_config();
  void init_camera();
  void timer_callback();

  // Config params
  std::string device_;
  int width_;
  int height_;
  int fps_;
  std::string topic_name_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // OpenCV
  cv::VideoCapture cap_;
};

#endif  // CAMERA_NODE_HPP_
