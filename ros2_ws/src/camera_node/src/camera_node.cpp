#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

class CameraNode : public rclcpp::Node {
public:
  CameraNode()
  : Node("camera_node")
  {
    load_config();
    init_camera();

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      topic_name_, 10); 

    auto period = std::chrono::milliseconds(1000 / fps_);
    timer_ = this->create_wall_timer(
        period, std::bind(&CameraNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Camera node started.");
  }

  ~CameraNode()
  {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  void load_config()
  {
    YAML::Node config = YAML::LoadFile("/config/system.yaml");

    device_ = config["camera"]["device"].as<std::string>();
    width_  = config["camera"]["width"].as<int>();
    height_ = config["camera"]["height"].as<int>();
    fps_    = config["camera"]["fps"].as<int>();
    topic_name_ = config["topics"]["nn_input_image"].as<std::string>();
  }

  void init_camera()
  {
    std::stringstream pipeline;

    pipeline << "v4l2src device=" << device_
             << " ! video/x-raw,format=YUY2,width=" << width_
             << ",height=" << height_
             << ",framerate=" << fps_ << "/1"
             << " ! appsink sync=false";

    cap_.open(pipeline.str(), cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open camera.");
      rclcpp::shutdown();
    }
  }

  void timer_callback()
  {
    cv::Mat frame_yuyv;
    if (!cap_.read(frame_yuyv)) {
      RCLCPP_WARN(this->get_logger(), "Frame capture failed.");
      return;
    }

    cv::Mat frame_bgr;
    cv::cvtColor(frame_yuyv, frame_bgr, cv::COLOR_YUV2BGR_YUY2);

    auto msg = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::BGR8,
        frame_bgr).toImageMsg();

    msg->header.stamp = this->now();
    image_pub_->publish(*msg);
  }

  std::string device_;
  int width_;
  int height_;
  int fps_;
  std::string topic_name_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

int main(int argc, char ** argv)
{ 
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
