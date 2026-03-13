#include "drone_pipeline/camera_capture.hpp"

#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace drone_pipeline
{

// ─────────────────────────────────────────────────────────────
//  Config
// ─────────────────────────────────────────────────────────────

CameraConfig CameraCapture::loadConfig() {
  const std::string share_dir =
    ament_index_cpp::get_package_share_directory("mavros_config");
  const std::string config_path = share_dir + "/config/control_params.yaml";

  RCLCPP_INFO(get_logger(), "Loading config from: %s", config_path.c_str());

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to parse YAML: %s", e.what());
    throw;
  }

  CameraConfig cfg;

  if (!root["camera"])
    throw std::runtime_error("Missing 'camera' section");

  const auto cam = root["camera"];
  cfg.width        = cam["width"].as<int>();
  cfg.height       = cam["height"].as<int>();
  cfg.fps          = cam["fps"].as<int>();
  cfg.device_path  = cam["device_path"].as<std::string>();
  cfg.pixel_format = cam["pixel_format"].as<std::string>();

  if (cfg.pixel_format != "YUY2" && cfg.pixel_format != "MJPEG")
    throw std::runtime_error("pixel_format must be 'YUY2' or 'MJPEG'");

  if (!root["drone_id"])
    throw std::runtime_error("Missing 'drone_id'");
  cfg.drone_id = static_cast<uint8_t>(root["drone_id"].as<int>());

  if (!root["custom_topics"] || !root["custom_topics"]["raw_images"])
    throw std::runtime_error("Missing 'custom_topics/raw_images'");
  cfg.raw_images_topic = root["custom_topics"]["raw_images"].as<std::string>();

  RCLCPP_INFO(
    get_logger(),
    "Config → device=%s  format=%s  resolution=%dx%d  fps=%d  "
    "drone_id=%u  topic=%s",
    cfg.device_path.c_str(), cfg.pixel_format.c_str(),
    cfg.width, cfg.height, cfg.fps,
    cfg.drone_id, cfg.raw_images_topic.c_str());

  return cfg;
}

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────

CameraCapture::CameraCapture(const rclcpp::NodeOptions & options)
: Node("camera_capture", options) {
  config_ = loadConfig();

  const std::string topic =
    "/drone_" + std::to_string(config_.drone_id) +
    "/" + config_.raw_images_topic;

  RCLCPP_INFO(get_logger(), "Publishing on: %s", topic.c_str());
  image_pub_ = image_transport::create_publisher(this, topic);

  cap_.open(config_.device_path, cv::CAP_V4L2);
  if (!cap_.isOpened())
    throw std::runtime_error("Camera open failed: " + config_.device_path);

  if (config_.pixel_format == "YUY2") {
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','2'));
  } else {
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  config_.width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
  cap_.set(cv::CAP_PROP_FPS,          config_.fps);

  RCLCPP_INFO(get_logger(),
    "Negotiated: %.0fx%.0f @ %.1f fps  fourcc=%.4s",
    cap_.get(cv::CAP_PROP_FRAME_WIDTH),
    cap_.get(cv::CAP_PROP_FRAME_HEIGHT),
    cap_.get(cv::CAP_PROP_FPS),
    [&]() -> std::string {
      int fcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
      return std::string(reinterpret_cast<char*>(&fcc), 4);
    }().c_str());

  running_ = true;
  capture_thread_ = std::thread(&CameraCapture::captureThread, this);

  RCLCPP_INFO(get_logger(), "camera_capture node ready.");
}

CameraCapture::~CameraCapture()
{
  running_ = false;
  if (capture_thread_.joinable())
    capture_thread_.join();
  cap_.release();
}

// ─────────────────────────────────────────────────────────────
//  Capture thread
// ─────────────────────────────────────────────────────────────

void CameraCapture::captureThread()
{
  const std::string encoding = "bgr8";

  while (running_ && rclcpp::ok()) {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN(get_logger(), "Failed to grab frame — skipping.");
      continue;
    }

    std_msgs::msg::Header header;
    header.stamp    = now();
    header.frame_id = "drone_" + std::to_string(config_.drone_id) + "_camera";

    auto msg = cv_bridge::CvImage(header, encoding, frame).toImageMsg();
    image_pub_.publish(*msg);
  }
}

}  // namespace drone_pipeline

// ─────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<drone_pipeline::CameraCapture>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("camera_capture"),
      "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}