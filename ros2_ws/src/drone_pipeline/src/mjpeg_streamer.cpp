#include "drone_pipeline/mjpeg_streamer.hpp"

#include <cstdint>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace drone_pipeline
{

namespace
{

MjpegStreamerConfig loadMjpegStreamerConfig(
  rclcpp::Node & node,
  const std::string & param_override)
{
  const std::string share_dir =
    ament_index_cpp::get_package_share_directory("mavros_config");
  const std::string config_path = share_dir + "/config/control_params.yaml";

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_FATAL(node.get_logger(), "Failed to parse YAML: %s", e.what());
    throw;
  }

  MjpegStreamerConfig cfg;
  cfg.drone_id = static_cast<uint8_t>(root["drone_id"].as<int>());

  const auto cam = root["camera"];
  cfg.width = cam["width"].as<int>();
  cfg.height = cam["height"].as<int>();

  const std::string configured_codec =
    root["streaming"] && root["streaming"]["codec"] ?
    root["streaming"]["codec"].as<std::string>() : "mjpeg";
  const std::string requested_codec =
    param_override.empty() ? configured_codec : param_override;

  if (!parseStreamCodec(requested_codec, cfg.stream_codec)) {
    RCLCPP_WARN(node.get_logger(),
      "Unknown stream codec '%s'; falling back to 'mjpeg'",
      requested_codec.c_str());
    cfg.stream_codec = StreamCodec::kMjpeg;
  }

  cfg.frames_topic = "/drone_" + std::to_string(cfg.drone_id) +
    "/" + root["custom_topics"]["images"].as<std::string>();

  return cfg;
}

}  // namespace

MjpegStreamer::MjpegStreamer(const rclcpp::NodeOptions & options)
: Node("mjpeg_streamer", options)
{
  declare_parameter<std::string>("stream_codec", "");
  config_ = loadConfig();
  enabled_ = config_.stream_codec == StreamCodec::kMjpeg;

  if (!enabled_) {
    RCLCPP_INFO(get_logger(),
      "mjpeg_streamer disabled; selected stream codec is '%s'",
      streamCodecName(config_.stream_codec));
    return;
  }

  const std::string dp = "/drone_" + std::to_string(config_.drone_id);
  const auto stream_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  frame_sub_ = create_subscription<drone_msgs::msg::FrameData>(
    config_.frames_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
    [this](drone_msgs::msg::FrameData::ConstSharedPtr msg) { frameCallback(msg); });

  stream_cmd_sub_ = create_subscription<drone_msgs::msg::Toggle>(
    dp + "/camera/stream/cmd", reliable_qos,
    [this](const drone_msgs::msg::Toggle::SharedPtr msg) { streamCmdCallback(msg); });

  stream_state_pub_ = create_publisher<drone_msgs::msg::Toggle>(
    dp + "/camera/stream/active", reliable_qos);

  stream_out_pub_ = create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
    dp + "/camera/stream/out", stream_qos);

  publishStreamState();
  RCLCPP_INFO(get_logger(),
    "mjpeg_streamer enabled on %s -> %s/camera/stream/out",
    config_.frames_topic.c_str(), dp.c_str());
}

MjpegStreamerConfig MjpegStreamer::loadConfig()
{
  const std::string stream_codec_override =
    get_parameter("stream_codec").as_string();
  return loadMjpegStreamerConfig(*this, stream_codec_override);
}

void MjpegStreamer::frameCallback(drone_msgs::msg::FrameData::ConstSharedPtr msg)
{
  if (!streaming_.load(std::memory_order_relaxed)) {
    return;
  }

  auto out = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
  out->header = msg->image.header;
  out->width = config_.width;
  out->height = config_.height;
  out->encoding = "mjpeg";
  out->pts = static_cast<int64_t>(msg->image.header.stamp.sec) * 1000000000LL +
    static_cast<int64_t>(msg->image.header.stamp.nanosec);
  out->flags = 1;
  out->data = msg->image.data;

  stream_out_pub_->publish(std::move(out));
}

void MjpegStreamer::streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
{
  const bool new_state = !streaming_.load(std::memory_order_relaxed);
  streaming_.store(new_state, std::memory_order_relaxed);
  RCLCPP_INFO(get_logger(), "MJPEG stream -> %s", new_state ? "ON" : "OFF");
  publishStreamState();
}

void MjpegStreamer::publishStreamState()
{
  if (!stream_state_pub_) {
    return;
  }

  drone_msgs::msg::Toggle msg;
  msg.state = streaming_.load(std::memory_order_relaxed);
  stream_state_pub_->publish(msg);
}

}  // namespace drone_pipeline

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_pipeline::MjpegStreamer)
