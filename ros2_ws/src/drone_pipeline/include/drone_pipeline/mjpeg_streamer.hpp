#ifndef DRONE_PIPELINE__MJPEG_STREAMER_HPP_
#define DRONE_PIPELINE__MJPEG_STREAMER_HPP_

#include <atomic>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "drone_msgs/msg/frame_data.hpp"
#include "drone_msgs/msg/toggle.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"

#include "drone_pipeline/stream_codec.hpp"

namespace drone_pipeline
{

struct MjpegStreamerConfig
{
  uint8_t     drone_id{};
  int         width{};
  int         height{};
  std::string frames_topic;
  StreamCodec stream_codec{StreamCodec::kMjpeg};
};

class MjpegStreamer : public rclcpp::Node
{
public:
  explicit MjpegStreamer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  MjpegStreamerConfig loadConfig();

  void frameCallback(drone_msgs::msg::FrameData::ConstSharedPtr msg);
  void streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  void publishStreamState();

  MjpegStreamerConfig config_;
  bool                enabled_{false};
  std::atomic<bool>   streaming_{false};

  rclcpp::CallbackGroup::SharedPtr frame_cb_group_;
  rclcpp::CallbackGroup::SharedPtr stream_cmd_cb_group_;

  rclcpp::Subscription<drone_msgs::msg::FrameData>::SharedPtr frame_sub_;
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr    stream_cmd_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr       stream_state_pub_;
  rclcpp::Publisher<
    ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr stream_out_pub_;
};

}  // namespace drone_pipeline

#endif  // DRONE_PIPELINE__MJPEG_STREAMER_HPP_
