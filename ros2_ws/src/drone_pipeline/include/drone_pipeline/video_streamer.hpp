#ifndef DRONE_PIPELINE__VIDEO_STREAMER_HPP_
#define DRONE_PIPELINE__VIDEO_STREAMER_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "drone_msgs/msg/frame_data.hpp"
#include "drone_msgs/msg/toggle.hpp"

#include "drone_pipeline/h264_encoder.hpp"
#include "drone_pipeline/stream_codec.hpp"

namespace drone_pipeline
{

struct VideoStreamerConfig
{
  uint8_t     drone_id{};
  int         width{};
  int         height{};
  int         fps{};
  std::string frames_topic;
  StreamCodec stream_codec{StreamCodec::kMjpeg};
  std::string video_transport{"udp"};
  int         video_port{50020};
  std::string gcs_host;
};

class VideoStreamer : public rclcpp::Node
{
public:
  explicit VideoStreamer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VideoStreamer() override;

private:
  class NetworkVideoSink;

  VideoStreamerConfig loadConfig();
  void frameCallback(drone_msgs::msg::FrameData::ConstSharedPtr msg);
  void streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  void publishStreamState();
  void streamerLoop();
  bool ensureStreamOpen();
  void closeStream();
  bool writeMjpegFrame(const drone_msgs::msg::FrameData::ConstSharedPtr & msg);
  bool writeH264Frame(const drone_msgs::msg::FrameData::ConstSharedPtr & msg);

  VideoStreamerConfig config_;

  std::atomic<bool> streaming_{false};
  std::atomic<bool> running_{true};
  int64_t mjpeg_pts_{0};
  rclcpp::Time last_open_attempt_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<NetworkVideoSink> sink_;
  H264Encoder h264_encoder_;
  std::mutex stream_mtx_;

  std::mutex queue_mtx_;
  std::condition_variable queue_cv_;
  std::deque<drone_msgs::msg::FrameData::ConstSharedPtr> queue_;
  std::thread streamer_thread_;

  rclcpp::CallbackGroup::SharedPtr frame_cb_group_;
  rclcpp::CallbackGroup::SharedPtr stream_cmd_cb_group_;

  rclcpp::Subscription<drone_msgs::msg::FrameData>::SharedPtr frame_sub_;
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr stream_cmd_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr stream_state_pub_;
};

}  // namespace drone_pipeline

#endif  // DRONE_PIPELINE__VIDEO_STREAMER_HPP_
