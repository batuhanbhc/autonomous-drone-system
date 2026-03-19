#ifndef DRONE_PIPELINE__VISION_PIPELINE_HPP_
#define DRONE_PIPELINE__VISION_PIPELINE_HPP_

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <turbojpeg.h>

#include "rclcpp/rclcpp.hpp"
#include "drone_msgs/msg/frame_data.hpp"
#include "drone_msgs/msg/toggle.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "drone_pipeline/h264_encoder.hpp"

namespace drone_pipeline
{

// ─────────────────────────────────────────────────────────────────────────────
//  Config
// ─────────────────────────────────────────────────────────────────────────────

struct VisionConfig
{
  uint8_t     drone_id{};
  std::string frames_topic;
  int         width{};
  int         height{};
  int         fps{};
};

// ─────────────────────────────────────────────────────────────────────────────
//  Frame buffer
// ─────────────────────────────────────────────────────────────────────────────

enum class SlotState : uint8_t { FREE, PENDING, PROCESSING, PROCESSED };

struct FrameSlot
{
  std::vector<uint8_t>    rgb;            // width * height * 3
  uint32_t                stamp_sec{};
  uint32_t                stamp_nanosec{};
  std::atomic<SlotState>  state{SlotState::FREE};
  std::mutex              cv_mtx;
  std::condition_variable cv;             // encoder waits here
};

// ─────────────────────────────────────────────────────────────────────────────
//  Node
// ─────────────────────────────────────────────────────────────────────────────

class VisionPipeline : public rclcpp::Node
{
public:
  explicit VisionPipeline(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VisionPipeline();

private:
  // ── Constants ─────────────────────────────────────────────────────────────
  static constexpr int kNumSlots   = 30;
  static constexpr int kNumWorkers = 4;

  // ── Config ────────────────────────────────────────────────────────────────
  VisionConfig loadConfig();
  VisionConfig config_;

  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::Subscription<drone_msgs::msg::FrameData>::SharedPtr      frame_sub_;
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr         stream_cmd_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr            stream_state_pub_;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr stream_out_pub_;

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void frameCallback(drone_msgs::msg::FrameData::ConstSharedPtr msg);
  void streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  void publishStreamState();

  // ── Frame buffer ──────────────────────────────────────────────────────────
  std::array<FrameSlot, kNumSlots> buffer_;

  // Finds a FREE slot and marks it PENDING atomically.
  // Returns -1 if no slot is available (buffer full — frame dropped).
  int acquireSlot();

  tjhandle tj_decompress_{nullptr};

  // ── Work queue (PENDING slot indices for workers) ─────────────────────────
  std::queue<int>         work_queue_;
  std::mutex              work_queue_mtx_;
  std::condition_variable work_queue_cv_;

  // ── Encoder queue (slot indices in insertion order for encoder) ────────────
  std::queue<int>         encoder_queue_;
  std::mutex              encoder_queue_mtx_;
  std::condition_variable encoder_queue_cv_;

  // ── Worker threads ────────────────────────────────────────────────────────
  std::array<std::thread, kNumWorkers> worker_threads_;
  std::atomic<bool>                    workers_running_{false};
  void workerLoop();

  // ── Encoder thread ────────────────────────────────────────────────────────
  std::thread       encoder_thread_;
  std::atomic<bool> encoder_running_{false};
  std::atomic<bool> streaming_{false};
  H264Encoder       h264_encoder_;
  std::mutex        encoder_mtx_;         // guards h264_encoder_
  void encoderLoop();
};

}  // namespace drone_pipeline
#endif  // DRONE_PIPELINE__VISION_PIPELINE_HPP_