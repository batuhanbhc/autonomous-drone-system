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

// HailoRT high-level C++ API
#include "hailo/hailort.hpp"

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

// INFERENCING is between PROCESSING and PROCESSED.
// Worker transitions PROCESSING → INFERENCING when it fires the async call.
// Hailo callback transitions INFERENCING → PROCESSED.
// Encoder waits for PROCESSED exactly as before.
enum class SlotState : uint8_t
{
  FREE,
  PENDING,
  PROCESSING,
  INFERENCING,
  PROCESSED
};

static constexpr int kHailoInputW  = 640;
static constexpr int kHailoInputH  = 640;
static constexpr int kHailoInputC  = 3;   // RGB24

struct FrameSlot
{
  // ── Original frame ───────────────────────────────────────────────────────
  std::vector<uint8_t>    rgb;            // cfg.width * cfg.height * 3

  // ── Letterboxed input for Hailo ──────────────────────────────────────────
  // Pre-allocated at construction. workerLoop writes here, then binds to Hailo.
  std::vector<uint8_t>    letterbox_buf;  // 640 * 640 * 3

  // ── NMS output from Hailo ────────────────────────────────────────────────
  // Size is determined from the HEF output vstream info at init time.
  std::vector<float>      nms_output_buf;

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
  static constexpr int kNumSlots      = 30;
  static constexpr int kNumWorkers    = 4;
  
  // Path to the compiled HEF model
  static constexpr const char * kHefPath =
    "/usr/share/hailo-models/yolov8s_h8l.hef";

  // COCO class index for "person"
  static constexpr int   kPersonClassIdx  = 0;
  static constexpr float kScoreThreshold  = 0.5f;

  // ── Config ────────────────────────────────────────────────────────────────
  VisionConfig loadConfig();
  VisionConfig config_;

  // ── De-letterbox constants (computed once in constructor) ─────────────────
  // These map 640x640 detection coordinates back to original frame pixels.
  float lb_scale_{};       // scale applied to the longer side
  int   lb_pad_top_{};     // black rows added at top
  int   lb_pad_left_{};    // black cols added at left (0 for 4:3 landscape)

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
  int acquireSlot();
  tjhandle tj_decompress_{nullptr};

  // ── Work queue ────────────────────────────────────────────────────────────
  std::queue<int>         work_queue_;
  std::mutex              work_queue_mtx_;
  std::condition_variable work_queue_cv_;

  // ── Encoder queue ─────────────────────────────────────────────────────────
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
  std::mutex        encoder_mtx_;
  void encoderLoop();

  // ── Hailo ─────────────────────────────────────────────────────────────────
  std::unique_ptr<hailort::VDevice> hailo_vdevice_;
  hailort::ConfiguredInferModel hailo_infer_model_;
  std::array<hailort::ConfiguredInferModel::Bindings, kNumSlots> hailo_bindings_;
  size_t hailo_output_frame_size_{0};

  void initHailo();
};

}  // namespace drone_pipeline
#endif  // DRONE_PIPELINE__VISION_PIPELINE_HPP_