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
#include "drone_pipeline/ray_ground_intersection.hpp"

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
  std::string hef_path;
  float       score_threshold{};

  // Camera intrinsics
  double fx{}, fy{}, cx{}, cy{};

  // Distortion (OpenCV 5-parameter)
  double k1{}, k2{}, p1{}, p2{}, k3{};
};

// ─────────────────────────────────────────────────────────────────────────────
//  Frame buffer
// ─────────────────────────────────────────────────────────────────────────────

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
  std::vector<uint8_t>    letterbox_buf;  // 640 * 640 * 3

  // ── NMS output from Hailo ────────────────────────────────────────────────
  std::vector<float>      nms_output_buf;

  // ── Timestamp ────────────────────────────────────────────────────────────
  uint32_t                stamp_sec{};
  uint32_t                stamp_nanosec{};

  // ── Odometry snapshot (copied from FrameData at capture time) ────────────
  double                  pos_x{};        ///< ENU East  (m)
  double                  pos_y{};        ///< ENU North (m)
  double                  pos_z{};        ///< ENU Up    (m)
  double                  quat_x{};
  double                  quat_y{};
  double                  quat_z{};
  double                  quat_w{1.0};
  bool                    odom_valid{false};

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

  // COCO class index for "person"
  static constexpr int kPersonClassIdx = 0;

  // Yaw calibration: collect this many samples during startup
  static constexpr int    kYawCalibSamples  = 10;
  static constexpr double kYawCalibTimeout  = 30.0;  ///< seconds

  // ── Config ────────────────────────────────────────────────────────────────
  VisionConfig loadConfig();
  VisionConfig config_;

  // ── De-letterbox constants (computed once in constructor) ─────────────────
  float lb_scale_{};
  int   lb_pad_top_{};
  int   lb_pad_left_{};

  // ── Ground projector ──────────────────────────────────────────────────────
  // Heap-allocated so we can construct it after loadConfig().
  std::unique_ptr<GroundProjector> projector_;

  // ── Yaw calibration ───────────────────────────────────────────────────────
  // The offset between the EKF's earth-north yaw and the local ENU frame yaw
  // established at arming. Computed once at startup from the first N frames.
  //
  // Corrected yaw = raw_yaw - yaw_offset_
  // When drone is stationary facing local north: raw_yaw == yaw_offset_
  double             yaw_offset_{0.0};
  std::atomic<bool>  yaw_calibrated_{false};

  // Shared state written by frameCallback, read by calibration thread
  struct LatestQuat { double x=0, y=0, z=0, w=1; bool valid=false; };
  LatestQuat         latest_quat_;
  std::mutex         latest_quat_mtx_;

  std::thread        yaw_calib_thread_;
  void               runYawCalibration();   ///< blocks for up to kYawCalibTimeout s

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