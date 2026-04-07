#ifndef DRONE_PIPELINE__VISION_PIPELINE_HPP_
#define DRONE_PIPELINE__VISION_PIPELINE_HPP_

#include <array>
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <turbojpeg.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

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

  double gimbal_pitch_angle;

  std::string hef_path;
  float       score_threshold{};
  int         model_input_size{};
  int         person_class_idx{};

  // Camera intrinsics
  double fx{}, fy{}, cx{}, cy{};

  // Distortion (OpenCV 5-parameter)
  double k1{}, k2{}, p1{}, p2{}, k3{};

  // Recording paths (from flight_params in YAML)
  std::string logs_path;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Detection — one bounding box from inference, with ground projection
// ─────────────────────────────────────────────────────────────────────────────

struct Detection
{
  float   x_min{};
  float   y_min{};
  float   x_max{};
  float   y_max{};
  float   score{};
  int     id{0};        // placeholder; will be replaced by tracker ID later
  double  world_x{};    // ENU East  (m); NaN if projection invalid / no odom
  double  world_y{};    // ENU North (m); NaN if projection invalid / no odom
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

struct FrameSlot
{
  // ── Original frame ───────────────────────────────────────────────────────
  std::vector<uint8_t>    rgb;            // cfg.width * cfg.height * 3

  // ── Letterboxed input for Hailo ──────────────────────────────────────────
  std::vector<uint8_t>    letterbox_buf;

  // ── NMS output from Hailo ────────────────────────────────────────────────
  std::vector<float>      nms_output_buf;

  // ── Detections (populated inside Hailo callback, read by encoder thread) ─
  std::vector<Detection>  detections;

  // ── Timestamp ────────────────────────────────────────────────────────────
  uint32_t                stamp_sec{};
  uint32_t                stamp_nanosec{};

  // ── Odometry snapshot (copied from FrameData at capture time) ────────────
  double                  pos_x{};
  double                  pos_y{};
  double                  pos_z{};
  double                  quat_x{};
  double                  quat_y{};
  double                  quat_z{};
  double                  quat_w{1.0};
  bool                    odom_valid{false};

  std::atomic<SlotState>  state{SlotState::FREE};
  std::mutex              cv_mtx;
  std::condition_variable cv;             // encoder thread waits here
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

  static constexpr int    kYawCalibSamples = 10;
  static constexpr double kYawCalibTimeout = 30.0;  ///< seconds

  // ── Config ────────────────────────────────────────────────────────────────
  VisionConfig loadConfig();
  VisionConfig config_;
  double gimbal_pitch_rad_{0.0};

  // ── De-letterbox constants (computed once in constructor) ─────────────────
  float lb_scale_{};
  int   lb_pad_top_{};
  int   lb_pad_left_{};

  // ── Ground projector ──────────────────────────────────────────────────────
  std::unique_ptr<GroundProjector> projector_;

  // ── Yaw calibration ───────────────────────────────────────────────────────
  double             yaw_offset_{0.0};
  std::atomic<bool>  yaw_calibrated_{false};

  struct LatestQuat { double x=0, y=0, z=0, w=1; bool valid=false; };
  LatestQuat         latest_quat_;
  std::mutex         latest_quat_mtx_;

  std::thread        yaw_calib_thread_;
  void               runYawCalibration();

  // ── ROS interfaces — streaming ────────────────────────────────────────────
  rclcpp::Subscription<drone_msgs::msg::FrameData>::SharedPtr      frame_sub_;
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr         stream_cmd_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr            stream_state_pub_;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr stream_out_pub_;

  // ── ROS interfaces — recording (ownership migrated from RecordVideo) ──────
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr  record_cmd_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr     record_state_pub_;

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void frameCallback(drone_msgs::msg::FrameData::ConstSharedPtr msg);
  void streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  void publishStreamState();
  void recordCmdCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  void publishRecordState();

  // ── Frame buffer ──────────────────────────────────────────────────────────
  std::array<FrameSlot, kNumSlots> buffer_;
  int      acquireSlot();
  tjhandle tj_decompress_{nullptr};

  // ── Work queue (frame callback → worker threads) ──────────────────────────
  std::queue<int>         work_queue_;
  std::mutex              work_queue_mtx_;
  std::condition_variable work_queue_cv_;

  // ── Encoder queue (frame callback → encoder thread) ───────────────────────
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
  std::mutex        encoder_mtx_;    ///< guards h264_encoder_ open/close vs encode
  void encoderLoop();

  // ── H264/MP4 recording ───────────────────────────────────────────────────
  // All MP4 muxer state is touched exclusively from the encoder thread.
  std::atomic<bool>   recording_{false};
  int                 clip_index_{0};
  std::string         session_dir_;
  std::string         videos_dir_;
  std::string         data_dir_;

  AVFormatContext * rec_fmt_ctx_{nullptr};
  AVStream        * rec_video_stream_{nullptr};
  int64_t           rec_pts_{0};

  std::string resolveSessionDir(const std::string & logs_path);
  void        openRecordClip();
  void        closeRecordClip();
  // Returns the MP4 frame index (PTS) actually written, or -1 if nothing was written.
  // Used by the encoder loop to correlate detection CSV rows with video frames.
  int64_t     writeRecordPacket(const H264Encoder::EncodeResult & result,
                                uint32_t stamp_sec, uint32_t stamp_nanosec);

  // ── Detection CSV ─────────────────────────────────────────────────────────
  // Written exclusively from the encoder thread; buffered and flushed by timer.
  // Format (one row per detection):
  //   frame_sec, frame_nanosec, id, x_min, y_min, x_max, y_max, score, world_x, world_y
  std::ofstream              det_csv_file_;
  std::vector<std::string>   det_csv_buffer_;
  std::mutex                 det_csv_mtx_;   ///< guards buffer + file during flush
  rclcpp::TimerBase::SharedPtr det_flush_timer_;

  static constexpr size_t kDetFlushSize = 200;  ///< lines before an eager flush

  // ── Hailo ─────────────────────────────────────────────────────────────────
  std::unique_ptr<hailort::VDevice> hailo_vdevice_;
  hailort::ConfiguredInferModel hailo_infer_model_;
  std::array<hailort::ConfiguredInferModel::Bindings, kNumSlots> hailo_bindings_;
  size_t hailo_output_frame_size_{0};

  void initHailo();
};

}  // namespace drone_pipeline
#endif  // DRONE_PIPELINE__VISION_PIPELINE_HPP_