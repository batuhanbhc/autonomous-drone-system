#ifndef DRONE_PIPELINE__VISION_PIPELINE_HPP_
#define DRONE_PIPELINE__VISION_PIPELINE_HPP_

#include <array>
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <future>
#include <map>
#include <memory>
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
#include "drone_pipeline/mjpeg_writer.hpp"
#include "drone_pipeline/ray_ground_intersection.hpp"
#include "drone_pipeline/stream_codec.hpp"
#include "drone_pipeline/tracker.hpp"

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

  double camera_mount_angle{};
  bool   use_gimbal{false};
  bool   reverse_mounted{false};

  std::string hef_path;
  float       score_threshold{};
  int         model_input_size{};
  int         person_class_idx{};

  // Camera intrinsics
  double fx{}, fy{}, cx{}, cy{};

  // Distortion (OpenCV 5-parameter)
  double k1{}, k2{}, p1{}, p2{}, k3{};

  // Root directory where session folders live
  std::string logs_path;

  // When true, use agl_m from the MCU vertical estimate (if mcu_valid) for
  // ground projection instead of the odometry pos_z.
  bool use_mcu_height_estimate{false};
  double projection_altitude_lpf_alpha{0.35};
  TrackerConfig tracker;
  StreamCodec  stream_codec{StreamCodec::kMjpeg};
};

// ─────────────────────────────────────────────────────────────────────────────
//  Detection — one bounding box from inference
// ─────────────────────────────────────────────────────────────────────────────

struct Detection
{
  float  x_min{};
  float  y_min{};
  float  x_max{};
  float  y_max{};
  float  score{};
};

// ─────────────────────────────────────────────────────────────────────────────
//  Frame slot
//
//  Lifecycle: FREE → acquired by frameCallback → dispatched to active
//  consumers (MJPEG writer, optional H264 encoder, Hailo workers).
//  Each consumer atomically decrements consumers_remaining_.  Whoever
//  decrements to zero sets the slot back to FREE.
// ─────────────────────────────────────────────────────────────────────────────

struct FrameSlot
{
  // ── Pixel data ───────────────────────────────────────────────────────────
  // Zero-copy reference to the intra-process message from CameraCapture.
  // All active consumers (MJPEG writer, encoder, worker) read image.data from
  // here.  Cleared in releaseSlot() when consumers_remaining hits 0.
  drone_msgs::msg::FrameData::ConstSharedPtr msg;

  // ── Letterboxed input / NMS output for Hailo ─────────────────────────────
  std::vector<uint8_t>   letterbox_buf;
  std::vector<float>     nms_output_buf;

  // ── Results (written by Hailo callback, read by track_results writer) ────
  std::vector<Detection> detections;

  // ── Metadata ─────────────────────────────────────────────────────────────
  uint32_t stamp_sec{};
  uint32_t stamp_nanosec{};

  // Odometry snapshot (copied from FrameData at capture time)
  double pos_x{}, pos_y{}, pos_z{};
  double quat_x{}, quat_y{}, quat_z{}, quat_w{1.0};
  double vel_x{}, vel_y{}, vel_z{};
  double ang_vel_x{}, ang_vel_y{}, ang_vel_z{};
  bool   odom_valid{false};

  // GPS snapshot
  int32_t lat{};
  int32_t lon{};
  bool    gps_valid{false};

  // MCU vertical estimate snapshot
  float agl_m{};
  float vz_mps{};
  bool  mcu_valid{false};

  // Monotonically increasing sequence number assigned at acquire time.
  // Used by the track_results writer to drain detections in order.
  uint64_t seq{0};

  // ── Slot lifecycle ───────────────────────────────────────────────────────
  // Set to 2 or 3 at acquire time depending on whether H264 streaming is
  // active for that frame. Each consumer decrements; whoever hits 0 sets
  // in_use to false.
  std::atomic<int>  consumers_remaining{0};
  std::atomic<bool> in_use{false};
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
  static constexpr int kNumWorkers = 2;

  static constexpr int    kYawCalibSamples = 10;
  static constexpr double kYawCalibTimeout = 30.0;   // seconds

  static constexpr size_t kCsvFlushSize = 2000;

  // ── Config ────────────────────────────────────────────────────────────────
  VisionConfig loadConfig();
  VisionConfig config_;
  double mount_angle_rad_{0.0};

  // ── De-letterbox constants (computed once in constructor) ─────────────────
  float lb_scale_{};
  int   lb_pad_top_{};
  int   lb_pad_left_{};

  // ── Ground projector ──────────────────────────────────────────────────────
  std::unique_ptr<GroundProjector> projector_;
  std::unique_ptr<MultiObjectTracker> tracker_;

  // ── Yaw calibration ───────────────────────────────────────────────────────
  double            yaw_offset_{0.0};
  std::atomic<bool> yaw_calibrated_{false};

  struct LatestQuat { double x=0, y=0, z=0, w=1; bool valid=false; };
  LatestQuat  latest_quat_;
  std::mutex  latest_quat_mtx_;

  std::thread yaw_calib_thread_;
  void        runYawCalibration();

  // ── Session / clip directories ────────────────────────────────────────────
  std::string resolveSessionDir(const std::string & logs_path);
  std::string session_dir_;
  std::string videos_dir_;
  std::string data_dir_;
  int         clip_index_{0};

  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr frame_cb_group_;
  rclcpp::CallbackGroup::SharedPtr stream_cmd_cb_group_;
  rclcpp::CallbackGroup::SharedPtr record_cmd_cb_group_;
  rclcpp::CallbackGroup::SharedPtr record_flush_cb_group_;
  rclcpp::CallbackGroup::SharedPtr track_flush_cb_group_;

  rclcpp::Subscription<drone_msgs::msg::FrameData>::SharedPtr  frame_sub_;

  // Streaming
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr     stream_cmd_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr        stream_state_pub_;
  rclcpp::Publisher<
    ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr stream_out_pub_;

  // Recording
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr     record_cmd_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr        record_state_pub_;
  bool                                                         h264_streaming_enabled_{false};

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void frameCallback(drone_msgs::msg::FrameData::ConstSharedPtr msg);
  void streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  void recordCmdCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  void publishStreamState();
  void publishRecordState();

  // ── Frame buffer ──────────────────────────────────────────────────────────
  std::array<FrameSlot, kNumSlots> buffer_;
  std::atomic<uint64_t>            next_seq_{0};
  // NOTE: tjhandle is NOT thread-safe — each consumer thread owns a
  // thread_local handle (defined in vision_pipeline.cpp).

  // Returns slot index, or -1 if all slots are in use.
  int  acquireSlot(int consumers);
  // Called by each consumer when it finishes.  Frees the slot when all done.
  void releaseSlot(int idx);

  // ── MJPEG writer thread ───────────────────────────────────────────────────
  // Consumes: slot.msg->image.data (zero-copy JPEG), slot odom/gps/mcu snapshot.
  // Writes:   .avi file via MjpegWriter, data.csv — all buffered.
  struct MjpegTask { int idx; };

  std::queue<MjpegTask>   mjpeg_queue_;
  std::mutex              mjpeg_queue_mtx_;
  std::condition_variable mjpeg_queue_cv_;
  std::thread             mjpeg_thread_;
  std::atomic<bool>       mjpeg_running_{false};
  std::atomic<bool>       recording_{false};

  MjpegWriter   mjpeg_writer_;
  std::ofstream data_csv_file_;

  enum class DataCsvCommandType { Open, AppendLine, Flush, Close, Stop };
  struct DataCsvCommand
  {
    DataCsvCommandType type{DataCsvCommandType::Flush};
    std::string path;
    std::string line;
    std::shared_ptr<std::promise<bool>> completion;
  };

  std::mutex                 data_csv_mtx_;
  std::condition_variable    data_csv_cv_;
  std::queue<DataCsvCommand> data_csv_cmd_queue_;
  std::thread                data_csv_thread_;

  void mjpegLoop();
  void dataCsvLoggerLoop();
  void enqueueDataCsvLine(std::string line);
  bool openDataCsv(const std::string & path);
  void flushDataCsv();
  void closeDataCsv();
  void openRecordClip();
  void closeRecordClip();

  rclcpp::TimerBase::SharedPtr record_flush_timer_;

  // ── H264 encoder thread ───────────────────────────────────────────────────
  // Consumes: slot.msg JPEG bytes → tjDecompressToYUVPlanes → libx264.
  // Publishes FFMPEGPacket if streaming_ is on. The pending queue is capped
  // so old frames are dropped instead of accumulating stream latency.
  struct EncoderTask { int idx; };

  std::queue<EncoderTask>  encoder_queue_;
  std::mutex               encoder_queue_mtx_;
  std::condition_variable  encoder_queue_cv_;
  std::thread              encoder_thread_;
  std::atomic<bool>        encoder_running_{false};
  std::atomic<bool>        streaming_{false};

  H264Encoder h264_encoder_;
  std::mutex  encoder_open_mtx_;   // guards open/close of h264_encoder_

  void encoderLoop();

  // ── Hailo worker threads ──────────────────────────────────────────────────
  // Consume: slot.msg JPEG bytes → tjDecompress2(RGB) → rotate → letterbox → async Hailo.
  // On completion they push a PendingResult into the track_results ordering
  // queue, then call releaseSlot().
  struct WorkerTask { int idx; };

  std::queue<WorkerTask>               worker_queue_;
  std::mutex                           worker_queue_mtx_;
  std::condition_variable              worker_queue_cv_;
  std::array<std::thread, kNumWorkers> worker_threads_;
  std::atomic<bool>                    workers_running_{false};

  void workerLoop();

  // ── track_results.csv — in-order detection writer ────────────────────────
  // Hailo callbacks complete out of order (multiple worker threads). Each
  // completed slot deposits a PendingResult here; the ordered drain performs
  // projection, covariance estimation, tracking, and confirmed-track CSV
  // writing in sequence-number order.
  struct PendingResult
  {
    uint64_t               seq;
    uint32_t               stamp_sec;
    uint32_t               stamp_nanosec;
    double                 pos_x{};
    double                 pos_y{};
    double                 pos_z{};
    double                 quat_x{};
    double                 quat_y{};
    double                 quat_z{};
    double                 quat_w{1.0};
    double                 ang_vel_x{};
    double                 ang_vel_y{};
    double                 ang_vel_z{};
    bool                   odom_valid{false};
    float                  agl_m{};
    bool                   mcu_valid{false};
    std::vector<Detection> detections;
  };

  // Priority queue: smallest seq on top
  struct PendingResultCmp {
    bool operator()(const PendingResult & a, const PendingResult & b) const {
      return a.seq > b.seq;
    }
  };

  std::priority_queue<PendingResult,
                      std::vector<PendingResult>,
                      PendingResultCmp> results_pq_;
  uint64_t          next_result_seq_{0};  // next seq we expect to write
  std::mutex        results_mtx_;         // guards pq + next_result_seq_ + tracker state
  bool              track_csv_active_{false};

  std::ofstream              track_csv_file_;
  enum class TrackCsvCommandType { Open, AppendLines, Flush, Close, Stop };
  struct TrackCsvCommand
  {
    TrackCsvCommandType type{TrackCsvCommandType::Flush};
    std::string path;
    std::vector<std::string> lines;
    std::shared_ptr<std::promise<bool>> completion;
  };

  std::mutex                  track_csv_mtx_;
  std::condition_variable     track_csv_cv_;
  std::queue<TrackCsvCommand> track_csv_cmd_queue_;
  std::thread                 track_csv_thread_;
  rclcpp::TimerBase::SharedPtr track_flush_timer_;
  bool   has_prev_track_timestamp_{false};
  double prev_track_timestamp_s_{0.0};
  bool   has_smoothed_projection_height_{false};
  double smoothed_projection_height_m_{0.0};

  void depositResult(PendingResult && r);   // called from Hailo callback
  void drainResultsLocked(std::vector<std::string> & ready_lines);
  void trackCsvLoggerLoop();
  void enqueueTrackCsvLines(std::vector<std::string> lines);
  bool openTrackCsvLogger(const std::string & path);
  void flushTrackCsv();
  void closeTrackCsvLogger();
  void openTrackCsv(const std::string & path);
  void closeTrackCsv();
  void resetTrackingStateLocked();
  double computeAngularVelocityDegPerSecLocked(const PendingResult & result) const;
  double smoothProjectionHeightLocked(double raw_height_m);
  std::vector<DetectionMeasurement> buildTrackMeasurementsLocked(
    const PendingResult & result,
    double noise_scale);
  void writeTrackRowsLocked(
    const PendingResult & result,
    const std::vector<TrackEstimate> & confirmed_tracks,
    double noise_scale,
    double omega_deg_s,
    std::vector<std::string> & ready_lines);

  // ── Hailo ─────────────────────────────────────────────────────────────────
  std::unique_ptr<hailort::VDevice>   hailo_vdevice_;
  hailort::ConfiguredInferModel       hailo_infer_model_;
  std::array<hailort::ConfiguredInferModel::Bindings, kNumSlots> hailo_bindings_;
  size_t hailo_output_frame_size_{0};

  void initHailo();
};

}  // namespace drone_pipeline
#endif  // DRONE_PIPELINE__VISION_PIPELINE_HPP_
