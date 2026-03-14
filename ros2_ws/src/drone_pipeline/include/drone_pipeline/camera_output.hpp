#pragma once

#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/msg/gpsraw.hpp"
#include "drone_msgs/msg/toggle.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"

#include "drone_pipeline/mjpeg_writer.hpp"
#include "drone_pipeline/h264_encoder.hpp"

namespace drone_pipeline
{

// ─────────────────────────────────────────────────────────────────────────────
//  Config
// ─────────────────────────────────────────────────────────────────────────────

struct VideoConfig
{
  uint8_t     drone_id{};
  std::string odom_topic;
  std::string gps1_raw_topic;
  std::string images_topic;
  std::string logs_path;
  int         width{};
  int         height{};
  int         fps{};
};

// ─────────────────────────────────────────────────────────────────────────────
//  Sensor snapshots
// ─────────────────────────────────────────────────────────────────────────────

struct OdomSnapshot
{
  rclcpp::Time stamp;
  double pos_x{}, pos_y{}, pos_z{};
  double quat_x{}, quat_y{}, quat_z{}, quat_w{1.0};
  double vel_x{}, vel_y{}, vel_z{};
};

struct GpsSnapshot
{
  rclcpp::Time stamp;
  int32_t lat{};   // deg * 1e7
  int32_t lon{};   // deg * 1e7
};

// ─────────────────────────────────────────────────────────────────────────────
//  Work items pushed to the async threads
// ─────────────────────────────────────────────────────────────────────────────

struct RecordTask
{
  std::vector<uint8_t> jpeg_data;
  uint32_t     stamp_sec{};
  uint32_t     stamp_nanosec{};
  OdomSnapshot odom;
  GpsSnapshot  gps;
};

struct StreamTask
{
  std::vector<uint8_t> jpeg_data;
  uint32_t stamp_sec{};
  uint32_t stamp_nanosec{};
};

// ─────────────────────────────────────────────────────────────────────────────
//  CameraOutput node
// ─────────────────────────────────────────────────────────────────────────────

class CameraOutput : public rclcpp::Node
{
public:
  explicit CameraOutput(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~CameraOutput() override;

private:
  // ── Config & directories ──────────────────────────────────────────────────
  VideoConfig config_;
  std::string session_dir_;
  std::string videos_dir_;
  std::string data_dir_;

  VideoConfig loadConfig();
  std::string resolveSessionDir(const std::string & logs_path);

  static constexpr std::size_t kMaxQueueDepth = 10;

  // ── Recording pipeline ────────────────────────────────────────────────────
  std::atomic<bool> recording_{false};
  int               clip_index_{0};

  MjpegWriter   mjpeg_writer_;
  std::ofstream csv_file_;
  std::mutex    clip_mtx_;

  void openClip();
  void closeClip();

  std::queue<RecordTask>  record_queue_;
  std::mutex              record_queue_mtx_;
  std::condition_variable record_queue_cv_;
  std::atomic<bool>       record_thread_running_{false};
  std::thread             record_thread_;

  void recordLoop();
  void startRecordThread();
  void stopRecordThread();

  // ── Streaming pipeline ────────────────────────────────────────────────────
  std::atomic<bool> streaming_{false};

  H264Encoder h264_encoder_;
  std::mutex  encoder_mtx_;

  std::queue<StreamTask>  stream_queue_;
  std::mutex              stream_queue_mtx_;
  std::condition_variable stream_queue_cv_;
  std::atomic<bool>       stream_thread_running_{false};
  std::thread             stream_thread_;

  void streamLoop();
  void startStreamThread();
  void stopStreamThread();

  // ── Sensor caches (recording only) ───────────────────────────────────────
  std::optional<OdomSnapshot> latest_odom_;
  std::optional<GpsSnapshot>  latest_gps_;
  std::mutex                  odom_mtx_;
  std::mutex                  gps_mtx_;

  // ── ROS interface ─────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;

  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr record_cmd_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr    record_state_pub_;

  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr stream_cmd_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr    stream_state_pub_;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr stream_out_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr gps_sub_;

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void imageCallback    (sensor_msgs::msg::CompressedImage::UniquePtr msg);
  void recordCmdCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  void streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  void odomCallback     (const nav_msgs::msg::Odometry::SharedPtr msg);
  void gpsCallback      (const mavros_msgs::msg::GPSRAW::SharedPtr msg);

  void publishRecordState();
  void publishStreamState();
};

}  // namespace drone_pipeline