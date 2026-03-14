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
#include <opencv2/videoio.hpp>

namespace drone_pipeline
{

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
//  A single unit of work pushed from the subscriber callback to the encoder
//  thread.  Carrying sensor snapshots alongside the raw JPEG bytes avoids any
//  additional locking on the hot path.
// ─────────────────────────────────────────────────────────────────────────────
struct EncodeTask
{
  // JPEG-compressed bytes straight from the CompressedImage message.
  // Stored in a vector so ownership is unambiguous after the message is gone.
  std::vector<uint8_t> jpeg_data;

  uint32_t     stamp_sec{};
  uint32_t     stamp_nanosec{};
  OdomSnapshot odom;
  GpsSnapshot  gps;
};

class SaveVideo : public rclcpp::Node
{
public:
  explicit SaveVideo(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~SaveVideo() override;

private:
  // ── Config & directories ──────────────────────────────────────────────────
  VideoConfig config_;
  std::string session_dir_;
  std::string videos_dir_;
  std::string data_dir_;

  VideoConfig loadConfig();
  std::string resolveSessionDir(const std::string & logs_path);

  // ── Recording state ───────────────────────────────────────────────────────
  std::atomic<bool> recording_{false};
  int               clip_index_{0};

  // Active clip resources — only touched by the encoder thread after openClip()
  cv::VideoWriter writer_;
  std::ofstream   csv_file_;
  std::mutex      clip_mtx_;   // guards writer_ / csv_file_ open/close

  void openClip();
  void closeClip();

  // ── Async encoder thread ──────────────────────────────────────────────────
  // The subscriber callback pushes EncodeTask objects here; the encoder
  // thread drains the queue and does all cv::imdecode + writer_.write() work,
  // keeping the ROS callback thread free.
  std::queue<EncodeTask>  encode_queue_;
  std::mutex              queue_mtx_;
  std::condition_variable queue_cv_;
  std::atomic<bool>       encoder_running_{false};
  std::thread             encoder_thread_;

  void encoderLoop();           // runs on encoder_thread_
  void startEncoderThread();
  void stopEncoderThread();

  // ── Sensor caches ─────────────────────────────────────────────────────────
  std::optional<OdomSnapshot> latest_odom_;
  std::optional<GpsSnapshot>  latest_gps_;
  std::mutex                  odom_mtx_;
  std::mutex                  gps_mtx_;

  // ── Subscriptions & publisher ─────────────────────────────────────────────
  // CompressedImage subscription uses UniquePtr callback — when the publisher
  // lives in the same process and publishes with unique_ptr, ROS2 intra-process
  // machinery delivers the message with zero copies.  For inter-process topics
  // the runtime transparently falls back to a shared-ownership copy.
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr            toggle_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr               state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr           gps_sub_;

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void toggleCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  // UniquePtr overload — enables zero-copy intra-process delivery
  void imageCallback(sensor_msgs::msg::CompressedImage::UniquePtr msg);
  void odomCallback (const nav_msgs::msg::Odometry::SharedPtr msg);
  void gpsCallback  (const mavros_msgs::msg::GPSRAW::SharedPtr msg);

  void publishState();
};

}  // namespace drone_pipeline