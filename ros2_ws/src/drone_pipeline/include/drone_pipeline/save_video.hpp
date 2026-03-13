#pragma once

#include <atomic>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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
  std::string raw_images_topic;
  std::string logs_path;
  int         width{};
  int         height{};
  int         fps{};
};

// Snapshot of the latest odometry data
struct OdomSnapshot
{
  rclcpp::Time stamp;
  double pos_x{}, pos_y{}, pos_z{};
  double quat_x{}, quat_y{}, quat_z{}, quat_w{1.0};
  double vel_x{}, vel_y{}, vel_z{};
};

// Snapshot of the latest GPS data
struct GpsSnapshot
{
  rclcpp::Time stamp;
  int32_t  lat{};   // deg * 1e7
  int32_t  lon{};   // deg * 1e7
};

class SaveVideo : public rclcpp::Node
{
public:
  explicit SaveVideo(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~SaveVideo() override;

private:
  // ── Config & directories ─────────────────────────────────────────────────
  VideoConfig  config_;
  std::string  session_dir_;      // e.g. <logs_path>/0001
  std::string  videos_dir_;       // <session_dir>/videos
  std::string  data_dir_;         // <session_dir>/data

  VideoConfig  loadConfig();
  std::string  resolveSessionDir(const std::string & logs_path);

  // ── State ────────────────────────────────────────────────────────────────
  std::atomic<bool> recording_{false};
  int               clip_index_{0};   // increments each time recording starts

  // Active clip resources (valid only while recording_==true)
  cv::VideoWriter  writer_;
  std::ofstream    csv_file_;
  std::mutex       clip_mtx_;

  void openClip();
  void closeClip();

  // ── Sensor caches ────────────────────────────────────────────────────────
  std::optional<OdomSnapshot> latest_odom_;
  std::optional<GpsSnapshot>  latest_gps_;
  std::mutex                  odom_mtx_;
  std::mutex                  gps_mtx_;

  // ── Subscriptions & publisher ────────────────────────────────────────────
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr         toggle_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr            state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr        gps_sub_;

  // ── Callbacks ────────────────────────────────────────────────────────────
  void toggleCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  void imageCallback (const sensor_msgs::msg::Image::SharedPtr msg);
  void odomCallback  (const nav_msgs::msg::Odometry::SharedPtr msg);
  void gpsCallback   (const mavros_msgs::msg::GPSRAW::SharedPtr msg);

  void publishState();
};

}  // namespace drone_pipeline