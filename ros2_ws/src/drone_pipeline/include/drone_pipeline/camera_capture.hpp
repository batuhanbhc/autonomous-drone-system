#ifndef DRONE_PIPELINE__CAMERA_CAPTURE_HPP_
#define DRONE_PIPELINE__CAMERA_CAPTURE_HPP_

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/msg/gpsraw.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "drone_msgs/msg/frame_data.hpp"

namespace drone_pipeline
{

struct CameraConfig
{
  int         width;
  int         height;
  int         fps;
  std::string device_path;
  uint8_t     drone_id;
  std::string frames_topic;
  std::string odom_topic;
  std::string gps1_raw_topic;
  std::string mcu_vertical_topic;
};

// Lightweight snapshots stored under their respective mutexes.
struct OdomSnapshot
{
  double pos_x{}, pos_y{}, pos_z{};
  double quat_x{}, quat_y{}, quat_z{}, quat_w{1.0};
  double vel_x{}, vel_y{}, vel_z{};
  double ang_vel_x{}, ang_vel_y{}, ang_vel_z{};
};

struct GpsSnapshot
{
  int32_t lat{};
  int32_t lon{};
};

struct McuSnapshot
{
  float agl_m{};    // Vector3Stamped.z
  float vz_mps{};   // Vector3Stamped.y
};

class CameraCapture : public rclcpp::Node
{
public:
  explicit CameraCapture(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraCapture();

private:
  // ── Config / device ───────────────────────────────────────
  CameraConfig loadConfig();
  void         openDevice();
  void         startStreaming();
  void         stopStreaming();
  void         captureThread();

  // ── Sensor callbacks ──────────────────────────────────────
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gpsCallback (const mavros_msgs::msg::GPSRAW::SharedPtr msg);
  void mcuCallback (const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  // ── Members ───────────────────────────────────────────────
  CameraConfig config_;

  rclcpp::Publisher<drone_msgs::msg::FrameData>::SharedPtr frame_pub_;

  rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
  rclcpp::CallbackGroup::SharedPtr gps_cb_group_;
  rclcpp::CallbackGroup::SharedPtr mcu_cb_group_;
  rclcpp::CallbackGroup::SharedPtr odom_timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr gps_timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr mcu_timer_cb_group_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr         gps_sub_;
  rclcpp::Subscription<
    geometry_msgs::msg::Vector3Stamped>::SharedPtr                  mcu_sub_;

  // Staleness timers — each fires independently at ~1 Hz
  rclcpp::TimerBase::SharedPtr odom_staleness_timer_;
  rclcpp::TimerBase::SharedPtr gps_staleness_timer_;
  rclcpp::TimerBase::SharedPtr mcu_staleness_timer_;

  std::mutex               odom_mtx_;
  std::optional<OdomSnapshot> latest_odom_;   // nullopt → stale / never received
  std::atomic<bool>        odom_valid_{false};

  std::mutex               gps_mtx_;
  std::optional<GpsSnapshot>  latest_gps_;
  std::atomic<bool>        gps_valid_{false};

  std::mutex               mcu_mtx_;
  std::optional<McuSnapshot>  latest_mcu_;
  std::atomic<bool>        mcu_valid_{false};

  int                fd_{-1};
  std::thread        capture_thread_;
  std::atomic<bool>  running_{false};
};

}  // namespace drone_pipeline
#endif  // DRONE_PIPELINE__CAMERA_CAPTURE_HPP_
