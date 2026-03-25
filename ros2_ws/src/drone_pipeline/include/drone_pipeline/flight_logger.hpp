#ifndef DRONE_PIPELINE__FLIGHT_LOGGER_HPP_
#define DRONE_PIPELINE__FLIGHT_LOGGER_HPP_

#include <fstream>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/msg/gpsraw.hpp"

namespace drone_pipeline
{

struct LoggerConfig
{
  uint8_t     drone_id;
  std::string odom_topic;      // full topic path from yaml (mavros_topics/odom)
  std::string gps1_raw_topic;  // full topic path from yaml (mavros_topics/gps1_raw)
  std::string logs_path;       // root directory where session folders live
};

class FlightLogger : public rclcpp::Node
{
public:
  explicit FlightLogger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FlightLogger();

private:
  // ── init ──────────────────────────────────────────────────
  LoggerConfig loadConfig();
  std::string  createSessionDir(const std::string & logs_path);

  // write buffer
  std::vector<std::string> odom_buffer_;
  std::vector<std::string> gps_buffer_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  static constexpr size_t kBufferFlushSize = 100;

  // ── callbacks ─────────────────────────────────────────────
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gpsCallback (const mavros_msgs::msg::GPSRAW::SharedPtr msg);

  // ── state ─────────────────────────────────────────────────
  LoggerConfig config_;

  std::ofstream odom_file_;
  std::ofstream gps_file_;
  std::mutex    odom_mtx_;
  std::mutex    gps_mtx_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr gps_sub_;
};

}  // namespace drone_pipeline

#endif  // DRONE_PIPELINE__FLIGHT_LOGGER_HPP_