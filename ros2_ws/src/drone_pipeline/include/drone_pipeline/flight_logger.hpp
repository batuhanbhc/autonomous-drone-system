#ifndef DRONE_PIPELINE__FLIGHT_LOGGER_HPP_
#define DRONE_PIPELINE__FLIGHT_LOGGER_HPP_

#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "drone_msgs/msg/frame_data.hpp"

namespace drone_pipeline
{

struct LoggerConfig
{
  uint8_t     drone_id;
  std::string frames_topic;    // full topic path from yaml (custom_topics/images)
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
  std::vector<std::string> frame_buffer_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  static constexpr size_t kBufferFlushSize = 100;

  // ── callbacks ─────────────────────────────────────────────
  void frameCallback(const drone_msgs::msg::FrameData::SharedPtr msg);

  // ── state ─────────────────────────────────────────────────
  LoggerConfig config_;

  std::ofstream frame_file_;
  std::mutex    frame_mtx_;

  rclcpp::CallbackGroup::SharedPtr frame_cb_group_;
  rclcpp::CallbackGroup::SharedPtr flush_cb_group_;

  rclcpp::Subscription<drone_msgs::msg::FrameData>::SharedPtr frame_sub_;
};

}  // namespace drone_pipeline

#endif  // DRONE_PIPELINE__FLIGHT_LOGGER_HPP_
