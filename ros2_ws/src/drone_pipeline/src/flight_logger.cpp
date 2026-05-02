#include "drone_pipeline/flight_logger.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace fs = std::filesystem;

namespace drone_pipeline
{

// ─────────────────────────────────────────────────────────────────────────────
//  Config loading
// ─────────────────────────────────────────────────────────────────────────────

LoggerConfig FlightLogger::loadConfig() {
  const std::string share_dir =
    ament_index_cpp::get_package_share_directory("mavros_config");
  const std::string config_path = share_dir + "/config/control_params.yaml";

  RCLCPP_INFO(get_logger(), "Loading config from: %s", config_path.c_str());

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to parse YAML: %s", e.what());
    throw;
  }

  LoggerConfig cfg;

  if (!root["drone_id"])
    throw std::runtime_error("Missing 'drone_id' in config");
  cfg.drone_id = static_cast<uint8_t>(root["drone_id"].as<int>());

  if (!root["mavros_topics"])
    throw std::runtime_error("Missing 'mavros_topics' section");

  if (!root["custom_topics"] || !root["custom_topics"]["images"])
    throw std::runtime_error("Missing 'custom_topics/images'");
  cfg.frames_topic = "/drone_" + std::to_string(cfg.drone_id) +
                     "/" + root["custom_topics"]["images"].as<std::string>();

  if (!root["flight_params"]["logs_path"])
    throw std::runtime_error("Missing 'flight_params/logs_path'");
  cfg.logs_path = root["flight_params"]["logs_path"].as<std::string>();

  RCLCPP_INFO(
    get_logger(),
    "Config → drone_id=%u  frames=%s  logs=%s",
    cfg.drone_id,
    cfg.frames_topic.c_str(),
    cfg.logs_path.c_str());

  return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Session directory creation
//  Counts existing sub-directories and creates the next zero-padded one.
//  e.g. 0001/ 0002/ 0003/ already exist → creates 0004/
// ─────────────────────────────────────────────────────────────────────────────

std::string FlightLogger::createSessionDir(const std::string & logs_path) {
  const std::string configured_session_dir = get_parameter("session_dir").as_string();
  if (!configured_session_dir.empty()) {
    fs::create_directories(configured_session_dir);
    RCLCPP_INFO(get_logger(), "Session directory: %s", configured_session_dir.c_str());
    return configured_session_dir;
  }

  fs::create_directories(logs_path);

  std::size_t dir_count = 0;
  for (const auto & entry : fs::directory_iterator(logs_path)) {
    if (entry.is_directory()) {
      ++dir_count;
    }
  }

  std::ostringstream oss;
  oss << std::setw(4) << std::setfill('0') << (dir_count + 1);
  const std::string session_path = logs_path + "/" + oss.str();

  fs::create_directory(session_path);

  RCLCPP_INFO(get_logger(), "Session directory: %s", session_path.c_str());
  return session_path;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────

FlightLogger::FlightLogger(const rclcpp::NodeOptions & options)
: Node("flight_logger", options) {
  declare_parameter<std::string>("session_dir", "");
  config_ = loadConfig();

  const std::string session_dir = createSessionDir(config_.logs_path);

  // ── Open CSV files ────────────────────────────────────────
  // '\n' is used in callbacks (not std::endl) — no per-line flush.
  // The OS page cache batches writes; explicit flush + close on clean shutdown.

  const std::string frame_path = session_dir + "/flight_data.csv";

  frame_file_.open(frame_path, std::ios::out | std::ios::app);
  if (!frame_file_.is_open())
    throw std::runtime_error("Cannot open frame CSV: " + frame_path);

  // Write header only for brand-new files
  frame_file_.seekp(0, std::ios::end);
  if (frame_file_.tellp() == 0) {
    frame_file_
      << "timestamp_sec,timestamp_nanosec,"
      << "pos_x,pos_y,pos_z,"
      << "quat_x,quat_y,quat_z,quat_w,"
      << "vel_x,vel_y,vel_z,"
      << "ang_vel_x,ang_vel_y,ang_vel_z,"
      << "odom_valid,"
      << "lat_deg_e7,lon_deg_e7,"
      << "gps_valid,"
      << "agl_m,vz_mps,"
      << "mcu_valid\n";
  }

  // ── Subscriptions ─────────────────────────────────────────
  const auto qos = rclcpp::SensorDataQoS();

  frame_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  flush_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions frame_sub_opts;
  frame_sub_opts.callback_group = frame_cb_group_;

  frame_sub_ = create_subscription<drone_msgs::msg::FrameData>(
    config_.frames_topic, qos,
    [this](const drone_msgs::msg::FrameData::SharedPtr msg) {
      frameCallback(msg);
    },
    frame_sub_opts);

  RCLCPP_INFO(get_logger(), "flight_logger ready. frames→%s",
    config_.frames_topic.c_str());

  // ── Flush timer ───────────────────────────────────────────
  // Flushes buffers to disk every 3 seconds, or when buffer size exceeds threshold.
  // This is a safeguard to prevent data loss on crash, since the destructor won't run.
  flush_timer_ = create_wall_timer(
    std::chrono::seconds(3),
    [this]() {
      {
        std::lock_guard<std::mutex> lk(frame_mtx_);
        for (const auto & line : frame_buffer_)
          frame_file_ << line;
        frame_buffer_.clear();
        frame_file_.flush();
      }
    },
    flush_cb_group_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Destructor — flush & close on clean shutdown
// ─────────────────────────────────────────────────────────────────────────────

FlightLogger::~FlightLogger()
{
  // Flush any remaining buffered lines
  for (const auto & line : frame_buffer_) frame_file_ << line;

  if (frame_file_.is_open()) { frame_file_.flush(); frame_file_.close(); }
}

void FlightLogger::frameCallback(const drone_msgs::msg::FrameData::SharedPtr msg)
{
  std::ostringstream oss;
  oss << msg->image.header.stamp.sec     << ','
      << msg->image.header.stamp.nanosec << ','
      << msg->pos_x      << ',' << msg->pos_y      << ',' << msg->pos_z      << ','
      << msg->quat_x     << ',' << msg->quat_y     << ',' << msg->quat_z     << ','
      << msg->quat_w     << ','
      << msg->vel_x      << ',' << msg->vel_y      << ',' << msg->vel_z      << ','
      << msg->ang_vel_x  << ',' << msg->ang_vel_y  << ',' << msg->ang_vel_z  << ','
      << msg->odom_valid << ','
      << msg->lat        << ',' << msg->lon        << ','
      << msg->gps_valid  << ','
      << msg->agl_m      << ',' << msg->vz_mps     << ','
      << msg->mcu_valid  << '\n';

  std::lock_guard<std::mutex> lk(frame_mtx_);
  frame_buffer_.push_back(oss.str());
}
}  // namespace drone_pipeline

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<drone_pipeline::FlightLogger>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("flight_logger"),
      "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
