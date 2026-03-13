#include "drone_pipeline/flight_logger.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

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

  const auto & mv = root["mavros_topics"];

  if (!mv["odom"])
    throw std::runtime_error("Missing 'mavros_topics/odom'");
  cfg.odom_topic = "/drone_" + std::to_string(cfg.drone_id) +
                   mv["odom"].as<std::string>();

  if (!mv["gps1_raw"])
    throw std::runtime_error("Missing 'mavros_topics/gps1_raw'");
  cfg.gps1_raw_topic = "/drone_" + std::to_string(cfg.drone_id) +
                       mv["gps1_raw"].as<std::string>();

  if (!root["flight_params"]["logs_path"])
    throw std::runtime_error("Missing 'flight_params/logs_path'");
  cfg.logs_path = root["flight_params"]["logs_path"].as<std::string>();

  RCLCPP_INFO(
    get_logger(),
    "Config → drone_id=%u  odom=%s  gps=%s  logs=%s",
    cfg.drone_id,
    cfg.odom_topic.c_str(),
    cfg.gps1_raw_topic.c_str(),
    cfg.logs_path.c_str());

  return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Session directory creation
//  Counts existing sub-directories and creates the next zero-padded one.
//  e.g. 0001/ 0002/ 0003/ already exist → creates 0004/
// ─────────────────────────────────────────────────────────────────────────────

std::string FlightLogger::createSessionDir(const std::string & logs_path) {
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
  config_ = loadConfig();

  const std::string session_dir = createSessionDir(config_.logs_path);

  // ── Open CSV files ────────────────────────────────────────
  // '\n' is used in callbacks (not std::endl) — no per-line flush.
  // The OS page cache batches writes; explicit flush + close on clean shutdown.

  const std::string odom_path = session_dir + "/odom.csv";
  const std::string gps_path  = session_dir + "/gps.csv";

  odom_file_.open(odom_path, std::ios::out | std::ios::app);
  if (!odom_file_.is_open())
    throw std::runtime_error("Cannot open odom CSV: " + odom_path);

  gps_file_.open(gps_path, std::ios::out | std::ios::app);
  if (!gps_file_.is_open())
    throw std::runtime_error("Cannot open gps CSV: " + gps_path);

  // Write header only for brand-new files
  odom_file_.seekp(0, std::ios::end);
  if (odom_file_.tellp() == 0) {
    odom_file_
      << "timestamp_sec,timestamp_nanosec,"
      << "pos_x,pos_y,pos_z,"
      << "quat_x,quat_y,quat_z,quat_w,"
      << "vel_lin_x,vel_lin_y,vel_lin_z\n";
  }

  gps_file_.seekp(0, std::ios::end);
  if (gps_file_.tellp() == 0) {
    gps_file_
      << "timestamp_sec,timestamp_nanosec,"
      << "lat_deg_e7,lon_deg_e7,alt_mm,"
      << "vel_m_s,cog_cdeg,"
      << "satellites_visible,fix_type\n";
  }

  // ── Subscriptions ─────────────────────────────────────────
  const auto qos = rclcpp::SensorDataQoS();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    config_.odom_topic, qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odomCallback(msg);
    });

  gps_sub_ = create_subscription<mavros_msgs::msg::GPSRAW>(
    config_.gps1_raw_topic, qos,
    [this](const mavros_msgs::msg::GPSRAW::SharedPtr msg) {
      gpsCallback(msg);
    });

  RCLCPP_INFO(get_logger(), "flight_logger ready.  odom→%s  gps→%s",
    config_.odom_topic.c_str(), config_.gps1_raw_topic.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Destructor — flush & close on clean shutdown
// ─────────────────────────────────────────────────────────────────────────────

FlightLogger::~FlightLogger()
{
  if (odom_file_.is_open()) {
    odom_file_.flush();
    odom_file_.close();
  }
  if (gps_file_.is_open()) {
    gps_file_.flush();
    gps_file_.close();
  }
  RCLCPP_INFO(get_logger(), "flight_logger: CSV files flushed and closed.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Odom callback
// ─────────────────────────────────────────────────────────────────────────────

void FlightLogger::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  const auto & v = msg->twist.twist.linear;

  std::lock_guard<std::mutex> lk(odom_mtx_);

  odom_file_
    << msg->header.stamp.sec     << ','
    << msg->header.stamp.nanosec << ','
    << p.x << ',' << p.y << ',' << p.z << ','
    << q.x << ',' << q.y << ',' << q.z << ',' << q.w << ','
    << v.x << ',' << v.y << ',' << v.z
    << '\n';
}

// ─────────────────────────────────────────────────────────────────────────────
//  GPS callback
// ─────────────────────────────────────────────────────────────────────────────

void FlightLogger::gpsCallback(const mavros_msgs::msg::GPSRAW::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(gps_mtx_);

  gps_file_
    << msg->header.stamp.sec     << ','
    << msg->header.stamp.nanosec << ','
    << msg->lat                  << ','   // deg * 1e7  (int32)
    << msg->lon                  << ','   // deg * 1e7  (int32)
    << msg->alt                  << ','   // mm above MSL (int32)
    << msg->vel                  << ','   // cm/s ground speed (uint16)
    << msg->cog                  << ','   // course over ground, cdeg (uint16)
    << static_cast<int>(msg->satellites_visible) << ','
    << static_cast<int>(msg->fix_type)
    << '\n';
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
    rclcpp::spin(node);
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