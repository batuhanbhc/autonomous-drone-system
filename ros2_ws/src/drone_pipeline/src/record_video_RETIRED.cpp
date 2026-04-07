#include "drone_pipeline/record_video.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace fs = std::filesystem;

namespace drone_pipeline
{

// ─────────────────────────────────────────────────────────────────────────────
//  Config loading
// ─────────────────────────────────────────────────────────────────────────────

VideoConfig RecordVideo::loadConfig()
{
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

  VideoConfig cfg;
  cfg.drone_id     = static_cast<uint8_t>(root["drone_id"].as<int>());
  cfg.frames_topic = "/drone_" + std::to_string(cfg.drone_id) +
                     "/" + root["custom_topics"]["images"].as<std::string>();
  cfg.logs_path    = root["flight_params"]["logs_path"].as<std::string>();
  const auto cam   = root["camera"];
  cfg.width        = cam["width"].as<int>();
  cfg.height       = cam["height"].as<int>();
  cfg.fps          = cam["fps"].as<int>();
  return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Session directory
// ─────────────────────────────────────────────────────────────────────────────

std::string RecordVideo::resolveSessionDir(const std::string & logs_path)
{
  fs::create_directories(logs_path);

  std::size_t dir_count = 0;
  for (const auto & entry : fs::directory_iterator(logs_path))
    if (entry.is_directory()) ++dir_count;

  std::size_t session_num = (dir_count == 0) ? 1 : dir_count;

  std::ostringstream oss;
  oss << std::setw(4) << std::setfill('0') << session_num;
  std::string candidate = logs_path + "/" + oss.str();

  if (!fs::exists(candidate)) fs::create_directory(candidate);

  RCLCPP_INFO(get_logger(), "Session directory: %s", candidate.c_str());
  return candidate;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────

RecordVideo::RecordVideo(const rclcpp::NodeOptions & options)
: Node("record_video", options)
{
  config_      = loadConfig();
  session_dir_ = resolveSessionDir(config_.logs_path);
  videos_dir_  = session_dir_ + "/videos";
  data_dir_    = session_dir_ + "/data";
  fs::create_directories(videos_dir_);
  fs::create_directories(data_dir_);

  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const std::string dp    = "/drone_" + std::to_string(config_.drone_id);

  frame_sub_ = create_subscription<drone_msgs::msg::FrameData>(
    config_.frames_topic, rclcpp::SensorDataQoS(),
    [this](drone_msgs::msg::FrameData::ConstSharedPtr msg) {
      frameCallback(msg);
    });

  record_cmd_sub_   = create_subscription<drone_msgs::msg::Toggle>(
    dp + "/camera/record/cmd", reliable_qos,
    [this](const drone_msgs::msg::Toggle::SharedPtr msg) { recordCmdCallback(msg); });
  record_state_pub_ = create_publisher<drone_msgs::msg::Toggle>(
    dp + "/camera/record/active", reliable_qos);

  publishRecordState();
  RCLCPP_INFO(get_logger(), "record_video ready.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Destructor
// ─────────────────────────────────────────────────────────────────────────────

RecordVideo::~RecordVideo()
{
  if (recording_.load()) {
    recording_.store(false);
    stopRecordThread();
    closeClip();
  } else {
    stopRecordThread();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Recording clip management
// ─────────────────────────────────────────────────────────────────────────────

void RecordVideo::openClip()
{
  ++clip_index_;

  std::ostringstream idx;
  idx << std::setw(3) << std::setfill('0') << clip_index_;

  const std::string video_path = videos_dir_ + "/" + idx.str() + ".avi";
  const std::string csv_path   = data_dir_   + "/" + idx.str() + ".csv";

  mjpeg_writer_.open(video_path, config_.width, config_.height, config_.fps);

  csv_file_.open(csv_path, std::ios::out | std::ios::trunc);
  if (!csv_file_.is_open())
    throw std::runtime_error("Cannot open CSV: " + csv_path);

  csv_file_ << "frame_timestamp_sec,frame_timestamp_nanosec,"
             "pos_x,pos_y,pos_z,"
             "quat_x,quat_y,quat_z,quat_w,"
             "vel_x,vel_y,vel_z,"
             "lat_deg_e7,lon_deg_e7,"
             "odom_valid,gps_valid\n";

  RCLCPP_INFO(get_logger(), "Recording started → %s  %s",
    video_path.c_str(), csv_path.c_str());
}

void RecordVideo::closeClip()
{
  std::lock_guard<std::mutex> lk(clip_mtx_);
  mjpeg_writer_.close();
  if (csv_file_.is_open()) { csv_file_.flush(); csv_file_.close(); }

  std::ostringstream idx;
  idx << std::setw(3) << std::setfill('0') << clip_index_;
  RCLCPP_INFO(get_logger(), "Recording stopped → clip %s finalized.", idx.str().c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Recording thread
// ─────────────────────────────────────────────────────────────────────────────

void RecordVideo::startRecordThread()
{
  record_thread_running_.store(true);
  record_thread_ = std::thread(&RecordVideo::recordLoop, this);
}

void RecordVideo::stopRecordThread()
{
  { std::lock_guard<std::mutex> lk(record_queue_mtx_); record_thread_running_.store(false); }
  record_queue_cv_.notify_one();
  if (record_thread_.joinable()) record_thread_.join();
}

void RecordVideo::recordLoop()
{
  while (true) {
    RecordTask task;
    {
      std::unique_lock<std::mutex> lk(record_queue_mtx_);
      record_queue_cv_.wait(lk, [this] {
        return !record_queue_.empty() || !record_thread_running_.load();
      });
      if (record_queue_.empty()) break;
      task = std::move(record_queue_.front());
      record_queue_.pop();
    }

    std::lock_guard<std::mutex> lk(clip_mtx_);
    if (!mjpeg_writer_.isOpen() || !csv_file_.is_open()) continue;

    const auto & m = *task.msg;

    mjpeg_writer_.writeFrame(m.image.data);

    csv_file_
      << m.image.header.stamp.sec  << ',' << m.image.header.stamp.nanosec << ','
      << m.pos_x  << ',' << m.pos_y  << ',' << m.pos_z  << ','
      << m.quat_x << ',' << m.quat_y << ','
      << m.quat_z << ',' << m.quat_w << ','
      << m.vel_x  << ',' << m.vel_y  << ',' << m.vel_z  << ','
      << m.lat    << ',' << m.lon    << ','
      << m.odom_valid << ',' << m.gps_valid << '\n';
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  State publisher
// ─────────────────────────────────────────────────────────────────────────────

void RecordVideo::publishRecordState()
{
  drone_msgs::msg::Toggle msg;
  msg.state = recording_.load();
  record_state_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Toggle callback
// ─────────────────────────────────────────────────────────────────────────────

void RecordVideo::recordCmdCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
{
  if (!recording_.load()) {
    try {
      openClip();
      startRecordThread();
      recording_.store(true);
      RCLCPP_INFO(get_logger(), "Record → ON  (clip %03d)", clip_index_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to start recording: %s", e.what());
      return;
    }
  } else {
    recording_.store(false);
    stopRecordThread();
    closeClip();
    RCLCPP_INFO(get_logger(), "Record → OFF");
  }
  publishRecordState();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Frame callback 
// ─────────────────────────────────────────────────────────────────────────────

void RecordVideo::frameCallback(drone_msgs::msg::FrameData::ConstSharedPtr msg)
{
  if (!recording_.load()) return;

  bool notify = false;
  {
    std::lock_guard<std::mutex> lk(record_queue_mtx_);
    if (record_queue_.size() >= kMaxQueueDepth) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Record queue full — dropping frame");
    } else {
      record_queue_.push({msg});
      notify = true;
    }
  }
  if (notify) record_queue_cv_.notify_one();
}

}  // namespace drone_pipeline

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_pipeline::RecordVideo)