#include "drone_pipeline/save_video.hpp"

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

VideoConfig SaveVideo::loadConfig()
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

  if (!root["drone_id"])
    throw std::runtime_error("Missing 'drone_id' in config");
  cfg.drone_id = static_cast<uint8_t>(root["drone_id"].as<int>());

  const auto & mv = root["mavros_topics"];
  cfg.odom_topic     = "/drone_" + std::to_string(cfg.drone_id) + mv["odom"].as<std::string>();
  cfg.gps1_raw_topic = "/drone_" + std::to_string(cfg.drone_id) + mv["gps1_raw"].as<std::string>();

  cfg.images_topic = "/drone_" + std::to_string(cfg.drone_id) +
                     "/" + root["custom_topics"]["images"].as<std::string>();
  cfg.logs_path    = root["flight_params"]["logs_path"].as<std::string>();

  const auto cam = root["camera"];
  cfg.width  = cam["width"].as<int>();
  cfg.height = cam["height"].as<int>();
  cfg.fps    = cam["fps"].as<int>();

  RCLCPP_INFO(get_logger(),
    "Config → drone_id=%u  odom=%s  gps=%s  images=%s  logs=%s  res=%dx%d@%dfps",
    cfg.drone_id,
    cfg.odom_topic.c_str(), cfg.gps1_raw_topic.c_str(),
    cfg.images_topic.c_str(), cfg.logs_path.c_str(),
    cfg.width, cfg.height, cfg.fps);

  return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Session directory
// ─────────────────────────────────────────────────────────────────────────────

std::string SaveVideo::resolveSessionDir(const std::string & logs_path)
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

SaveVideo::SaveVideo(const rclcpp::NodeOptions & options)
: Node("save_video", options)
{
  config_      = loadConfig();
  session_dir_ = resolveSessionDir(config_.logs_path);
  videos_dir_  = session_dir_ + "/videos";
  data_dir_    = session_dir_ + "/data";

  fs::create_directories(videos_dir_);
  fs::create_directories(data_dir_);

  RCLCPP_INFO(get_logger(), "Videos → %s", videos_dir_.c_str());
  RCLCPP_INFO(get_logger(), "Data   → %s", data_dir_.c_str());

  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const auto sensor_qos   = rclcpp::SensorDataQoS();
  const std::string dp    = "/drone_" + std::to_string(config_.drone_id);

  image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    config_.images_topic, sensor_qos,
    [this](sensor_msgs::msg::CompressedImage::UniquePtr msg) {
      imageCallback(std::move(msg));
    });

  record_cmd_sub_   = create_subscription<drone_msgs::msg::Toggle>(
    dp + "/camera/record/cmd", reliable_qos,
    [this](const drone_msgs::msg::Toggle::SharedPtr msg) { recordCmdCallback(msg); });
  record_state_pub_ = create_publisher<drone_msgs::msg::Toggle>(
    dp + "/camera/record/active", reliable_qos);

  stream_cmd_sub_   = create_subscription<drone_msgs::msg::Toggle>(
    dp + "/camera/stream/cmd", reliable_qos,
    [this](const drone_msgs::msg::Toggle::SharedPtr msg) { streamCmdCallback(msg); });
  stream_state_pub_ = create_publisher<drone_msgs::msg::Toggle>(
    dp + "/camera/stream/active", reliable_qos);
  stream_out_pub_   = create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
    dp + "/camera/stream/out",
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    config_.odom_topic, sensor_qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });
  gps_sub_  = create_subscription<mavros_msgs::msg::GPSRAW>(
    config_.gps1_raw_topic, reliable_qos,
    [this](const mavros_msgs::msg::GPSRAW::SharedPtr msg) { gpsCallback(msg); });

  publishRecordState();
  publishStreamState();

  RCLCPP_INFO(get_logger(),
    "save_video ready.\n"
    "  record : %s/camera/record/{cmd,active}\n"
    "  stream : %s/camera/stream/{cmd,active,out}",
    dp.c_str(), dp.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Destructor
// ─────────────────────────────────────────────────────────────────────────────

SaveVideo::~SaveVideo()
{
  if (recording_.load()) {
    recording_.store(false);
    stopRecordThread();
    closeClip();
  } else {
    stopRecordThread();
  }

  if (streaming_.load()) {
    streaming_.store(false);
    stopStreamThread();
    std::lock_guard<std::mutex> lk(encoder_mtx_);
    h264_encoder_.close();
  } else {
    stopStreamThread();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Recording clip management
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::openClip()
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
               "lat_deg_e7,lon_deg_e7\n";

  RCLCPP_INFO(get_logger(), "Recording started → %s  %s",
    video_path.c_str(), csv_path.c_str());
}

void SaveVideo::closeClip()
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

void SaveVideo::startRecordThread()
{
  record_thread_running_.store(true);
  record_thread_ = std::thread(&SaveVideo::recordLoop, this);
}

void SaveVideo::stopRecordThread()
{
  { std::lock_guard<std::mutex> lk(record_queue_mtx_); record_thread_running_.store(false); }
  record_queue_cv_.notify_one();
  if (record_thread_.joinable()) record_thread_.join();
}

void SaveVideo::recordLoop()
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

    mjpeg_writer_.writeFrame(task.jpeg_data);

    csv_file_
      << task.stamp_sec     << ',' << task.stamp_nanosec  << ','
      << task.odom.pos_x    << ',' << task.odom.pos_y     << ',' << task.odom.pos_z  << ','
      << task.odom.quat_x   << ',' << task.odom.quat_y    << ','
      << task.odom.quat_z   << ',' << task.odom.quat_w    << ','
      << task.odom.vel_x    << ',' << task.odom.vel_y     << ',' << task.odom.vel_z  << ','
      << task.gps.lat       << ',' << task.gps.lon        << '\n';
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Streaming thread
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::startStreamThread()
{
  stream_thread_running_.store(true);
  stream_thread_ = std::thread(&SaveVideo::streamLoop, this);
}

void SaveVideo::stopStreamThread()
{
  { std::lock_guard<std::mutex> lk(stream_queue_mtx_); stream_thread_running_.store(false); }
  stream_queue_cv_.notify_one();
  if (stream_thread_.joinable()) stream_thread_.join();
}

void SaveVideo::streamLoop()
{
  while (true) {
    StreamTask task;
    {
      std::unique_lock<std::mutex> lk(stream_queue_mtx_);
      stream_queue_cv_.wait(lk, [this] {
        return !stream_queue_.empty() || !stream_thread_running_.load();
      });
      if (stream_queue_.empty()) break;
      task = std::move(stream_queue_.front());
      stream_queue_.pop();
    }

    H264Encoder::EncodeResult result;
    {
      std::lock_guard<std::mutex> lk(encoder_mtx_);
      if (!h264_encoder_.isOpen()) continue;
      result = h264_encoder_.encode(task.jpeg_data);
    }

    if (result.data.empty()) continue;

    auto out_msg = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
    out_msg->header.stamp.sec     = task.stamp_sec;
    out_msg->header.stamp.nanosec = task.stamp_nanosec;
    out_msg->width                = config_.width;
    out_msg->height               = config_.height;
    out_msg->encoding             = "h264";
    out_msg->pts                  = task.stamp_sec * 1000000000ULL + task.stamp_nanosec;
    // flags bit 0 == AV_PKT_FLAG_KEY, matching the ffmpeg_image_transport convention
    out_msg->flags                = result.is_key_frame ? 1 : 0;
    out_msg->data                 = std::move(result.data);

    stream_out_pub_->publish(std::move(out_msg));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  State publishers
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::publishRecordState()
{
  drone_msgs::msg::Toggle msg;
  msg.state = recording_.load();
  record_state_pub_->publish(msg);
}

void SaveVideo::publishStreamState()
{
  drone_msgs::msg::Toggle msg;
  msg.state = streaming_.load();
  stream_state_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Toggle callbacks
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::recordCmdCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
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

void SaveVideo::streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
{
  if (!streaming_.load()) {
    try {
      {
        std::lock_guard<std::mutex> lk(encoder_mtx_);
        h264_encoder_.open(config_.width, config_.height, config_.fps, config_.fps);
      }
      startStreamThread();
      streaming_.store(true);
      RCLCPP_INFO(get_logger(), "Stream → ON  (%dx%d @ %d fps, GOP=%d)",
        config_.width, config_.height, config_.fps, config_.fps);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to start streaming: %s", e.what());
      return;
    }
  } else {
    streaming_.store(false);
    stopStreamThread();
    {
      std::lock_guard<std::mutex> lk(encoder_mtx_);
      h264_encoder_.close();
    }
    RCLCPP_INFO(get_logger(), "Stream → OFF");
  }
  publishStreamState();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Sensor callbacks
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  const auto & v = msg->twist.twist.linear;

  OdomSnapshot snap;
  snap.stamp  = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
  snap.pos_x  = p.x;  snap.pos_y  = p.y;  snap.pos_z  = p.z;
  snap.quat_x = q.x;  snap.quat_y = q.y;  snap.quat_z = q.z;  snap.quat_w = q.w;
  snap.vel_x  = v.x;  snap.vel_y  = v.y;  snap.vel_z  = v.z;

  std::lock_guard<std::mutex> lk(odom_mtx_);
  latest_odom_ = snap;
}

void SaveVideo::gpsCallback(const mavros_msgs::msg::GPSRAW::SharedPtr msg)
{
  GpsSnapshot snap;
  snap.stamp = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
  snap.lat   = msg->lat;
  snap.lon   = msg->lon;

  std::lock_guard<std::mutex> lk(gps_mtx_);
  latest_gps_ = snap;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Image callback  — fans frame out to active pipelines
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::imageCallback(sensor_msgs::msg::CompressedImage::UniquePtr msg)
{
  const bool do_record = recording_.load();
  const bool do_stream = streaming_.load();

  if (!do_record && !do_stream) return;

  if (do_record) {
    OdomSnapshot odom_snap;
    GpsSnapshot  gps_snap;
    bool have_odom, have_gps;

    { std::lock_guard<std::mutex> lk(odom_mtx_); have_odom = latest_odom_.has_value(); if (have_odom) odom_snap = *latest_odom_; }
    { std::lock_guard<std::mutex> lk(gps_mtx_);  have_gps  = latest_gps_.has_value();  if (have_gps)  gps_snap  = *latest_gps_;  }

    if (!have_odom || !have_gps) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for odom + gps — record frame dropped.");
    } else {
      RecordTask rtask;
      rtask.jpeg_data     = do_stream ? msg->data : std::move(msg->data);
      rtask.stamp_sec     = msg->header.stamp.sec;
      rtask.stamp_nanosec = msg->header.stamp.nanosec;
      rtask.odom          = odom_snap;
      rtask.gps           = gps_snap;

      { std::lock_guard<std::mutex> lk(record_queue_mtx_); record_queue_.push(std::move(rtask)); }
      record_queue_cv_.notify_one();
    }
  }

  if (do_stream) {
    StreamTask stask;
    stask.jpeg_data     = std::move(msg->data);
    stask.stamp_sec     = msg->header.stamp.sec;
    stask.stamp_nanosec = msg->header.stamp.nanosec;

    { std::lock_guard<std::mutex> lk(stream_queue_mtx_); stream_queue_.push(std::move(stask)); }
    stream_queue_cv_.notify_one();
  }
}

}  // namespace drone_pipeline

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_pipeline::SaveVideo)