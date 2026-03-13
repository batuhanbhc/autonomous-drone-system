#include "drone_pipeline/save_video.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;

namespace drone_pipeline
{

// ─────────────────────────────────────────────────────────────────────────────
//  Config loading  (mirrors flight_logger / camera_capture pattern)
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

  // ── drone_id ─────────────────────────────────────────────────────────────
  if (!root["drone_id"])
    throw std::runtime_error("Missing 'drone_id' in config");
  cfg.drone_id = static_cast<uint8_t>(root["drone_id"].as<int>());

  // ── mavros_topics (odom + gps) ────────────────────────────────────────────
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

  // ── custom_topics (raw images) ────────────────────────────────────────────
  if (!root["custom_topics"] || !root["custom_topics"]["raw_images"])
    throw std::runtime_error("Missing 'custom_topics/raw_images'");
  cfg.raw_images_topic = "/drone_" + std::to_string(cfg.drone_id) +
                         "/" + root["custom_topics"]["raw_images"].as<std::string>();

  // ── flight_params/logs_path ───────────────────────────────────────────────
  if (!root["flight_params"] || !root["flight_params"]["logs_path"])
    throw std::runtime_error("Missing 'flight_params/logs_path'");
  cfg.logs_path = root["flight_params"]["logs_path"].as<std::string>();

  // ── camera (width / height / fps) ─────────────────────────────────────────
  if (!root["camera"])
    throw std::runtime_error("Missing 'camera' section");
  const auto cam = root["camera"];
  cfg.width  = cam["width"].as<int>();
  cfg.height = cam["height"].as<int>();
  cfg.fps    = cam["fps"].as<int>();

  RCLCPP_INFO(
    get_logger(),
    "Config → drone_id=%u  odom=%s  gps=%s  images=%s  "
    "logs=%s  res=%dx%d@%dfps",
    cfg.drone_id,
    cfg.odom_topic.c_str(),
    cfg.gps1_raw_topic.c_str(),
    cfg.raw_images_topic.c_str(),
    cfg.logs_path.c_str(),
    cfg.width, cfg.height, cfg.fps);

  return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Session directory resolution
//  Counts existing sub-directories → picks the SAME session already used by
//  flight_logger (they count the same way and run in the same launch).
//  The session directory is created here only if it doesn't exist yet.
// ─────────────────────────────────────────────────────────────────────────────

std::string SaveVideo::resolveSessionDir(const std::string & logs_path)
{
  fs::create_directories(logs_path);

  std::size_t dir_count = 0;
  for (const auto & entry : fs::directory_iterator(logs_path)) {
    if (entry.is_directory()) ++dir_count;
  }

  // If flight_logger already created the session directory it will show up in
  // the count.  If this node starts first, it creates a new one (same logic).
  std::size_t session_num = (dir_count == 0) ? 1 : dir_count;

  // Peek: does the highest-numbered directory already exist?
  // Re-use it rather than creating a new one (sibling nodes share a session).
  std::ostringstream oss;
  oss << std::setw(4) << std::setfill('0') << session_num;
  std::string candidate = logs_path + "/" + oss.str();

  if (!fs::exists(candidate)) {
    fs::create_directory(candidate);
  }

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

  // ── Reliable QoS for toggle ───────────────────────────────────────────────
  const auto reliable_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  const std::string drone_prefix =
    "/drone_" + std::to_string(config_.drone_id);

  toggle_sub_ = create_subscription<drone_msgs::msg::Toggle>(
    drone_prefix + "/camera/record_input", reliable_qos,
    [this](const drone_msgs::msg::Toggle::SharedPtr msg) {
      toggleCallback(msg);
    });

  state_pub_ = create_publisher<drone_msgs::msg::Toggle>(
    drone_prefix + "/camera/record_state", reliable_qos);

  // ── Sensor-data QoS for image / odom / gps ────────────────────────────────
  const auto sensor_qos = rclcpp::SensorDataQoS();

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    config_.raw_images_topic, sensor_qos,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      imageCallback(msg);
    });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    config_.odom_topic, sensor_qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odomCallback(msg);
    });

  gps_sub_ = create_subscription<mavros_msgs::msg::GPSRAW>(
    config_.gps1_raw_topic, sensor_qos,
    [this](const mavros_msgs::msg::GPSRAW::SharedPtr msg) {
      gpsCallback(msg);
    });

  // Publish initial state (mode = off)
  publishState();

  RCLCPP_INFO(get_logger(),
    "save_video ready.  toggle=%s/camera/record_input  "
    "state=%s/camera/record_state",
    drone_prefix.c_str(), drone_prefix.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Destructor
// ─────────────────────────────────────────────────────────────────────────────

SaveVideo::~SaveVideo()
{
  if (recording_.load()) {
    closeClip();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Open / close a recording clip
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::openClip()
{
  ++clip_index_;

  std::ostringstream idx;
  idx << std::setw(3) << std::setfill('0') << clip_index_;

  const std::string video_path = videos_dir_ + "/" + idx.str() + ".mp4";
  const std::string csv_path   = data_dir_   + "/" + idx.str() + ".csv";

  // H.264 / mp4
  const int fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
  writer_.open(video_path, fourcc, config_.fps,
               cv::Size(config_.width, config_.height));

  if (!writer_.isOpened())
    throw std::runtime_error("VideoWriter could not open: " + video_path);

  csv_file_.open(csv_path, std::ios::out | std::ios::trunc);
  if (!csv_file_.is_open())
    throw std::runtime_error("Cannot open CSV: " + csv_path);

  csv_file_
    << "frame_timestamp_sec,frame_timestamp_nanosec,"
    << "pos_x,pos_y,pos_z,"
    << "quat_x,quat_y,quat_z,quat_w,"
    << "vel_x,vel_y,vel_z,"
    << "lat_deg_e7,lon_deg_e7\n";

  RCLCPP_INFO(get_logger(),
    "Recording started → video: %s  csv: %s",
    video_path.c_str(), csv_path.c_str());
}

void SaveVideo::closeClip()
{
  std::lock_guard<std::mutex> lk(clip_mtx_);

  if (writer_.isOpened()) {
    writer_.release();
  }
  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
  }

  std::ostringstream idx;
  idx << std::setw(3) << std::setfill('0') << clip_index_;
  RCLCPP_INFO(get_logger(),
    "Recording stopped → clip %s finalized.", idx.str().c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  publishState
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::publishState()
{
  drone_msgs::msg::Toggle msg;
  msg.state = recording_.load();
  state_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Toggle callback — flips recording on/off regardless of msg.state value
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::toggleCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
{
  if (!recording_.load()) {
    // OFF → ON
    try {
      {
        std::lock_guard<std::mutex> lk(clip_mtx_);
        openClip();
      }
      recording_.store(true);
      RCLCPP_INFO(get_logger(), "Mode → ON  (clip %03d)", clip_index_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to start recording: %s", e.what());
      return;
    }
  } else {
    // ON → OFF
    recording_.store(false);
    closeClip();
    RCLCPP_INFO(get_logger(), "Mode → OFF");
  }

  publishState();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Odom callback — cache latest snapshot
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  const auto & v = msg->twist.twist.linear;

  OdomSnapshot snap;
  // Convert header stamp to rclcpp::Time using ROS_TIME clock for comparison
  snap.stamp  = rclcpp::Time(msg->header.stamp.sec,
                              msg->header.stamp.nanosec,
                              RCL_ROS_TIME);
  snap.pos_x  = p.x;  snap.pos_y = p.y;  snap.pos_z = p.z;
  snap.quat_x = q.x;  snap.quat_y = q.y; snap.quat_z = q.z; snap.quat_w = q.w;
  snap.vel_x  = v.x;  snap.vel_y = v.y;  snap.vel_z  = v.z;

  std::lock_guard<std::mutex> lk(odom_mtx_);
  latest_odom_ = snap;
}

// ─────────────────────────────────────────────────────────────────────────────
//  GPS callback — cache latest snapshot
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::gpsCallback(const mavros_msgs::msg::GPSRAW::SharedPtr msg)
{
  GpsSnapshot snap;
  snap.stamp = rclcpp::Time(msg->header.stamp.sec,
                             msg->header.stamp.nanosec,
                             RCL_ROS_TIME);
  snap.lat = msg->lat;
  snap.lon = msg->lon;

  std::lock_guard<std::mutex> lk(gps_mtx_);
  latest_gps_ = snap;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Image callback — encode frame + write CSV row when recording
//
//  Timestamp strategy:
//    camera_capture stamps frames with now() using the node's clock.
//    odom and gps are stamped by MAVROS from the FCU clock.
//    Both are kept in ROS_TIME (sim time or wall time depending on the
//    launch configuration), so they share the same reference frame.
//    We simply grab the latest cached snapshot; at typical odom/gps rates
//    (≥10 Hz) the age is well below one video frame period.
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!recording_.load()) return;

  // ── Grab sensor snapshots ─────────────────────────────────────────────────
  OdomSnapshot odom_snap;
  GpsSnapshot  gps_snap;
  bool have_odom, have_gps;

  {
    std::lock_guard<std::mutex> lk(odom_mtx_);
    have_odom = latest_odom_.has_value();
    if (have_odom) odom_snap = *latest_odom_;
  }
  {
    std::lock_guard<std::mutex> lk(gps_mtx_);
    have_gps = latest_gps_.has_value();
    if (have_gps) gps_snap = *latest_gps_;
  }

  if (!have_odom || !have_gps) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Waiting for odom + gps data — frame dropped.");
    return;
  }

  // ── Decode image ──────────────────────────────────────────────────────────
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // Accept bgr8 (from camera_capture) or yuv422 / mono8 fallback
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
    return;
  }

  cv::Mat frame = cv_ptr->image;
  if (frame.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty frame — skipping.");
    return;
  }

  // Resize if the live negotiated resolution differs from config
  if (frame.cols != config_.width || frame.rows != config_.height) {
    cv::resize(frame, frame, cv::Size(config_.width, config_.height));
  }

  // ── Write video frame + CSV row ───────────────────────────────────────────
  std::lock_guard<std::mutex> lk(clip_mtx_);

  // Guard against a race where recording_ was set false between the check
  // at the top of the callback and acquiring clip_mtx_.
  if (!writer_.isOpened() || !csv_file_.is_open()) return;

  writer_.write(frame);

  csv_file_
    << msg->header.stamp.sec     << ','
    << msg->header.stamp.nanosec << ','
    << odom_snap.pos_x  << ',' << odom_snap.pos_y  << ',' << odom_snap.pos_z  << ','
    << odom_snap.quat_x << ',' << odom_snap.quat_y << ','
    << odom_snap.quat_z << ',' << odom_snap.quat_w << ','
    << odom_snap.vel_x  << ',' << odom_snap.vel_y  << ',' << odom_snap.vel_z  << ','
    << gps_snap.lat     << ',' << gps_snap.lon
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
    auto node = std::make_shared<drone_pipeline::SaveVideo>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("save_video"),
      "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}