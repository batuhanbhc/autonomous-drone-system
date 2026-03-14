#include "drone_pipeline/save_video.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include <opencv2/imgcodecs.hpp>   // cv::imdecode
#include <opencv2/imgproc.hpp>     // cv::resize

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

  if (!root["custom_topics"] || !root["custom_topics"]["images"])
    throw std::runtime_error("Missing 'custom_topics/images'");
  cfg.images_topic = "/drone_" + std::to_string(cfg.drone_id) +
                         "/" + root["custom_topics"]["images"].as<std::string>();

  if (!root["flight_params"] || !root["flight_params"]["logs_path"])
    throw std::runtime_error("Missing 'flight_params/logs_path'");
  cfg.logs_path = root["flight_params"]["logs_path"].as<std::string>();

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
    cfg.images_topic.c_str(),
    cfg.logs_path.c_str(),
    cfg.width, cfg.height, cfg.fps);

  return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Session directory resolution
// ─────────────────────────────────────────────────────────────────────────────

std::string SaveVideo::resolveSessionDir(const std::string & logs_path)
{
  fs::create_directories(logs_path);

  std::size_t dir_count = 0;
  for (const auto & entry : fs::directory_iterator(logs_path)) {
    if (entry.is_directory()) ++dir_count;
  }

  std::size_t session_num = (dir_count == 0) ? 1 : dir_count;

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
  // Must be set before any VideoWriter is opened
  ::setenv("OPENCV_FFMPEG_WRITER_OPTIONS",
           "preset;ultrafast|tune;zerolatency|crf;28",
           /*overwrite=*/0);

  config_      = loadConfig();
  session_dir_ = resolveSessionDir(config_.logs_path);
  videos_dir_  = session_dir_ + "/videos";
  data_dir_    = session_dir_ + "/data";

  fs::create_directories(videos_dir_);
  fs::create_directories(data_dir_);

  RCLCPP_INFO(get_logger(), "Videos → %s", videos_dir_.c_str());
  RCLCPP_INFO(get_logger(), "Data   → %s", data_dir_.c_str());

  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const auto sensor_qos   = rclcpp::SensorDataQoS();   // best-effort

  const std::string drone_prefix =
    "/drone_" + std::to_string(config_.drone_id);

  // ── Toggle / state ────────────────────────────────────────────────────────
  toggle_sub_ = create_subscription<drone_msgs::msg::Toggle>(
    drone_prefix + "/camera/record_input", reliable_qos,
    [this](const drone_msgs::msg::Toggle::SharedPtr msg) {
      toggleCallback(msg);
    });

  state_pub_ = create_publisher<drone_msgs::msg::Toggle>(
    drone_prefix + "/camera/record_state", reliable_qos);

  // ── CompressedImage — UniquePtr callback for zero-copy intra-process ──────
  // ROS2 intra-process transport will call this overload directly with a
  // moved unique_ptr when the publisher is in the same process, avoiding any
  // heap allocation or copy.  For inter-process topics the middleware creates
  // a new unique_ptr from the deserialized bytes, so the callback signature
  // remains identical in both cases.
  image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    config_.images_topic,
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CompressedImage::UniquePtr msg) {
      imageCallback(std::move(msg));
    });

  // ── Odometry & GPS ────────────────────────────────────────────────────────
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
    recording_.store(false);
    stopEncoderThread();
    closeClip();
  } else {
    stopEncoderThread();
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

  // ── Configure VideoWriter for H.264 / ultrafast ───────────────────────────
  // cv::VideoWriter accepts a CAP_PROP_HW_ACCELERATION hint and extra params
  // via the VideoCaptureAPIs overload.  The cleaner portable path is to pass
  // encoder parameters through the environment or rely on OpenCV's GStreamer /
  // FFmpeg back-end selecting x264 automatically for avc1 / mp4v fourcc.
  //
  // For the lowest possible CPU cost on an embedded platform:
  //   fourcc = 'avc1' (H.264 baseline, handled by libx264 in FFmpeg back-end)
  //   OPENCV_FFMPEG_WRITER_OPTIONS env var sets x264 options at process start
  //   (set in main() below).
  //
  // Alternatively, if the platform has hardware H.264 (V4L2/VAAPI/OMX) the
  // GStreamer pipeline override approach is more reliable, but that requires
  // knowing the exact HW codec name at deploy time.  We keep it portable here.

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
  if (writer_.isOpened()) writer_.release();
  if (csv_file_.is_open()) { csv_file_.flush(); csv_file_.close(); }

  std::ostringstream idx;
  idx << std::setw(3) << std::setfill('0') << clip_index_;
  RCLCPP_INFO(get_logger(),
    "Recording stopped → clip %s finalized.", idx.str().c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Encoder thread management
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::startEncoderThread()
{
  encoder_running_.store(true);
  encoder_thread_ = std::thread(&SaveVideo::encoderLoop, this);
}

void SaveVideo::stopEncoderThread()
{
  {
    std::lock_guard<std::mutex> lk(queue_mtx_);
    encoder_running_.store(false);
  }
  queue_cv_.notify_one();
  if (encoder_thread_.joinable()) {
    encoder_thread_.join();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Encoder loop
//
//  Runs on a dedicated thread.  Pulls EncodeTask objects from encode_queue_,
//  decodes JPEG → BGR with cv::imdecode, resizes if needed, writes the frame
//  to the VideoWriter, and appends a CSV row — all without touching the ROS
//  executor thread.
//
//  CPU notes:
//    • cv::imdecode for JPEG is libjpeg-turbo under the hood on most distros:
//      fast SIMD-accelerated decode, typically < 1 ms per 720p frame.
//    • x264 ultrafast + zerolatency cuts encode time to a fraction of what
//      the default medium preset costs; quality loss is acceptable for
//      telemetry video.
//    • We do NOT hold any mutex while encoding so the subscriber callback
//      never stalls waiting for the encoder.
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::encoderLoop()
{
  while (true) {
    EncodeTask task;

    {
      std::unique_lock<std::mutex> lk(queue_mtx_);
      queue_cv_.wait(lk, [this] {
        return !encode_queue_.empty() || !encoder_running_.load();
      });

      // Drain remaining frames even after stop signal so nothing is lost
      if (encode_queue_.empty()) {
        break;   // encoder_running_ == false and queue is empty → exit
      }

      task = std::move(encode_queue_.front());
      encode_queue_.pop();
    }

    // ── Decode JPEG bytes → BGR Mat ───────────────────────────────────────
    // cv::imdecode is allocation-light: it writes directly into the Mat
    // buffer.  IMREAD_COLOR forces BGR output regardless of JPEG colour space.
    cv::Mat frame = cv::imdecode(
      cv::InputArray(task.jpeg_data),
      cv::IMREAD_COLOR);

    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "imdecode produced an empty frame — skipping.");
      continue;
    }

    // Resize only when the JPEG resolution differs from the configured size
    if (frame.cols != config_.width || frame.rows != config_.height) {
      cv::resize(frame, frame, cv::Size(config_.width, config_.height));
    }

    // ── Write frame + CSV ─────────────────────────────────────────────────
    std::lock_guard<std::mutex> lk(clip_mtx_);

    if (!writer_.isOpened() || !csv_file_.is_open()) {
      // Recording was stopped between the push and now — discard gracefully
      continue;
    }

    writer_.write(frame);

    csv_file_
      << task.stamp_sec          << ','
      << task.stamp_nanosec      << ','
      << task.odom.pos_x         << ',' << task.odom.pos_y  << ',' << task.odom.pos_z  << ','
      << task.odom.quat_x        << ',' << task.odom.quat_y << ','
      << task.odom.quat_z        << ',' << task.odom.quat_w << ','
      << task.odom.vel_x         << ',' << task.odom.vel_y  << ',' << task.odom.vel_z  << ','
      << task.gps.lat            << ',' << task.gps.lon
      << '\n';
  }
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
//  Toggle callback
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::toggleCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
{
  if (!recording_.load()) {
    // OFF → ON
    try {
      openClip();
      startEncoderThread();
      recording_.store(true);
      RCLCPP_INFO(get_logger(), "Mode → ON  (clip %03d)", clip_index_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to start recording: %s", e.what());
      return;
    }
  } else {
    // ON → OFF
    // Stop accepting new frames first, then drain the encoder queue before
    // closing the clip so no buffered frames are lost.
    recording_.store(false);
    stopEncoderThread();
    closeClip();
    RCLCPP_INFO(get_logger(), "Mode → OFF");
  }

  publishState();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Odom callback
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  const auto & v = msg->twist.twist.linear;

  OdomSnapshot snap;
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
//  GPS callback
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
//  Image callback  (UniquePtr overload — zero-copy intra-process)
//
//  This callback runs on the ROS executor thread and must return quickly.
//  All it does is:
//    1. Snapshot sensor caches
//    2. Move the JPEG bytes into an EncodeTask
//    3. Push to the encoder queue
//    4. Return
//
//  The encoder thread owns the actual decode + write work.
// ─────────────────────────────────────────────────────────────────────────────

void SaveVideo::imageCallback(sensor_msgs::msg::CompressedImage::UniquePtr msg)
{
  if (!recording_.load()) return;

  // ── Snapshot sensor caches ────────────────────────────────────────────────
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

  // ── Build task and push to encoder queue ──────────────────────────────────
  // std::move(msg->data) avoids copying the JPEG bytes; the unique_ptr itself
  // is consumed here so there is no dangling reference.
  EncodeTask task;
  task.jpeg_data       = std::move(msg->data);   // zero-copy move of the vector
  task.stamp_sec       = msg->header.stamp.sec;
  task.stamp_nanosec   = msg->header.stamp.nanosec;
  task.odom            = odom_snap;
  task.gps             = gps_snap;

  {
    std::lock_guard<std::mutex> lk(queue_mtx_);
    encode_queue_.push(std::move(task));
  }
  queue_cv_.notify_one();
}

}  // namespace drone_pipeline
// ─────────────────────────────────────────────────────────────────────────────

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_pipeline::SaveVideo)