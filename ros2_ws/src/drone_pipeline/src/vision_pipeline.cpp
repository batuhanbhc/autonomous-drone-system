#include "drone_pipeline/vision_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/imgproc.hpp>

extern "C" {
#include <libavutil/opt.h>
#include <libavutil/avutil.h>
}

namespace fs = std::filesystem;

namespace drone_pipeline
{

namespace
{

// TEMP DEBUG: remove after investigating frame-buffer backpressure.
constexpr int kDebugNoSlot = -1;
constexpr int kDebugNumWorkers = 4;
constexpr auto kDebugLogInterval = std::chrono::seconds(1);

struct TempFrameBufferDebugState
{
  std::mutex mtx;
  int mjpeg_idx{kDebugNoSlot};
  int encoder_idx{kDebugNoSlot};
  std::array<int, kDebugNumWorkers> worker_indices{};
  std::chrono::steady_clock::time_point last_log_tp{};
  std::atomic<int> next_worker_id{0};

  TempFrameBufferDebugState()
  {
    worker_indices.fill(kDebugNoSlot);
    last_log_tp = std::chrono::steady_clock::now();
  }
};

TempFrameBufferDebugState g_temp_frame_buffer_debug;

template<typename T>
T yamlOr(const YAML::Node & node, const char * key, const T & default_value)
{
  return node[key] ? node[key].as<T>() : default_value;
}

int clampPixelCoord(int value, int limit)
{
  return std::max(0, std::min(limit - 1, value));
}

std::array<double, 4> estimateMeasurementCovariance(
  GroundProjector & projector,
  int u,
  int v,
  int image_width,
  int image_height,
  double noise_scale,
  double pixel_std_x,
  double pixel_std_y,
  double base_meas_noise_m)
{
  const auto p0 = projector.projectOne(Pixel{u, v});
  if (!p0.valid) {
    const double s = 100.0 * noise_scale;
    return {s, 0.0, 0.0, s};
  }

  const int du = std::max(1, static_cast<int>(std::lround(pixel_std_x)));
  const int dv = std::max(1, static_cast<int>(std::lround(pixel_std_y)));
  const int pu_u = clampPixelCoord(u + du, image_width);
  const int pv_v = clampPixelCoord(v + dv, image_height);

  const auto pu = projector.projectOne(Pixel{pu_u, v});
  const auto pv = projector.projectOne(Pixel{u, pv_v});
  if (!pu.valid || !pv.valid) {
    const double s = 25.0 * noise_scale;
    return {s, 0.0, 0.0, s};
  }

  const double step_u = static_cast<double>(pu_u - u);
  const double step_v = static_cast<double>(pv_v - v);
  if (std::abs(step_u) < 1e-9 || std::abs(step_v) < 1e-9) {
    const double s = 25.0 * noise_scale;
    return {s, 0.0, 0.0, s};
  }

  const double j00 = (pu.world_x - p0.world_x) / step_u;
  const double j10 = (pu.world_y - p0.world_y) / step_u;
  const double j01 = (pv.world_x - p0.world_x) / step_v;
  const double j11 = (pv.world_y - p0.world_y) / step_v;

  const double sigma_u2 = pixel_std_x * pixel_std_x;
  const double sigma_v2 = pixel_std_y * pixel_std_y;

  std::array<double, 4> s = {
    j00 * j00 * sigma_u2 + j01 * j01 * sigma_v2,
    j00 * j10 * sigma_u2 + j01 * j11 * sigma_v2,
    j10 * j00 * sigma_u2 + j11 * j01 * sigma_v2,
    j10 * j10 * sigma_u2 + j11 * j11 * sigma_v2
  };

  const double base_noise2 = base_meas_noise_m * base_meas_noise_m;
  s[0] += base_noise2;
  s[3] += base_noise2;

  for (double & value : s) {
    value *= noise_scale;
  }
  return s;
}

void resetTempFrameBufferDebugState()
{
  std::lock_guard<std::mutex> lk(g_temp_frame_buffer_debug.mtx);
  g_temp_frame_buffer_debug.mjpeg_idx = kDebugNoSlot;
  g_temp_frame_buffer_debug.encoder_idx = kDebugNoSlot;
  g_temp_frame_buffer_debug.worker_indices.fill(kDebugNoSlot);
  g_temp_frame_buffer_debug.last_log_tp = std::chrono::steady_clock::now();
  g_temp_frame_buffer_debug.next_worker_id.store(0, std::memory_order_relaxed);
}

int registerTempWorkerDebugThread()
{
  return g_temp_frame_buffer_debug.next_worker_id.fetch_add(1, std::memory_order_relaxed);
}

void setTempMjpegDebugIndex(int idx)
{
  std::lock_guard<std::mutex> lk(g_temp_frame_buffer_debug.mtx);
  g_temp_frame_buffer_debug.mjpeg_idx = idx;
}

void clearTempMjpegDebugIndex(int idx)
{
  std::lock_guard<std::mutex> lk(g_temp_frame_buffer_debug.mtx);
  if (g_temp_frame_buffer_debug.mjpeg_idx == idx) {
    g_temp_frame_buffer_debug.mjpeg_idx = kDebugNoSlot;
  }
}

void setTempEncoderDebugIndex(int idx)
{
  std::lock_guard<std::mutex> lk(g_temp_frame_buffer_debug.mtx);
  g_temp_frame_buffer_debug.encoder_idx = idx;
}

void clearTempEncoderDebugIndex(int idx)
{
  std::lock_guard<std::mutex> lk(g_temp_frame_buffer_debug.mtx);
  if (g_temp_frame_buffer_debug.encoder_idx == idx) {
    g_temp_frame_buffer_debug.encoder_idx = kDebugNoSlot;
  }
}

void setTempWorkerDebugIndex(int worker_id, int idx)
{
  if (worker_id < 0 || worker_id >= kDebugNumWorkers) {
    return;
  }

  std::lock_guard<std::mutex> lk(g_temp_frame_buffer_debug.mtx);
  g_temp_frame_buffer_debug.worker_indices[worker_id] = idx;
}

void clearTempWorkerDebugIndex(int worker_id, int idx)
{
  if (worker_id < 0 || worker_id >= kDebugNumWorkers) {
    return;
  }

  std::lock_guard<std::mutex> lk(g_temp_frame_buffer_debug.mtx);
  if (g_temp_frame_buffer_debug.worker_indices[worker_id] == idx) {
    g_temp_frame_buffer_debug.worker_indices[worker_id] = kDebugNoSlot;
  }
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
//  Config
// ─────────────────────────────────────────────────────────────────────────────

VisionConfig VisionPipeline::loadConfig()
{
  const std::string share_dir =
    ament_index_cpp::get_package_share_directory("mavros_config");
  const std::string config_path = share_dir + "/config/control_params.yaml";

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to parse YAML: %s", e.what());
    throw;
  }

  VisionConfig cfg;
  cfg.drone_id     = static_cast<uint8_t>(root["drone_id"].as<int>());
  cfg.frames_topic = "/drone_" + std::to_string(cfg.drone_id) +
                     "/" + root["custom_topics"]["images"].as<std::string>();

  const auto cam = root["camera"];
  cfg.width              = cam["width"].as<int>();
  cfg.height             = cam["height"].as<int>();
  cfg.fps                = cam["fps"].as<int>();
  cfg.camera_mount_angle = cam["camera_mount_angle"].as<double>();
  cfg.use_gimbal         = cam["use_gimbal"].as<bool>();
  cfg.reverse_mounted    = cam["reverse_mounted"].as<bool>();

  const auto vp = root["vision_pipeline"];
  cfg.hef_path         = vp["obj_det_hef_path"].as<std::string>();
  cfg.score_threshold  = vp["obj_det_score_thresh"].as<float>();
  cfg.model_input_size = vp["obj_det_model_input_size"].as<int>();
  cfg.person_class_idx = vp["obj_det_person_class_idx"].as<int>();

  const auto intr = vp["camera_intrinsics"];
  cfg.fx = intr["fx"].as<double>();
  cfg.fy = intr["fy"].as<double>();
  cfg.cx = intr["cx"].as<double>();
  cfg.cy = intr["cy"].as<double>();

  const auto dist = vp["distortion"];
  cfg.k1 = dist["k1"].as<double>();
  cfg.k2 = dist["k2"].as<double>();
  cfg.p1 = dist["p1"].as<double>();
  cfg.p2 = dist["p2"].as<double>();
  cfg.k3 = dist["k3"].as<double>();

  cfg.logs_path = root["flight_params"]["logs_path"].as<std::string>();

  cfg.use_mcu_height_estimate = yamlOr<bool>(vp, "use_mcu_height_estimate", false);
  cfg.projection_altitude_lpf_alpha =
    yamlOr<double>(vp, "projection_altitude_lpf_alpha", cfg.projection_altitude_lpf_alpha);

  const auto tracker = vp["tracker"];
  if (tracker) {
    cfg.tracker.max_association_distance_m = yamlOr<double>(
      tracker, "max_association_distance_m", cfg.tracker.max_association_distance_m);
    cfg.tracker.mahalanobis_gate = yamlOr<double>(
      tracker, "mahalanobis_gate", cfg.tracker.mahalanobis_gate);
    cfg.tracker.max_missed_frames = yamlOr<int>(
      tracker, "max_missed_frames", cfg.tracker.max_missed_frames);
    cfg.tracker.min_hits_to_confirm = yamlOr<int>(
      tracker, "min_hits_to_confirm", cfg.tracker.min_hits_to_confirm);
    cfg.tracker.process_noise_std_pos = yamlOr<double>(
      tracker, "process_noise_std_pos", cfg.tracker.process_noise_std_pos);
    cfg.tracker.process_noise_std_vel = yamlOr<double>(
      tracker, "process_noise_std_vel", cfg.tracker.process_noise_std_vel);
    cfg.tracker.base_meas_noise_m = yamlOr<double>(
      tracker, "base_meas_noise_m", cfg.tracker.base_meas_noise_m);
    cfg.tracker.pixel_std_x = yamlOr<double>(
      tracker, "pixel_std_x", cfg.tracker.pixel_std_x);
    cfg.tracker.pixel_std_y = yamlOr<double>(
      tracker, "pixel_std_y", cfg.tracker.pixel_std_y);
    cfg.tracker.angular_vel_low_deg_s = yamlOr<double>(
      tracker, "angular_vel_low_deg_s", cfg.tracker.angular_vel_low_deg_s);
    cfg.tracker.angular_vel_high_deg_s = yamlOr<double>(
      tracker, "angular_vel_high_deg_s", cfg.tracker.angular_vel_high_deg_s);
    cfg.tracker.angular_vel_max_scale = yamlOr<double>(
      tracker, "angular_vel_max_scale", cfg.tracker.angular_vel_max_scale);
    cfg.tracker.new_track_min_dist_calm_m = yamlOr<double>(
      tracker, "new_track_min_dist_calm_m", cfg.tracker.new_track_min_dist_calm_m);
    cfg.tracker.new_track_min_dist_noisy_m = yamlOr<double>(
      tracker, "new_track_min_dist_noisy_m", cfg.tracker.new_track_min_dist_noisy_m);
  }

  RCLCPP_INFO(get_logger(),
    "Config → drone_id=%u  frames=%s  res=%dx%d@%dfps  hef=%s  "
    "score_thresh=%.2f  model_input=%d  person_class=%d  use_mcu_height=%s  alt_lpf=%.2f",
    cfg.drone_id, cfg.frames_topic.c_str(),
    cfg.width, cfg.height, cfg.fps,
    cfg.hef_path.c_str(), cfg.score_threshold,
    cfg.model_input_size, cfg.person_class_idx,
    cfg.use_mcu_height_estimate ? "true" : "false",
    cfg.projection_altitude_lpf_alpha);

  RCLCPP_INFO(get_logger(), "Camera mount angle=%.2f deg  use_gimbal=%s  reverse_mounted=%s",
    cfg.camera_mount_angle, cfg.use_gimbal ? "true" : "false",
    cfg.reverse_mounted ? "true" : "false");

  RCLCPP_INFO(get_logger(),
    "Camera intrinsics → fx=%.4f fy=%.4f cx=%.4f cy=%.4f",
    cfg.fx, cfg.fy, cfg.cx, cfg.cy);

  RCLCPP_INFO(get_logger(),
    "Distortion → k1=%.6f k2=%.6f p1=%.6f p2=%.6f k3=%.6f",
    cfg.k1, cfg.k2, cfg.p1, cfg.p2, cfg.k3);

  RCLCPP_INFO(get_logger(),
    "Tracker → assoc=%.2fm  maha=%.2f  missed=%d  hits=%d  "
    "proc=(%.2f,%.2f)  meas=%.2f  pixel_std=(%.2f,%.2f)  omega_deg_s=(%.2f,%.2f)->x%.2f",
    cfg.tracker.max_association_distance_m,
    cfg.tracker.mahalanobis_gate,
    cfg.tracker.max_missed_frames,
    cfg.tracker.min_hits_to_confirm,
    cfg.tracker.process_noise_std_pos,
    cfg.tracker.process_noise_std_vel,
    cfg.tracker.base_meas_noise_m,
    cfg.tracker.pixel_std_x,
    cfg.tracker.pixel_std_y,
    cfg.tracker.angular_vel_low_deg_s,
    cfg.tracker.angular_vel_high_deg_s,
    cfg.tracker.angular_vel_max_scale);

  return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Session directory
//  FlightLogger is disabled, so VisionPipeline now owns session creation.
//  Always creates dir_count+1 to avoid colliding with any pre-existing dirs.
// ─────────────────────────────────────────────────────────────────────────────

std::string VisionPipeline::resolveSessionDir(const std::string & logs_path)
{
  fs::create_directories(logs_path);

  std::size_t dir_count = 0;
  for (const auto & entry : fs::directory_iterator(logs_path))
    if (entry.is_directory()) ++dir_count;

  std::ostringstream oss;
  oss << std::setw(4) << std::setfill('0') << (dir_count + 1);
  const std::string candidate = logs_path + "/" + oss.str();
  fs::create_directory(candidate);

  RCLCPP_INFO(get_logger(), "Session directory: %s", candidate.c_str());
  return candidate;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Hailo initialisation
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::initHailo()
{
  auto vdevice_exp = hailort::VDevice::create();
  if (!vdevice_exp)
    throw std::runtime_error("VDevice::create failed: " +
      std::to_string(static_cast<int>(vdevice_exp.status())));
  hailo_vdevice_ = vdevice_exp.release();

  auto infer_model_exp = hailo_vdevice_->create_infer_model(config_.hef_path);
  if (!infer_model_exp)
    throw std::runtime_error("create_infer_model failed: " +
      std::to_string(static_cast<int>(infer_model_exp.status())));
  auto infer_model = infer_model_exp.release();

  infer_model->set_batch_size(1);

  if (!infer_model->get_output_names().empty()) {
    auto out_name = infer_model->get_output_names().front();
    auto out = infer_model->output(out_name);
    if (out) {
      out->set_nms_score_threshold(config_.score_threshold);
      hailo_output_frame_size_ = out->get_frame_size();
    }
  }

  if (hailo_output_frame_size_ == 0)
    throw std::runtime_error("Failed to determine Hailo output frame size");

  auto configured_exp = infer_model->configure();
  if (!configured_exp)
    throw std::runtime_error("InferModel::configure failed: " +
      std::to_string(static_cast<int>(configured_exp.status())));
  hailo_infer_model_ = configured_exp.release();

  const size_t nms_output_floats = hailo_output_frame_size_ / sizeof(float);

  const auto & input_names  = infer_model->get_input_names();
  const auto & output_names = infer_model->get_output_names();

  if (input_names.empty())  throw std::runtime_error("Hailo model has no inputs");
  if (output_names.empty()) throw std::runtime_error("Hailo model has no outputs");

  const auto   input_name       = input_names.front();
  const auto   output_name      = output_names.front();
  const size_t input_frame_size = infer_model->input(input_name)->get_frame_size();

  for (int i = 0; i < kNumSlots; ++i) {
    buffer_[i].letterbox_buf.resize(input_frame_size, 0);
    buffer_[i].nms_output_buf.resize(nms_output_floats, 0.0f);

    auto bindings_exp = hailo_infer_model_.create_bindings();
    if (!bindings_exp)
      throw std::runtime_error("create_bindings failed for slot " +
        std::to_string(i) + ": " +
        std::to_string(static_cast<int>(bindings_exp.status())));

    auto bindings = bindings_exp.release();

    auto in_status = bindings.input(input_name)->set_buffer(
      hailort::MemoryView(buffer_[i].letterbox_buf.data(),
                          buffer_[i].letterbox_buf.size()));
    if (in_status != HAILO_SUCCESS)
      throw std::runtime_error("set_buffer (input) failed for slot " +
        std::to_string(i));

    auto out_status = bindings.output(output_name)->set_buffer(
      hailort::MemoryView(
        reinterpret_cast<uint8_t *>(buffer_[i].nms_output_buf.data()),
        buffer_[i].nms_output_buf.size() * sizeof(float)));
    if (out_status != HAILO_SUCCESS)
      throw std::runtime_error("set_buffer (output) failed for slot " +
        std::to_string(i));

    hailo_bindings_[i] = std::move(bindings);
  }

  RCLCPP_INFO(get_logger(), "Hailo initialized successfully");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Yaw calibration
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::runYawCalibration()
{
  yaw_offset_ = 0.0;
  yaw_calibrated_.store(true);
  RCLCPP_INFO(get_logger(), "Yaw calibration skipped — using true ENU heading.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────

VisionPipeline::VisionPipeline(const rclcpp::NodeOptions & options)
: Node("vision_pipeline", options)
{
  resetTempFrameBufferDebugState();
  config_           = loadConfig();
  mount_angle_rad_ = config_.camera_mount_angle * M_PI / 180.0;

  // ── De-letterbox constants ────────────────────────────────────────────────
  lb_scale_ = static_cast<float>(config_.model_input_size) /
              static_cast<float>(std::max(config_.width, config_.height));
  const int scaled_w = static_cast<int>(std::round(config_.width  * lb_scale_));
  const int scaled_h = static_cast<int>(std::round(config_.height * lb_scale_));
  lb_pad_left_ = (config_.model_input_size - scaled_w) / 2;
  lb_pad_top_  = (config_.model_input_size - scaled_h) / 2;

  RCLCPP_INFO(get_logger(),
    "Letterbox: scale=%.4f  scaled=%dx%d  pad_left=%d  pad_top=%d",
    lb_scale_, scaled_w, scaled_h, lb_pad_left_, lb_pad_top_);

  // ── Ground projector ──────────────────────────────────────────────────────
  {
    CameraParams cp;
    cp.fx = config_.fx;  cp.fy = config_.fy;
    cp.cx = config_.cx;  cp.cy = config_.cy;
    DistortionCoeffs dc;
    dc.k1 = config_.k1;  dc.k2 = config_.k2;
    dc.p1 = config_.p1;  dc.p2 = config_.p2;
    dc.k3 = config_.k3;
    projector_ = std::make_unique<GroundProjector>(cp, dc);
  }
  tracker_ = std::make_unique<MultiObjectTracker>(config_.tracker);

  // ── Session directory ─────────────────────────────────────────────────────
  session_dir_ = resolveSessionDir(config_.logs_path);
  videos_dir_  = session_dir_ + "/videos";
  data_dir_    = session_dir_ + "/data";
  fs::create_directories(videos_dir_);
  fs::create_directories(data_dir_);

  // Write a metadata file so post-processing knows rotation state
  {
    std::ofstream meta(session_dir_ + "/metadata.txt");
    meta << "reverse_mounted: " << (config_.reverse_mounted ? "true" : "false") << "\n"
         << "width: "  << config_.width  << "\n"
         << "height: " << config_.height << "\n"
         << "fps: "    << config_.fps    << "\n";
  }

  // ── Pre-allocate slot letterbox/NMS buffers — done inside initHailo() ──────

  // ── Hailo ─────────────────────────────────────────────────────────────────
  initHailo();

  // ── ROS interfaces ────────────────────────────────────────────────────────
  const auto sensor_qos   = rclcpp::SensorDataQoS();
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const std::string dp    = "/drone_" + std::to_string(config_.drone_id);

  frame_sub_ = create_subscription<drone_msgs::msg::FrameData>(
    config_.frames_topic, sensor_qos,
    [this](drone_msgs::msg::FrameData::ConstSharedPtr msg) { frameCallback(msg); });

  stream_cmd_sub_ = create_subscription<drone_msgs::msg::Toggle>(
    dp + "/camera/stream/cmd", reliable_qos,
    [this](const drone_msgs::msg::Toggle::SharedPtr msg) { streamCmdCallback(msg); });

  stream_state_pub_ = create_publisher<drone_msgs::msg::Toggle>(
    dp + "/camera/stream/active", reliable_qos);

  stream_out_pub_ = create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
    dp + "/camera/stream/out", sensor_qos);

  record_cmd_sub_ = create_subscription<drone_msgs::msg::Toggle>(
    dp + "/camera/record/cmd", reliable_qos,
    [this](const drone_msgs::msg::Toggle::SharedPtr msg) { recordCmdCallback(msg); });

  record_state_pub_ = create_publisher<drone_msgs::msg::Toggle>(
    dp + "/camera/record/active", reliable_qos);

  publishStreamState();
  publishRecordState();

  // ── Periodic flush timer for recording data CSV ───────────────────────────
  // Runs on the ROS timer thread; the mjpeg loop itself is the only other
  // writer of this buffer, so we coordinate via mjpeg_queue_mtx_.
  record_flush_timer_ = create_wall_timer(
    std::chrono::seconds(3),
    [this]() {
      std::lock_guard<std::mutex> lk(mjpeg_queue_mtx_);
      if (data_csv_file_.is_open()) {
        for (const auto & line : data_csv_buf_) data_csv_file_ << line;
        data_csv_buf_.clear();
        data_csv_file_.flush();
      }
    });

  // ── Periodic flush timer for track_results CSV ────────────────────────────
  track_flush_timer_ = create_wall_timer(
    std::chrono::seconds(3),
    [this]() {
      std::lock_guard<std::mutex> lk(results_mtx_);
      if (track_csv_file_.is_open()) {
        for (const auto & line : track_csv_buf_) track_csv_file_ << line;
        track_csv_buf_.clear();
        track_csv_file_.flush();
      }
    });

  // ── Start threads ─────────────────────────────────────────────────────────

  // Yaw calibration
  yaw_calib_thread_ = std::thread(&VisionPipeline::runYawCalibration, this);

  // MJPEG writer
  mjpeg_running_.store(true);
  mjpeg_thread_ = std::thread(&VisionPipeline::mjpegLoop, this);

  // H264 encoder (for streaming)
  encoder_running_.store(true);
  encoder_thread_ = std::thread(&VisionPipeline::encoderLoop, this);

  // Hailo workers
  workers_running_.store(true);
  for (auto & t : worker_threads_)
    t = std::thread(&VisionPipeline::workerLoop, this);

  RCLCPP_INFO(get_logger(), "vision_pipeline ready.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Destructor
// ─────────────────────────────────────────────────────────────────────────────

VisionPipeline::~VisionPipeline()
{
  // ── Stop worker threads ───────────────────────────────────────────────────
  workers_running_.store(false);
  worker_queue_cv_.notify_all();
  for (auto & t : worker_threads_)
    if (t.joinable()) t.join();

  // Hailo must be destroyed after workers have stopped using it
  hailo_vdevice_.reset();

  // ── Stop encoder thread ───────────────────────────────────────────────────
  encoder_running_.store(false);
  encoder_queue_cv_.notify_all();
  if (encoder_thread_.joinable()) encoder_thread_.join();
  {
    std::lock_guard<std::mutex> lk(encoder_open_mtx_);
    if (h264_encoder_.isOpen()) h264_encoder_.close();
  }

  // ── Stop MJPEG writer thread ──────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lk(mjpeg_queue_mtx_);
    mjpeg_running_.store(false);
  }
  mjpeg_queue_cv_.notify_all();
  if (mjpeg_thread_.joinable()) mjpeg_thread_.join();

  // closeRecordClip() is idempotent; call in case recording was still active
  closeRecordClip();

  // ── Flush and close track_results CSV ────────────────────────────────────
  closeTrackCsv();

  // ── Yaw calibration thread ────────────────────────────────────────────────
  if (yaw_calib_thread_.joinable()) yaw_calib_thread_.join();

  resetTempFrameBufferDebugState();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Slot management
// ─────────────────────────────────────────────────────────────────────────────

int VisionPipeline::acquireSlot()
{
  for (int i = 0; i < kNumSlots; ++i) {
    bool expected = false;
    if (buffer_[i].in_use.compare_exchange_strong(
          expected, true,
          std::memory_order_acquire,
          std::memory_order_relaxed))
    {
      buffer_[i].consumers_remaining.store(3, std::memory_order_release);
      buffer_[i].seq = next_seq_.fetch_add(1, std::memory_order_relaxed);
      return i;
    }
  }
  return -1;
}

void VisionPipeline::releaseSlot(int idx)
{
  // Decrement; whoever reaches 0 frees the slot.
  const int remaining =
    buffer_[idx].consumers_remaining.fetch_sub(1, std::memory_order_acq_rel) - 1;
  if (remaining == 0) {
    buffer_[idx].msg.reset();   // release the intra-process message reference
    buffer_[idx].in_use.store(false, std::memory_order_release);
  }
}

int VisionPipeline::countSlotsInUse() const
{
  int count = 0;
  for (const auto & slot : buffer_) {
    if (slot.in_use.load(std::memory_order_acquire)) {
      ++count;
    }
  }
  return count;
}

void VisionPipeline::logTempFrameBufferDebug(const char * reason, bool force)
{
  int mjpeg_idx = kDebugNoSlot;
  int encoder_idx = kDebugNoSlot;
  std::array<int, kDebugNumWorkers> worker_indices{};
  worker_indices.fill(kDebugNoSlot);
  std::size_t mjpeg_q_size = 0;
  std::size_t encoder_q_size = 0;
  std::size_t worker_q_size = 0;

  {
    std::lock_guard<std::mutex> lk(g_temp_frame_buffer_debug.mtx);
    const auto now = std::chrono::steady_clock::now();
    if (!force && (now - g_temp_frame_buffer_debug.last_log_tp) < kDebugLogInterval) {
      return;
    }

    g_temp_frame_buffer_debug.last_log_tp = now;
    mjpeg_idx = g_temp_frame_buffer_debug.mjpeg_idx;
    encoder_idx = g_temp_frame_buffer_debug.encoder_idx;
    worker_indices = g_temp_frame_buffer_debug.worker_indices;
  }

  {
    std::lock_guard<std::mutex> lk(mjpeg_queue_mtx_);
    mjpeg_q_size = mjpeg_queue_.size();
  }

  {
    std::lock_guard<std::mutex> lk(encoder_queue_mtx_);
    encoder_q_size = encoder_queue_.size();
  }

  {
    std::lock_guard<std::mutex> lk(worker_queue_mtx_);
    worker_q_size = worker_queue_.size();
  }

  const int slots_in_use = countSlotsInUse();

  if (force) {
    RCLCPP_WARN(
      get_logger(),
      "[temp-debug] %s: mjpeg=%d encoder=%d worker0=%d worker1=%d worker2=%d worker3=%d "
      "mjpeg_q=%zu encoder_q=%zu worker_q=%zu slots_in_use=%d",
      reason, mjpeg_idx, encoder_idx,
      worker_indices[0], worker_indices[1], worker_indices[2], worker_indices[3],
      mjpeg_q_size, encoder_q_size, worker_q_size, slots_in_use);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "[temp-debug] %s: mjpeg=%d encoder=%d worker0=%d worker1=%d worker2=%d worker3=%d "
    "mjpeg_q=%zu encoder_q=%zu worker_q=%zu slots_in_use=%d",
    reason, mjpeg_idx, encoder_idx,
    worker_indices[0], worker_indices[1], worker_indices[2], worker_indices[3],
    mjpeg_q_size, encoder_q_size, worker_q_size, slots_in_use);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Frame callback
//  Runs in the ROS subscription thread.  Must return as fast as possible.
//  Does:
//    1. Feed yaw calibration if needed
//    2. Acquire a slot (drop frame if full)
//    3. Store ConstSharedPtr (zero-copy) + copy scalar metadata into slot
//    4. Push slot index to all three consumer queues
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::frameCallback(drone_msgs::msg::FrameData::ConstSharedPtr msg) {
  const int idx = acquireSlot();
  if (idx == -1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Frame buffer full — dropping frame");
    logTempFrameBufferDebug("frame buffer full", true);
    return;
  }

  FrameSlot & slot = buffer_[idx];

  // ── Zero-copy: store the intra-process message pointer ────────────────────
  // All three consumers read slot.msg->image.data directly.
  // No JPEG copy, no decode, no rotate here — each consumer does its own work.
  slot.msg = msg;

  // ── Metadata ──────────────────────────────────────────────────────────────
  slot.stamp_sec     = msg->image.header.stamp.sec;
  slot.stamp_nanosec = msg->image.header.stamp.nanosec;

  slot.pos_x      = msg->pos_x;
  slot.pos_y      = msg->pos_y;
  slot.pos_z      = msg->pos_z;
  slot.quat_x     = msg->quat_x;
  slot.quat_y     = msg->quat_y;
  slot.quat_z     = msg->quat_z;
  slot.quat_w     = msg->quat_w;
  slot.vel_x      = msg->vel_x;
  slot.vel_y      = msg->vel_y;
  slot.vel_z      = msg->vel_z;
  slot.ang_vel_x  = msg->ang_vel_x;
  slot.ang_vel_y  = msg->ang_vel_y;
  slot.ang_vel_z  = msg->ang_vel_z;
  slot.odom_valid = msg->odom_valid;

  slot.lat       = msg->lat;
  slot.lon       = msg->lon;
  slot.gps_valid = msg->gps_valid;

  slot.agl_m     = msg->agl_m;
  slot.vz_mps    = msg->vz_mps;
  slot.mcu_valid = msg->mcu_valid;

  slot.detections.clear();

  // ── Dispatch to all three consumer queues ─────────────────────────────────
  {
    std::lock_guard<std::mutex> lk(mjpeg_queue_mtx_);
    mjpeg_queue_.push({idx});
  }
  mjpeg_queue_cv_.notify_one();

  {
    std::lock_guard<std::mutex> lk(encoder_queue_mtx_);
    encoder_queue_.push({idx});
  }
  encoder_queue_cv_.notify_one();

  {
    std::lock_guard<std::mutex> lk(worker_queue_mtx_);
    worker_queue_.push({idx});
  }
  worker_queue_cv_.notify_one();

  logTempFrameBufferDebug("periodic");
}

// ─────────────────────────────────────────────────────────────────────────────
//  MJPEG writer thread
//  Writes raw JPEG bytes directly to AVI — no pixel work.
//  Also writes one combined frame-data row to session_data.csv, buffered.
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::mjpegLoop()
{
  while (true) {
    MjpegTask task;
    {
      std::unique_lock<std::mutex> lk(mjpeg_queue_mtx_);
      mjpeg_queue_cv_.wait(lk, [this] {
        return !mjpeg_queue_.empty() || !mjpeg_running_.load();
      });
      if (mjpeg_queue_.empty()) break;
      task = mjpeg_queue_.front();
      mjpeg_queue_.pop();
    }

    const int idx = task.idx;
    const FrameSlot & slot = buffer_[idx];
    setTempMjpegDebugIndex(idx);

    if (recording_.load() && mjpeg_writer_.isOpen()) {
      // ── Write JPEG frame to AVI ───────────────────────────────────────────
      mjpeg_writer_.writeFrame(slot.msg->image.data);

      // ── Combined data CSV row (all FrameData fields) ──────────────────────
      {
        std::ostringstream oss;
        oss << slot.stamp_sec      << ','
            << slot.stamp_nanosec  << ','
            << slot.pos_x   << ',' << slot.pos_y   << ',' << slot.pos_z   << ','
            << slot.quat_x  << ',' << slot.quat_y  << ','
            << slot.quat_z  << ',' << slot.quat_w  << ','
            << slot.vel_x   << ',' << slot.vel_y   << ',' << slot.vel_z   << ','
            << slot.ang_vel_x << ',' << slot.ang_vel_y << ',' << slot.ang_vel_z << ','
            << slot.odom_valid << ','
            << slot.lat     << ','
            << slot.lon     << ','
            << slot.gps_valid << ','
            << slot.agl_m   << ','
            << slot.vz_mps  << ','
            << slot.mcu_valid << '\n';

        std::lock_guard<std::mutex> lk(mjpeg_queue_mtx_);
        data_csv_buf_.push_back(oss.str());

        if (data_csv_buf_.size() >= kCsvFlushSize && data_csv_file_.is_open()) {
          for (const auto & line : data_csv_buf_) data_csv_file_ << line;
          data_csv_buf_.clear();
          data_csv_file_.flush();
        }
      }
    }

    clearTempMjpegDebugIndex(idx);
    releaseSlot(idx);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Recording clip open / close
//  Called from recordCmdCallback (ROS callback thread).
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::openRecordClip()
{
  ++clip_index_;

  std::ostringstream idx;
  idx << std::setw(3) << std::setfill('0') << clip_index_;
  const std::string prefix = idx.str();

  const std::string video_path     = videos_dir_ + "/" + prefix + ".avi";
  const std::string data_csv_path  = data_dir_   + "/" + prefix + "_data.csv";
  const std::string track_csv_path = data_dir_   + "/" + prefix + "_track_results.csv";

  // Open AVI — MjpegWriter throws on failure
  mjpeg_writer_.open(video_path, config_.width, config_.height, config_.fps);

  // Open data CSV (under mjpeg_queue_mtx_ so the flush timer sees a consistent state)
  {
    std::lock_guard<std::mutex> lk(mjpeg_queue_mtx_);

    data_csv_file_.open(data_csv_path, std::ios::out | std::ios::trunc);
    if (!data_csv_file_.is_open())
      throw std::runtime_error("Cannot open data CSV: " + data_csv_path);

    data_csv_file_
      << "stamp_sec,stamp_nanosec,"
      << "pos_x,pos_y,pos_z,"
      << "quat_x,quat_y,quat_z,quat_w,"
      << "vel_x,vel_y,vel_z,"
      << "ang_vel_x,ang_vel_y,ang_vel_z,"
      << "odom_valid,"
      << "lat_deg_e7,lon_deg_e7,"
      << "gps_valid,"
      << "agl_m,vz_mps,"
      << "mcu_valid\n";

    data_csv_buf_.clear();
  }

  // Open track_results CSV
  openTrackCsv(track_csv_path);

  RCLCPP_INFO(get_logger(),
    "Recording started → %s | %s | %s",
    video_path.c_str(), data_csv_path.c_str(), track_csv_path.c_str());
}

void VisionPipeline::closeRecordClip()
{
  if (!mjpeg_writer_.isOpen()) return;

  // Flush and close MJPEG + data CSV
  mjpeg_writer_.close();

  {
    std::lock_guard<std::mutex> lk(mjpeg_queue_mtx_);
    if (data_csv_file_.is_open()) {
      for (const auto & line : data_csv_buf_) data_csv_file_ << line;
      data_csv_buf_.clear();
      data_csv_file_.flush();
      data_csv_file_.close();
    }
  }

  // Flush and close track_results CSV
  closeTrackCsv();

  std::ostringstream idx;
  idx << std::setw(3) << std::setfill('0') << clip_index_;
  RCLCPP_INFO(get_logger(), "Recording stopped → clip %s finalized.", idx.str().c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  H264 encoder thread — streaming only
//  Receives slot.msg JPEG bytes, decodes directly to YUV420P via
//  libjpeg-turbo (no RGB intermediate), encodes with libx264.
//  Never waits for Hailo; fully independent of the worker pipeline.
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::encoderLoop()
{
  while (true) {
    EncoderTask task;
    {
      std::unique_lock<std::mutex> lk(encoder_queue_mtx_);
      encoder_queue_cv_.wait(lk, [this] {
        return !encoder_queue_.empty() || !encoder_running_.load();
      });
      if (!encoder_running_.load() && encoder_queue_.empty()) break;
      task = encoder_queue_.front();
      encoder_queue_.pop();
    }

    const int idx = task.idx;
    const FrameSlot & slot = buffer_[idx];
    setTempEncoderDebugIndex(idx);

    if (streaming_.load()) {
      H264Encoder::EncodeResult result;
      {
        std::lock_guard<std::mutex> lk(encoder_open_mtx_);
        if (h264_encoder_.isOpen()) {
          result = h264_encoder_.encode(
            slot.msg->image.data.data(),
            slot.msg->image.data.size());
        }
      }

      if (!result.data.empty()) {
        auto out = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
        out->header.stamp.sec     = slot.stamp_sec;
        out->header.stamp.nanosec = slot.stamp_nanosec;
        out->width                = config_.width;
        out->height               = config_.height;
        out->encoding             = "h264";
        out->pts                  = result.pts;
        out->flags                = result.is_key_frame ? 1 : 0;
        out->data                 = std::move(result.data);
        stream_out_pub_->publish(std::move(out));
      }
    }

    clearTempEncoderDebugIndex(idx);
    releaseSlot(idx);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Hailo worker loop
//  Letterboxes the RGB image, submits to Hailo async inference.
//  The callback parses detections, deposits a PendingResult for in-order CSV
//  writing, then calls releaseSlot().
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::workerLoop()
{
  // Each worker thread owns its own libjpeg-turbo handle — tjhandle is NOT
  // thread-safe and must not be shared across threads.
  const int worker_id = registerTempWorkerDebugThread();
  tjhandle tj = tjInitDecompress();
  if (!tj) {
    RCLCPP_FATAL(get_logger(), "workerLoop: tjInitDecompress failed");
    return;
  }

  // Per-thread RGB buffer — allocated once, reused every frame.
  std::vector<uint8_t> rgb(config_.width * config_.height * 3);

  while (true) {
    WorkerTask task;
    {
      std::unique_lock<std::mutex> lk(worker_queue_mtx_);
      worker_queue_cv_.wait(lk, [this] {
        return !worker_queue_.empty() || !workers_running_.load();
      });
      if (!workers_running_.load() && worker_queue_.empty()) break;
      task = worker_queue_.front();
      worker_queue_.pop();
    }

    const int idx = task.idx;
    FrameSlot & slot = buffer_[idx];
    setTempWorkerDebugIndex(worker_id, idx);

    // ── JPEG decompress → RGB ─────────────────────────────────────────────
    const auto & jpeg = slot.msg->image.data;
    const int ret = tjDecompress2(
      tj,
      jpeg.data(),
      static_cast<unsigned long>(jpeg.size()),
      rgb.data(),
      config_.width, 0, config_.height,
      TJPF_RGB, TJFLAG_FASTDCT);

    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "workerLoop JPEG decompress failed: %s", tjGetErrorStr2(tj));
      // Deposit empty result so ordering is preserved, then release slot.
      PendingResult pending;
      pending.seq           = slot.seq;
      pending.stamp_sec     = slot.stamp_sec;
      pending.stamp_nanosec = slot.stamp_nanosec;
      pending.pos_x         = slot.pos_x;
      pending.pos_y         = slot.pos_y;
      pending.pos_z         = slot.pos_z;
      pending.quat_x        = slot.quat_x;
      pending.quat_y        = slot.quat_y;
      pending.quat_z        = slot.quat_z;
      pending.quat_w        = slot.quat_w;
      pending.ang_vel_x     = slot.ang_vel_x;
      pending.ang_vel_y     = slot.ang_vel_y;
      pending.ang_vel_z     = slot.ang_vel_z;
      pending.odom_valid    = slot.odom_valid;
      pending.agl_m         = slot.agl_m;
      pending.mcu_valid     = slot.mcu_valid;
      depositResult(std::move(pending));
      clearTempWorkerDebugIndex(worker_id, idx);
      releaseSlot(idx);
      continue;
    }

    // ── Optional 180° rotation ────────────────────────────────────────────
    if (config_.reverse_mounted) {
      cv::Mat rgb_mat(config_.height, config_.width, CV_8UC3, rgb.data());
      cv::rotate(rgb_mat, rgb_mat, cv::ROTATE_180);
    }

    // ── Letterbox resize ──────────────────────────────────────────────────
    const int scaled_w = static_cast<int>(std::round(config_.width  * lb_scale_));
    const int scaled_h = static_cast<int>(std::round(config_.height * lb_scale_));

    // Read directly from the per-thread rgb buffer — no copy into slot needed.
    const cv::Mat src(config_.height, config_.width, CV_8UC3, rgb.data());
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(scaled_w, scaled_h), 0, 0, cv::INTER_LINEAR);

    cv::Mat letterbox(config_.model_input_size, config_.model_input_size,
                      CV_8UC3, slot.letterbox_buf.data());
    letterbox.setTo(cv::Scalar(0, 0, 0));
    resized.copyTo(letterbox(cv::Rect(lb_pad_left_, lb_pad_top_, scaled_w, scaled_h)));

    // ── Async Hailo inference ─────────────────────────────────────────────
    auto callback = [this, idx](const hailort::AsyncInferCompletionInfo & info)
    {
      FrameSlot & cb_slot = buffer_[idx];

      PendingResult pending;
      pending.seq           = cb_slot.seq;
      pending.stamp_sec     = cb_slot.stamp_sec;
      pending.stamp_nanosec = cb_slot.stamp_nanosec;
      pending.pos_x         = cb_slot.pos_x;
      pending.pos_y         = cb_slot.pos_y;
      pending.pos_z         = cb_slot.pos_z;
      pending.quat_x        = cb_slot.quat_x;
      pending.quat_y        = cb_slot.quat_y;
      pending.quat_z        = cb_slot.quat_z;
      pending.quat_w        = cb_slot.quat_w;
      pending.ang_vel_x     = cb_slot.ang_vel_x;
      pending.ang_vel_y     = cb_slot.ang_vel_y;
      pending.ang_vel_z     = cb_slot.ang_vel_z;
      pending.odom_valid    = cb_slot.odom_valid;
      pending.agl_m         = cb_slot.agl_m;
      pending.mcu_valid     = cb_slot.mcu_valid;

      if (info.status != HAILO_SUCCESS) {
        RCLCPP_WARN(get_logger(), "Hailo async infer failed (slot %d): %d",
          idx, static_cast<int>(info.status));
        // Deposit empty result so ordering is preserved
        depositResult(std::move(pending));
        releaseSlot(idx);
        return;
      }

      // ── Parse NMS output ────────────────────────────────────────────────
      static constexpr int kMaxBboxPerClass = 100;
      static constexpr int kClassStride     = 1 + kMaxBboxPerClass * 5;

      const float * nms     = cb_slot.nms_output_buf.data();
      const float * cls_ptr = nms + config_.person_class_idx * kClassStride;
      const int     num_dets = static_cast<int>(cls_ptr[0]);

      if (num_dets > 0) {
        pending.detections.reserve(num_dets);

        for (int d = 0; d < num_dets; ++d) {
          const float * det      = cls_ptr + 1 + d * 5;
          const float y_min_norm = det[0];
          const float x_min_norm = det[1];
          const float y_max_norm = det[2];
          const float x_max_norm = det[3];
          const float score      = det[4];

          const float x_min_lb = x_min_norm * config_.model_input_size;
          const float y_min_lb = y_min_norm * config_.model_input_size;
          const float x_max_lb = x_max_norm * config_.model_input_size;
          const float y_max_lb = y_max_norm * config_.model_input_size;

          const float x_min = (x_min_lb - lb_pad_left_) / lb_scale_;
          const float y_min = (y_min_lb - lb_pad_top_)  / lb_scale_;
          const float x_max = (x_max_lb - lb_pad_left_) / lb_scale_;
          const float y_max = (y_max_lb - lb_pad_top_)  / lb_scale_;

          Detection det_obj;
          det_obj.x_min   = std::max(0.0f, x_min);
          det_obj.y_min   = std::max(0.0f, y_min);
          det_obj.x_max   = std::min(static_cast<float>(config_.width  - 1), x_max);
          det_obj.y_max   = std::min(static_cast<float>(config_.height - 1), y_max);
          det_obj.score   = score;

          pending.detections.push_back(det_obj);
        }
      }

      depositResult(std::move(pending));
      releaseSlot(idx);
    };  // end callback lambda

    auto job_exp = hailo_infer_model_.run_async(hailo_bindings_[idx], callback);
    if (!job_exp) {
      RCLCPP_ERROR(get_logger(), "run_async failed (slot %d): %d",
        idx, static_cast<int>(job_exp.status()));
      // Deposit empty result so ordering is preserved
      PendingResult pending;
      pending.seq           = slot.seq;
      pending.stamp_sec     = slot.stamp_sec;
      pending.stamp_nanosec = slot.stamp_nanosec;
      pending.pos_x         = slot.pos_x;
      pending.pos_y         = slot.pos_y;
      pending.pos_z         = slot.pos_z;
      pending.quat_x        = slot.quat_x;
      pending.quat_y        = slot.quat_y;
      pending.quat_z        = slot.quat_z;
      pending.quat_w        = slot.quat_w;
      pending.ang_vel_x     = slot.ang_vel_x;
      pending.ang_vel_y     = slot.ang_vel_y;
      pending.ang_vel_z     = slot.ang_vel_z;
      pending.odom_valid    = slot.odom_valid;
      pending.agl_m         = slot.agl_m;
      pending.mcu_valid     = slot.mcu_valid;
      depositResult(std::move(pending));
      clearTempWorkerDebugIndex(worker_id, idx);
      releaseSlot(idx);
    }

    clearTempWorkerDebugIndex(worker_id, idx);
    (void)job_exp;  // job lifetime managed by HailoRT
  }

  tjDestroy(tj);
}

// ─────────────────────────────────────────────────────────────────────────────
//  track_results CSV — in-order drain
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::resetTrackingStateLocked()
{
  if (tracker_) {
    tracker_->reset();
  }
  has_prev_track_timestamp_ = false;
  prev_track_timestamp_s_ = 0.0;
  has_smoothed_projection_height_ = false;
  smoothed_projection_height_m_ = 0.0;

  while (!results_pq_.empty()) {
    results_pq_.pop();
  }

  track_csv_buf_.clear();
  next_result_seq_ = next_seq_.load(std::memory_order_relaxed);
}

double VisionPipeline::computeAngularVelocityDegPerSecLocked(
  const PendingResult & result) const
{
  const double wx = result.ang_vel_x;
  const double wy = result.ang_vel_y;
  const double wz = result.ang_vel_z;
  // nav_msgs/Odometry twist uses ROS SI units, so MAVROS body rates arrive in rad/s.
  const double omega_rad_s = std::sqrt(wx * wx + wy * wy + wz * wz);
  return omega_rad_s * 180.0 / M_PI;
}

double VisionPipeline::smoothProjectionHeightLocked(double raw_height_m)
{
  const double alpha = std::max(0.0, std::min(1.0, config_.projection_altitude_lpf_alpha));
  if (!has_smoothed_projection_height_ || alpha >= 0.999) {
    smoothed_projection_height_m_ = raw_height_m;
    has_smoothed_projection_height_ = true;
    return smoothed_projection_height_m_;
  }

  smoothed_projection_height_m_ =
    alpha * raw_height_m + (1.0 - alpha) * smoothed_projection_height_m_;
  return smoothed_projection_height_m_;
}

std::vector<DetectionMeasurement> VisionPipeline::buildTrackMeasurementsLocked(
  const PendingResult & result,
  double noise_scale)
{
  std::vector<DetectionMeasurement> measurements;
  if (!result.odom_valid || result.detections.empty()) {
    return measurements;
  }

  tf2::Quaternion tf_q(
    result.quat_x, result.quat_y,
    result.quat_z, result.quat_w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  yaw += yaw_offset_;

  const double projection_pitch = config_.use_gimbal ? 0.0 : pitch;
  const double projection_roll = config_.use_gimbal ? 0.0 : roll;
  const double raw_projection_height =
    (config_.use_mcu_height_estimate && result.mcu_valid)
    ? static_cast<double>(result.agl_m)
    : result.pos_z;

  if (!std::isfinite(raw_projection_height) || raw_projection_height <= 1e-3) {
    has_smoothed_projection_height_ = false;
    return measurements;
  }

  const double projection_height = smoothProjectionHeightLocked(raw_projection_height);
  projector_->setPose(
    result.pos_x, result.pos_y, projection_height,
    yaw, projection_pitch, projection_roll, mount_angle_rad_);

  measurements.reserve(result.detections.size());
  for (const auto & det : result.detections) {
    const int foot_u = clampPixelCoord(
      static_cast<int>(std::lround(0.5 * (det.x_min + det.x_max))),
      config_.width);
    const int foot_v = clampPixelCoord(
      static_cast<int>(std::lround(det.y_max)),
      config_.height);

    const auto hit = projector_->projectOne(Pixel{foot_u, foot_v});
    if (!hit.valid) {
      continue;
    }

    DetectionMeasurement measurement;
    measurement.world_xy = {hit.world_x, hit.world_y};
    measurement.meas_cov = estimateMeasurementCovariance(
      *projector_,
      foot_u,
      foot_v,
      config_.width,
      config_.height,
      noise_scale,
      config_.tracker.pixel_std_x,
      config_.tracker.pixel_std_y,
      config_.tracker.base_meas_noise_m);
    measurement.image_box = {det.x_min, det.y_min, det.x_max, det.y_max};
    measurement.score = det.score;
    measurements.push_back(measurement);
  }

  return measurements;
}

void VisionPipeline::writeTrackRowsLocked(
  const PendingResult & result,
  const std::vector<TrackEstimate> & confirmed_tracks,
  double noise_scale,
  double omega_deg_s)
{
  if (confirmed_tracks.empty()) {
    std::ostringstream oss;
    oss << result.seq << ','
        << result.stamp_sec << ',' << result.stamp_nanosec;
    for (int i = 0; i < 14; ++i) {
      oss << ',';
    }
    oss << ',';
    oss << noise_scale << ',' << omega_deg_s << '\n';
    track_csv_buf_.push_back(oss.str());
    return;
  }

  for (const auto & track : confirmed_tracks) {
    const double raw_world_x = track.matched_in_frame
      ? track.raw_world_xy[0]
      : std::numeric_limits<double>::quiet_NaN();
    const double raw_world_y = track.matched_in_frame
      ? track.raw_world_xy[1]
      : std::numeric_limits<double>::quiet_NaN();
    std::ostringstream oss;
    oss << result.seq << ','
        << result.stamp_sec << ',' << result.stamp_nanosec << ','
        << track.track_id << ','
        << track.image_box[0] << ',' << track.image_box[1] << ','
        << track.image_box[2] << ',' << track.image_box[3] << ','
        << track.score << ','
        << raw_world_x << ',' << raw_world_y << ','
        << track.state[0] << ',' << track.state[1] << ','
        << track.state[2] << ',' << track.state[3] << ','
        << track.hits << ',' << track.missed << ','
        << noise_scale << ',' << omega_deg_s << '\n';
    track_csv_buf_.push_back(oss.str());
  }
}

void VisionPipeline::openTrackCsv(const std::string & path)
{
  std::lock_guard<std::mutex> lk(results_mtx_);
  resetTrackingStateLocked();
  track_csv_file_.open(path, std::ios::out | std::ios::trunc);
  if (!track_csv_file_.is_open())
    throw std::runtime_error("Cannot open track_results CSV: " + path);

  track_csv_file_
    << "seq,"
    << "stamp_sec,stamp_nanosec,"
    << "track_id,"
    << "x_min,y_min,x_max,y_max,"
    << "score,"
    << "raw_world_x,raw_world_y,"
    << "world_x,world_y,"
    << "world_vx,world_vy,"
    << "hits,missed,"
    << "noise_scale,omega_deg_s\n";
}

void VisionPipeline::closeTrackCsv()
{
  std::lock_guard<std::mutex> lk(results_mtx_);
  if (!track_csv_file_.is_open()) return;

  while (!results_pq_.empty()) {
    const PendingResult & result = results_pq_.top();
    const double timestamp_s =
      static_cast<double>(result.stamp_sec) +
      static_cast<double>(result.stamp_nanosec) * 1e-9;
    const double dt = has_prev_track_timestamp_
      ? std::max(1e-3, timestamp_s - prev_track_timestamp_s_)
      : (1.0 / static_cast<double>(config_.fps));
    has_prev_track_timestamp_ = true;
    prev_track_timestamp_s_ = timestamp_s;

    const double omega_deg_s = computeAngularVelocityDegPerSecLocked(result);
    const double low = config_.tracker.angular_vel_low_deg_s;
    const double high = config_.tracker.angular_vel_high_deg_s;
    double noise_scale = 1.0;
    if (omega_deg_s >= high) {
      noise_scale = config_.tracker.angular_vel_max_scale;
    } else if (omega_deg_s > low) {
      const double t = (omega_deg_s - low) / std::max(1e-6, high - low);
      noise_scale = 1.0 + t * (config_.tracker.angular_vel_max_scale - 1.0);
    }

    const auto measurements = buildTrackMeasurementsLocked(result, noise_scale);
    const auto confirmed_tracks = tracker_->step(measurements, dt, noise_scale);
    writeTrackRowsLocked(result, confirmed_tracks, noise_scale, omega_deg_s);
    results_pq_.pop();
  }

  for (const auto & line : track_csv_buf_) track_csv_file_ << line;
  track_csv_buf_.clear();
  track_csv_file_.flush();
  track_csv_file_.close();
}

void VisionPipeline::depositResult(PendingResult && r)
{
  std::lock_guard<std::mutex> lk(results_mtx_);
  results_pq_.push(std::move(r));
  drainResultsLocked();
}

void VisionPipeline::drainResultsLocked()
{
  if (!track_csv_file_.is_open()) {
    while (!results_pq_.empty())
      results_pq_.pop();
    next_result_seq_ = next_seq_.load(std::memory_order_relaxed);
    return;
  }

  while (!results_pq_.empty() &&
         results_pq_.top().seq == next_result_seq_)
  {
    const PendingResult & result = results_pq_.top();
    const double timestamp_s =
      static_cast<double>(result.stamp_sec) +
      static_cast<double>(result.stamp_nanosec) * 1e-9;
    const double dt = has_prev_track_timestamp_
      ? std::max(1e-3, timestamp_s - prev_track_timestamp_s_)
      : (1.0 / static_cast<double>(config_.fps));
    has_prev_track_timestamp_ = true;
    prev_track_timestamp_s_ = timestamp_s;

    const double omega_deg_s = computeAngularVelocityDegPerSecLocked(result);
    const double low = config_.tracker.angular_vel_low_deg_s;
    const double high = config_.tracker.angular_vel_high_deg_s;
    double noise_scale = 1.0;
    if (omega_deg_s >= high) {
      noise_scale = config_.tracker.angular_vel_max_scale;
    } else if (omega_deg_s > low) {
      const double t = (omega_deg_s - low) / std::max(1e-6, high - low);
      noise_scale = 1.0 + t * (config_.tracker.angular_vel_max_scale - 1.0);
    }

    const auto measurements = buildTrackMeasurementsLocked(result, noise_scale);
    const auto confirmed_tracks = tracker_->step(measurements, dt, noise_scale);
    writeTrackRowsLocked(result, confirmed_tracks, noise_scale, omega_deg_s);

    results_pq_.pop();
    ++next_result_seq_;

    if (track_csv_buf_.size() >= kCsvFlushSize) {
      for (const auto & line : track_csv_buf_) track_csv_file_ << line;
      track_csv_buf_.clear();
      track_csv_file_.flush();
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Stream toggle
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
{
  const bool was_streaming = streaming_.load();

  if (!was_streaming) {
    try {
      std::lock_guard<std::mutex> lk(encoder_open_mtx_);
      h264_encoder_.open(config_.width, config_.height, config_.fps, config_.fps / 2);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to open H264 encoder: %s", e.what());
      return;
    }
    streaming_.store(true);
    RCLCPP_INFO(get_logger(), "Stream → ON  (%dx%d @ %d fps)",
      config_.width, config_.height, config_.fps);
  } else {
    streaming_.store(false);
    std::lock_guard<std::mutex> lk(encoder_open_mtx_);
    h264_encoder_.close();
    RCLCPP_INFO(get_logger(), "Stream → OFF");
  }
  publishStreamState();
}

void VisionPipeline::publishStreamState()
{
  drone_msgs::msg::Toggle msg;
  msg.state = streaming_.load();
  stream_state_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Record toggle
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::recordCmdCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
{
  const bool was_recording = recording_.load();

  if (!was_recording) {
    try {
      openRecordClip();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to open recording clip: %s", e.what());
      return;
    }
    recording_.store(true);
    RCLCPP_INFO(get_logger(), "Record → ON  (clip %03d)", clip_index_);
  } else {
    recording_.store(false);
    closeRecordClip();
    RCLCPP_INFO(get_logger(), "Record → OFF");
  }
  logTempFrameBufferDebug("record toggled", true);
  publishRecordState();
}

void VisionPipeline::publishRecordState()
{
  drone_msgs::msg::Toggle msg;
  msg.state = recording_.load();
  record_state_pub_->publish(msg);
}

}  // namespace drone_pipeline

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_pipeline::VisionPipeline)
