#include "drone_pipeline/vision_pipeline.hpp"

#include <stdexcept>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

// OpenCV for letterbox resize
#include <opencv2/imgproc.hpp>

extern "C" {
#include <libavutil/opt.h>
#include <libavutil/avutil.h>
}

namespace fs = std::filesystem;

namespace drone_pipeline
{

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
  cfg.gimbal_pitch_angle = cam["gimbal_pitch_angle"].as<double>();

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

  RCLCPP_INFO(get_logger(),
    "Config → drone_id=%u  frames=%s  res=%dx%d@%dfps  hef=%s  score_thresh=%.2f  "
    "model_input=%d  person_class=%d",
    cfg.drone_id, cfg.frames_topic.c_str(),
    cfg.width, cfg.height, cfg.fps,
    cfg.hef_path.c_str(), cfg.score_threshold, cfg.model_input_size, cfg.person_class_idx);

  RCLCPP_INFO(get_logger(), "Gimbal pitch angle=%.2f deg", cfg.gimbal_pitch_angle);

  RCLCPP_INFO(get_logger(),
    "Camera intrinsics → fx=%.4f fy=%.4f cx=%.4f cy=%.4f",
    cfg.fx, cfg.fy, cfg.cx, cfg.cy);

  RCLCPP_INFO(get_logger(),
    "Distortion → k1=%.6f k2=%.6f p1=%.6f p2=%.6f k3=%.6f",
    cfg.k1, cfg.k2, cfg.p1, cfg.p2, cfg.k3);

  return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Session directory (mirrors RecordVideo::resolveSessionDir logic)
//  Joins the existing highest-numbered session directory created by
//  FlightLogger so all artefacts for one flight share one folder.
// ─────────────────────────────────────────────────────────────────────────────

std::string VisionPipeline::resolveSessionDir(const std::string & logs_path)
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
//  Hailo initialisation
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::initHailo()
{
  auto vdevice_exp = hailort::VDevice::create();
  if (!vdevice_exp) {
    throw std::runtime_error(
      "VDevice::create failed: " +
      std::to_string(static_cast<int>(vdevice_exp.status())));
  }
  hailo_vdevice_ = vdevice_exp.release();

  auto infer_model_exp = hailo_vdevice_->create_infer_model(config_.hef_path);
  if (!infer_model_exp) {
    throw std::runtime_error(
      "create_infer_model failed: " +
      std::to_string(static_cast<int>(infer_model_exp.status())));
  }
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
  if (!configured_exp) {
    throw std::runtime_error(
      "InferModel::configure failed: " +
      std::to_string(static_cast<int>(configured_exp.status())));
  }
  hailo_infer_model_ = configured_exp.release();

  const size_t nms_output_floats = hailo_output_frame_size_ / sizeof(float);

  const auto & input_names  = infer_model->get_input_names();
  const auto & output_names = infer_model->get_output_names();

  if (input_names.empty())  throw std::runtime_error("Hailo model has no inputs");
  if (output_names.empty()) throw std::runtime_error("Hailo model has no outputs");

  const auto input_name  = input_names.front();
  const auto output_name = output_names.front();
  const size_t input_frame_size = infer_model->input(input_name)->get_frame_size();

  for (int i = 0; i < kNumSlots; ++i) {
    buffer_[i].letterbox_buf.resize(input_frame_size, 0);
    buffer_[i].nms_output_buf.resize(nms_output_floats, 0.0f);

    auto bindings_exp = hailo_infer_model_.create_bindings();
    if (!bindings_exp) {
      throw std::runtime_error(
        "create_bindings failed for slot " + std::to_string(i) + ": " +
        std::to_string(static_cast<int>(bindings_exp.status())));
    }

    auto bindings = bindings_exp.release();

    auto in_status = bindings.input(input_name)->set_buffer(
      hailort::MemoryView(buffer_[i].letterbox_buf.data(), buffer_[i].letterbox_buf.size()));
    if (in_status != HAILO_SUCCESS) {
      throw std::runtime_error(
        "Failed to set input buffer for slot " + std::to_string(i) + ": " +
        std::to_string(static_cast<int>(in_status)));
    }

    auto out_status = bindings.output(output_name)->set_buffer(
      hailort::MemoryView(
        reinterpret_cast<uint8_t *>(buffer_[i].nms_output_buf.data()),
        buffer_[i].nms_output_buf.size() * sizeof(float)));
    if (out_status != HAILO_SUCCESS) {
      throw std::runtime_error(
        "Failed to set output buffer for slot " + std::to_string(i) + ": " +
        std::to_string(static_cast<int>(out_status)));
    }

    hailo_bindings_[i] = std::move(bindings);
  }

  RCLCPP_INFO(get_logger(), "Hailo initialized successfully");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Yaw calibration — runs in its own thread, blocks up to kYawCalibTimeout s
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::runYawCalibration()
{
  RCLCPP_INFO(get_logger(),
    "Yaw calibration started — collecting %d samples over %.0f s max",
    kYawCalibSamples, kYawCalibTimeout);

  std::vector<double> yaw_samples;
  yaw_samples.reserve(kYawCalibSamples);

  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::duration<double>(kYawCalibTimeout);

  while (static_cast<int>(yaw_samples.size()) < kYawCalibSamples) {
    if (std::chrono::steady_clock::now() >= deadline) break;

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    LatestQuat q;
    {
      std::lock_guard<std::mutex> lk(latest_quat_mtx_);
      q = latest_quat_;
    }

    if (!q.valid) continue;

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll_unused, pitch_unused, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll_unused, pitch_unused, yaw);

    yaw_samples.push_back(yaw);

    {
      std::lock_guard<std::mutex> lk(latest_quat_mtx_);
      latest_quat_.valid = false;
    }
  }

  if (yaw_samples.empty()) {
    RCLCPP_WARN(get_logger(),
      "Yaw calibration: no odometry received within %.0f s — defaulting yaw_offset=0",
      kYawCalibTimeout);
    yaw_offset_ = 0.0;
  } else {
    double sum_sin = 0.0, sum_cos = 0.0;
    for (double y : yaw_samples) { sum_sin += std::sin(y); sum_cos += std::cos(y); }
    yaw_offset_ = std::atan2(sum_sin / static_cast<double>(yaw_samples.size()),
                             sum_cos / static_cast<double>(yaw_samples.size()));

    if (static_cast<int>(yaw_samples.size()) < kYawCalibSamples) {
      RCLCPP_WARN(get_logger(),
        "Yaw calibration: only %zu/%d samples before timeout",
        yaw_samples.size(), kYawCalibSamples);
    }

    RCLCPP_INFO(get_logger(),
      "Yaw calibration complete — %zu samples, yaw_offset=%.4f rad (%.2f deg)",
      yaw_samples.size(), yaw_offset_, yaw_offset_ * 180.0 / M_PI);
  }

  yaw_calibrated_.store(true);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────

VisionPipeline::VisionPipeline(const rclcpp::NodeOptions & options)
: Node("vision_pipeline", options)
{
  config_          = loadConfig();
  gimbal_pitch_rad_ = config_.gimbal_pitch_angle * M_PI / 180.0;

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

  // ── Ground projector ─────────────────────────────────────────────────────
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

  // ── Session directory for recording ──────────────────────────────────────
  session_dir_ = resolveSessionDir(config_.logs_path);
  videos_dir_  = session_dir_ + "/videos";
  data_dir_    = session_dir_ + "/data";
  fs::create_directories(videos_dir_);
  fs::create_directories(data_dir_);

  // ── libjpeg-turbo ────────────────────────────────────────────────────────
  tj_decompress_ = tjInitDecompress();
  if (!tj_decompress_) throw std::runtime_error("VisionPipeline: tjInitDecompress failed");

  // ── Pre-allocate RGB slots ────────────────────────────────────────────────
  for (auto & slot : buffer_)
    slot.rgb.resize(config_.width * config_.height * 3);

  // ── Hailo ─────────────────────────────────────────────────────────────────
  initHailo();

  // ── ROS subscriptions / publishers ───────────────────────────────────────
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

  // Recording command / state (ownership migrated from RecordVideo node)
  record_cmd_sub_ = create_subscription<drone_msgs::msg::Toggle>(
    dp + "/camera/record/cmd", reliable_qos,
    [this](const drone_msgs::msg::Toggle::SharedPtr msg) { recordCmdCallback(msg); });

  record_state_pub_ = create_publisher<drone_msgs::msg::Toggle>(
    dp + "/camera/record/active", reliable_qos);

  publishStreamState();
  publishRecordState();

  // ── Yaw calibration thread ────────────────────────────────────────────────
  yaw_calib_thread_ = std::thread(&VisionPipeline::runYawCalibration, this);

  // ── Worker threads ────────────────────────────────────────────────────────
  workers_running_.store(true);
  for (auto & t : worker_threads_)
    t = std::thread(&VisionPipeline::workerLoop, this);

  // ── Encoder thread ────────────────────────────────────────────────────────
  encoder_running_.store(true);
  encoder_thread_ = std::thread(&VisionPipeline::encoderLoop, this);

  // ── Detection CSV periodic flush timer (every 3 s) ───────────────────────
  det_flush_timer_ = create_wall_timer(
    std::chrono::seconds(3),
    [this]() {
      std::lock_guard<std::mutex> lk(det_csv_mtx_);
      if (!det_csv_file_.is_open()) return;
      for (const auto & line : det_csv_buffer_)
        det_csv_file_ << line;
      det_csv_buffer_.clear();
      det_csv_file_.flush();
    });

  RCLCPP_INFO(get_logger(), "vision_pipeline ready.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Destructor
// ─────────────────────────────────────────────────────────────────────────────

VisionPipeline::~VisionPipeline()
{
  if (tj_decompress_) {
    tjDestroy(tj_decompress_);
    tj_decompress_ = nullptr;
  }

  workers_running_.store(false);
  work_queue_cv_.notify_all();
  for (auto & t : worker_threads_)
    if (t.joinable()) t.join();

  hailo_vdevice_.reset();

  encoder_running_.store(false);
  encoder_queue_cv_.notify_all();
  for (auto & slot : buffer_)
    slot.cv.notify_all();
  if (encoder_thread_.joinable()) encoder_thread_.join();

  // Flush and close detection CSV
  {
    std::lock_guard<std::mutex> lk(det_csv_mtx_);
    if (det_csv_file_.is_open()) {
      for (const auto & line : det_csv_buffer_) det_csv_file_ << line;
      det_csv_buffer_.clear();
      det_csv_file_.flush();
      det_csv_file_.close();
    }
  }

  // Close any open recording clip
  if (rec_fmt_ctx_) closeRecordClip();

  if (yaw_calib_thread_.joinable()) yaw_calib_thread_.join();

  {
    std::lock_guard<std::mutex> lk(encoder_mtx_);
    if (h264_encoder_.isOpen()) h264_encoder_.close();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Recording — clip open / close / write
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::openRecordClip()
{
  ++clip_index_;

  std::ostringstream idx;
  idx << std::setw(3) << std::setfill('0') << clip_index_;

  const std::string video_path = videos_dir_ + "/" + idx.str() + ".mp4";
  const std::string csv_path   = data_dir_   + "/" + idx.str() + "_detections.csv";

  // ── FFmpeg MP4 muxer ──────────────────────────────────────────────────────
  if (avformat_alloc_output_context2(
        &rec_fmt_ctx_, nullptr, "mp4", video_path.c_str()) < 0)
    throw std::runtime_error("VisionPipeline: cannot allocate MP4 output context");

  const AVCodec * h264_codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  if (!h264_codec)
    throw std::runtime_error("VisionPipeline: H264 codec descriptor not found in FFmpeg");

  rec_video_stream_ = avformat_new_stream(rec_fmt_ctx_, h264_codec);
  if (!rec_video_stream_)
    throw std::runtime_error("VisionPipeline: cannot allocate video stream for MP4");

  AVCodecParameters * par = rec_video_stream_->codecpar;
  par->codec_type = AVMEDIA_TYPE_VIDEO;
  par->codec_id   = AV_CODEC_ID_H264;
  par->width      = config_.width;
  par->height     = config_.height;
  par->format     = AV_PIX_FMT_YUV420P;

  rec_video_stream_->time_base      = AVRational{1, 90000};   // standard MP4 video timescale
  rec_video_stream_->avg_frame_rate = AVRational{config_.fps, 1};

  // Copy SPS/PPS extradata from encoder into codecpar.
  // AV_CODEC_FLAG_GLOBAL_HEADER makes libx264 populate codec_ctx_->extradata
  // after avcodec_open2. Without this the moov atom has no codec config
  // and the file cannot be played back.
  {
    std::lock_guard<std::mutex> lk(encoder_mtx_);
    const uint8_t * ed    = h264_encoder_.extradata();
    const int       ed_sz = h264_encoder_.extradata_size();
    if (ed && ed_sz > 0) {
      par->extradata = static_cast<uint8_t *>(
        av_mallocz(ed_sz + AV_INPUT_BUFFER_PADDING_SIZE));
      if (par->extradata) {
        std::memcpy(par->extradata, ed, ed_sz);
        par->extradata_size = ed_sz;
      }
    } else {
      RCLCPP_WARN(get_logger(),
        "openRecordClip: encoder extradata empty — MP4 may not play. "
        "Is AV_CODEC_FLAG_GLOBAL_HEADER set before avcodec_open2?");
    }
  }

  AVDictionary * opts = nullptr;
  av_dict_set(&opts, "movflags", "faststart", 0);

  if (!(rec_fmt_ctx_->oformat->flags & AVFMT_NOFILE)) {
    if (avio_open(&rec_fmt_ctx_->pb, video_path.c_str(), AVIO_FLAG_WRITE) < 0) {
      av_dict_free(&opts);
      throw std::runtime_error("VisionPipeline: cannot open MP4 file " + video_path);
    }
  }

  if (avformat_write_header(rec_fmt_ctx_, &opts) < 0) {
    av_dict_free(&opts);
    throw std::runtime_error("VisionPipeline: avformat_write_header failed");
  }
  av_dict_free(&opts);
  rec_pts_ = 0;

  // ── Detection CSV ─────────────────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lk(det_csv_mtx_);
    det_csv_file_.open(csv_path, std::ios::out | std::ios::trunc);
    if (!det_csv_file_.is_open())
      throw std::runtime_error("VisionPipeline: cannot open detection CSV: " + csv_path);

    det_csv_file_  << 
      "frame_sec,frame_nanosec,"
      "frame_idx,"
      "id,"
      "x_min,y_min,x_max,y_max,"
      "score,"
      "world_x,world_y\n";
  }

  RCLCPP_INFO(get_logger(), "Recording started → %s | %s",
    video_path.c_str(), csv_path.c_str());
}

void VisionPipeline::closeRecordClip()
{
  if (!rec_fmt_ctx_) return;

  av_write_trailer(rec_fmt_ctx_);

  if (!(rec_fmt_ctx_->oformat->flags & AVFMT_NOFILE))
    avio_closep(&rec_fmt_ctx_->pb);

  avformat_free_context(rec_fmt_ctx_);
  rec_fmt_ctx_       = nullptr;
  rec_video_stream_  = nullptr;
  rec_pts_           = 0;

  // Flush detection CSV
  {
    std::lock_guard<std::mutex> lk(det_csv_mtx_);
    if (det_csv_file_.is_open()) {
      for (const auto & line : det_csv_buffer_) det_csv_file_ << line;
      det_csv_buffer_.clear();
      det_csv_file_.flush();
      det_csv_file_.close();
    }
  }

  std::ostringstream idx;
  idx << std::setw(3) << std::setfill('0') << clip_index_;
  RCLCPP_INFO(get_logger(), "Recording stopped → clip %s finalized.", idx.str().c_str());
}

int64_t VisionPipeline::writeRecordPacket(
  const H264Encoder::EncodeResult & result,
  uint32_t stamp_sec,
  uint32_t stamp_nanosec)
{
  if (!rec_fmt_ctx_ || result.data.empty()) return -1;

  const int64_t used_pts = rec_pts_;

  AVPacket * pkt = av_packet_alloc();
  if (!pkt) return -1;

  uint8_t * buf = static_cast<uint8_t *>(av_malloc(result.data.size()));
  if (!buf) { av_packet_free(&pkt); return -1; }
  std::memcpy(buf, result.data.data(), result.data.size());

  av_packet_from_data(pkt, buf, static_cast<int>(result.data.size()));
  pkt->stream_index = rec_video_stream_->index;

  // Rescale PTS from 1/fps units to 1/90000 units
  pkt->pts      = av_rescale_q(used_pts, AVRational{1, config_.fps}, rec_video_stream_->time_base);
  pkt->dts      = pkt->pts;
  pkt->duration = av_rescale_q(1,        AVRational{1, config_.fps}, rec_video_stream_->time_base);
  if (result.is_key_frame) pkt->flags |= AV_PKT_FLAG_KEY;
  ++rec_pts_;

  (void)stamp_sec;
  (void)stamp_nanosec;

  av_interleaved_write_frame(rec_fmt_ctx_, pkt);
  av_packet_free(&pkt);
  return used_pts;
}


// ─────────────────────────────────────────────────────────────────────────────
//  Slot acquisition
// ─────────────────────────────────────────────────────────────────────────────

int VisionPipeline::acquireSlot()
{
  for (int i = 0; i < kNumSlots; ++i) {
    SlotState expected = SlotState::FREE;
    if (buffer_[i].state.compare_exchange_strong(
          expected, SlotState::PENDING,
          std::memory_order_acquire,
          std::memory_order_relaxed))
    {
      return i;
    }
  }
  return -1;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Frame callback
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::frameCallback(drone_msgs::msg::FrameData::ConstSharedPtr msg)
{
  // Feed latest quaternion to calibration thread (non-blocking)
  if (!yaw_calibrated_.load() && msg->odom_valid) {
    std::lock_guard<std::mutex> lk(latest_quat_mtx_);
    latest_quat_ = {msg->quat_x, msg->quat_y, msg->quat_z, msg->quat_w, true};
  }

  const int idx = acquireSlot();
  if (idx == -1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Frame buffer full — dropping frame");
    return;
  }

  FrameSlot & slot   = buffer_[idx];
  slot.stamp_sec     = msg->image.header.stamp.sec;
  slot.stamp_nanosec = msg->image.header.stamp.nanosec;

  slot.pos_x      = msg->pos_x;
  slot.pos_y      = msg->pos_y;
  slot.pos_z      = msg->pos_z;
  slot.quat_x     = msg->quat_x;
  slot.quat_y     = msg->quat_y;
  slot.quat_z     = msg->quat_z;
  slot.quat_w     = msg->quat_w;
  slot.odom_valid = msg->odom_valid;

  slot.detections.clear();

  const int ret = tjDecompress2(
    tj_decompress_,
    msg->image.data.data(),
    static_cast<unsigned long>(msg->image.data.size()),
    slot.rgb.data(),
    config_.width, 0, config_.height,
    TJPF_RGB, TJFLAG_FASTDCT);

  if (ret != 0) {
    slot.state.store(SlotState::FREE, std::memory_order_release);
    RCLCPP_WARN(get_logger(), "JPEG decompress failed: %s", tjGetErrorStr2(tj_decompress_));
    return;
  }

  {
    std::lock_guard<std::mutex> lk(work_queue_mtx_);
    work_queue_.push(idx);
  }
  work_queue_cv_.notify_one();

  {
    std::lock_guard<std::mutex> lk(encoder_queue_mtx_);
    encoder_queue_.push(idx);
  }
  encoder_queue_cv_.notify_one();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Worker loop — letterbox + async Hailo dispatch
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::workerLoop()
{
  while (true) {
    int idx = -1;
    {
      std::unique_lock<std::mutex> lk(work_queue_mtx_);
      work_queue_cv_.wait(lk, [this] {
        return !work_queue_.empty() || !workers_running_.load();
      });
      if (!workers_running_.load() && work_queue_.empty()) break;
      idx = work_queue_.front();
      work_queue_.pop();
    }

    FrameSlot & slot = buffer_[idx];

    SlotState expected = SlotState::PENDING;
    if (!slot.state.compare_exchange_strong(
          expected, SlotState::PROCESSING,
          std::memory_order_acquire,
          std::memory_order_relaxed))
    {
      continue;
    }

    // ── Letterbox resize ─────────────────────────────────────────────────
    const cv::Mat src(config_.height, config_.width, CV_8UC3, slot.rgb.data());
    const int scaled_w = static_cast<int>(std::round(config_.width  * lb_scale_));
    const int scaled_h = static_cast<int>(std::round(config_.height * lb_scale_));

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(scaled_w, scaled_h), 0, 0, cv::INTER_LINEAR);

    cv::Mat letterbox(config_.model_input_size, config_.model_input_size,
                      CV_8UC3, slot.letterbox_buf.data());
    letterbox.setTo(cv::Scalar(0, 0, 0));

    const cv::Rect roi(lb_pad_left_, lb_pad_top_, scaled_w, scaled_h);
    resized.copyTo(letterbox(roi));

    slot.state.store(SlotState::INFERENCING, std::memory_order_release);

    // ── Async Hailo inference ─────────────────────────────────────────────
    auto callback = [this, idx](const hailort::AsyncInferCompletionInfo & info)
    {
      FrameSlot & cb_slot = buffer_[idx];

      if (info.status != HAILO_SUCCESS) {
        RCLCPP_WARN(get_logger(), "Hailo async infer failed (slot %d): %d",
          idx, static_cast<int>(info.status));
        {
          std::lock_guard<std::mutex> lk(cb_slot.cv_mtx);
          cb_slot.state.store(SlotState::PROCESSED, std::memory_order_release);
        }
        cb_slot.cv.notify_one();
        return;
      }

      // ── Parse NMS output ──────────────────────────────────────────────
      static constexpr int kMaxBboxPerClass = 100;
      static constexpr int kClassStride     = 1 + kMaxBboxPerClass * 5;

      const float * nms     = cb_slot.nms_output_buf.data();
      const float * cls_ptr = nms + config_.person_class_idx * kClassStride;
      const int     num_dets = static_cast<int>(cls_ptr[0]);

      cb_slot.detections.clear();

      if (num_dets > 0) {
        // ── Decode pose for ground projection ─────────────────────────
        bool can_project = cb_slot.odom_valid;
        double yaw = 0.0;
        if (can_project) {
          tf2::Quaternion tf_q(
            cb_slot.quat_x, cb_slot.quat_y,
            cb_slot.quat_z, cb_slot.quat_w);
          double roll_unused, pitch_unused;
          tf2::Matrix3x3(tf_q).getRPY(roll_unused, pitch_unused, yaw);
          yaw -= yaw_offset_;
          projector_->setPose(
            cb_slot.pos_x, cb_slot.pos_y, cb_slot.pos_z,
            yaw, gimbal_pitch_rad_, 0.0);
        }

        cb_slot.detections.reserve(num_dets);

        for (int d = 0; d < num_dets; ++d) {
          const float * det      = cls_ptr + 1 + d * 5;
          const float y_min_norm = det[0];
          const float x_min_norm = det[1];
          const float y_max_norm = det[2];
          const float x_max_norm = det[3];
          const float score      = det[4];

          // Letterbox → original frame pixel coords
          const float x_min_lb = x_min_norm * config_.model_input_size;
          const float y_min_lb = y_min_norm * config_.model_input_size;
          const float x_max_lb = x_max_norm * config_.model_input_size;
          const float y_max_lb = y_max_norm * config_.model_input_size;

          const float x_min = (x_min_lb - lb_pad_left_) / lb_scale_;
          const float y_min = (y_min_lb - lb_pad_top_)  / lb_scale_;
          const float x_max = (x_max_lb - lb_pad_left_) / lb_scale_;
          const float y_max = (y_max_lb - lb_pad_top_)  / lb_scale_;

          const int ix0 = std::max(0,                  static_cast<int>(x_min));
          //const int iy0 = std::max(0,                  static_cast<int>(y_min));
          const int ix1 = std::min(config_.width  - 1, static_cast<int>(x_max));
          const int iy1 = std::min(config_.height - 1, static_cast<int>(y_max));

          // ── Ground projection (foot point = bottom-centre of bbox) ─────
          const int foot_u = (ix0 + ix1) / 2;
          const int foot_v = iy1;

          double world_x = std::numeric_limits<double>::quiet_NaN();
          double world_y = std::numeric_limits<double>::quiet_NaN();

          if (can_project) {
            const auto res = projector_->projectOne(Pixel{foot_u, foot_v});
            if (res.valid) {
              world_x = res.world_x;
              world_y = res.world_y;
            }
          }

          Detection det_obj;
          det_obj.x_min   = std::max(0.0f, (x_min_lb - lb_pad_left_) / lb_scale_);
          det_obj.y_min   = std::max(0.0f, (y_min_lb - lb_pad_top_)  / lb_scale_);
          det_obj.x_max   = std::min((float)(config_.width  - 1), (x_max_lb - lb_pad_left_) / lb_scale_);
          det_obj.y_max   = std::min((float)(config_.height - 1), (y_max_lb - lb_pad_top_)  / lb_scale_);
          det_obj.score   = score;
          det_obj.id      = 0;      // placeholder; tracker will assign real IDs
          det_obj.world_x = world_x;
          det_obj.world_y = world_y;

          cb_slot.detections.push_back(det_obj);
        }
      }

      // ── INFERENCING → PROCESSED ───────────────────────────────────────
      {
        std::lock_guard<std::mutex> lk(cb_slot.cv_mtx);
        cb_slot.state.store(SlotState::PROCESSED, std::memory_order_release);
      }
      cb_slot.cv.notify_one();
    };  // end callback lambda

    auto job_exp = hailo_infer_model_.run_async(hailo_bindings_[idx], callback);
    if (!job_exp) {
      RCLCPP_ERROR(get_logger(), "run_async failed (slot %d): %d",
        idx, static_cast<int>(job_exp.status()));
      {
        std::lock_guard<std::mutex> lk(slot.cv_mtx);
        slot.state.store(SlotState::PROCESSED, std::memory_order_release);
      }
      slot.cv.notify_one();
      continue;
    }

    auto job = job_exp.release();
    (void)job;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Encoder loop
//  Runs for both streaming and recording — the H264 encoder is opened when
//  either streaming or recording is active, and closed when both are off.
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::encoderLoop()
{
  while (true) {
    int idx = -1;
    {
      std::unique_lock<std::mutex> lk(encoder_queue_mtx_);
      encoder_queue_cv_.wait(lk, [this] {
        return !encoder_queue_.empty() || !encoder_running_.load();
      });
      if (!encoder_running_.load() && encoder_queue_.empty()) break;
      idx = encoder_queue_.front();
      encoder_queue_.pop();
    }

    FrameSlot & slot = buffer_[idx];

    // Wait until Hailo inference has finished for this slot
    {
      std::unique_lock<std::mutex> lk(slot.cv_mtx);
      slot.cv.wait(lk, [&slot, this] {
        return slot.state.load(std::memory_order_acquire) == SlotState::PROCESSED
               || !encoder_running_.load();
      });
    }

    if (!encoder_running_.load()) {
      slot.state.store(SlotState::FREE, std::memory_order_release);
      break;
    }

    const bool do_stream = streaming_.load();
    const bool do_record = recording_.load();
    int64_t written_pts = -1;

    if (do_stream || do_record) {
      H264Encoder::EncodeResult result;
      {
        std::lock_guard<std::mutex> lk(encoder_mtx_);
        result = h264_encoder_.encode(slot.rgb.data(), config_.width, config_.height);
      }

      if (!result.data.empty()) {
        // ── Publish stream packet ─────────────────────────────────────────
        if (do_stream) {
          auto out = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
          out->header.stamp.sec     = slot.stamp_sec;
          out->header.stamp.nanosec = slot.stamp_nanosec;
          out->width                = config_.width;
          out->height               = config_.height;
          out->encoding             = "h264";
          out->pts                  = result.pts;
          out->flags                = result.is_key_frame ? 1 : 0;
          out->data                 = result.data;     // copy; recording may also need it
          stream_out_pub_->publish(std::move(out));
        }

        // ── Write to MP4 recording; capture the frame index used ──────────
        if (do_record) {
          written_pts = writeRecordPacket(result, slot.stamp_sec, slot.stamp_nanosec);
        }
      }
    }

    // ── Detection CSV — one row per detection, buffered ───────────────────
    // Guard on written_pts >= 0 so frame_idx in the CSV always matches a real
    // written MP4 frame (encoder may return empty data on the first few frames).
    if (do_record && written_pts >= 0 && !slot.detections.empty()) {
      std::lock_guard<std::mutex> lk(det_csv_mtx_);

      for (const auto & det : slot.detections) {
        std::ostringstream oss;
        oss << slot.stamp_sec     << ','
            << slot.stamp_nanosec << ','
            << written_pts        << ','   // MP4 frame index — use with cap.set(CAP_PROP_POS_FRAMES, frame_idx)
            << det.id             << ','
            << det.x_min          << ','
            << det.y_min          << ','
            << det.x_max          << ','
            << det.y_max          << ','
            << det.score          << ',';

        if (std::isnan(det.world_x)) oss << "nan"; else oss << det.world_x;
        oss << ',';
        if (std::isnan(det.world_y)) oss << "nan"; else oss << det.world_y;
        oss << '\n';

        det_csv_buffer_.push_back(oss.str());
      }

      // Eager flush if buffer is large
      if (det_csv_buffer_.size() >= kDetFlushSize && det_csv_file_.is_open()) {
        for (const auto & line : det_csv_buffer_) det_csv_file_ << line;
        det_csv_buffer_.clear();
        det_csv_file_.flush();
      }
    }

    slot.state.store(SlotState::FREE, std::memory_order_release);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Stream toggle
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
{
  const bool was_streaming = streaming_.load();
  const bool is_recording  = recording_.load();

  if (!was_streaming) {
    // Open encoder only if it isn't already open (recording may have opened it)
    if (!is_recording) {
      try {
        std::lock_guard<std::mutex> lk(encoder_mtx_);
        h264_encoder_.open(config_.width, config_.height, config_.fps, config_.fps / 2);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to open H264 encoder: %s", e.what());
        return;
      }
    }
    streaming_.store(true);
    RCLCPP_INFO(get_logger(), "Stream → ON  (%dx%d @ %d fps)", config_.width, config_.height, config_.fps);
  } else {
    streaming_.store(false);
    // Close encoder only if recording is also off
    if (!is_recording) {
      std::lock_guard<std::mutex> lk(encoder_mtx_);
      h264_encoder_.close();
    }
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
//  Record toggle (migrated from RecordVideo node)
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::recordCmdCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
{
  const bool was_recording = recording_.load();
  const bool is_streaming  = streaming_.load();

  if (!was_recording) {
    // Open encoder if not already open (streaming may have done it already)
    if (!is_streaming) {
      try {
        std::lock_guard<std::mutex> lk(encoder_mtx_);
        h264_encoder_.open(config_.width, config_.height, config_.fps, config_.fps / 2);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to open H264 encoder: %s", e.what());
        return;
      }
    }

    try {
      openRecordClip();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to open recording clip: %s", e.what());
      // Roll back encoder if we opened it just now
      if (!is_streaming) {
        std::lock_guard<std::mutex> lk(encoder_mtx_);
        h264_encoder_.close();
      }
      return;
    }

    recording_.store(true);
    RCLCPP_INFO(get_logger(), "Record → ON  (clip %03d)", clip_index_);
  } else {
    recording_.store(false);
    closeRecordClip();

    // Close encoder if streaming is also off
    if (!is_streaming) {
      std::lock_guard<std::mutex> lk(encoder_mtx_);
      h264_encoder_.close();
    }
    RCLCPP_INFO(get_logger(), "Record → OFF");
  }
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