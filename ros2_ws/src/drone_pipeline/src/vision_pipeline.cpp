#include "drone_pipeline/vision_pipeline.hpp"

#include <stdexcept>
#include <cstdio>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

// OpenCV for letterbox resize
#include <opencv2/imgproc.hpp>

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
  cfg.width  = cam["width"].as<int>();
  cfg.height = cam["height"].as<int>();
  cfg.fps    = cam["fps"].as<int>();

  const auto vp = root["vision_pipeline"];
  cfg.hef_path        = vp["obj_det_hef_path"].as<std::string>();
  cfg.score_threshold = vp["obj_det_score_thresh"].as<float>();

  RCLCPP_INFO(get_logger(),
    "Config → drone_id=%u  frames=%s  res=%dx%d@%dfps  hef=%s  score_thresh=%.2f",
    cfg.drone_id, cfg.frames_topic.c_str(),
    cfg.width, cfg.height, cfg.fps,
    cfg.hef_path.c_str(), cfg.score_threshold);

  return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Hailo initialisation
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::initHailo()
{
  // 1. Open VDevice
  auto vdevice_exp = hailort::VDevice::create();
  if (!vdevice_exp) {
    throw std::runtime_error(
      "VDevice::create failed: " +
      std::to_string(static_cast<int>(vdevice_exp.status())));
  }
  hailo_vdevice_ = vdevice_exp.release();

  // 2. Load HEF and create InferModel
  auto infer_model_exp = hailo_vdevice_->create_infer_model(config_.hef_path);
  if (!infer_model_exp) {
    throw std::runtime_error(
      "create_infer_model failed: " +
      std::to_string(static_cast<int>(infer_model_exp.status())));
  }
  auto infer_model = infer_model_exp.release();

  // 3. Configure model parameters BEFORE configure()
  infer_model->set_batch_size(1);

  // Optional NMS threshold override if supported by your HEF/output
  if (!infer_model->get_output_names().empty()) {
    auto out_name = infer_model->get_output_names().front();
    auto out = infer_model->output(out_name);
    if (out) {
      out->set_nms_score_threshold(config_.score_threshold);
      hailo_output_frame_size_ = out->get_frame_size();
    }
  }

  if (hailo_output_frame_size_ == 0) {
    throw std::runtime_error("Failed to determine Hailo output frame size");
  }

  // 4. Configure -> ConfiguredInferModel
  auto configured_exp = infer_model->configure();
  if (!configured_exp) {
    throw std::runtime_error(
      "InferModel::configure failed: " +
      std::to_string(static_cast<int>(configured_exp.status())));
  }
  hailo_infer_model_ = configured_exp.release();

  // 5. Pre-allocate per-slot bindings and output buffers
  const size_t nms_output_floats = hailo_output_frame_size_ / sizeof(float);

  const auto &input_names = infer_model->get_input_names();
  const auto &output_names = infer_model->get_output_names();

  if (input_names.empty()) {
    throw std::runtime_error("Hailo model has no inputs");
  }
  if (output_names.empty()) {
    throw std::runtime_error("Hailo model has no outputs");
  }

  const auto input_name = input_names.front();
  const auto output_name = output_names.front();
  const size_t input_frame_size = infer_model->input(input_name)->get_frame_size();

  for (int i = 0; i < kNumSlots; ++i) {
    buffer_[i].letterbox_buf.resize(input_frame_size, 0);
    buffer_[i].nms_output_buf.resize(nms_output_floats, 0.0f);

    auto bindings_exp = hailo_infer_model_.create_bindings();
    if (!bindings_exp) {
      throw std::runtime_error(
        "create_bindings failed for slot " + std::to_string(i) +
        ": " + std::to_string(static_cast<int>(bindings_exp.status())));
    }

    auto bindings = bindings_exp.release();

    auto in_status = bindings.input(input_name)->set_buffer(
      hailort::MemoryView(buffer_[i].letterbox_buf.data(), buffer_[i].letterbox_buf.size()));
    if (in_status != HAILO_SUCCESS) {
      throw std::runtime_error(
        "Failed to set input buffer for slot " + std::to_string(i) +
        ": " + std::to_string(static_cast<int>(in_status)));
    }

    auto out_status = bindings.output(output_name)->set_buffer(
      hailort::MemoryView(
        reinterpret_cast<uint8_t*>(buffer_[i].nms_output_buf.data()),
        buffer_[i].nms_output_buf.size() * sizeof(float)));
    if (out_status != HAILO_SUCCESS) {
      throw std::runtime_error(
        "Failed to set output buffer for slot " + std::to_string(i) +
        ": " + std::to_string(static_cast<int>(out_status)));
    }

    hailo_bindings_[i] = std::move(bindings);
  }

  RCLCPP_INFO(get_logger(), "Hailo initialized successfully");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────

VisionPipeline::VisionPipeline(const rclcpp::NodeOptions & options)
: Node("vision_pipeline", options)
{
  config_ = loadConfig();

  // ── De-letterbox constants ────────────────────────────────────────────────
  // The longer side of the original frame is scaled to 640.
  // For 1280×960: scale=0.5, scaled_h=480, pad_top=80, pad_left=0.
  // These are computed from the actual loaded config so any resolution works.
  lb_scale_     = static_cast<float>(kHailoInputW) /
                  static_cast<float>(std::max(config_.width, config_.height));
  const int scaled_w = static_cast<int>(std::round(config_.width  * lb_scale_));
  const int scaled_h = static_cast<int>(std::round(config_.height * lb_scale_));
  lb_pad_left_  = (kHailoInputW - scaled_w) / 2;
  lb_pad_top_   = (kHailoInputH - scaled_h) / 2;

  RCLCPP_INFO(get_logger(),
    "Letterbox: scale=%.4f  scaled=%dx%d  pad_left=%d  pad_top=%d",
    lb_scale_, scaled_w, scaled_h, lb_pad_left_, lb_pad_top_);

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
    [this](drone_msgs::msg::FrameData::ConstSharedPtr msg) {
      frameCallback(msg);
    });

  stream_cmd_sub_ = create_subscription<drone_msgs::msg::Toggle>(
    dp + "/camera/stream/cmd", reliable_qos,
    [this](const drone_msgs::msg::Toggle::SharedPtr msg) {
      streamCmdCallback(msg);
    });

  stream_state_pub_ = create_publisher<drone_msgs::msg::Toggle>(
    dp + "/camera/stream/active", reliable_qos);

  stream_out_pub_ = create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
    dp + "/camera/stream/out", sensor_qos);

  publishStreamState();

  // ── Worker threads ────────────────────────────────────────────────────────
  workers_running_.store(true);
  for (auto & t : worker_threads_)
    t = std::thread(&VisionPipeline::workerLoop, this);

  // ── Encoder thread ────────────────────────────────────────────────────────
  encoder_running_.store(true);
  encoder_thread_ = std::thread(&VisionPipeline::encoderLoop, this);

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

  // Stop workers first — they may be mid-async-call
  workers_running_.store(false);
  work_queue_cv_.notify_all();
  for (auto & t : worker_threads_)
    if (t.joinable()) t.join();

  // At this point no new async jobs can be submitted.
  // Any in-flight Hailo callbacks will complete naturally since
  // ConfiguredInferModel destructor waits for pending jobs.
  // Release Hailo resources before stopping encoder.
  // (ConfiguredInferModel dtor flushes the async queue.)
  hailo_vdevice_.reset();   // also destroys configured model via RAII

  // Stop encoder
  encoder_running_.store(false);
  encoder_queue_cv_.notify_all();
  for (auto & slot : buffer_)
    slot.cv.notify_all();
  if (encoder_thread_.joinable()) encoder_thread_.join();

  std::lock_guard<std::mutex> lk(encoder_mtx_);
  if (h264_encoder_.isOpen()) h264_encoder_.close();
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
  const int idx = acquireSlot();
  if (idx == -1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Frame buffer full — dropping frame");
    return;
  }

  FrameSlot & slot = buffer_[idx];
  slot.stamp_sec     = msg->image.header.stamp.sec;
  slot.stamp_nanosec = msg->image.header.stamp.nanosec;

  const int ret = tjDecompress2(
    tj_decompress_,
    msg->image.data.data(),
    static_cast<unsigned long>(msg->image.data.size()),
    slot.rgb.data(),
    config_.width, 0, config_.height,
    TJPF_RGB, TJFLAG_FASTDCT
  );

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

    // CAS PENDING → PROCESSING
    SlotState expected = SlotState::PENDING;
    if (!slot.state.compare_exchange_strong(
          expected, SlotState::PROCESSING,
          std::memory_order_acquire,
          std::memory_order_relaxed))
    {
      continue;
    }

    // ── Letterbox resize: RGB24 → 640×640 RGB24 ──────────────────────────
    // Wrap the raw slot RGB buffer in a cv::Mat (no copy).
    const cv::Mat src(config_.height, config_.width, CV_8UC3, slot.rgb.data());

    // Scale so the longer side becomes 640.
    const int scaled_w = static_cast<int>(std::round(config_.width  * lb_scale_));
    const int scaled_h = static_cast<int>(std::round(config_.height * lb_scale_));

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(scaled_w, scaled_h), 0, 0, cv::INTER_LINEAR);

    // Wrap the pre-allocated letterbox buffer as a 640×640 Mat (no alloc).
    cv::Mat letterbox(kHailoInputH, kHailoInputW, CV_8UC3,
                      slot.letterbox_buf.data());
    letterbox.setTo(cv::Scalar(0, 0, 0));   // black padding

    // Copy resized image into the centre of the letterbox canvas.
    const cv::Rect roi(lb_pad_left_, lb_pad_top_, scaled_w, scaled_h);
    resized.copyTo(letterbox(roi));

    // ── Transition PROCESSING → INFERENCING ─────────────────────────────
    slot.state.store(SlotState::INFERENCING, std::memory_order_release);

    // ── Fire async Hailo inference ───────────────────────────────────────
    // Capture everything by value except 'this' — the lambda runs on a
    // HailoRT-internal thread after this worker has already returned.
    auto callback = [this, idx](const hailort::AsyncInferCompletionInfo & info)
    {
      FrameSlot & cb_slot = buffer_[idx];

      if (info.status != HAILO_SUCCESS) {
        RCLCPP_WARN(
          get_logger(),
          "Hailo async infer failed (slot %d): %d",
          idx,
          static_cast<int>(info.status));

        {
          std::lock_guard<std::mutex> lk(cb_slot.cv_mtx);
          cb_slot.state.store(SlotState::PROCESSED, std::memory_order_release);
        }
        cb_slot.cv.notify_one();
        return;
      }

      // ── Parse NMS output (BY_CLASS layout) ──────────────────────────
      // Layout for each class:
      //   float[0]        = number of valid detections (cast to int)
      //   float[1..5*N]   = N detections × [y_min, x_min, y_max, x_max, score]
      //                     coordinates are normalised to [0, 1] in 640×640 space
      //
      // Stride per class = 1 + max_bboxes_per_class * 5 = 501 floats
      static constexpr int kMaxBboxPerClass = 100;
      static constexpr int kClassStride     = 1 + kMaxBboxPerClass * 5;

      const float * nms = cb_slot.nms_output_buf.data();

      // Only look at person class (index 0)
      const float * cls_ptr = nms + kPersonClassIdx * kClassStride;
      const int num_dets = static_cast<int>(cls_ptr[0]);

      if (num_dets > 0) {
        // Draw detections in-place on the RGB buffer (no copy — cv::Mat is just a view).
        cv::Mat frame(config_.height, config_.width, CV_8UC3, cb_slot.rgb.data());

        for (int d = 0; d < num_dets; ++d) {
          // Each detection: [y_min, x_min, y_max, x_max, score] (normalised, 640-space)
          const float * det      = cls_ptr + 1 + d * 5;
          const float y_min_norm = det[0];
          const float x_min_norm = det[1];
          const float y_max_norm = det[2];
          const float x_max_norm = det[3];
          const float score      = det[4];

          // Convert from normalised [0,1] to pixel coords in 640×640 space
          const float x_min_lb = x_min_norm * kHailoInputW;
          const float y_min_lb = y_min_norm * kHailoInputH;
          const float x_max_lb = x_max_norm * kHailoInputW;
          const float y_max_lb = y_max_norm * kHailoInputH;

          // De-letterbox back to original frame coordinates
          const float x_min = (x_min_lb - lb_pad_left_) / lb_scale_;
          const float y_min = (y_min_lb - lb_pad_top_)  / lb_scale_;
          const float x_max = (x_max_lb - lb_pad_left_) / lb_scale_;
          const float y_max = (y_max_lb - lb_pad_top_)  / lb_scale_;

          // Clamp to frame bounds
          const int ix0 = std::max(0,                  static_cast<int>(x_min));
          const int iy0 = std::max(0,                  static_cast<int>(y_min));
          const int ix1 = std::min(config_.width  - 1, static_cast<int>(x_max));
          const int iy1 = std::min(config_.height - 1, static_cast<int>(y_max));

          // Bounding box — bright green in RGB
          cv::rectangle(frame,
            cv::Point(ix0, iy0), cv::Point(ix1, iy1),
            cv::Scalar(0, 255, 0), 2);

          // Score label
          char label[16];
          std::snprintf(label, sizeof(label), "%.2f", score);
          cv::putText(frame, label,
            cv::Point(ix0, std::max(iy0 - 4, 0)),
            cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
      }

      // ── Transition INFERENCING → PROCESSED, wake encoder ─────────────
      {
        std::lock_guard<std::mutex> lk(cb_slot.cv_mtx);
        cb_slot.state.store(SlotState::PROCESSED, std::memory_order_release);
      }
      cb_slot.cv.notify_one();
    };  // end callback lambda

    auto job_exp = hailo_infer_model_.run_async(hailo_bindings_[idx], callback);
    if (!job_exp) {
      RCLCPP_ERROR(
        get_logger(),
        "run_async failed (slot %d): %d",
        idx,
        static_cast<int>(job_exp.status()));

      {
        std::lock_guard<std::mutex> lk(slot.cv_mtx);
        slot.state.store(SlotState::PROCESSED, std::memory_order_release);
      }
      slot.cv.notify_one();
      continue;
    }

    auto job = job_exp.release();
    (void)job;

    // Worker returns here. Slot stays INFERENCING until the Hailo callback fires.
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Encoder loop — unchanged logic, new state predicate
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

    // Wait until Hailo callback transitions slot to PROCESSED
    // (or until we are shutting down)
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

    // Encode and publish if streaming
    if (streaming_.load()) {
      H264Encoder::EncodeResult result;
      {
        std::lock_guard<std::mutex> lk(encoder_mtx_);
        result = h264_encoder_.encode(
          slot.rgb.data(), config_.width, config_.height);
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

    slot.state.store(SlotState::FREE, std::memory_order_release);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Stream toggle
// ─────────────────────────────────────────────────────────────────────────────

void VisionPipeline::streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
{
  if (!streaming_.load()) {
    try {
      std::lock_guard<std::mutex> lk(encoder_mtx_);
      h264_encoder_.open(config_.width, config_.height,
                         config_.fps, config_.fps / 2);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to open H264 encoder: %s", e.what());
      return;
    }
    streaming_.store(true);
    RCLCPP_INFO(get_logger(), "Stream → ON  (%dx%d @ %d fps)",
      config_.width, config_.height, config_.fps);
  } else {
    streaming_.store(false);
    std::lock_guard<std::mutex> lk(encoder_mtx_);
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

}  // namespace drone_pipeline

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_pipeline::VisionPipeline)