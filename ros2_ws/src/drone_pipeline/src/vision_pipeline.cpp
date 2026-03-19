#include "drone_pipeline/vision_pipeline.hpp"

#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

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

  RCLCPP_INFO(get_logger(),
    "Config → drone_id=%u  frames=%s  res=%dx%d@%dfps",
    cfg.drone_id, cfg.frames_topic.c_str(),
    cfg.width, cfg.height, cfg.fps);

  return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────

VisionPipeline::VisionPipeline(const rclcpp::NodeOptions & options)
: Node("vision_pipeline", options)
{
  config_ = loadConfig();

  tj_decompress_ = tjInitDecompress();
  if (!tj_decompress_) throw std::runtime_error("VisionPipeline: tjInitDecompress failed");

  // Pre-allocate every RGB slot so frameCallback never allocates on the hot path
  for (auto & slot : buffer_)
    slot.rgb.resize(config_.width * config_.height * 3);

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

  // Publish initial state — streaming starts disabled
  publishStreamState();

  // Start worker threads
  workers_running_.store(true);
  for (auto & t : worker_threads_)
    t = std::thread(&VisionPipeline::workerLoop, this);

  // Start encoder thread — h264_encoder_ opened on first stream/cmd toggle
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
  // Stop workers
  workers_running_.store(false);
  work_queue_cv_.notify_all();
  for (auto & t : worker_threads_)
    if (t.joinable()) t.join();

  // Stop encoder
  encoder_running_.store(false);
  encoder_queue_cv_.notify_all();
  // Wake any slot CVs the encoder might be waiting on
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
    // CAS: only one caller can transition FREE → PENDING
    if (buffer_[i].state.compare_exchange_strong(
          expected, SlotState::PENDING,
          std::memory_order_acquire,
          std::memory_order_relaxed))
    {
      return i;
    }
  }
  return -1;  // buffer full
}

// ─────────────────────────────────────────────────────────────────────────────
//  Frame callback — runs on rclcpp executor thread
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

  // ── Decompress JPEG → RGB24 with libjpeg-turbo ───────────────────────────
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

  // Slot is written — hand off to workers and encoder
  // State goes PENDING → stays PENDING (workers will CAS to PROCESSING)
  // Push to both queues while holding their locks
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
//  Worker loop  (runs on kNumWorkers threads)
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

    // CAS PENDING → PROCESSING (another worker may have grabbed it — shouldn't
    // happen since each index appears once in the queue, but be safe)
    SlotState expected = SlotState::PENDING;
    if (!slot.state.compare_exchange_strong(
          expected, SlotState::PROCESSING,
          std::memory_order_acquire,
          std::memory_order_relaxed))
    {
      // Someone else got it — skip
      continue;
    }

    // ── Dummy processing ──────────────────────────────────────────────────
    // Replace this block with Hailo async inference later.
    // For now: just simulate work with a short sleep.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // ── Mark done and wake encoder ────────────────────────────────────────
    {
      std::lock_guard<std::mutex> lk(slot.cv_mtx);
      slot.state.store(SlotState::PROCESSED, std::memory_order_release);
    }
    slot.cv.notify_one();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Encoder loop  (single thread, processes slots in insertion order)
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

    // ── Wait until workers have finished this slot ────────────────────────
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

    // ── Encode and publish if streaming is enabled ────────────────────────
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

    // ── Free slot ─────────────────────────────────────────────────────────
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