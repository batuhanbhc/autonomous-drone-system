#include "drone_pipeline/video_streamer.hpp"

#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

extern "C" {
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/opt.h>
}

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace drone_pipeline
{

namespace
{

template<typename T>
T yamlOr(const YAML::Node & node, const char * key, const T & default_value)
{
  return node[key] ? node[key].as<T>() : default_value;
}

std::string ffmpegErrorString(int errnum)
{
  char buf[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(errnum, buf, sizeof(buf));
  return std::string(buf);
}

VideoStreamerConfig loadVideoStreamerConfig(
  rclcpp::Node & node,
  const std::string & codec_override,
  const std::string & gcs_host_override)
{
  const std::string share_dir =
    ament_index_cpp::get_package_share_directory("mavros_config");
  const std::string config_path = share_dir + "/config/control_params.yaml";

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_FATAL(node.get_logger(), "Failed to parse YAML: %s", e.what());
    throw;
  }

  VideoStreamerConfig cfg;
  cfg.drone_id = static_cast<uint8_t>(root["drone_id"].as<int>());

  const auto cam = root["camera"];
  cfg.width = cam["width"].as<int>();
  cfg.height = cam["height"].as<int>();
  cfg.fps = cam["fps"].as<int>();

  const std::string configured_codec =
    root["streaming"] && root["streaming"]["codec"] ?
    root["streaming"]["codec"].as<std::string>() : "mjpeg";
  const std::string requested_codec =
    codec_override.empty() ? configured_codec : codec_override;

  if (!parseStreamCodec(requested_codec, cfg.stream_codec)) {
    RCLCPP_WARN(node.get_logger(),
      "Unknown stream codec '%s'; falling back to 'mjpeg'",
      requested_codec.c_str());
    cfg.stream_codec = StreamCodec::kMjpeg;
  }

  cfg.frames_topic = "/drone_" + std::to_string(cfg.drone_id) +
    "/" + root["custom_topics"]["images"].as<std::string>();

  const auto link = root["link"];
  const auto video = link["video"];
  cfg.video_transport = yamlOr<std::string>(video, "transport", "udp");
  cfg.video_port = yamlOr<int>(video, "port", 50020);

  cfg.gcs_host = gcs_host_override;
  if (cfg.gcs_host.empty() && link["gcs_host"]) {
    cfg.gcs_host = link["gcs_host"].as<std::string>();
  }

  return cfg;
}

class VideoStreamer::NetworkVideoSink
{
public:
  NetworkVideoSink() = default;
  ~NetworkVideoSink() { close(); }

  bool isOpen() const { return fmt_ctx_ != nullptr; }

  void openMjpeg(const VideoStreamerConfig & cfg)
  {
    openCommon(cfg, AV_CODEC_ID_MJPEG);
    AVCodecParameters * par = stream_->codecpar;
    par->codec_type = AVMEDIA_TYPE_VIDEO;
    par->codec_id = AV_CODEC_ID_MJPEG;
    par->format = AV_PIX_FMT_YUVJ422P;
    par->width = cfg.width;
    par->height = cfg.height;
    writeHeader();
  }

  void openH264(const VideoStreamerConfig & cfg, const H264Encoder & encoder)
  {
    openCommon(cfg, AV_CODEC_ID_H264);
    if (!encoder.copyCodecParameters(stream_->codecpar)) {
      throw std::runtime_error("Failed to copy H264 codec parameters");
    }
    stream_->time_base = AVRational{1, cfg.fps};
    writeHeader();
  }

  void close()
  {
    if (!fmt_ctx_) {
      return;
    }

    if (header_written_) {
      av_write_trailer(fmt_ctx_);
    }
    if (!(fmt_ctx_->oformat->flags & AVFMT_NOFILE) && fmt_ctx_->pb) {
      avio_closep(&fmt_ctx_->pb);
    }
    avformat_free_context(fmt_ctx_);
    fmt_ctx_ = nullptr;
    stream_ = nullptr;
    header_written_ = false;
  }

  void writePacket(const uint8_t * data, size_t size, int64_t pts, bool is_key_frame)
  {
    if (!fmt_ctx_ || !stream_) {
      throw std::runtime_error("video sink is not open");
    }

    AVPacket pkt;
    av_init_packet(&pkt);
    pkt.data = nullptr;
    pkt.size = 0;

    const int alloc_rc = av_new_packet(&pkt, static_cast<int>(size));
    if (alloc_rc < 0) {
      throw std::runtime_error("av_new_packet failed: " + ffmpegErrorString(alloc_rc));
    }

    std::memcpy(pkt.data, data, size);
    pkt.stream_index = stream_->index;
    pkt.pts = pts;
    pkt.dts = pts;
    pkt.duration = 1;
    if (is_key_frame) {
      pkt.flags |= AV_PKT_FLAG_KEY;
    }

    const int rc = av_interleaved_write_frame(fmt_ctx_, &pkt);
    av_packet_unref(&pkt);
    if (rc < 0) {
      throw std::runtime_error("av_interleaved_write_frame failed: " + ffmpegErrorString(rc));
    }
  }

private:
  void openCommon(const VideoStreamerConfig & cfg, AVCodecID codec_id)
  {
    close();

    const std::string url = buildUrl(cfg);
    AVFormatContext * ctx = nullptr;
    int rc = avformat_alloc_output_context2(&ctx, nullptr, "nut", url.c_str());
    if (rc < 0 || !ctx) {
      throw std::runtime_error("avformat_alloc_output_context2 failed: " + ffmpegErrorString(rc));
    }

    fmt_ctx_ = ctx;
    fmt_ctx_->flags |= AVFMT_FLAG_FLUSH_PACKETS;

    stream_ = avformat_new_stream(fmt_ctx_, nullptr);
    if (!stream_) {
      close();
      throw std::runtime_error("avformat_new_stream failed");
    }

    stream_->time_base = AVRational{1, cfg.fps};
    stream_->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    stream_->codecpar->codec_id = codec_id;
    stream_->codecpar->width = cfg.width;
    stream_->codecpar->height = cfg.height;

    AVDictionary * options = nullptr;
    if (cfg.video_transport == "tcp") {
      av_dict_set(&options, "tcp_nodelay", "1", 0);
    } else {
      av_dict_set(&options, "pkt_size", "1316", 0);
      av_dict_set(&options, "buffer_size", "1048576", 0);
    }

    rc = avio_open2(&fmt_ctx_->pb, url.c_str(), AVIO_FLAG_WRITE, nullptr, &options);
    av_dict_free(&options);
    if (rc < 0) {
      close();
      throw std::runtime_error("avio_open2 failed for " + url + ": " + ffmpegErrorString(rc));
    }
  }

  void writeHeader()
  {
    const int rc = avformat_write_header(fmt_ctx_, nullptr);
    if (rc < 0) {
      throw std::runtime_error("avformat_write_header failed: " + ffmpegErrorString(rc));
    }
    header_written_ = true;
  }

  static std::string buildUrl(const VideoStreamerConfig & cfg)
  {
    if (cfg.gcs_host.empty()) {
      throw std::runtime_error("gcs_host is empty");
    }

    if (cfg.video_transport == "tcp") {
      return "tcp://" + cfg.gcs_host + ":" + std::to_string(cfg.video_port);
    }

    return "udp://" + cfg.gcs_host + ":" + std::to_string(cfg.video_port) +
      "?pkt_size=1316&buffer_size=1048576";
  }

  AVFormatContext * fmt_ctx_{nullptr};
  AVStream * stream_{nullptr};
  bool header_written_{false};
};

}  // namespace

VideoStreamer::VideoStreamer(const rclcpp::NodeOptions & options)
: Node("video_streamer", options)
{
  declare_parameter<std::string>("stream_codec", "");
  declare_parameter<std::string>("gcs_host", "");

  config_ = loadConfig();
  sink_ = std::make_unique<NetworkVideoSink>();

  avformat_network_init();

  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const std::string dp = "/drone_" + std::to_string(config_.drone_id);

  frame_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  stream_cmd_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions frame_sub_opts;
  frame_sub_opts.callback_group = frame_cb_group_;

  rclcpp::SubscriptionOptions stream_cmd_sub_opts;
  stream_cmd_sub_opts.callback_group = stream_cmd_cb_group_;

  frame_sub_ = create_subscription<drone_msgs::msg::FrameData>(
    config_.frames_topic,
    sensor_qos,
    [this](drone_msgs::msg::FrameData::ConstSharedPtr msg) { frameCallback(msg); },
    frame_sub_opts);

  stream_cmd_sub_ = create_subscription<drone_msgs::msg::Toggle>(
    dp + "/camera/stream/cmd",
    reliable_qos,
    [this](const drone_msgs::msg::Toggle::SharedPtr msg) { streamCmdCallback(msg); },
    stream_cmd_sub_opts);

  stream_state_pub_ = create_publisher<drone_msgs::msg::Toggle>(
    dp + "/camera/stream/active",
    reliable_qos);

  publishStreamState();
  streamer_thread_ = std::thread(&VideoStreamer::streamerLoop, this);

  RCLCPP_INFO(get_logger(),
    "video_streamer ready → topic=%s codec=%s transport=%s port=%d host=%s",
    config_.frames_topic.c_str(),
    streamCodecName(config_.stream_codec),
    config_.video_transport.c_str(),
    config_.video_port,
    config_.gcs_host.empty() ? "<unset>" : config_.gcs_host.c_str());
}

VideoStreamer::~VideoStreamer()
{
  running_.store(false);
  queue_cv_.notify_all();
  if (streamer_thread_.joinable()) {
    streamer_thread_.join();
  }
  {
    std::lock_guard<std::mutex> stream_lk(stream_mtx_);
    closeStream();
  }
  avformat_network_deinit();
}

VideoStreamerConfig VideoStreamer::loadConfig()
{
  const std::string codec_override = get_parameter("stream_codec").as_string();
  const std::string gcs_host_override = get_parameter("gcs_host").as_string();
  return loadVideoStreamerConfig(*this, codec_override, gcs_host_override);
}

void VideoStreamer::frameCallback(drone_msgs::msg::FrameData::ConstSharedPtr msg)
{
  if (!streaming_.load(std::memory_order_relaxed)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lk(queue_mtx_);
    queue_.clear();
    queue_.push_back(std::move(msg));
  }
  queue_cv_.notify_one();
}

void VideoStreamer::streamCmdCallback(const drone_msgs::msg::Toggle::SharedPtr /*msg*/)
{
  const bool new_state = !streaming_.load(std::memory_order_relaxed);
  streaming_.store(new_state, std::memory_order_relaxed);
  if (!new_state) {
    {
      std::lock_guard<std::mutex> lk(queue_mtx_);
      queue_.clear();
    }
    std::lock_guard<std::mutex> stream_lk(stream_mtx_);
    closeStream();
  }
  publishStreamState();
  RCLCPP_INFO(get_logger(), "Stream -> %s", new_state ? "ON" : "OFF");
}

void VideoStreamer::publishStreamState()
{
  drone_msgs::msg::Toggle msg;
  msg.state = streaming_.load(std::memory_order_relaxed);
  stream_state_pub_->publish(msg);
}

bool VideoStreamer::ensureStreamOpen()
{
  if (sink_->isOpen()) {
    return true;
  }

  const rclcpp::Time now = now();
  if ((now - last_open_attempt_).seconds() < 1.0) {
    return false;
  }
  last_open_attempt_ = now;

  try {
    if (config_.stream_codec == StreamCodec::kH264) {
      if (!h264_encoder_.isOpen()) {
        h264_encoder_.open(config_.width, config_.height, config_.fps, 60);
      }
      sink_->openH264(config_, h264_encoder_);
    } else {
      sink_->openMjpeg(config_);
    }
    mjpeg_pts_ = 0;
    return true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Failed to open stream output: %s", e.what());
    closeStream();
    return false;
  }
}

void VideoStreamer::closeStream()
{
  if (sink_) {
    sink_->close();
  }
  if (h264_encoder_.isOpen()) {
    h264_encoder_.close();
  }
  mjpeg_pts_ = 0;
}

bool VideoStreamer::writeMjpegFrame(const drone_msgs::msg::FrameData::ConstSharedPtr & msg)
{
  sink_->writePacket(
    msg->image.data.data(),
    msg->image.data.size(),
    mjpeg_pts_++,
    true);
  return true;
}

bool VideoStreamer::writeH264Frame(const drone_msgs::msg::FrameData::ConstSharedPtr & msg)
{
  const auto result = h264_encoder_.encode(
    msg->image.data.data(),
    msg->image.data.size());
  if (result.data.empty()) {
    return false;
  }

  sink_->writePacket(
    result.data.data(),
    result.data.size(),
    result.pts,
    result.is_key_frame);
  return true;
}

void VideoStreamer::streamerLoop()
{
  while (running_.load(std::memory_order_relaxed)) {
    drone_msgs::msg::FrameData::ConstSharedPtr msg;
    {
      std::unique_lock<std::mutex> lk(queue_mtx_);
      queue_cv_.wait(lk, [this] {
        return !queue_.empty() || !running_.load(std::memory_order_relaxed);
      });
      if (!running_.load(std::memory_order_relaxed)) {
        break;
      }
      msg = queue_.back();
      queue_.clear();
    }

    if (!streaming_.load(std::memory_order_relaxed)) {
      continue;
    }

    std::lock_guard<std::mutex> stream_lk(stream_mtx_);
    if (!ensureStreamOpen()) {
      continue;
    }

    try {
      if (config_.stream_codec == StreamCodec::kH264) {
        (void)writeH264Frame(msg);
      } else {
        (void)writeMjpegFrame(msg);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Stream write failed: %s", e.what());
      closeStream();
    }
  }
}

}  // namespace drone_pipeline

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(drone_pipeline::VideoStreamer)
