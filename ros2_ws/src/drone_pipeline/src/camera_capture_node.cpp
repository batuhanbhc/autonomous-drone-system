#include "drone_pipeline/camera_capture.hpp"

#include <stdexcept>
#include <string>
#include <vector>
#include <cstring>
#include <cerrno>

// V4L2 / mmap
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rmw/qos_profiles.h"

namespace drone_pipeline
{

// ─────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────

namespace
{

// Number of kernel MMAP buffers to request.
// 2 is the minimum; 4 keeps the kernel busy while we process.
constexpr unsigned int kNumBuffers = 4;

struct V4L2Buffer
{
  void *   start  = nullptr;
  size_t   length = 0;
};

// Thin wrapper so the ioctl boilerplate stays out of the main code.
int xioctl(int fd, unsigned long request, void * arg)
{
  int r;
  do { r = ::ioctl(fd, request, arg); } while (r == -1 && errno == EINTR);
  return r;
}

}  // namespace

// ─────────────────────────────────────────────────────────────
//  Config
// ─────────────────────────────────────────────────────────────

CameraConfig CameraCapture::loadConfig()
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

  CameraConfig cfg;

  if (!root["camera"])
    throw std::runtime_error("Missing 'camera' section");

  const auto cam = root["camera"];
  cfg.width       = cam["width"].as<int>();
  cfg.height      = cam["height"].as<int>();
  cfg.fps         = cam["fps"].as<int>();
  cfg.device_path = cam["device_path"].as<std::string>();
  // pixel_format is intentionally NOT read — MJPEG is hardcoded.

  if (!root["drone_id"])
    throw std::runtime_error("Missing 'drone_id'");
  cfg.drone_id = static_cast<uint8_t>(root["drone_id"].as<int>());

  if (!root["custom_topics"] || !root["custom_topics"]["raw_images"])
    throw std::runtime_error("Missing 'custom_topics/raw_images'");
  cfg.raw_images_topic = root["custom_topics"]["raw_images"].as<std::string>();

  RCLCPP_INFO(
    get_logger(),
    "Config → device=%s  format=MJPEG(hardcoded)  resolution=%dx%d  fps=%d  "
    "drone_id=%u  topic=%s",
    cfg.device_path.c_str(), cfg.width, cfg.height, cfg.fps,
    cfg.drone_id, cfg.raw_images_topic.c_str());

  return cfg;
}

// ─────────────────────────────────────────────────────────────
//  V4L2 device open + configure
// ─────────────────────────────────────────────────────────────

// V4L2 MMAP buffers — kept as a member vector in the translation unit.
// Declared here so captureThread() and stopStreaming() share them.
static std::vector<V4L2Buffer> g_buffers;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

void CameraCapture::openDevice()
{
  fd_ = ::open(config_.device_path.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd_ == -1)
    throw std::runtime_error("Cannot open device: " + config_.device_path +
                             "  errno=" + std::to_string(errno));

  // ── 1. Query capabilities ─────────────────────────────────
  v4l2_capability cap{};
  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) == -1)
    throw std::runtime_error("VIDIOC_QUERYCAP failed");
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    throw std::runtime_error("Device is not a video capture device");
  if (!(cap.capabilities & V4L2_CAP_STREAMING))
    throw std::runtime_error("Device does not support streaming I/O");

  // ── 2. Set MJPEG format ───────────────────────────────────
  v4l2_format fmt{};
  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = static_cast<__u32>(config_.width);
  fmt.fmt.pix.height      = static_cast<__u32>(config_.height);
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;

  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) == -1)
    throw std::runtime_error("VIDIOC_S_FMT (MJPEG) failed");

  // Driver may adjust dimensions — log what we actually got.
  RCLCPP_INFO(get_logger(),
    "Negotiated: %ux%u  fourcc=%c%c%c%c",
    fmt.fmt.pix.width, fmt.fmt.pix.height,
    (fmt.fmt.pix.pixelformat >>  0) & 0xFF,
    (fmt.fmt.pix.pixelformat >>  8) & 0xFF,
    (fmt.fmt.pix.pixelformat >> 16) & 0xFF,
    (fmt.fmt.pix.pixelformat >> 24) & 0xFF);

  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
    throw std::runtime_error("Driver rejected MJPEG — unsupported by this camera");

  // ── 3. Set frame rate ─────────────────────────────────────
  v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator   = 1;
  parm.parm.capture.timeperframe.denominator = static_cast<__u32>(config_.fps);
  // Ignore EINVAL — some drivers are read-only for frame rate.
  xioctl(fd_, VIDIOC_S_PARM, &parm);

  // ── 4. Request MMAP buffers ───────────────────────────────
  v4l2_requestbuffers req{};
  req.count  = kNumBuffers;
  req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (xioctl(fd_, VIDIOC_REQBUFS, &req) == -1)
    throw std::runtime_error("VIDIOC_REQBUFS failed");
  if (req.count < 2)
    throw std::runtime_error("Insufficient MMAP buffers from driver");

  // ── 5. Map buffers into user space ────────────────────────
  g_buffers.resize(req.count);
  for (unsigned int i = 0; i < req.count; ++i) {
    v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = i;

    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1)
      throw std::runtime_error("VIDIOC_QUERYBUF failed for index " + std::to_string(i));

    g_buffers[i].length = buf.length;
    g_buffers[i].start  = ::mmap(nullptr, buf.length,
                                  PROT_READ | PROT_WRITE, MAP_SHARED,
                                  fd_, buf.m.offset);
    if (g_buffers[i].start == MAP_FAILED)
      throw std::runtime_error("mmap failed for buffer " + std::to_string(i));
  }
}

void CameraCapture::startStreaming()
{
  // Enqueue all buffers, then start the stream.
  for (unsigned int i = 0; i < g_buffers.size(); ++i) {
    v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = i;
    if (xioctl(fd_, VIDIOC_QBUF, &buf) == -1)
      throw std::runtime_error("VIDIOC_QBUF failed for index " + std::to_string(i));
  }

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1)
    throw std::runtime_error("VIDIOC_STREAMON failed");
}

void CameraCapture::stopStreaming()
{
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(fd_, VIDIOC_STREAMOFF, &type);  // best-effort, ignore error

  for (auto & buf : g_buffers) {
    if (buf.start && buf.start != MAP_FAILED)
      ::munmap(buf.start, buf.length);
  }
  g_buffers.clear();

  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

// ─────────────────────────────────────────────────────────────
//  Constructor / Destructor
// ─────────────────────────────────────────────────────────────

CameraCapture::CameraCapture(const rclcpp::NodeOptions & options)
: Node("camera_capture", options)
{
  config_ = loadConfig();

  // Build topic name.
  // Convention: <base_topic>/compressed  so tools like image_view,
  // web_video_server, etc. recognise it automatically.
  const std::string topic =
    "/drone_" + std::to_string(config_.drone_id) +
    "/" + config_.raw_images_topic + "/compressed";

  RCLCPP_INFO(get_logger(), "Publishing on: %s", topic.c_str());

  // Sensor-data QoS (best-effort, small depth) is appropriate for live video.
  // UniquePtr publish path is used in captureThread() which enables
  // zero-copy intra-process delivery when both ends opt into it via
  // rclcpp::NodeOptions().use_intra_process_comms(true).
  image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    topic, rclcpp::SensorDataQoS());

  openDevice();
  startStreaming();

  running_        = true;
  capture_thread_ = std::thread(&CameraCapture::captureThread, this);

  RCLCPP_INFO(get_logger(), "camera_capture node ready.");
}

CameraCapture::~CameraCapture()
{
  running_ = false;
  if (capture_thread_.joinable())
    capture_thread_.join();
  stopStreaming();
}

// ─────────────────────────────────────────────────────────────
//  Capture thread
// ─────────────────────────────────────────────────────────────

void CameraCapture::captureThread()
{
  const std::string frame_id =
    "drone_" + std::to_string(config_.drone_id) + "_camera";

  while (running_ && rclcpp::ok()) {

    // ── Wait for a frame to become ready (with timeout) ─────
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);

    timeval tv{};
    tv.tv_sec  = 2;   // 2-second timeout — far longer than any sane frame interval
    tv.tv_usec = 0;

    int r = ::select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (r == -1) {
      if (errno == EINTR) continue;
      RCLCPP_ERROR(get_logger(), "select() error: %s", std::strerror(errno));
      break;
    }
    if (r == 0) {
      RCLCPP_WARN(get_logger(), "select() timeout — no frame received");
      continue;
    }

    // ── Dequeue the filled buffer ────────────────────────────
    v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_DQBUF, &buf) == -1) {
      if (errno == EAGAIN) continue;  // spurious wakeup
      RCLCPP_WARN(get_logger(), "VIDIOC_DQBUF error: %s", std::strerror(errno));
      continue;
    }

    // ── Build CompressedImage and publish ───────────────────
    // The MMAP buffer contains a complete MJPEG frame.
    // We copy it once into the ROS message; with intra-process zero-copy
    // enabled that message unique_ptr is moved directly to the subscriber
    // without any further copies or serialisation.
    auto msg          = std::make_unique<sensor_msgs::msg::CompressedImage>();
    msg->header.stamp    = now();
    msg->header.frame_id = frame_id;
    msg->format          = "jpeg";   // ROS convention for MJPEG/JPEG

    const auto * src = static_cast<const uint8_t *>(g_buffers[buf.index].start);
    msg->data.assign(src, src + buf.bytesused);

    image_pub_->publish(std::move(msg));  // zero-copy path when intra-process

    // ── Re-enqueue the buffer immediately ───────────────────
    if (xioctl(fd_, VIDIOC_QBUF, &buf) == -1)
      RCLCPP_WARN(get_logger(), "VIDIOC_QBUF error: %s", std::strerror(errno));
  }
}

}  // namespace drone_pipeline

// ─────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    // use_intra_process_comms is intentionally left at its default (false)
    // here so the node works standalone without any change.
    // When composing nodes in the same process, construct via
    //   rclcpp::NodeOptions().use_intra_process_comms(true)
    // and the publisher's unique_ptr path will deliver frames to any
    // same-process subscriber with zero copies.
    auto node = std::make_shared<drone_pipeline::CameraCapture>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("camera_capture"),
      "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}