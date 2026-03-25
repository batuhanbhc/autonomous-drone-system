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

  if (!root["drone_id"])
    throw std::runtime_error("Missing 'drone_id'");
  cfg.drone_id = static_cast<uint8_t>(root["drone_id"].as<int>());

  if (!root["custom_topics"] || !root["custom_topics"]["images"])
    throw std::runtime_error("Missing 'custom_topics/images'");
  cfg.frames_topic = root["custom_topics"]["images"].as<std::string>();

  const auto & mv    = root["mavros_topics"];
  const std::string dp = "/drone_" + std::to_string(cfg.drone_id);
  cfg.odom_topic     = dp + mv["odom"].as<std::string>();
  cfg.gps1_raw_topic = dp + mv["gps1_raw"].as<std::string>();

  RCLCPP_INFO(
    get_logger(),
    "Config → device=%s  format=MJPEG(hardcoded)  resolution=%dx%d  fps=%d  "
    "drone_id=%u  topic=%s",
    cfg.device_path.c_str(), cfg.width, cfg.height, cfg.fps,
    cfg.drone_id, cfg.frames_topic.c_str());

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

  const std::string dp    = "/drone_" + std::to_string(config_.drone_id);
  const std::string topic = dp + "/" + config_.frames_topic;

  frame_pub_ = create_publisher<drone_msgs::msg::FrameData>(topic, rclcpp::SensorDataQoS());

  const auto sensor_qos   = rclcpp::SensorDataQoS();
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    config_.odom_topic, sensor_qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });

  gps_sub_ = create_subscription<mavros_msgs::msg::GPSRAW>(
    config_.gps1_raw_topic, reliable_qos,
    [this](const mavros_msgs::msg::GPSRAW::SharedPtr msg) { gpsCallback(msg); });

  // Staleness timers — fire every 1 s; reset snapshot to nullopt if no fresh data arrived.
  // Each timer is independent: GPS can go stale while odom remains valid, and vice-versa.
  odom_staleness_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      // If odom_valid_ is already false the sensor was already stale — no-op.
      // If it was true, a fresh odomCallback() will have reset it to true since the
      // last timer fire; if not, mark it stale now.
      // We use a simple flag: odomCallback sets odom_valid_ = true each time it fires.
      // The timer checks and then unconditionally clears the flag.
      // That gives a ~1 s window of tolerance.
      if (!odom_valid_.exchange(false)) {
        std::lock_guard<std::mutex> lk(odom_mtx_);
        latest_odom_.reset();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Odom stale — FrameData will carry odom_valid=false");
      }
    });

  gps_staleness_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      if (!gps_valid_.exchange(false)) {
        std::lock_guard<std::mutex> lk(gps_mtx_);
        latest_gps_.reset();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "GPS stale — FrameData will carry gps_valid=false");
      }
    });

  openDevice();
  startStreaming();

  running_        = true;
  capture_thread_ = std::thread(&CameraCapture::captureThread, this);

  RCLCPP_INFO(get_logger(), "camera_capture ready → %s", topic.c_str());
}

CameraCapture::~CameraCapture()
{
  running_ = false;
  if (capture_thread_.joinable())
    capture_thread_.join();
  stopStreaming();
}

void CameraCapture::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  const auto & v = msg->twist.twist.linear;

  OdomSnapshot snap;
  snap.pos_x  = p.x;  snap.pos_y  = p.y;  snap.pos_z  = p.z;
  snap.quat_x = q.x;  snap.quat_y = q.y;  snap.quat_z = q.z;  snap.quat_w = q.w;
  snap.vel_x  = v.x;  snap.vel_y  = v.y;  snap.vel_z  = v.z;

  {
    std::lock_guard<std::mutex> lk(odom_mtx_);
    latest_odom_ = snap;
  }
  odom_valid_.store(true);   // tell the staleness timer we got fresh data
}

void CameraCapture::gpsCallback(const mavros_msgs::msg::GPSRAW::SharedPtr msg)
{
  GpsSnapshot snap;
  snap.lat = msg->lat;
  snap.lon = msg->lon;

  {
    std::lock_guard<std::mutex> lk(gps_mtx_);
    latest_gps_ = snap;
  }
  gps_valid_.store(true);
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
      if (errno == EAGAIN) continue;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,  // max once per 5000ms
        "VIDIOC_DQBUF error: %s", std::strerror(errno));
      continue;
    }

    auto msg = std::make_unique<drone_msgs::msg::FrameData>();

    // ── Image fields ────────────────────────────────────────
    msg->image.header.stamp    = now();
    msg->image.header.frame_id = frame_id;
    msg->image.format          = "jpeg";
    const auto * src = static_cast<const uint8_t *>(g_buffers[buf.index].start);
    msg->image.data.assign(src, src + buf.bytesused);

    // ── Odom fields ─────────────────────────────────────────
    {
      std::lock_guard<std::mutex> lk(odom_mtx_);
      if (latest_odom_.has_value()) {
        const auto & o  = *latest_odom_;
        msg->pos_x      = o.pos_x;  msg->pos_y  = o.pos_y;  msg->pos_z  = o.pos_z;
        msg->quat_x     = o.quat_x; msg->quat_y = o.quat_y;
        msg->quat_z     = o.quat_z; msg->quat_w = o.quat_w;
        msg->vel_x      = o.vel_x;  msg->vel_y  = o.vel_y;  msg->vel_z  = o.vel_z;
        msg->odom_valid = true;
      } else {
        // Dummy: identity quaternion, everything else zero
        msg->quat_w     = 1.0;
        msg->odom_valid = false;
      }
    }

    // ── GPS fields ──────────────────────────────────────────
    {
      std::lock_guard<std::mutex> lk(gps_mtx_);
      if (latest_gps_.has_value()) {
        msg->lat       = latest_gps_->lat;
        msg->lon       = latest_gps_->lon;
        msg->gps_valid = true;
      } else {
        msg->lat       = 0;
        msg->lon       = 0;
        msg->gps_valid = false;
      }
    }

    frame_pub_->publish(std::move(msg));

    // ── Re-enqueue the buffer immediately ───────────────────
    if (xioctl(fd_, VIDIOC_QBUF, &buf) == -1)
      RCLCPP_WARN(get_logger(), "VIDIOC_QBUF error: %s", std::strerror(errno));
  }
}

}  // namespace drone_pipeline

// ─────────────────────────────────────────────────────────────

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_pipeline::CameraCapture)