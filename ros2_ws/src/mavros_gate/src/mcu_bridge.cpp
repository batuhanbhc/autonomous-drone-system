#include "mavros_gate/mcu_bridge.hpp"

#include <cerrno>
#include <cstring>
#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/qos.hpp>


// ============================================================================
// constructor / destructor
// ============================================================================

McuBridgeNode::McuBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mcu_bridge", options)
{
  RCLCPP_INFO(get_logger(), "mcu_bridge starting");

  // ── drone namespace ──────────────────────────────────────────────────────
  this->declare_parameter<int>("drone_id", 0);
  int drone_id = 0;
  this->get_parameter("drone_id", drone_id);
  const std::string base_ns = "/drone_" + std::to_string(drone_id);

  // ── serial port ──────────────────────────────────────────────────────────
  this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  this->get_parameter("serial_port", serial_port_);

  // ── config ───────────────────────────────────────────────────────────────
  if (!loadConfig()) {
    RCLCPP_FATAL(get_logger(), "Failed to load config. Shutting down.");
    rclcpp::shutdown();
    throw std::runtime_error("mcu_bridge init failed");
  }

  pub_topic_ = base_ns + pub_topic_;   // prepend namespace

  // ── publisher (best-effort, estimated ~20 Hz) ────────────────────────────
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  pub_vertical_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(pub_topic_, qos);

  RCLCPP_INFO(get_logger(), "Publishing vertical estimates on: %s", pub_topic_.c_str());

  // ── open serial port ──────────────────────────────────────────────────────
  if (!openSerial(serial_port_)) {
    RCLCPP_FATAL(get_logger(), "Could not open serial port '%s'. Shutting down.", serial_port_.c_str());
    rclcpp::shutdown();
    throw std::runtime_error("mcu_bridge init failed");
  }

  // ── start reader thread ───────────────────────────────────────────────────
  running_.store(true, std::memory_order_relaxed);
  read_thread_ = std::thread(&McuBridgeNode::serialReadLoop, this);

  RCLCPP_INFO(get_logger(), "mcu_bridge ready. Serial: %s", serial_port_.c_str());
}

McuBridgeNode::~McuBridgeNode()
{
  running_.store(false, std::memory_order_relaxed);
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}


// ============================================================================
// loadConfig
// ============================================================================

bool McuBridgeNode::loadConfig()
{
  const std::string pkg_share =
    ament_index_cpp::get_package_share_directory("mavros_config");
  const std::string yaml_path = pkg_share + "/config/control_params.yaml";

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load YAML '%s': %s", yaml_path.c_str(), e.what());
    return false;
  }

  try {
    pub_topic_ = root["custom_topics"]["mcu_bridge"].as<std::string>();
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(get_logger(), "YAML missing custom_topics.mcu_bridge: %s", e.what());
    return false;
  }

  return true;
}


// ============================================================================
// openSerial
// ============================================================================

bool McuBridgeNode::openSerial(const std::string & port)
{
  serial_fd_ = ::open(port.c_str(), O_RDONLY | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "open('%s') failed: %s", port.c_str(), std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, B460800);
  cfsetospeed(&tty, B460800);

  tty.c_cflag |=  (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |=  CS8;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN]  = 1;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}


// ============================================================================
// CRC-16/CCITT-FALSE
// ============================================================================

uint16_t McuBridgeNode::crc16CcittFalse(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000)
            ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}


// ============================================================================
// readExact / findSync
// ============================================================================

bool McuBridgeNode::readExact(uint8_t * buf, size_t len)
{
  size_t total = 0;
  while (total < len && running_.load(std::memory_order_relaxed)) {
    const ssize_t n = ::read(serial_fd_, buf + total, len - total);
    if (n < 0) {
      if (errno == EINTR) continue;
      RCLCPP_ERROR(get_logger(), "read error: %s", std::strerror(errno));
      return false;
    }
    if (n == 0) continue;
    total += static_cast<size_t>(n);
  }
  return running_.load(std::memory_order_relaxed);
}

bool McuBridgeNode::findSync()
{
  uint8_t b = 0;
  while (running_.load(std::memory_order_relaxed)) {
    const ssize_t n = ::read(serial_fd_, &b, 1);
    if (n < 0) {
      if (errno == EINTR) continue;
      RCLCPP_ERROR(get_logger(), "read error while syncing: %s", std::strerror(errno));
      return false;
    }
    if (n == 0 || b != SYNC0) continue;

    uint8_t b2 = 0;
    if (!readExact(&b2, 1)) return false;
    if (b2 == SYNC1) return true;
  }
  return false;
}


// ============================================================================
// serialReadLoop  (runs in dedicated thread)
// ============================================================================

void McuBridgeNode::serialReadLoop()
{
  RCLCPP_INFO(get_logger(), "Serial read loop started.");

  while (running_.load(std::memory_order_relaxed)) {

    if (!findSync()) break;

    // ── read header: [MSG_ID][PAYLOAD_LEN][SEQ] ───────────────────────────
    uint8_t header[3]{};
    if (!readExact(header, sizeof(header))) break;

    const uint8_t msgId      = header[0];
    const uint8_t payloadLen = header[1];
    const uint8_t seq        = header[2];

    // ── skip unknown / wrong-length messages ─────────────────────────────
    if (msgId != MSG_VERTICAL || payloadLen != EXPECTED_PAYLOAD_LEN) {
      const size_t skip_n = static_cast<size_t>(payloadLen) + 2;  // payload + 2 CRC bytes
      std::vector<uint8_t> skip(skip_n);
      if (!readExact(skip.data(), skip_n)) break;
      if (msgId != MSG_VERTICAL) {
        RCLCPP_WARN(get_logger(), "Skipped unknown msg_id=0x%02X", msgId);
      } else {
        RCLCPP_WARN(get_logger(), "Skipped packet: unexpected payload_len=%u", payloadLen);
      }
      continue;
    }

    // ── payload ───────────────────────────────────────────────────────────
    uint8_t payloadBytes[EXPECTED_PAYLOAD_LEN]{};
    if (!readExact(payloadBytes, EXPECTED_PAYLOAD_LEN)) break;

    // ── CRC ───────────────────────────────────────────────────────────────
    uint8_t crcBytes[2]{};
    if (!readExact(crcBytes, 2)) break;

    const uint16_t crcRx =
      static_cast<uint16_t>(crcBytes[0]) |
      (static_cast<uint16_t>(crcBytes[1]) << 8);

    uint8_t crcBuf[3 + EXPECTED_PAYLOAD_LEN]{};
    crcBuf[0] = msgId;
    crcBuf[1] = payloadLen;
    crcBuf[2] = seq;
    std::memcpy(crcBuf + 3, payloadBytes, EXPECTED_PAYLOAD_LEN);

    const uint16_t crcCalc = crc16CcittFalse(crcBuf, sizeof(crcBuf));
    if (crcCalc != crcRx) {
      RCLCPP_WARN(get_logger(), "CRC mismatch: rx=0x%04X calc=0x%04X — dropping packet",
                  crcRx, crcCalc);
      continue;
    }

    // ── deserialise ───────────────────────────────────────────────────────
    VerticalEstimatePayload p{};
    std::memcpy(&p, payloadBytes, sizeof(p));

    RCLCPP_DEBUG(get_logger(),
      "seq=%u t_ms=%u z=%.3f vz=%.3f agl=%.3f init=%u lidar=%u",
      seq, p.timestamp_ms, p.z_world_m, p.vz_world_mps, p.agl_m,
      p.ekf_initialized, p.lidar_accepted);

    // ── publish ───────────────────────────────────────────────────────────
    //   x = z_world_m  (height above origin)
    //   y = vz_world_mps (vertical velocity)
    //   z = agl_m      (above ground level)
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp    = this->now();
    msg.header.frame_id = "world";
    msg.vector.x = static_cast<double>(p.z_world_m);
    msg.vector.y = static_cast<double>(p.vz_world_mps);
    msg.vector.z = static_cast<double>(p.agl_m);

    pub_vertical_->publish(msg);
  }

  RCLCPP_INFO(get_logger(), "Serial read loop exited.");
}


// ============================================================================
// main
// ============================================================================

#ifndef BUILDING_COMPOSITOR
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<McuBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
#endif