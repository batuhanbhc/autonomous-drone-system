#ifndef MAVROS_GATE__MCU_BRIDGE_HPP_
#define MAVROS_GATE__MCU_BRIDGE_HPP_

#include <cstdint>
#include <string>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>


// ---------------------------------------------------------------------------
// Packet format (little-endian, packed):
//
//  [SYNC0=0xA5][SYNC1=0x5A][MSG_ID][PAYLOAD_LEN][SEQ][PAYLOAD…][CRC16_LO][CRC16_HI]
//
//  MSG_ID 0x01 – Vertical estimate
//    uint32_t  timestamp_ms
//    float     z_world_m
//    float     vz_world_mps
//    float     agl_m
//    uint8_t   ekf_initialized
//    uint8_t   lidar_accepted
// ---------------------------------------------------------------------------

class McuBridgeNode : public rclcpp::Node
{
public:
  explicit McuBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~McuBridgeNode() override;

private:
  // ── packet constants ────────────────────────────────────────────────────
  static constexpr uint8_t SYNC0               = 0xA5;
  static constexpr uint8_t SYNC1               = 0x5A;
  static constexpr uint8_t MSG_VERTICAL        = 0x01;
  static constexpr uint8_t EXPECTED_PAYLOAD_LEN = 18;

#pragma pack(push, 1)
  struct VerticalEstimatePayload
  {
    uint32_t timestamp_ms;
    float    z_world_m;
    float    vz_world_mps;
    float    agl_m;
    uint8_t  ekf_initialized;
    uint8_t  lidar_accepted;
  };
#pragma pack(pop)

  // ── helpers ─────────────────────────────────────────────────────────────
  bool loadConfig();
  bool openSerial(const std::string & port);
  void serialReadLoop();

  static uint16_t crc16CcittFalse(const uint8_t * data, size_t len);
  bool            readExact(uint8_t * buf, size_t len);
  bool            findSync();

  // ── state ───────────────────────────────────────────────────────────────
  int         serial_fd_{-1};
  std::string serial_port_;
  std::string pub_topic_;

  std::thread         read_thread_;
  std::atomic<bool>   running_{false};

  // ── publisher ───────────────────────────────────────────────────────────
  //
  // We publish on geometry_msgs/Vector3Stamped (built-in, no custom msg needed):
  //   vector.x  →  z_world_m      (height above origin, m)
  //   vector.y  →  vz_world_mps   (vertical velocity,   m/s)
  //   vector.z  →  agl_m          (above-ground-level,  m)
  //
  // The header stamp carries the ROS wall-clock time of reception.
  // ekf_initialized and lidar_accepted are logged at DEBUG level only;
  // add a custom message if you need them on the wire.
  //
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vertical_;
};

#endif  // MAVROS_GATE__MCU_BRIDGE_HPP_