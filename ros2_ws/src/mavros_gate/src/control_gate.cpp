#include "mavros_gate/control_gate.hpp"

#include <rclcpp/qos.hpp>
#include <functional>
#include <stdexcept>


ControlGateNode::ControlGateNode(): rclcpp::Node("control_gate") {
  RCLCPP_INFO(get_logger(), "control_gate started");
  initialization_phase_ = true;

  if (!loadConfig()) {
    RCLCPP_FATAL(get_logger(), "Could not retrieve config contents. Shutting down.");
    rclcpp::shutdown();
    throw std::runtime_error("control_gate init failed");
  }

  // -----------------------------------
  // --- Drone ID (ros param) ---
  int drone_id = 0;

  // declare + read
  this->declare_parameter<int>("drone_id", 0);
  this->get_parameter("drone_id", drone_id);

  const std::string base_ns = "/drone_" + std::to_string(drone_id);
  base_ns_ = base_ns;

  // topic paths
  topics_.manual_command = base_ns + topics_.manual_command;
  topics_.manual_action = base_ns + topics_.manual_action;
  topics_.autonomous_action = base_ns + topics_.autonomous_action;
  std::string topic_mavros_state = base_ns + std::string("/mavros/state");
  std::string topic_setpoint_local = base_ns + std::string("/mavros/setpoint_raw/local");
  std::string topic_cmd_gate_state = base_ns + std::string("/cmd_gate/state");
  std::string topic_cmd_gate_info = base_ns + std::string("/cmd_gate/info");
  std::string topic_gcs_heartbeat = base_ns + std::string("/gcs/heartbeat");
  std::string topic_mavros_extended_state = base_ns + std::string("/mavros/extended_state");

  // -----------------------------------
  // QoS profiles
  const auto qos_command = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto qos_action = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  auto qos_sensor_data = rclcpp::SensorDataQoS();
  const auto qos_state_pub = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  const auto qos_info_latched =rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  // -----------------------------------
  // --- Subscriptions ---
  sub_teleop_command_ = this->create_subscription<TeleopCmd>(
    topics_.manual_command, qos_command,
    std::bind(&ControlGateNode::onTeleopCommand, this, std::placeholders::_1));

  sub_teleop_action_ = this->create_subscription<TeleopAct>(
    topics_.manual_action, qos_action,
    std::bind(&ControlGateNode::onTeleopAction, this, std::placeholders::_1));

  sub_mavros_state_ = this->create_subscription<mavros_msgs::msg::State>(
    topic_mavros_state, qos_command,
    std::bind(&ControlGateNode::onMavrosState, this, std::placeholders::_1));

  sub_gcs_heartbeat_ = this->create_subscription<GcsHeartbeat>(
    topic_gcs_heartbeat,
    qos_sensor_data,
    std::bind(&ControlGateNode::onGcsHeartbeat, this, std::placeholders::_1));

  // -----------------------------------
  // --- Publishers ---
  pub_setpoint_raw_local_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(topic_setpoint_local, qos_sensor_data);
  pub_control_state_ = this->create_publisher<DroneState>(topic_cmd_gate_state, qos_state_pub);
  pub_drone_info_ = this->create_publisher<DroneInfo>(topic_cmd_gate_info, qos_info_latched);

  // -----------------------------------
  if (!initializationRoutine()) {
    publishInfo(DroneInfo::LEVEL_ERROR, "Non-fixable error occured during initialization routine.");
    RCLCPP_FATAL(get_logger(), "Non-fixable error occured during initialization routine.");
    rclcpp::shutdown();
    throw std::runtime_error("control_gate init failed");
  } 
  
  // -----------------------------------
  // called mainly for logging whether setpoint commands can be sent or not
  const InternalState st0 = snapshotState();
  updateSetpointBlockStateAndMaybePublish(isSetpointBlocked(st0), true);

  // 1 Hz state publisher timer (best-effort)
  state_pub_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&ControlGateNode::onPublishStateTimer, this));

  // 0.5 Hz GCS watchdog timer
  gcs_watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>((1.0 / failsafe_watchdog_hz_) * 1000)),
    std::bind(&ControlGateNode::onFailsafeWatchdog, this));
      
  // update initialization phase flag to false, so that workflow starts
  RCLCPP_INFO(get_logger(), "Initialization complete.");
  publishInfo(DroneInfo::LEVEL_INFO, "Initialization complete.");
  initialization_phase_ = false;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlGateNode>());
  rclcpp::shutdown();
  return 0;
}