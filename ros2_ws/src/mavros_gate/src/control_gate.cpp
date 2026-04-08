#include "mavros_gate/control_gate.hpp"

#include <rclcpp/qos.hpp>
#include <functional>
#include <stdexcept>


ControlGateNode::ControlGateNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("control_gate", options)
{
  RCLCPP_INFO(get_logger(), "control_gate started");
  initialization_phase_ = true;

  if (!loadConfig()) {
    RCLCPP_FATAL(get_logger(), "Could not retrieve config contents. Shutting down.");
    rclcpp::shutdown();
    throw std::runtime_error("control_gate init failed");
  }

  // -----------------------------------
  // --- Drone ID (ros param) ---
  this->declare_parameter<int>("drone_id", 0);
  int drone_id = 0;
  this->get_parameter("drone_id", drone_id);

  const std::string base_ns = "/drone_" + std::to_string(drone_id);
  base_ns_ = base_ns;

  // prepend namespace to all topic paths
  topics_.manual_command    = base_ns + topics_.manual_command;
  topics_.manual_action     = base_ns + topics_.manual_action;
  topics_.autonomous_action = base_ns + topics_.autonomous_action;
  topics_.mcu_bridge        = base_ns + topics_.mcu_bridge;
  topics_.alt_ctrl_input    = base_ns + topics_.alt_ctrl_input;
  topics_.alt_ctrl_output   = base_ns + topics_.alt_ctrl_output;

  const std::string topic_mavros_state   = base_ns + "/mavros/state";
  const std::string topic_setpoint_local = base_ns + "/mavros/setpoint_raw/local";
  const std::string topic_cmd_gate_state = base_ns + "/cmd_gate/state";
  const std::string topic_cmd_gate_info  = base_ns + "/cmd_gate/info";
  const std::string topic_gcs_heartbeat  = base_ns + "/gcs/heartbeat";

  // -----------------------------------
  // QoS profiles
  const auto qos_command      = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto qos_action       = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  const auto qos_sensor_data  = rclcpp::SensorDataQoS();
  const auto qos_state_pub    = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  const auto qos_info_latched = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
  const auto qos_best_effort  = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  const auto qos_reliable     = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

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
    topic_gcs_heartbeat, qos_sensor_data,
    std::bind(&ControlGateNode::onGcsHeartbeat, this, std::placeholders::_1));

  // control_gate still subscribes to mcu_bridge to cache vertical estimates
  // (needed for hover-target snapshots, takeoff monitor, etc.)
  // It does NOT forward these to altitude_controller anymore.
  sub_vertical_estimate_ = this->create_subscription<VerticalEstimate>(
    topics_.mcu_bridge, qos_best_effort,
    std::bind(&ControlGateNode::onVerticalEstimate, this, std::placeholders::_1));

  // Altitude controller output — sensor QoS (best-effort, keep-last 1)
  sub_alt_ctrl_output_ = this->create_subscription<AltCtrlOutput>(
    topics_.alt_ctrl_output, qos_sensor_data,
    std::bind(&ControlGateNode::onAltCtrlOutput, this, std::placeholders::_1));

  // -----------------------------------
  // --- Publishers ---
  pub_setpoint_raw_local_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
    topic_setpoint_local, qos_sensor_data);
  pub_control_state_ = this->create_publisher<DroneState>(
    topic_cmd_gate_state, qos_state_pub);
  pub_drone_info_ = this->create_publisher<DroneInfo>(
    topic_cmd_gate_info, qos_info_latched);

  // Altitude controller input — reliable (commands must arrive)
  pub_alt_ctrl_input_ = this->create_publisher<AltCtrlInput>(
    topics_.alt_ctrl_input, qos_reliable);

  // -----------------------------------
  if (!initializationRoutine()) {
    publishInfo(DroneInfo::LEVEL_ERROR, "Non-fixable error occured during initialization routine.");
    RCLCPP_FATAL(get_logger(), "Non-fixable error occured during initialization routine.");
    rclcpp::shutdown();
    throw std::runtime_error("control_gate init failed");
  }

  // -----------------------------------
  const InternalState st0 = snapshotState();
  updateSetpointBlockStateAndMaybePublish(isSetpointBlocked(st0), true);

  // 1 Hz state publisher
  state_pub_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&ControlGateNode::onPublishStateTimer, this));

  // GCS failsafe watchdog
  gcs_watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>((1.0 / failsafe_watchdog_hz_) * 1000)),
    std::bind(&ControlGateNode::onFailsafeWatchdog, this));

  // Guided timeout watchdog (default 10 Hz, configurable)
  guided_timeout_watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(
      static_cast<int64_t>((1.0 / guided_timeout_watchdog_hz_) * 1000)),
    std::bind(&ControlGateNode::onGuidedTimeoutWatchdog, this));

  // Takeoff monitor timer — created here but immediately cancelled;
  // it is re-armed by executeTakeoff() after a successful takeoff service call.
  takeoff_monitor_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),   // 10 Hz check
    std::bind(&ControlGateNode::onTakeoffMonitorTick, this));
  takeoff_monitor_timer_->cancel();   // dormant until takeoff accepted

  // -----------------------------------
  RCLCPP_INFO(get_logger(), "Initialization complete.");
  publishInfo(DroneInfo::LEVEL_INFO, "Initialization complete.");
  initialization_phase_ = false;
}

#ifndef BUILDING_COMPOSITOR
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlGateNode>());
  rclcpp::shutdown();
  return 0;
}
#endif