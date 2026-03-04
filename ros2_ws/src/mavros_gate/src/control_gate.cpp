#include "mavros_gate/control_gate.hpp"

#include <rclcpp/qos.hpp>
#include <functional>
#include <stdexcept>


ControlGateNode::ControlGateNode(): rclcpp::Node("control_gate") {
  using DroneInfo = drone_msgs::msg::DroneInfo;

  
  RCLCPP_INFO(get_logger(), "control_gate started");
  initialization_phase_ = true;

  if (!loadConfig()) {
    RCLCPP_FATAL(get_logger(), "Could not retrieve config contents. Shutting down.");
    rclcpp::shutdown();
    throw std::runtime_error("control_gate init failed");
  }

  // --- Drone ID (env) ---
  int drone_id = 0;
  if (const char* env = std::getenv("DRONE_ID")) {
    try {
      drone_id = std::stoi(env);
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Invalid DRONE_ID='%s', defaulting to 0.", env);
      drone_id = 0;
    }
  }

  const std::string base_ns = "/drone_" + std::to_string(drone_id);

  // QoS profiles
  const auto qos_command = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto qos_action = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  auto qos_setpoint_local = rclcpp::SensorDataQoS();
  const auto qos_state_pub = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();


  // --- Subscriptions ---
  sub_teleop_command_ = this->create_subscription<drone_msgs::msg::TeleopCommand>(
    topics_.manual_command, qos_command,
    std::bind(&ControlGateNode::onTeleopCommand, this, std::placeholders::_1));

  sub_teleop_action_ = this->create_subscription<drone_msgs::msg::TeleopAction>(
    topics_.manual_action, qos_action,
    std::bind(&ControlGateNode::onTeleopAction, this, std::placeholders::_1));

  sub_mavros_state_ = this->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", qos_command,
    std::bind(&ControlGateNode::onMavrosState, this, std::placeholders::_1));

  // --- Publishers ---
  pub_setpoint_raw_local_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
    "/mavros/setpoint_raw/local", qos_setpoint_local);

  pub_control_state_ = this->create_publisher<drone_msgs::msg::ControlState>(
    base_ns + "/state", qos_state_pub);

  pub_drone_info_ = this->create_publisher<drone_msgs::msg::DroneInfo>(
    base_ns + "/info", qos_command);

    
  if (!initializationRoutine()) {
    publishInfo(DroneInfo::LEVEL_ERROR, "Non-fixable error occured during initialization routine.");
    RCLCPP_FATAL(get_logger(), "Non-fixable error occured during initialization routine.");
    rclcpp::shutdown();
    throw std::runtime_error("control_gate init failed");
  } 

  // called mainly for logging whether setpoint commands can be sent or not
  const InternalState st0 = snapshotState();
  updateSetpointBlockStateAndMaybePublish(isSetpointBlocked(st0), true);

  // 1 Hz state publisher timer (best-effort)
  state_pub_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&ControlGateNode::onPublishStateTimer, this));
      
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