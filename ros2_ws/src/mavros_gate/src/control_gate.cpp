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

  // QoS profiles
  const auto qos_command = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto qos_action = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  const auto qos_state = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto qos_setpoint_local = rclcpp::SensorDataQoS();
  
  // --- Subscriptions ---
  sub_teleop_command_ = this->create_subscription<drone_msgs::msg::TeleopCommand>(
    topics_.manual_command, qos_command,
    std::bind(&ControlGateNode::onTeleopCommand, this, std::placeholders::_1));

  sub_teleop_action_ = this->create_subscription<drone_msgs::msg::TeleopAction>(
    topics_.manual_action, qos_action,
    std::bind(&ControlGateNode::onTeleopAction, this, std::placeholders::_1));

  sub_mavros_state_ = this->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", qos_state,
    std::bind(&ControlGateNode::onMavrosState, this, std::placeholders::_1));

  // --- Publishers ---
  pub_setpoint_raw_local_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
    "/mavros/setpoint_raw/local", qos_setpoint_local);

  if (!initializationRoutine()) {
    RCLCPP_FATAL(get_logger(), "Non-fixable error occured during initialization routine.");
    rclcpp::shutdown();
    throw std::runtime_error("control_gate init failed");

  } else {
    // update initialization phase flag to false, so that workflow starts
    RCLCPP_INFO(get_logger(), "Initialization complete");
    initialization_phase_ = false;
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlGateNode>());
  rclcpp::shutdown();
  return 0;
}