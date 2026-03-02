#include "mavros_gate/command_gate.hpp"

#include <rclcpp/qos.hpp>
#include <functional>
#include <stdexcept>

CommandGateNode::CommandGateNode(): rclcpp::Node("command_gate") {
  RCLCPP_INFO(get_logger(), "command_gate started");
  initialization_phase_ = true;

  if (!loadTopics()) {
    RCLCPP_FATAL(get_logger(), "Could not retrieve ros2 topics from config. Shutting down.");
    rclcpp::shutdown();
    throw std::runtime_error("command_gate init failed");
  }

  // QoS profiles
  const auto qos_command = rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
  const auto qos_action = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort().durability_volatile();
  const auto qos_state = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // --- Subscriptions ---
  sub_teleop_command_ = this->create_subscription<teleop_msgs::msg::TeleopCommand>(
    topics_.manual_command, qos_command,
    std::bind(&CommandGateNode::onTeleopCommand, this, std::placeholders::_1));

  sub_teleop_action_ = this->create_subscription<teleop_msgs::msg::TeleopAction>(
    topics_.manual_action, qos_action,
    std::bind(&CommandGateNode::onTeleopAction, this, std::placeholders::_1));

  sub_mavros_state_ = this->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", qos_state,
    std::bind(&CommandGateNode::onMavrosState, this, std::placeholders::_1));

  // --- Publishers ---
  pub_setpoint_raw_local_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
    "/mavros/setpoint_raw/local", qos_command);

  if (!initializationRoutine()) {
    RCLCPP_FATAL(get_logger(), "Non-fixable error occured during initialization routine.");
    rclcpp::shutdown();
    throw std::runtime_error("command_gate init failed");

  } else {
    // update initialization phase flag to false, so that workflow starts
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      initialization_phase_ = false;
    }
    RCLCPP_INFO(get_logger(), "Initialization complete");
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandGateNode>());
  rclcpp::shutdown();
  return 0;
}