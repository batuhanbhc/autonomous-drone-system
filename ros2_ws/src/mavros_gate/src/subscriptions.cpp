#include "mavros_gate/control_gate.hpp"

using DroneInfo = drone_msgs::msg::DroneInfo;


void ControlGateNode::onTeleopCommand(const drone_msgs::msg::TeleopCommand::SharedPtr msg) {
  if (inInitializationPhase()) return;

  // Take a snapshot of the current state 
  InternalState current_state = snapshotState();
  const char* command_name = commandName(msg->command_id);


  if (!current_state.connected) {
    //RCLCPP_WARN(get_logger(), "Rejected (id=%d, %s): Mavlink connection failed.", msg->command_id, command_name);
    publishInfo(DroneInfo::LEVEL_WARN, stringf("Rejected (id=%d, %s): Mavlink connection failed.", msg->command_id, command_name));
    return;
  } else if (current_state.system_killed) {
    //RCLCPP_ERROR(get_logger(), "System is killed, power cycle is required.");
    publishInfo(DroneInfo::LEVEL_ERROR, "System killed, power cycle required.");
    return;
  }

  if (isCommandAlwaysEnabled(msg->command_id) == false) {
      // Command is not always enabled. Check other criteria to decide whether it should be executed.

      if (current_state.control_mode == ControlMode::Auto) {
        // Current mode is autonomous, discard command
        //RCLCPP_WARN(get_logger(), "Rejected (id=%d, %s): Command not permitted in AUTO mode.", msg->command_id, command_name);
        publishInfo(DroneInfo::LEVEL_WARN, stringf("Rejected (id=%d, %s): Command not permitted in AUTO mode.", msg->command_id, command_name));
        return;
      } else if (current_state.keyboard_on == false) {
        // Keyboard is off, control state cannot accept this command
        publishInfo(DroneInfo::LEVEL_WARN, stringf("Rejected (id=%d, %s): Keyboard off.", msg->command_id, command_name));
        //RCLCPP_WARN(get_logger(), "Rejected (id=%d, %s): Keyboard is off.", msg->command_id, command_name);
        return;
      }
  }

  // Find the handler from given command id
  auto it = cmd_handlers_.find(msg->command_id);
  if (it == cmd_handlers_.end()) {
    RCLCPP_WARN(get_logger(), "Requested (id=%d, %s): No handler found.", msg->command_id, command_name);
    return;
  }

  // execute command
  ControlGateNode::CommandResult result = it->second(*msg, current_state);
  if (result.success) {
    RCLCPP_INFO(get_logger(), "Requested (id=%d, %s): %s", msg->command_id, command_name, result.text.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Failed (id=%d, %s): %s", msg->command_id, command_name, result.text.c_str());
  }
}


// Sends incoming velocity command to FCU
void ControlGateNode::onTeleopAction(const drone_msgs::msg::TeleopAction::SharedPtr msg) {
    if (inInitializationPhase()) return;

    // Take a snapshot of current state
    const InternalState current_state = snapshotState();

    // Update transition state (publishes only when blocked/enabled flips)
    const bool blocked = isSetpointBlocked(current_state);
    updateSetpointBlockStateAndMaybePublish(blocked, false);

    if (blocked) return;
    
    mavros_msgs::msg::PositionTarget sp;
    sp.header.stamp = msg->stamp;
    sp.header.frame_id = "base_link";
    sp.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;

    sp.type_mask =
    mavros_msgs::msg::PositionTarget::IGNORE_PX |
    mavros_msgs::msg::PositionTarget::IGNORE_PY |
    mavros_msgs::msg::PositionTarget::IGNORE_PZ |
    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
    mavros_msgs::msg::PositionTarget::IGNORE_YAW;

    sp.velocity.x = msg->vx;
    sp.velocity.y = msg->vy;
    sp.velocity.z = msg->vz;
    sp.yaw_rate = msg->yaw_rate;

    pub_setpoint_raw_local_->publish(sp);

    // update last action time
    InternalStateUpdate update;
    update.last_action_t = std::chrono::steady_clock::now();
    updateInternalStateAtomic(update);
}

// Updates internal state from /mavros/state
void ControlGateNode::onMavrosState(const mavros_msgs::msg::State::SharedPtr msg) {
    if (inInitializationPhase()) return;
    InternalStateUpdate update;
    update.connected = msg->connected;
    update.armed = msg->armed;
    update.guided = msg->guided;
    updateInternalStateAtomic(update);
}
