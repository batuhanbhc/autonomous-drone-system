#include "mavros_gate/control_gate.hpp"


void ControlGateNode::onTeleopCommand(const drone_msgs::msg::TeleopCommand::SharedPtr msg) {
  if (inInitializationPhase()) return;

  // Take a snapshot of the current state 
  InternalState current_state = snapshotState();
  const char* command_name = commandName(msg->command_id);

  // Check mavlink connection
  if (!current_state.connected) {
    RCLCPP_WARN(get_logger(), "Rejected (id=%d, %s): Mavlink connection failed.", msg->command_id, command_name);
    return;
  }

  // Find the handler from given command id
  auto it = cmd_handlers_.find(msg->command_id);
  if (it == cmd_handlers_.end()) {
    RCLCPP_WARN(get_logger(), "Requested (id=%d, %s): No handler found.", msg->command_id, command_name);
    return;
  }

  // Check whether system is alive
  if (current_state.system_killed) {
    RCLCPP_ERROR(get_logger(), "System is killed, reboot is required.");
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
    InternalState current_state = snapshotState();
    bool send_action = true;

    // The only difference below is that for hover command, we do not print logs unnecessarily
    if (msg->command_id == drone_msgs::msg::TeleopCommand::HOVER) {
      if (current_state.connected == false || current_state.system_killed || 
            current_state.armed == false || current_state.control_mode == ControlMode::Auto) {
          send_action = false;
      }
    } else {
      if (current_state.control_mode == ControlMode::Auto) {
          RCLCPP_WARN(get_logger(), "Control is in AUTONOMOUS mode, velocity setpoint ignored.");
          send_action = false;
      } else if (current_state.connected == false) {
          RCLCPP_WARN(get_logger(), "Rejected (id=%d, VEL_YAW): Mavlink connection failed.", msg->command_id);
          send_action = false;
      } else if (current_state.system_killed) {
          RCLCPP_ERROR(get_logger(), "System is killed, reboot is required.");
          send_action = false;
      } else if (current_state.armed == false) {
          RCLCPP_WARN(get_logger(), "Action failed: Motors not armed.");
          send_action = false;
      }
    }

    if (send_action == false) return;
    
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
