#include "mavros_gate/command_gate.hpp"


void CommandGateNode::onTeleopCommand(const teleop_msgs::msg::TeleopCommand::SharedPtr msg) {
  if (inInitializationPhase()) return;

  // Take a snapshot of the current state 
  InternalState current_state = snapshotState();
  const char* command_name = commandName(msg->command_id);

  if (!current_state.connected) {
    RCLCPP_WARN(get_logger(), "MAVROS not connected to FCU. Command rejected: id=%d (%s)", msg->command_id, command_name);
    return;
  }

  // Find the handler from given command id
  auto it = cmd_handlers_.find(msg->command_id);
  if (it == cmd_handlers_.end()) {
    RCLCPP_WARN(get_logger(), "No handler for command_id=%d (%s)", msg->command_id, command_name);
    return;
  }

  // execute command
  CommandGateNode::CommandResult result = it->second(*msg, current_state);
  if (result.success) {
    RCLCPP_INFO(get_logger(), "Command requested: {%s}", result.text.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Command failed: {%s}", result.text.c_str());
  }

  InternalStateUpdate update;
  update.last_command_id = msg->command_id;
  updateInternalStateAtomic(update);
}


// Sends incoming velocity command to FCU
void CommandGateNode::onTeleopAction(const teleop_msgs::msg::TeleopAction::SharedPtr msg) {
    if (inInitializationPhase()) return;

    bool hover_cmd = msg->hover;
    
    if (hover_cmd) {
      //RCLCPP_INFO(get_logger(), "HOVER");
    }

    // Take a snapshot of current state
    InternalState current_state = snapshotState();
    if (current_state.connected == false) {
      RCLCPP_WARN(get_logger(), "Action failed: Vehicle not connected to FCU.");
      return;
    } else if (current_state.armed == false) {
      //RCLCPP_WARN(get_logger(), "Action failed: Motors not armed.");
      return;
    }


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
}

// Updates internal state from /mavros/state
void CommandGateNode::onMavrosState(const mavros_msgs::msg::State::SharedPtr msg) {
    if (inInitializationPhase()) return;

    InternalStateUpdate update;
    update.connected = msg->connected;
    update.armed = msg->armed;
    update.guided = msg->guided;
    updateInternalStateAtomic(update);
}
