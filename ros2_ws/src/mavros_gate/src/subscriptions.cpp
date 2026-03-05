#include "mavros_gate/control_gate.hpp"


using DroneInfo = drone_msgs::msg::DroneInfo;
using MavrosExtendedState = mavros_msgs::msg::ExtendedState;

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
      } else if (critical_state_.load(std::memory_order_relaxed)) {
        // Keyboard is off, control state cannot accept this command
        publishInfo(DroneInfo::LEVEL_ERROR, stringf("Rejected (id=%d, %s): System in critical state.", msg->command_id, command_name));
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
    updateLastAct(std::chrono::steady_clock::now());
}

// Updates internal state from /mavros/state
void ControlGateNode::onMavrosState(const mavros_msgs::msg::State::SharedPtr msg) {
    if (inInitializationPhase()) return;
    InternalStateUpdate update;
    update.connected = msg->connected;
    update.armed = msg->armed;
    update.guided = msg->guided;
    updateInternalStateAtomic(update);

    fcu_state_.store(msg->system_status, std::memory_order_relaxed);
}

void ControlGateNode::onGcsHeartbeat(const GcsHeartbeat::SharedPtr msg) {
  const auto now_ns   = this->now().nanoseconds();
  const auto stamp_ns = rclcpp::Time(msg->stamp).nanoseconds();  // simple ctor

  const int64_t diff_ns = (now_ns > stamp_ns) ? (now_ns - stamp_ns) : (stamp_ns - now_ns);

  if (diff_ns > 3'000'000'000LL) {  // 3 seconds in ns
    return;
  }

  time_since_heartbeat_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
}


void ControlGateNode::onFailsafeWatchdog() {
  // --- 1. Critical state check  ---
  const uint8_t fcu = fcu_state_.load(std::memory_order_relaxed);
  if (fcu >= 5u) {  // MAV_STATE_CRITICAL and above
    if (!critical_state_.load(std::memory_order_relaxed)) {
      critical_state_.store(true, std::memory_order_relaxed);
      RCLCPP_ERROR(get_logger(),
        "FCU entered critical state (system_status=%u).", fcu);
      publishInfo(DroneInfo::LEVEL_ERROR,
        stringf("FCU critical state (system_status=%u).", fcu));
    }
  }

  // --- 2. GCS failsafe scope ---
  const int64_t last_ns = time_since_heartbeat_ns_.load(std::memory_order_relaxed);
  const rclcpp::Time last_heartbeat(last_ns, RCL_ROS_TIME);
  const double elapsed_s = (this->now() - last_heartbeat).seconds();
  if (last_ns == -1) return;  // no GCS ever connected, don't trigger failsafe

  if (elapsed_s > gcs_failsafe_s_) {
    if (!gcs_failsafe_.load(std::memory_order_relaxed)) {
      gcs_failsafe_.store(true, std::memory_order_relaxed);
      RCLCPP_ERROR(get_logger(),
        "GCS heartbeat lost for %.1f s (threshold=%.1f s). GCS failsafe triggered.", elapsed_s, gcs_failsafe_s_);
      publishInfo(DroneInfo::LEVEL_ERROR,
        stringf("GCS failsafe: no heartbeat for %.1f s.", elapsed_s));
      publishInfo(DroneInfo::LEVEL_ERROR, "Sending LAND.");

      // send land
      drone_msgs::msg::TeleopCommand dummy_cmd{};
      executeRTL(dummy_cmd, snapshotState());
    }
  } else {
    if (gcs_failsafe_.load(std::memory_order_relaxed)) {
      gcs_failsafe_.store(false, std::memory_order_relaxed);
      RCLCPP_WARN(get_logger(),
        "GCS heartbeat recovered (%.1f s gap). GCS failsafe cleared.", elapsed_s);
      publishInfo(DroneInfo::LEVEL_WARN, "GCS failsafe cleared: heartbeat recovered.");
    }
  }
}