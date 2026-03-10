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
  // 1) Check is heartbeat is new
  const auto now_ns   = this->now().nanoseconds();
  const auto stamp_ns = rclcpp::Time(msg->stamp).nanoseconds();  // simple ctor
  const int64_t diff_ns = (now_ns > stamp_ns) ? (now_ns - stamp_ns) : (stamp_ns - now_ns);

  if (diff_ns > 3'000'000'000LL) {  // 3 seconds, heartbeat is old
    return;
  }

  // 2) Now we know heartbeat is new. Check if connected to any gcs
  if (!gcs_connected_.load(std::memory_order_relaxed)) {
    // Not connected to any GCS
    RCLCPP_INFO(get_logger(), "New GCS heartbeat received. GCS ID: %d", msg->gcs_id);
    publishInfo(DroneInfo::LEVEL_INFO, stringf("New GCS heartbeat received. GCS ID: %d", msg->gcs_id));
    gcs_id_.store(msg->gcs_id, std::memory_order_relaxed);
    gcs_connected_.store(true, std::memory_order_relaxed);
    time_since_heartbeat_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
    return;
  }

  // 3) Already connected to a GCS. Check if HB is from connected GCS
  if (gcs_id_.load(std::memory_order_relaxed) != msg->gcs_id) {
    // From different GCS
    RCLCPP_WARN(get_logger(), "Heartbeat from unknown GCS (id=%d), ignoring.", msg->gcs_id);
    return;
  }

  // 4) HB is from connected GCS
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
  // If no GCS connection & no failsafe, return.
  bool is_there_failsafe = gcs_failsafe_.load(std::memory_order_relaxed);
  bool is_there_conn = gcs_connected_.load(std::memory_order_relaxed);
  if ((is_there_conn == false) && (is_there_failsafe == false)) {
    return;
  } 

  // Either GCS connection exists, or there is failsafe
  // Check if current state triggers GCS failsafe
  const int64_t last_ns = time_since_heartbeat_ns_.load(std::memory_order_relaxed);
  const rclcpp::Time last_heartbeat(last_ns, RCL_ROS_TIME);
  const double elapsed_s = (this->now() - last_heartbeat).seconds();
  bool gcs_triggered = elapsed_s > gcs_failsafe_s_ ? true: false;
  
  if (gcs_triggered) {
    // GCS failsafe triggered.

    // Check if this is new
    if (is_there_failsafe == false) {
      // This is new, log it and set gcs failsafe to true
      RCLCPP_ERROR(get_logger(),
        "GCS heartbeat lost for %.1f s. GCS failsafe triggered.", elapsed_s);
      publishInfo(DroneInfo::LEVEL_ERROR,
        stringf("GCS failsafe: No heartbeat for %.1f s.", elapsed_s));
      
      // reset previous gcs info and set GCS failsafe on
      gcs_connected_.store(false, std::memory_order_relaxed);
      gcs_id_.store(-1, std::memory_order_relaxed);
      gcs_failsafe_.store(true, std::memory_order_relaxed);

      const InternalState st = snapshotState();
      drone_msgs::msg::TeleopCommand dummy_cmd{};
      if (!st.armed) {
          // nothing to do
      } else if (st.guided) {
          // change to RTL in future if drone is capable of flying normally
          // armed and guided, return to launch
          executeLand(dummy_cmd, st);
          publishInfo(DroneInfo::LEVEL_ERROR, "Sending LAND.");
      } else {
          // armed but not guided, possibly another failsafe, land directly
          executeLand(dummy_cmd, st);
          publishInfo(DroneInfo::LEVEL_ERROR, "Sending LAND.");
      }
    }
  } else {
    // Current state does not require failsafe
    // Clear GCS failsafe state if it is on
    if (is_there_failsafe) {
      gcs_failsafe_.store(false, std::memory_order_relaxed);
      RCLCPP_INFO(get_logger(), "GCS failsafe cleared.");
      publishInfo(DroneInfo::LEVEL_INFO, "GCS failsafe cleared: Heartbeat received.");
    }
  }
}