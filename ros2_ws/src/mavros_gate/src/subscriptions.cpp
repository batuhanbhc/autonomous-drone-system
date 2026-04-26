#include "mavros_gate/control_gate.hpp"


using DroneInfo = drone_msgs::msg::DroneInfo;


// ============================================================================
// onTeleopCommand
// ============================================================================

void ControlGateNode::onTeleopCommand(const drone_msgs::msg::TeleopCommand::SharedPtr msg) {
  if (inInitializationPhase()) return;

  const InternalState current_state = snapshotState();
  const char* command_name = commandName(msg->command_id);

  if (!current_state.connected) {
    publishInfo(DroneInfo::LEVEL_WARN,
      stringf("Rejected (id=%d, %s): Mavlink connection failed.", msg->command_id, command_name));
    return;
  } else if (current_state.system_killed) {
    publishInfo(DroneInfo::LEVEL_ERROR, "System killed, power cycle required.");
    return;
  }

  if (!isCommandAlwaysEnabled(msg->command_id)) {
    if (current_state.control_mode == ControlMode::Auto) {
      publishInfo(DroneInfo::LEVEL_WARN,
        stringf("Rejected (id=%d, %s): Command not permitted in AUTO mode.",
                msg->command_id, command_name));
      return;
    } else if (!current_state.keyboard_on) {
      publishInfo(DroneInfo::LEVEL_WARN,
        stringf("Rejected (id=%d, %s): Keyboard off.", msg->command_id, command_name));
      return;
    } else if (critical_state_.load(std::memory_order_relaxed)) {
      publishInfo(DroneInfo::LEVEL_ERROR,
        stringf("Rejected (id=%d, %s): System in critical state.",
                msg->command_id, command_name));
      return;
    }
  }

  auto it = cmd_handlers_.find(msg->command_id);
  if (it == cmd_handlers_.end()) {
    RCLCPP_WARN(get_logger(), "Requested (id=%d, %s): No handler found.",
      msg->command_id, command_name);
    return;
  }

  const CommandResult result = it->second(*msg, current_state);
  if (result.success) {
    RCLCPP_INFO(get_logger(), "Executed (id=%d, %s): %s",
      msg->command_id, command_name, result.text.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Failed (id=%d, %s): %s",
      msg->command_id, command_name, result.text.c_str());
  }
}


// ============================================================================
// onTeleopAction
// ============================================================================

void ControlGateNode::onTeleopAction(const drone_msgs::msg::TeleopAction::SharedPtr msg) {
  if (inInitializationPhase()) return;

  const InternalState current_state = snapshotState();
  const bool blocked = isSetpointBlocked(current_state);
  updateSetpointBlockStateAndMaybePublish(blocked, false);
  if (blocked) return;

  // Every teleop action still resets the general "last action" clock
  // (used by other watchdogs / state reporting).
  updateLastAct(std::chrono::steady_clock::now());

  const float eff_vx       = static_cast<float>(msg->vx);
  const float eff_vy       = static_cast<float>(msg->vy);
  const float eff_vz       = static_cast<float>(msg->vz);
  const float eff_yaw_rate = static_cast<float>(msg->yaw_rate);

  if (alt_ctrl_mode_ == AltCtrlMode::AltHold) {
    // Vx / Vy / yaw_rate always pass through — update guided cmd state.
    updateGuidedVx(eff_vx);
    updateGuidedVy(eff_vy);
    updateGuidedYawRate(eff_yaw_rate);

    // Vz: operator input resets the override clock and is forwarded directly.
    // The operator bypasses the PID and the ground-protection clamp intentionally.
    if (eff_vz != 0.0f) {
      last_vz_override_t_ = std::chrono::steady_clock::now();

      if (!alt_hold_operator_override_) {
        // Transition: PID was running → operator took over.
        alt_hold_operator_override_ = true;
        deactivatePid();

        RCLCPP_INFO(get_logger(),
          "[alt_hold] Operator Vz override (%.3f m/s) — PID deactivated.", eff_vz);
        publishInfo(DroneInfo::LEVEL_INFO, "AltHold: operator Vz override, PID paused.");
      }
      // Forward operator Vz directly — bypass updateGuidedVz (no ground clamp for operator).
      guided_cmd_.vz  = eff_vz;
      last_vz_update_ = std::chrono::steady_clock::now();
    } else {
      // vz == 0: zero out Vz so the setpoint timer doesn't replay the last nonzero value.
      guided_cmd_.vz  = 0.0f;
      last_vz_update_ = std::chrono::steady_clock::now();
    }

  } else {
    // AltHold Off — plain pass-through to the setpoint publisher.
    // We do NOT use the guided_cmd_ / timer path here; call publishSetpoint directly
    // so that non-althold teleop still works.
    publishSetpoint(eff_vx, eff_vy, eff_vz, eff_yaw_rate);
  }
}


// ============================================================================
// onMavrosState
// ============================================================================

void ControlGateNode::onMavrosState(const mavros_msgs::msg::State::SharedPtr msg) {
  if (inInitializationPhase()) return;

  const InternalState prev_state = snapshotState();

  InternalStateUpdate update;
  update.connected = msg->connected;
  update.armed     = msg->armed;
  update.guided    = msg->guided;
  update.fcu_mode  = msg->mode;
  updateInternalStateAtomic(update);

  fcu_state_.store(msg->system_status, std::memory_order_relaxed);

  const bool was_guided_mode =
    prev_state.guided || prev_state.fcu_mode == "GUIDED";
  const bool is_guided_mode =
    msg->guided || msg->mode == "GUIDED";

  if (was_guided_mode && !is_guided_mode) {
    const std::string prev_mode =
      prev_state.fcu_mode.empty() ? std::string("GUIDED") : prev_state.fcu_mode;
    const std::string next_mode =
      msg->mode.empty() ? std::string("<unknown>") : msg->mode;
    closeAltitudeController(
      stringf("FCU mode changed %s -> %s.", prev_mode.c_str(), next_mode.c_str()));
  }
}


// ============================================================================
// onGcsHeartbeat
// ============================================================================

void ControlGateNode::onGcsHeartbeat(const GcsHeartbeat::SharedPtr msg) {
  const auto now_ns   = this->now().nanoseconds();
  const auto stamp_ns = rclcpp::Time(msg->stamp).nanoseconds();
  const int64_t diff_ns = (now_ns > stamp_ns)
                          ? (now_ns - stamp_ns)
                          : (stamp_ns - now_ns);

  if (diff_ns > 3'000'000'000LL) return;  // stale heartbeat

  if (!gcs_connected_.load(std::memory_order_relaxed)) {
    RCLCPP_INFO(get_logger(), "New GCS heartbeat received. GCS ID: %d", msg->gcs_id);
    publishInfo(DroneInfo::LEVEL_INFO,
      stringf("New GCS heartbeat received. GCS ID: %d", msg->gcs_id));
    gcs_id_.store(msg->gcs_id, std::memory_order_relaxed);
    gcs_connected_.store(true, std::memory_order_relaxed);
    time_since_heartbeat_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
    return;
  }

  if (gcs_id_.load(std::memory_order_relaxed) != msg->gcs_id) {
    RCLCPP_WARN(get_logger(), "Heartbeat from unknown GCS (id=%d), ignoring.", msg->gcs_id);
    return;
  }

  time_since_heartbeat_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
}


// ============================================================================
// onFailsafeWatchdog
// ============================================================================

void ControlGateNode::onFailsafeWatchdog() {
  // --- 1. Critical state check ---
  const uint8_t fcu = fcu_state_.load(std::memory_order_relaxed);
  if (fcu >= 5u) {
    if (!critical_state_.load(std::memory_order_relaxed)) {
      critical_state_.store(true, std::memory_order_relaxed);
      RCLCPP_ERROR(get_logger(), "FCU entered critical state (system_status=%u).", fcu);
      publishInfo(DroneInfo::LEVEL_ERROR,
        stringf("FCU critical state (system_status=%u).", fcu));
    }
  }

  // --- 2. GCS failsafe ---
  const bool is_there_failsafe = gcs_failsafe_.load(std::memory_order_relaxed);
  const bool is_there_conn     = gcs_connected_.load(std::memory_order_relaxed);
  if (!is_there_conn && !is_there_failsafe) return;

  const int64_t last_ns = time_since_heartbeat_ns_.load(std::memory_order_relaxed);
  const rclcpp::Time last_heartbeat(last_ns, RCL_ROS_TIME);
  const double elapsed_s = (this->now() - last_heartbeat).seconds();
  const bool gcs_triggered = (elapsed_s > gcs_failsafe_s_);

  if (gcs_triggered) {
    if (!is_there_failsafe) {
      RCLCPP_ERROR(get_logger(),
        "GCS heartbeat lost for %.1f s. GCS failsafe triggered.", elapsed_s);
      publishInfo(DroneInfo::LEVEL_ERROR,
        stringf("GCS failsafe: No heartbeat for %.1f s.", elapsed_s));

      gcs_connected_.store(false, std::memory_order_relaxed);
      gcs_id_.store(-1, std::memory_order_relaxed);
      gcs_failsafe_.store(true, std::memory_order_relaxed);

      const InternalState st = snapshotState();
      drone_msgs::msg::TeleopCommand dummy_cmd{};
      if (!st.armed) {
        // nothing to do
      } else if (st.guided) {
        executeRTL(dummy_cmd, st);
        publishInfo(DroneInfo::LEVEL_ERROR, "Sending RTL.");
      } else {
        executeLand(dummy_cmd, st);
        publishInfo(DroneInfo::LEVEL_ERROR, "Sending LAND.");
      }
    }
  } else {
    if (is_there_failsafe) {
      gcs_failsafe_.store(false, std::memory_order_relaxed);
      RCLCPP_INFO(get_logger(), "GCS failsafe cleared.");
      publishInfo(DroneInfo::LEVEL_INFO, "GCS failsafe cleared: Heartbeat received.");
    }
  }
}


// ============================================================================
// onVerticalEstimate  — update internal cache ONLY
//
// control_gate caches the MCU vertical estimates so it can:
//   • snapshot a safe hold altitude when the vz-override timeout fires
//   • monitor takeoff progress
//
// The altitude_controller subscribes directly to the mcu_bridge topic at
// sensor QoS for its own measurements.
// ============================================================================

void ControlGateNode::onVerticalEstimate(const VerticalEstimate::SharedPtr msg) {
  if (inInitializationPhase()) return;

  std::lock_guard<std::mutex> lk(vert_est_mtx_);
  vert_est_.z_world_m = msg->z_world_m;
  vert_est_.vz_mps    = msg->vz_world_mps;
  vert_est_.agl_m     = msg->agl_m;
  vert_est_.valid     = true;
}


// ============================================================================
// onAltCtrlOutput  — receive Vz command from PID node
// ============================================================================

void ControlGateNode::onAltCtrlOutput(const AltCtrlOutput::SharedPtr msg) {
  if (inInitializationPhase()) return;

  // Ignore output when AltHold is off or operator override is active.
  if (alt_ctrl_mode_ != AltCtrlMode::AltHold || alt_hold_operator_override_) return;

  alt_ctrl_vz_output_    = msg->data;
  alt_ctrl_output_fresh_ = true;

  // Route PID output into the guided command state (with ground protection).
  updateGuidedVz(msg->data);
}


// ============================================================================
// onSetTargetHeight  — service callback (control_gate owns this)
//
// Sets a new altitude hold target. Rejects targets below alt_ctrl_min_agl_m_.
// If the PID is currently active, pushes the new target immediately with an
// integral reset. If AltHold is off entirely, the request is rejected.
// ============================================================================

void ControlGateNode::onSetTargetHeight(
  const std::shared_ptr<SetTargetHeight::Request>  req,
  std::shared_ptr<SetTargetHeight::Response>       res)
{
  if (alt_ctrl_mode_ == AltCtrlMode::Off) {
    res->success = false;
    res->message = "AltHold is not active.";
    RCLCPP_WARN(get_logger(), "SetTargetHeight rejected: %s", res->message.c_str());
    return;
  }

  if (req->target_agl_m < alt_ctrl_min_agl_m_) {
    res->success = false;
    res->message = stringf(
      "target_agl_m=%.2f is below minimum safe AGL of %.2f m.",
      req->target_agl_m, alt_ctrl_min_agl_m_);
    RCLCPP_WARN(get_logger(), "SetTargetHeight rejected: %s", res->message.c_str());
    return;
  }

  const float old_target = alt_ctrl_target_agl_;
  const std::string info_msg = stringf(
    "Target height updated: %.2f m → %.2f m", old_target, req->target_agl_m);

  RCLCPP_INFO(get_logger(), "[set_target_height] %s", info_msg.c_str());
  publishInfo(DroneInfo::LEVEL_INFO, info_msg);

  // Push new target. If PID is active send it immediately; if override is
  // active, just store it so it will be used when the timeout fires.
  if (!alt_hold_operator_override_) {
    activatePid(req->target_agl_m);   // activatePid stores target and resets integral
  } else {
    alt_ctrl_target_agl_ = req->target_agl_m;
  }

  res->success = true;
  res->message = info_msg;
}
