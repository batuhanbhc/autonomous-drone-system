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

  // Every teleop action resets the timeout clock
  updateLastAct(std::chrono::steady_clock::now());

  float eff_vx       = static_cast<float>(msg->vx);
  float eff_vy       = static_cast<float>(msg->vy);
  float eff_vz       = static_cast<float>(msg->vz);
  float eff_yaw_rate = static_cast<float>(msg->yaw_rate);

  if (alt_ctrl_mode_ == AltCtrlMode::AltHold) {
    const bool vz_nonzero = (eff_vz != 0.0f);

    // ── If timed out, any input cancels the full-hover sub-state ──────────
    // (exitTimedOut is already called by the watchdog when it sees elapsed < timeout,
    //  but we also call updateLastAct above so it will clear on next watchdog tick.
    //  No explicit action needed here — the watchdog handles it.)

    if (vz_nonzero) {
      // Operator pushing vz — stop controller, pass through operator value.
      // We stay in AltHold so it re-engages on the next vz==0 input.
      if (!alt_hold_last_vz_nonzero_) {
        // Transition: vz was zero → now nonzero. Deactivate PID.
        publishAltCtrlInput(false, 0.0f, 0.0f, 0.0f);
        alt_ctrl_output_fresh_    = false;
        alt_hold_last_vz_nonzero_ = true;
      }
      // eff_vz passes through unchanged

    } else {
      // vz == 0: controller holds altitude
      if (alt_hold_last_vz_nonzero_) {
        // Transition: vz was nonzero → now zero.
        // Snapshot current agl as the new hold target.
        const VerticalEstimateCache ve = snapshotVertEst();
        if (ve.valid) {
          alt_ctrl_target_agl_ = ve.agl_m;
          publishAltCtrlInput(true, alt_ctrl_target_agl_, ve.agl_m, ve.vz_mps);
        }
        alt_hold_last_vz_nonzero_ = false;
      }

      if (alt_ctrl_output_fresh_) {
        eff_vz = alt_ctrl_vz_output_;
      }
      // No controller output yet → eff_vz stays 0, safe hover
    }
  }

  publishSetpoint(eff_vx, eff_vy, eff_vz, eff_yaw_rate);
}


// ============================================================================
// onMavrosState
// ============================================================================

void ControlGateNode::onMavrosState(const mavros_msgs::msg::State::SharedPtr msg) {
  if (inInitializationPhase()) return;

  InternalStateUpdate update;
  update.connected = msg->connected;
  update.armed     = msg->armed;
  update.guided    = msg->guided;
  updateInternalStateAtomic(update);

  fcu_state_.store(msg->system_status, std::memory_order_relaxed);
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
// onVerticalEstimate  — update cache and forward ALL fields to altitude controller
//
// FIX: previously only (active, agl, vz_mps) were forwarded; the PID node
// received only the current velocity and the fixed target but not the live
// current_agl_m needed to compute the error.  Now we send all four fields:
//   active          = true  (controller stays running while mode is active)
//   target_agl_m    = alt_ctrl_target_agl_   (operator-set or snapshotted)
//   current_agl_m   = vert_est_.agl_m        (live measurement from MCU)
//   current_vz_mps  = vert_est_.vz_mps       (live velocity from MCU)
// ============================================================================

void ControlGateNode::onVerticalEstimate(const VerticalEstimate::SharedPtr msg) {
  if (inInitializationPhase()) return;

  {
    std::lock_guard<std::mutex> lk(vert_est_mtx_);
    vert_est_.z_world_m = static_cast<float>(msg->vector.x);
    vert_est_.vz_mps    = static_cast<float>(msg->vector.y);
    vert_est_.agl_m     = static_cast<float>(msg->vector.z);
    vert_est_.valid     = true;
  }

  // Forward to controller at MCU rate while any mode is active.
  // current_agl_m and current_vz_mps are now passed so the PID node
  // has everything it needs to compute error = target - current.
  if (alt_ctrl_mode_ != AltCtrlMode::Off) {
    publishAltCtrlInput(
      /*active=*/        true,
      /*target_agl_m=*/  alt_ctrl_target_agl_,
      /*current_agl_m=*/ vert_est_.agl_m,
      /*current_vz_mps=*/vert_est_.vz_mps);
  }
}


// ============================================================================
// onAltCtrlOutput  — receive vz from PID node
// ============================================================================

void ControlGateNode::onAltCtrlOutput(const AltCtrlOutput::SharedPtr msg) {
  if (inInitializationPhase()) return;

  alt_ctrl_vz_output_    = msg->data;
  alt_ctrl_output_fresh_ = true;

  // When timed out, drive setpoints directly from here (no teleop action
  // arriving) so the drone hovers even with no operator input.
  if (alt_ctrl_mode_ == AltCtrlMode::AltHold && alt_hold_timed_out_) {
    const InternalState st = snapshotState();
    if (isSetpointBlocked(st)) return;
    publishSetpoint(0.0f, 0.0f, alt_ctrl_vz_output_, 0.0f);
  }
}


// ============================================================================
// onSetTargetHeight  — service callback
//
// Lets an operator set a specific hold altitude on-the-fly.
// The new target is stored in alt_ctrl_target_agl_ and, if the controller
// is currently active, a fresh message is immediately pushed to it so the
// next PID tick uses the updated target.
// ============================================================================

void ControlGateNode::onSetTargetHeight(
  const std::shared_ptr<SetTargetHeight::Request>  req,
  std::shared_ptr<SetTargetHeight::Response>       res)
{
  if (req->target_agl_m < 0.0f) {
    res->success = false;
    res->message = "target_agl_m must be >= 0.";
    RCLCPP_WARN(get_logger(), "SetTargetHeight rejected: %s", res->message.c_str());
    return;
  }

  const float old_target = alt_ctrl_target_agl_;
  alt_ctrl_target_agl_   = req->target_agl_m;

  const std::string info_msg = stringf(
    "Target height updated: %.2f m → %.2f m", old_target, alt_ctrl_target_agl_);
  RCLCPP_INFO(get_logger(), "[set_target_height] %s", info_msg.c_str());
  publishInfo(DroneInfo::LEVEL_INFO, info_msg);

  // If the controller is running, push a new message immediately so the PID
  // node picks up the changed target before the next MCU packet arrives.
  if (alt_ctrl_mode_ != AltCtrlMode::Off) {
      const VerticalEstimateCache ve = snapshotVertEst();
      if (ve.valid) {
          publishAltCtrlInput(true, alt_ctrl_target_agl_, ve.agl_m, ve.vz_mps, true);
      }
  }

  res->success = true;
  res->message = info_msg;
}