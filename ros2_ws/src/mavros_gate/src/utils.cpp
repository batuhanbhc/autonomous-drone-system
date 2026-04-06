#include "mavros_gate/control_gate.hpp"


using Cmd       = drone_msgs::msg::TeleopCommand;
using DroneInfo = drone_msgs::msg::DroneInfo;
using MonotonicTime = std::chrono::steady_clock::time_point;

// ============================================================================
// commandName / isCommandAlwaysEnabled
// ============================================================================

const char* ControlGateNode::commandName(int8_t id) {
  switch (id) {
    case Cmd::KILL_SWITCH:           return "KILL_SWITCH";
    case Cmd::KILL_CONFIRM:          return "KILL_CONFIRM";
    case Cmd::CONTROL_TOGGLE:        return "CONTROL_TOGGLE";
    case Cmd::KEYBOARD_TOGGLE:       return "KEYBOARD_TOGGLE";
    case Cmd::ARM:                   return "ARM";
    case Cmd::DISARM:                return "DISARM";
    case Cmd::LAND:                  return "LAND";
    case Cmd::RTL:                   return "RTL";
    case Cmd::TAKEOFF:               return "TAKEOFF";
    case Cmd::GUIDED:                return "GUIDED";
    case Cmd::SPEED_UP_HORIZONTAL:   return "SPEED_UP_HORIZONTAL";
    case Cmd::SPEED_DOWN_HORIZONTAL: return "SPEED_DOWN_HORIZONTAL";
    case Cmd::SPEED_UP_VERTICAL:     return "SPEED_UP_VERTICAL";
    case Cmd::SPEED_DOWN_VERTICAL:   return "SPEED_DOWN_VERTICAL";
    case Cmd::SPEED_UP_YAW:          return "SPEED_UP_YAW";
    case Cmd::SPEED_DOWN_YAW:        return "SPEED_DOWN_YAW";
    case Cmd::PRESS_SAFETY_SWITCH:   return "PRESS_SAFETY_SWITCH";
    case Cmd::VEL_YAW:               return "VEL_YAW";
    case Cmd::HOVER:                 return "HOVER";
    case Cmd::ALT_SUPPORT_TOGGLE:    return "ALT_SUPPORT_TOGGLE";
    default:                         return "<unknown>";
  }
}

bool ControlGateNode::isCommandAlwaysEnabled(int8_t id) {
  switch (id) {
    case Cmd::KILL_SWITCH:
    case Cmd::KILL_CONFIRM:
    case Cmd::CONTROL_TOGGLE:
    case Cmd::KEYBOARD_TOGGLE:
    case Cmd::LAND:
    case Cmd::RTL:
      return true;
    default:
      return false;
  }
}

// ============================================================================
// Internal state helpers
// ============================================================================

void ControlGateNode::updateInternalStateAtomic(const InternalStateUpdate & update) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (update.control_mode)       state_.control_mode       = *update.control_mode;
  if (update.vel)                state_.vel                = *update.vel;
  if (update.keyboard_on)        state_.keyboard_on        = *update.keyboard_on;
  if (update.safety_switch_on)   state_.safety_switch_on   = *update.safety_switch_on;
  if (update.connected)          state_.connected          = *update.connected;
  if (update.armed)              state_.armed              = *update.armed;
  if (update.guided)             state_.guided             = *update.guided;
  if (update.kill_switch_window) state_.kill_switch_window = *update.kill_switch_window;
  if (update.system_killed)      state_.system_killed      = *update.system_killed;
}

ControlGateNode::InternalState ControlGateNode::snapshotState() const {
  std::lock_guard<std::mutex> lk(state_mtx_);
  return state_;
}

bool ControlGateNode::inInitializationPhase() const {
  return initialization_phase_;
}

float ControlGateNode::getTakeoffMeters() const {
  return takeoff_m_;
}

bool ControlGateNode::isAnyFailsafeActive() const {
  return gcs_failsafe_.load(std::memory_order_relaxed) ||
         critical_state_.load(std::memory_order_relaxed);
}

// ============================================================================
// Vertical estimate cache
// ============================================================================

ControlGateNode::VerticalEstimateCache ControlGateNode::snapshotVertEst() const {
  std::lock_guard<std::mutex> lk(vert_est_mtx_);
  return vert_est_;
}

// ============================================================================
// last_action helpers
// ============================================================================

void ControlGateNode::updateLastAct(const MonotonicTime& t) {
  std::lock_guard<std::mutex> lk(last_act_mtx_);
  last_action_t_ = t;
}

MonotonicTime ControlGateNode::readLastAct() const {
  std::lock_guard<std::mutex> lk(last_act_mtx_);
  return last_action_t_;
}

// ============================================================================
// loadConfig
// ============================================================================

bool ControlGateNode::loadConfig() {
  const std::string pkg_share = ament_index_cpp::get_package_share_directory("mavros_config");
  const std::string yaml_path = pkg_share + "/config/control_params.yaml";

  YAML::Node root = YAML::LoadFile(yaml_path);

  YAML::Node t = root["custom_topics"];
  if (!t || !t.IsMap()) {
    RCLCPP_WARN(get_logger(), "YAML missing/invalid 'custom_topics'");
    return false;
  }

  auto get_str = [&](const char* key, std::string& out) -> bool {
    auto n = t[key];
    if (!n || !n.IsScalar()) {
      RCLCPP_WARN(get_logger(), "YAML missing/invalid custom_topics.%s", key);
      return false;
    }
    out = n.as<std::string>();
    return true;
  };

  bool ok = true;
  ok &= get_str("autonomous_action", topics_.autonomous_action);
  ok &= get_str("manual_action",     topics_.manual_action);
  ok &= get_str("manual_command",    topics_.manual_command);
  ok &= get_str("mcu_bridge",        topics_.mcu_bridge);
  ok &= get_str("alt_ctrl_input",    topics_.alt_ctrl_input);
  ok &= get_str("alt_ctrl_output",   topics_.alt_ctrl_output);

  YAML::Node fp = root["flight_params"];
  auto load_f = [&](const char* key, auto& field) {
    try {
      field = fp[key].as<std::remove_reference_t<decltype(field)>>();
    } catch (const YAML::Exception&) {
      RCLCPP_WARN(get_logger(), "YAML missing flight_params.%s", key);
      ok = false;
    }
  };

  load_f("takeoff_m",                  takeoff_m_);
  load_f("gcs_failsafe_s",             gcs_failsafe_s_);
  load_f("failsafe_watchdog_hz",       failsafe_watchdog_hz_);
  load_f("guided_timeout_s",           guided_timeout_s_);
  load_f("guided_timeout_watchdog_hz", guided_timeout_watchdog_hz_);
  load_f("takeoff_reach_threshold_m",  takeoff_reach_threshold_m_);
  load_f("takeoff_timeout_s",          takeoff_timeout_s_);

  return ok;
}

// ============================================================================
// initCommandHandlers
// ============================================================================

void ControlGateNode::initCommandHandlers() {
  cmd_handlers_[TeleopCmd::ARM] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeArm(c, s); };
  cmd_handlers_[TeleopCmd::DISARM] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeDisarm(c, s); };
  cmd_handlers_[TeleopCmd::KILL_SWITCH] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeKillSwitch(c, s); };
  cmd_handlers_[TeleopCmd::KILL_CONFIRM] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeKillConfirm(c, s); };
  cmd_handlers_[TeleopCmd::LAND] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeLand(c, s); };
  cmd_handlers_[TeleopCmd::RTL] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeRTL(c, s); };
  cmd_handlers_[TeleopCmd::TAKEOFF] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeTakeoff(c, s); };
  cmd_handlers_[TeleopCmd::GUIDED] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeGuided(c, s); };
  cmd_handlers_[TeleopCmd::CONTROL_TOGGLE] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeControlToggle(c, s); };
  cmd_handlers_[TeleopCmd::KEYBOARD_TOGGLE] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeKeyboardToggle(c, s); };
  cmd_handlers_[TeleopCmd::SPEED_UP_HORIZONTAL] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeChangeSpeedHorizontal(c, s); };
  cmd_handlers_[TeleopCmd::SPEED_DOWN_HORIZONTAL] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeChangeSpeedHorizontal(c, s); };
  cmd_handlers_[TeleopCmd::SPEED_UP_VERTICAL] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeChangeSpeedVertical(c, s); };
  cmd_handlers_[TeleopCmd::SPEED_DOWN_VERTICAL] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeChangeSpeedVertical(c, s); };
  cmd_handlers_[TeleopCmd::SPEED_UP_YAW] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeChangeSpeedYaw(c, s); };
  cmd_handlers_[TeleopCmd::SPEED_DOWN_YAW] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeChangeSpeedYaw(c, s); };
  cmd_handlers_[TeleopCmd::PRESS_SAFETY_SWITCH] = [this](const TeleopCmd& c, const InternalState& s) {
    return executePressSafetySwitch(c, s); };
  cmd_handlers_[TeleopCmd::ALT_SUPPORT_TOGGLE] = [this](const TeleopCmd& c, const InternalState& s) {
    return executeAltSupportToggle(c, s); };
}

// ============================================================================
// initializationRoutine
// ============================================================================

bool ControlGateNode::initializationRoutine() {
  updateLastAct(std::chrono::steady_clock::now());

  const std::string arming_path       = base_ns_ + "/mavros/cmd/arming";
  const std::string command_long_path = base_ns_ + "/mavros/cmd/command";
  const std::string set_mode_path     = base_ns_ + "/mavros/set_mode";
  const std::string takeoff_path      = base_ns_ + "/mavros/cmd/takeoff";
  const std::string msg_interval_path = base_ns_ + "/mavros/set_message_interval";

  RCLCPP_INFO(get_logger(), "Waiting for mavros to come online...");
  auto arming_probe = this->create_client<mavros_msgs::srv::CommandBool>(arming_path);
  if (!arming_probe->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_FATAL(get_logger(), "mavros arming service not available after 60s");
    return false;
  }
  RCLCPP_INFO(get_logger(), "mavros is online.");
  arming_probe.reset();

  arming_client_       = this->create_client<mavros_msgs::srv::CommandBool>(arming_path);
  command_long_client_ = this->create_client<mavros_msgs::srv::CommandLong>(command_long_path);
  set_mode_client_     = this->create_client<mavros_msgs::srv::SetMode>(set_mode_path);
  takeoff_client_      = this->create_client<mavros_msgs::srv::CommandTOL>(takeoff_path);
  msg_interval_client_ = this->create_client<mavros_msgs::srv::MessageInterval>(msg_interval_path);

  auto requestMsgInterval = [&](int id, float rate_hz, const char* label) -> bool {
    if (!msg_interval_client_->wait_for_service(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(), "set_message_interval not available (%s)", label);
      return false;
    }
    auto req = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
    req->message_id   = id;
    req->message_rate = rate_hz;
    int retries = 5;
    while (retries-- > 0) {
      auto result = msg_interval_client_->async_send_request(req);
      if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), result,
            std::chrono::seconds(3)) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Requested %s stream: success=%s",
          label, result.get()->success ? "true" : "false");
        break;
      }
      RCLCPP_WARN(get_logger(), "set_message_interval timed out, retrying... (%d left)", retries);
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    if (retries <= 0) {
      RCLCPP_FATAL(get_logger(), "set_message_interval failed after all retries: %s", label);
      throw std::runtime_error("control_gate init failed");
    }
    return true;
  };

  if (!requestMsgInterval(245, 2.0f,  "EXTENDED_SYS_STATE")) return false;
  if (!requestMsgInterval(147, 1.0f,  "BATTERY_STATUS"))      return false;
  if (!requestMsgInterval(32,  30.0f, "LOCAL_POSITION_NED"))  return false;
  if (!requestMsgInterval(24,  5.0f,  "GPS_RAW_INT"))         return false;
  if (!requestMsgInterval(31,  30.0f, "ATTITUDE_QUATERNION")) return false;

  initCommandHandlers();
  return true;
}

// ============================================================================
// isSetpointBlocked / updateSetpointBlockStateAndMaybePublish
// ============================================================================

bool ControlGateNode::isSetpointBlocked(const InternalState& st) const {
  const bool enabled =
    st.connected && !st.system_killed && st.armed && st.guided &&
    (st.control_mode == ControlMode::Manual) && st.keyboard_on &&
    !critical_state_.load(std::memory_order_relaxed) &&
    !gcs_failsafe_.load(std::memory_order_relaxed);
  return !enabled;
}

void ControlGateNode::updateSetpointBlockStateAndMaybePublish(bool blocked, bool initial_publish) {
  if (!setpoint_blocked_initialized_) {
    setpoint_blocked_ = blocked;
    setpoint_blocked_initialized_ = true;
    if (initial_publish) {
      publishInfo(blocked ? DroneInfo::LEVEL_WARN : DroneInfo::LEVEL_INFO,
                  blocked ? "Setpoint commands are blocked."
                          : "Setpoint commands are enabled.");
    }
    return;
  }
  if (blocked != setpoint_blocked_) {
    setpoint_blocked_ = blocked;
    publishInfo(blocked ? DroneInfo::LEVEL_WARN : DroneInfo::LEVEL_INFO,
                blocked ? "Setpoint commands are now blocked."
                        : "Setpoint commands are now enabled.");
  }
}


// ============================================================================
// Altitude controller state transitions
// ============================================================================

void ControlGateNode::enterGuidedTimeout(const VerticalEstimateCache& ve) {
  alt_ctrl_mode_      = AltCtrlMode::GuidedTimeout;
  alt_ctrl_fixed_agl_ = ve.agl_m;

  RCLCPP_WARN(get_logger(),
    "[alt_ctrl] Guided timeout triggered — hover at agl=%.3f m",
    alt_ctrl_fixed_agl_);
  publishInfo(DroneInfo::LEVEL_WARN,
    stringf("Guided timeout. Hovering at agl=%.2f m", alt_ctrl_fixed_agl_));

  publishAltCtrlInput(true, alt_ctrl_fixed_agl_, ve.vz_mps);
}

void ControlGateNode::exitGuidedTimeout() {
  // Always publish stop first, then decide next mode.
  publishAltCtrlInput(false, 0.0f, 0.0f);
  alt_ctrl_mode_ = AltCtrlMode::Off;

  RCLCPP_INFO(get_logger(), "[alt_ctrl] Guided timeout cleared.");
  publishInfo(DroneInfo::LEVEL_INFO, "Guided timeout cleared.");

  // Note: if the user had AltSupport toggled on before the timeout occurred,
  // onTeleopAction handles the AltSupport entry on the same callback pass,
  // since alt_ctrl_mode_ is now Off and the vz==0 branch will re-enter it.
}

void ControlGateNode::enterAltSupport(float agl, float vz_mps) {
  alt_ctrl_mode_      = AltCtrlMode::AltSupport;
  alt_ctrl_fixed_agl_ = agl;

  RCLCPP_INFO(get_logger(),
    "[alt_ctrl] AltSupport active — target agl=%.3f m", agl);

  publishAltCtrlInput(true, alt_ctrl_fixed_agl_, vz_mps);
}

void ControlGateNode::exitAltSupport() {
  alt_ctrl_mode_ = AltCtrlMode::Off;
  publishAltCtrlInput(false, 0.0f, 0.0f);

  RCLCPP_INFO(get_logger(), "[alt_ctrl] AltSupport deactivated (vz override).");
}

// ============================================================================
// executeAltSupportToggle  (command handler)
// ============================================================================

ControlGateNode::CommandResult ControlGateNode::executeAltSupportToggle(
  const TeleopCmd&, const InternalState&)
{
  if (alt_ctrl_mode_ == AltCtrlMode::AltSupport) {
    exitAltSupport();
    return {true, "AltSupport disabled."};
  }

  if (alt_ctrl_mode_ == AltCtrlMode::GuidedTimeout) {
    return {false, "Cannot toggle AltSupport while guided timeout is active."};
  }

  // Mode is Off — arm AltSupport. Actual activation (snapshot + controller
  // start) happens on the first vz==0 teleop action in onTeleopAction.
  alt_ctrl_mode_ = AltCtrlMode::AltSupport;
  alt_support_last_vz_nonzero_ = true;  // force snapshot on next vz==0 action

  return {true, "AltSupport enabled. Will activate on next vz=0 action."};
}

// ============================================================================
// onGuidedTimeoutWatchdog  (timer callback, ~10 Hz)
// ============================================================================

void ControlGateNode::onGuidedTimeoutWatchdog() {
  if (inInitializationPhase()) return;

  const InternalState st = snapshotState();

  // If FCU left guided or a failsafe is active, clean up any active alt ctrl
  if (!st.guided || isAnyFailsafeActive()) {
    if (alt_ctrl_mode_ == AltCtrlMode::GuidedTimeout) {
      publishAltCtrlInput(false, 0.0f, 0.0f);
      alt_ctrl_mode_ = AltCtrlMode::Off;
      RCLCPP_INFO(get_logger(),
        "[alt_ctrl] Guided timeout cleared — FCU no longer in guided.");
    }
    return;
  }

  // Already in guided timeout — nothing more to trigger here
  if (alt_ctrl_mode_ == AltCtrlMode::GuidedTimeout) return;

  const double elapsed =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - readLastAct()).count();

  if (elapsed >= guided_timeout_s_) {
    const VerticalEstimateCache ve = snapshotVertEst();
    if (!ve.valid) {
      RCLCPP_WARN(get_logger(),
        "[alt_ctrl] Guided timeout elapsed but no MCU data yet — skipping.");
      return;
    }
    enterGuidedTimeout(ve);
  }
}

// ============================================================================
// onTakeoffMonitorTick  (dormant until re-armed by executeTakeoff)
// ============================================================================

void ControlGateNode::onTakeoffMonitorTick() {
  if (!takeoff_monitor_active_) {
    takeoff_monitor_timer_->cancel();
    return;
  }

  const auto now     = std::chrono::steady_clock::now();
  const float elapsed =
    std::chrono::duration<float>(now - takeoff_start_t_).count();

  if (elapsed >= takeoff_timeout_s_) {
    RCLCPP_WARN(get_logger(),
      "[takeoff_monitor] Hard timeout after %.1f s — stopping.", elapsed);
    publishInfo(DroneInfo::LEVEL_WARN, "Takeoff monitor timed out.");
    takeoff_monitor_active_ = false;
    takeoff_monitor_timer_->cancel();
    return;
  }

  const VerticalEstimateCache ve = snapshotVertEst();
  if (!ve.valid) return;

  if (std::abs(ve.agl_m - takeoff_m_) <= takeoff_reach_threshold_m_) {
    RCLCPP_INFO(get_logger(),
      "[takeoff_monitor] Target altitude reached (agl=%.3f m).", ve.agl_m);
    publishInfo(DroneInfo::LEVEL_INFO,
      stringf("Takeoff complete: agl=%.2f m", ve.agl_m));
    takeoff_monitor_active_ = false;
    takeoff_monitor_timer_->cancel();
    return;
  }

  // Still climbing — reset guided timeout clock
  updateLastAct(std::chrono::steady_clock::now());
}