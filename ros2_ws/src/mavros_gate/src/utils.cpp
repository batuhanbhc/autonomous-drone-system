#include "mavros_gate/control_gate.hpp"


using Cmd           = drone_msgs::msg::TeleopCommand;
using DroneInfo     = drone_msgs::msg::DroneInfo;
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
  // ALT_SUPPORT_TOGGLE removed — AltHold is always active after takeoff
}


// ============================================================================
// initializationRoutine
// ============================================================================

bool ControlGateNode::initializationRoutine() {
  // ── Service clients ────────────────────────────────────────────────────
  arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
    base_ns_ + "/mavros/cmd/arming");
  command_long_client_ = this->create_client<mavros_msgs::srv::CommandLong>(
    base_ns_ + "/mavros/cmd/command");
  set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
    base_ns_ + "/mavros/set_mode");
  takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(
    base_ns_ + "/mavros/cmd/takeoff");
  msg_interval_client_ = this->create_client<mavros_msgs::srv::MessageInterval>(
    base_ns_ + "/mavros/set_message_interval");

  // ── SetTargetHeight service server ────────────────────────────────────
  const std::string srv_name = base_ns_ + "/control_gate/set_target_height";
  srv_set_target_height_ = this->create_service<SetTargetHeight>(
    srv_name,
    std::bind(&ControlGateNode::onSetTargetHeight, this,
              std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(get_logger(),
    "Target height service ready at: %s", srv_name.c_str());

  // ── Wait for MAVROS services ───────────────────────────────────────────
  // wait_for_service() is safe to call before spin() — it does not need
  // the executor to be running; it just checks the ROS graph.
  const auto timeout = std::chrono::seconds(5);
  auto wait = [&](auto& client, const char* label) -> bool {
    RCLCPP_INFO(get_logger(), "Waiting for service: %s", label);
    while (!client->wait_for_service(timeout)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL(get_logger(), "Interrupted waiting for %s", label);
        return false;
      }
      RCLCPP_WARN(get_logger(), "Service %s not ready yet, retrying...", label);
    }
    RCLCPP_INFO(get_logger(), "Service ready: %s", label);
    return true;
  };

  if (!wait(arming_client_,       "arming"))        return false;
  if (!wait(command_long_client_,  "command_long"))  return false;
  if (!wait(set_mode_client_,      "set_mode"))      return false;
  if (!wait(takeoff_client_,       "takeoff"))       return false;
  if (!wait(msg_interval_client_,  "msg_interval"))  return false;

  // ── Request MAVLink message intervals ────────────────────────────────────
  // async_send_request() + wait_for() deadlocks when the node has not started
  // spinning yet, because the response callback can never be dispatched.
  // Fix: spin a temporary single-threaded executor on a side thread just long
  // enough to drive the service round-trip, then shut it down.
  auto requestMsgInterval = [&](uint32_t msg_id, float hz, const char* label) {
    auto req = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
    req->message_id   = msg_id;
    req->message_rate = hz;

    for (int attempt = 1; attempt <= 3; ++attempt) {
      // Spin a temporary executor on a background thread so the response
      // callback can be dispatched while we block on the future here.
      auto temp_exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      temp_exec->add_node(this->get_node_base_interface());

      auto fut = msg_interval_client_->async_send_request(req);

      // Drive the executor until the future is ready or we time out.
      const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (fut.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
        if (std::chrono::steady_clock::now() > deadline) break;
        temp_exec->spin_some(std::chrono::milliseconds(10));
      }

      temp_exec->remove_node(this->get_node_base_interface());

      if (fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        if (fut.get()->success) {
          RCLCPP_INFO(get_logger(), "msg_interval OK: %s @ %.1f Hz", label, hz);
        } else {
          RCLCPP_WARN(get_logger(),
            "msg_interval rejected for %s — FCU may not support this message ID", label);
        }
        return;
      }
      RCLCPP_WARN(get_logger(),
        "msg_interval timeout for %s (attempt %d/3)", label, attempt);
    }
    RCLCPP_WARN(get_logger(),
      "msg_interval gave up for %s after 3 attempts.", label);
  };

  requestMsgInterval(245, 2.0f,  "EXTENDED_SYS_STATE");
  requestMsgInterval(147, 1.0f,  "BATTERY_STATUS");
  requestMsgInterval(32,  30.0f, "LOCAL_POSITION_NED");
  requestMsgInterval(24,  2.0f,  "GPS_RAW_INT");
  requestMsgInterval(31,  30.0f, "ATTITUDE_QUATERNION");

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

void ControlGateNode::updateSetpointBlockStateAndMaybePublish(
  bool blocked, bool initial_publish)
{
  if (!setpoint_blocked_initialized_) {
    setpoint_blocked_             = blocked;
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
// Altitude controller state transitions  (unified AltHold design)
//
//  AltHold is entered automatically after takeoff whenever in guided mode.
//  It has two sub-states controlled by alt_hold_timed_out_:
//    false → operator flying; only vz is controlled, xy/yaw pass through
//    true  → no teleop for guided_timeout_s_; full hover (xy/yaw zeroed too)
// ============================================================================

void ControlGateNode::enterAltHold(
  float target_agl, float current_agl, float current_vz_mps)
{
  alt_ctrl_mode_             = AltCtrlMode::AltHold;
  alt_ctrl_target_agl_       = target_agl;
  alt_hold_timed_out_        = false;
  alt_hold_last_vz_nonzero_  = false;

  RCLCPP_INFO(get_logger(),
    "[alt_hold] Entered AltHold — target=%.3f m, current=%.3f m",
    target_agl, current_agl);
  publishInfo(DroneInfo::LEVEL_INFO,
    stringf("AltHold active. Holding %.2f m", target_agl));

  publishAltCtrlInput(/*active=*/true, alt_ctrl_target_agl_);
}

void ControlGateNode::exitAltHold()
{
  alt_ctrl_mode_      = AltCtrlMode::Off;
  alt_hold_timed_out_ = false;
  publishAltCtrlInput(/*active=*/false, /*target_agl_m=*/0.0f);

  RCLCPP_INFO(get_logger(), "[alt_hold] Exited AltHold.");
  publishInfo(DroneInfo::LEVEL_INFO, "AltHold deactivated.");
}

void ControlGateNode::enterTimedOut(const VerticalEstimateCache& ve)
{
  // Snapshot current agl as the hold target — "stay exactly where you are".
  // A subsequent set_target_height service call can override this.
  alt_ctrl_target_agl_ = ve.agl_m;
  alt_hold_timed_out_  = true;

  RCLCPP_WARN(get_logger(),
    "[alt_hold] Timed out — full hover at %.3f m", alt_ctrl_target_agl_);
  publishInfo(DroneInfo::LEVEL_WARN,
    stringf("AltHold timed out. Hovering at %.2f m", alt_ctrl_target_agl_));

  publishAltCtrlInput(/*active=*/true, alt_ctrl_target_agl_);
}

void ControlGateNode::exitTimedOut()
{
  alt_hold_timed_out_ = false;

  RCLCPP_INFO(get_logger(), "[alt_hold] Timeout cleared — operator resumed.");
  publishInfo(DroneInfo::LEVEL_INFO, "AltHold timeout cleared.");
}


// ============================================================================
// onGuidedTimeoutWatchdog  (timer callback, ~10 Hz)
//
// Responsibilities:
//   1. Enter AltHold when conditions are met (taken off, guided, no failsafe)
//   2. Exit AltHold when conditions are lost
//   3. Flip timed_out flag when operator goes quiet for guided_timeout_s_
// ============================================================================

void ControlGateNode::onGuidedTimeoutWatchdog() {
  if (inInitializationPhase()) return;

  const InternalState st = snapshotState();

  // Exit AltHold if conditions are lost
  if (!st.guided || !st.armed || isAnyFailsafeActive()) {
    if (alt_ctrl_mode_ == AltCtrlMode::AltHold) {
      exitAltHold();
    }
    return;
  }

  if (alt_ctrl_mode_ == AltCtrlMode::Off) return;  // ← nothing to do, user hasn't toggled it on

  // Inside AltHold: manage timed_out sub-state
  const double elapsed =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - readLastAct()).count();

  if (!alt_hold_timed_out_ && elapsed >= guided_timeout_s_) {
    const VerticalEstimateCache ve = snapshotVertEst();
    if (!ve.valid) return;
    enterTimedOut(ve);
  } else if (alt_hold_timed_out_ && elapsed < guided_timeout_s_) {
    exitTimedOut();
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

  const auto  now     = std::chrono::steady_clock::now();
  const float elapsed =
    std::chrono::duration<float>(now - takeoff_start_t_).count();

  if (elapsed >= takeoff_timeout_s_) {
    RCLCPP_WARN(get_logger(),
      "[takeoff_monitor] Hard timeout after %.1f s — stopping.", elapsed);
    publishInfo(DroneInfo::LEVEL_WARN, "Takeoff monitor timed out.");
    takeoff_monitor_active_ = false;
    has_taken_off_ = true;  
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
    has_taken_off_ = true;   // ← MOVE it here
    takeoff_monitor_timer_->cancel();
    return;
  }

  // Still climbing — reset guided timeout clock
  updateLastAct(std::chrono::steady_clock::now());
}