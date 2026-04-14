#include "mavros_gate/control_gate.hpp"


// ============================================================================
// onPublishStateTimer  (1 Hz)
// ============================================================================

void ControlGateNode::onPublishStateTimer() {
  if (inInitializationPhase()) return;

  const InternalState st       = snapshotState();
  const MonotonicTime last_act = readLastAct();

  DroneState msg;

  msg.control_mode =
    (st.control_mode == ControlMode::Auto)
      ? DroneState::CONTROL_MODE_AUTO
      : DroneState::CONTROL_MODE_MANUAL;

  // alt_ctrl_mode
  if (alt_ctrl_mode_ == AltCtrlMode::Off) {
    msg.alt_ctrl_mode = DroneState::ALT_CTRL_OFF;
  } else if (alt_hold_operator_override_) {
    msg.alt_ctrl_mode = DroneState::ALT_CTRL_SUPPORT;          // operator flying
  } else {
    msg.alt_ctrl_mode = DroneState::ALT_CTRL_GUIDED_TIMEOUT;   // PID holding
  }

  msg.velocity_h       = st.vel.hv;
  msg.velocity_v       = st.vel.vv;
  msg.velocity_yaw     = static_cast<float>(st.vel.yaw);
  msg.keyboard_on      = st.keyboard_on;
  msg.safety_switch_on = st.safety_switch_on;
  msg.system_killed    = st.system_killed;

  const auto now = std::chrono::steady_clock::now();
  if (last_act.time_since_epoch().count() == 0) {
    msg.time_since_action.sec     = 0;
    msg.time_since_action.nanosec = 0;
  } else {
    auto dt = now - last_act;
    if (dt < std::chrono::steady_clock::duration::zero())
      dt = std::chrono::steady_clock::duration::zero();
    const auto ns              = std::chrono::duration_cast<std::chrono::nanoseconds>(dt).count();
    msg.time_since_action.sec  = static_cast<int32_t>(ns / 1'000'000'000LL);
    msg.time_since_action.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
  }

  pub_control_state_->publish(msg);
}


// ============================================================================
// publishInfo
// ============================================================================

void ControlGateNode::publishInfo(uint8_t level, const std::string& text) {
  if (!pub_drone_info_) return;
  drone_msgs::msg::DroneInfo msg;
  msg.stamp = this->now();
  msg.level = level;
  msg.text  = text;
  pub_drone_info_->publish(msg);
}


// ============================================================================
// publishSetpoint  — low-level MAVLink setpoint builder
// Called only by onGuidedSetpointTimer (AltHold path) and directly from
// onTeleopAction (non-AltHold path).
// ============================================================================

void ControlGateNode::publishSetpoint(float vx, float vy, float vz, float yaw_rate) {
  mavros_msgs::msg::PositionTarget sp;
  sp.header.stamp     = this->now();
  sp.header.frame_id  = "base_link";
  sp.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
  sp.type_mask =
    mavros_msgs::msg::PositionTarget::IGNORE_PX  |
    mavros_msgs::msg::PositionTarget::IGNORE_PY  |
    mavros_msgs::msg::PositionTarget::IGNORE_PZ  |
    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
    mavros_msgs::msg::PositionTarget::IGNORE_YAW;
  sp.velocity.x = vx;
  sp.velocity.y = vy;
  sp.velocity.z = vz;
  sp.yaw_rate   = yaw_rate;
  pub_setpoint_raw_local_->publish(sp);
}


// ============================================================================
// onGuidedSetpointTimer  — single point of entry for guided setpoints
//
// Runs at alt_ctrl_setpoint_hz_ while AltHold is active.
// Reads the current guided_cmd_ state, applies stale-zeroing to each
// component that has not been updated within cmd_stale_timeout_s_, then
// publishes the combined setpoint.
//
// Stale rules:
//   Vx, Vy, yaw_rate — zeroed if not updated within cmd_stale_timeout_s_
//   Vz               — supplied by PID (updated via updateGuidedVz from
//                      onAltCtrlOutput); zeroed if PID output is stale.
//                      When operator override is active, Vz is always 0.
// ============================================================================

void ControlGateNode::onGuidedSetpointTimer() {
  if (inInitializationPhase()) return;
  if (alt_ctrl_mode_ != AltCtrlMode::AltHold) return;

  const InternalState st = snapshotState();
  if (isSetpointBlocked(st)) return;

  const auto   now           = std::chrono::steady_clock::now();
  const double stale_s       = cmd_stale_timeout_s_;

  // Zero stale horizontal / yaw components.
  auto age = [&](const std::chrono::steady_clock::time_point& t) -> double {
    if (t.time_since_epoch().count() == 0) return 1e9;
    return std::chrono::duration<double>(now - t).count();
  };

  const float vx       = (age(last_vx_update_)  < stale_s) ? guided_cmd_.vx       : 0.0f;
  const float vy       = (age(last_vy_update_)  < stale_s) ? guided_cmd_.vy       : 0.0f;
  const float yaw_rate = (age(last_yaw_update_) < stale_s) ? guided_cmd_.yaw_rate : 0.0f;

  // Vz: use guided_cmd_.vz if fresh — it holds either operator Vz (when override
  // is active) or PID output (when override is inactive). Either way, stale = 0.
  const float vz = (age(last_vz_update_) < stale_s) ? guided_cmd_.vz : 0.0f;

  publishSetpoint(vx, vy, vz, yaw_rate);
}


// ============================================================================
// publishAltCtrlInput
// ============================================================================

void ControlGateNode::publishAltCtrlInput(
  bool  active,
  float target_agl_m,
  bool  reset_integral)
{
  AltCtrlInput msg;
  msg.active         = active;
  msg.target_agl_m   = target_agl_m;
  msg.reset_integral = reset_integral;
  pub_alt_ctrl_input_->publish(msg);
}