#include "mavros_gate/control_gate.hpp"


// ============================================================================
// onPublishStateTimer  (1 Hz)
// ============================================================================

void ControlGateNode::onPublishStateTimer() {
  if (inInitializationPhase()) return;

  const InternalState st       = snapshotState();
  const MonotonicTime last_act = readLastAct();

  DroneState msg;

  // control_mode
  msg.control_mode =
    (st.control_mode == ControlMode::Auto)
      ? DroneState::CONTROL_MODE_AUTO
      : DroneState::CONTROL_MODE_MANUAL;

  // alt_ctrl_mode
  if (alt_ctrl_mode_ == AltCtrlMode::Off) {
    msg.alt_ctrl_mode = DroneState::ALT_CTRL_OFF;
  } else if (alt_hold_timed_out_) {
    msg.alt_ctrl_mode = DroneState::ALT_CTRL_GUIDED_TIMEOUT;  // reuse existing constant
  } else {
    msg.alt_ctrl_mode = DroneState::ALT_CTRL_SUPPORT;         // reuse: AltHold active
  }

  msg.velocity_h       = st.vel.hv;
  msg.velocity_v       = st.vel.vv;
  msg.velocity_yaw     = static_cast<float>(st.vel.yaw);
  msg.keyboard_on      = st.keyboard_on;
  msg.safety_switch_on = st.safety_switch_on;
  msg.system_killed    = st.system_killed;

  // time_since_action
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
// publishSetpoint  — single point of entry for all MAVLink setpoint publishing
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
// publishAltCtrlInput  — sends all four fields the PID node needs
//
//   active          – true  → controller should run; false → stop & reset
//   target_agl_m    – desired altitude above ground [m]
//   current_agl_m   – measured altitude above ground [m]  (from MCU)
//   current_vz_mps  – measured vertical velocity [m/s]    (from MCU)
//
// Previously only three values (active, agl, vz_mps) were sent and the
// meaning of the fields was ambiguous.  The PID node needs to know both
// the *target* and the *current* altitude to compute the error; without
// current_agl_m it cannot produce a meaningful output.
// ============================================================================

void ControlGateNode::publishAltCtrlInput(
  bool  active,
  float target_agl_m,
  float current_agl_m,
  float current_vz_mps,
  bool reset_integral)
{
  AltCtrlInput msg;
  msg.active         = active;
  msg.target_agl_m   = target_agl_m;
  msg.current_agl_m  = current_agl_m;
  msg.current_vz_mps = current_vz_mps;
  msg.reset_integral  = reset_integral;
  pub_alt_ctrl_input_->publish(msg);
}