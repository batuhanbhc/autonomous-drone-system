#include "mavros_gate/control_gate.hpp"


void ControlGateNode::onPublishStateTimer() {
  if (inInitializationPhase()) return;

  const InternalState st = snapshotState();

  drone_msgs::msg::ControlState msg;

  // control_mode enum mapping
  msg.control_mode =
    (st.control_mode == ControlMode::Auto)
      ? drone_msgs::msg::ControlState::CONTROL_MODE_AUTO
      : drone_msgs::msg::ControlState::CONTROL_MODE_MANUAL;

  msg.velocity_h = st.vel.hv;
  msg.velocity_v = st.vel.vv;

  msg.keyboard_on = st.keyboard_on;
  msg.safety_switch_on = st.safety_switch_on;
  msg.system_killed = st.system_killed;

  // time_since_action = now - last_action_t (steady clock)
  const auto now = std::chrono::steady_clock::now();

  // If last_action_t was never set, publish 0 duration
  if (st.last_action_t.time_since_epoch().count() == 0) {
    msg.time_since_action.sec = 0;
    msg.time_since_action.nanosec = 0;
  } else {
    auto dt = now - st.last_action_t;

    // Clamp negative just in case (shouldn't happen with steady_clock)
    if (dt < std::chrono::steady_clock::duration::zero()) {
      dt = std::chrono::steady_clock::duration::zero();
    }

    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(dt).count();
    const int64_t sec = ns / 1000000000LL;
    const int64_t nsec = ns % 1000000000LL;

    // builtin_interfaces/Duration uses int32 sec + uint32 nanosec
    msg.time_since_action.sec = static_cast<int32_t>(sec);
    msg.time_since_action.nanosec = static_cast<uint32_t>(nsec);
  }

  pub_control_state_->publish(msg);
}