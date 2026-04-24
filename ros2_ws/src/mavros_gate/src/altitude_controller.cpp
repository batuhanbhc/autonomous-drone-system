#include "mavros_gate/altitude_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <rclcpp/qos.hpp>


// ============================================================================
// constructor
// ============================================================================

AltitudeControllerNode::AltitudeControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("altitude_controller", options)
{
  RCLCPP_INFO(get_logger(), "altitude_controller starting");

  // ── drone namespace ──────────────────────────────────────────────────────
  this->declare_parameter<int>("drone_id", 0);
  int drone_id = 0;
  this->get_parameter("drone_id", drone_id);
  const std::string base_ns = "/drone_" + std::to_string(drone_id);

  // ── load config ──────────────────────────────────────────────────────────
  if (!loadConfig()) {
    RCLCPP_FATAL(get_logger(), "Failed to load config. Shutting down.");
    rclcpp::shutdown();
    throw std::runtime_error("altitude_controller init failed");
  }

  // Prepend namespace to topic paths loaded from config
  topic_cmd_    = base_ns + topic_cmd_;
  topic_output_ = base_ns + topic_output_;
  topic_mcu_    = base_ns + topic_mcu_;

  RCLCPP_INFO(get_logger(), "PID gains — kp=%.5f  ki=%.5f  kd=%.5f", kp_, ki_, kd_);
  RCLCPP_INFO(get_logger(), "D-term vz LPF tau: %.5f s", d_lpf_tau_s_);
  RCLCPP_INFO(get_logger(), "Output clamp  [%.2f, %.2f] m/s", output_min_, output_max_);
  RCLCPP_INFO(get_logger(), "Integral clamp[%.2f, %.2f]",      integral_min_, integral_max_);
  RCLCPP_INFO(get_logger(), "Subscribing  cmd    : %s", topic_cmd_.c_str());
  RCLCPP_INFO(get_logger(), "Subscribing  mcu    : %s", topic_mcu_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing   output : %s", topic_output_.c_str());

  // ── QoS ──────────────────────────────────────────────────────────────────
  //  cmd:    reliable — must not miss active/inactive/target transitions
  //  mcu:    sensor QoS (best-effort) — matches mcu_bridge publisher
  //  output: sensor QoS (best-effort) — matches control_gate subscription
  const auto qos_cmd    = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto qos_sensor = rclcpp::SensorDataQoS();

  // ── command subscriber (control_gate → altitude_controller) ──────────────
  sub_cmd_ = this->create_subscription<AltCtrlInput>(
    topic_cmd_, qos_cmd,
    std::bind(&AltitudeControllerNode::onCommand, this, std::placeholders::_1));

  // ── measurement subscriber (mcu_bridge → altitude_controller, direct) ────
  sub_mcu_ = this->create_subscription<VerticalEst>(
    topic_mcu_, qos_sensor,
    std::bind(&AltitudeControllerNode::onMcuEstimate, this, std::placeholders::_1));

  // ── output publisher ─────────────────────────────────────────────────────
  pub_output_ = this->create_publisher<AltCtrlOutput>(topic_output_, qos_sensor);

  // ── on-air tuning service ────────────────────────────────────────────────
  const std::string srv_name = base_ns + "/altitude_controller/set_pid_gains";
  srv_set_gains_ = this->create_service<SetPidGains>(
    srv_name,
    std::bind(&AltitudeControllerNode::onSetPidGains, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "PID gain tuning service ready at: %s", srv_name.c_str());
  RCLCPP_INFO(get_logger(), "altitude_controller ready.");
}


// ============================================================================
// loadConfig
// ============================================================================

bool AltitudeControllerNode::loadConfig()
{
  std::string yaml_path;
  try {
    const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("mavros_config");
    yaml_path = pkg_share + "/config/control_params.yaml";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Cannot find mavros_config package: %s", e.what());
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load YAML '%s': %s", yaml_path.c_str(), e.what());
    return false;
  }

  // ── topic paths ───────────────────────────────────────────────────────────
  bool ok = true;
  auto get_topic = [&](const char* key, std::string& out) {
    try {
      out = root["custom_topics"][key].as<std::string>();
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(get_logger(), "YAML missing custom_topics.%s: %s", key, e.what());
      ok = false;
    }
  };
  get_topic("alt_ctrl_input",  topic_cmd_);     // command from control_gate
  get_topic("alt_ctrl_output", topic_output_);  // output to control_gate
  get_topic("mcu_bridge",      topic_mcu_);     // direct MCU measurements
  if (!ok) return false;

  // ── controller parameters ────────────────────────────────────────────────
  YAML::Node ac = root["altitude_controller"];
  if (!ac || !ac.IsMap()) {
    RCLCPP_ERROR(get_logger(),
      "YAML missing 'altitude_controller' section in '%s'", yaml_path.c_str());
    return false;
  }

  auto load = [&](const char* key, float& field) {
    try {
      field = ac[key].as<float>();
    } catch (const YAML::Exception& e) {
      RCLCPP_WARN(get_logger(), "YAML altitude_controller.%s missing, using default %.5f — %s",
                  key, field, e.what());
    }
  };

  load("kp",             kp_);
  load("ki",             ki_);
  load("kd",             kd_);
  load("d_lpf_tau_s",    d_lpf_tau_s_);
  load("output_min",     output_min_);
  load("output_max",     output_max_);
  load("integral_min",   integral_min_);
  load("integral_max",   integral_max_);

  d_lpf_tau_s_ = std::max(0.0f, d_lpf_tau_s_);

  return true;
}


// ============================================================================
// PID helpers
// ============================================================================

void AltitudeControllerNode::resetPid()
{
  integral_        = 0.0f;
  pid_initialized_ = false;
  filtered_vz_.reset();
  RCLCPP_DEBUG(get_logger(), "[alt_ctrl_pid] State reset.");
}

float AltitudeControllerNode::computeDesiredVelocity(
  float target_agl, float current_agl, float current_vz, double dt_s)
{
  float kp, ki, kd;
  {
    std::lock_guard<std::mutex> lk(gains_mtx_);
    kp = kp_;
    ki = ki_;
    kd = kd_;
  }

  const float error  = target_agl - current_agl;
  const float p_term = kp * error;

  integral_ += ki * error * static_cast<float>(dt_s);
  integral_  = std::clamp(integral_, integral_min_, integral_max_);
  const float i_term = integral_;

  float d_input_vz = current_vz;
  if (d_lpf_tau_s_ > 0.0f) {
    if (!filtered_vz_.has_value()) {
      filtered_vz_ = current_vz;
    } else {
      const float alpha = static_cast<float>(dt_s / (static_cast<double>(d_lpf_tau_s_) + dt_s));
      filtered_vz_ = *filtered_vz_ + alpha * (current_vz - *filtered_vz_);
    }
    d_input_vz = *filtered_vz_;
  }

  const float d_term = -kd * d_input_vz;

  const float raw = p_term + i_term + d_term;
  const float out = std::clamp(raw, output_min_, output_max_);

  RCLCPP_DEBUG(get_logger(),
    "[pid] tgt=%.3f agl=%.3f err=%.3f vz=%.3f vz_f=%.3f P=%.3f I=%.3f D=%.3f → %.3f (%.3f clamped)",
    target_agl, current_agl, error, current_vz, d_input_vz, p_term, i_term, d_term, raw, out);

  return out;
}


// ============================================================================
// onCommand  — receives control commands from control_gate (reliable QoS)
//
// This callback is COMMAND-ONLY: it never performs a PID tick itself.
// It updates active_, target_agl_, and resets integral when asked.
// The actual PID computation is driven by onMcuEstimate().
// ============================================================================

void AltitudeControllerNode::onCommand(const AltCtrlInput::SharedPtr msg)
{
  if (!msg->active) {
    // Deactivate: stop producing output and reset all state.
    if (active_) {
      RCLCPP_INFO(get_logger(), "[alt_ctrl_pid] Deactivated. Resetting state.");
      active_ = false;
      resetPid();
    }
    return;
  }

  // Controller should be (or remain) active.
  if (!active_) {
    // Activation edge: log and initialise state.
    active_ = true;
    RCLCPP_INFO(get_logger(),
      "[alt_ctrl_pid] Activated. target_agl=%.3f m", msg->target_agl_m);
    // PID state will be initialised on the first MCU callback (pid_initialized_ == false).
    resetPid();
  }

  // Update target — always honour the latest command.
  const bool target_changed = (msg->target_agl_m != target_agl_);
  target_agl_ = msg->target_agl_m;

  if (target_changed) {
    RCLCPP_DEBUG(get_logger(),
      "[alt_ctrl_pid] Target updated to %.3f m", target_agl_);
  }

  // Handle explicit integral reset request.
  if (msg->reset_integral && pid_initialized_) {
    integral_ = 0.0f;
    RCLCPP_INFO(get_logger(),
      "[alt_ctrl_pid] Integral reset (target: %.3f m)", target_agl_);
  }
}


// ============================================================================
// onMcuEstimate  — measurement callback from mcu_bridge (sensor QoS)
//
// This is the PID tick: runs at MCU rate (~20 Hz) while the controller is
// active.  Measurement fields:
//   msg->vector.x  = z_world_m      (unused here)
//   msg->vector.y  = vz_world_mps   (vertical velocity, m/s)
//   msg->vector.z  = agl_m          (above-ground-level, m)
// ============================================================================

void AltitudeControllerNode::onMcuEstimate(const VerticalEst::SharedPtr msg)
{
  // Ignore all measurements while the controller is inactive.
  if (!active_) return;

  const float current_agl = static_cast<float>(msg->vector.z);  // agl_m
  const float current_vz  = static_cast<float>(msg->vector.y);  // vz_world_mps

  const auto now = std::chrono::steady_clock::now();

  if (!pid_initialized_) {
    RCLCPP_INFO(get_logger(),
      "[alt_ctrl_pid] First measurement: agl=%.3f m, vz=%.3f m/s — PID running.",
      current_agl, current_vz);
    integral_        = 0.0f;
    filtered_vz_     = current_vz;
    pid_initialized_ = true;
    last_stamp_      = now;
    return;  // skip first tick — no valid dt yet
  }

  const double dt_s = std::chrono::duration<double>(now - last_stamp_).count();
  last_stamp_ = now;

  if (dt_s <= 0.0 || dt_s > 1.0) {
    RCLCPP_WARN(get_logger(),
      "[alt_ctrl_pid] Abnormal dt=%.4f s — skipping tick.", dt_s);
    return;
  }

  const float vz_cmd = computeDesiredVelocity(
    target_agl_, current_agl, current_vz, dt_s);

  AltCtrlOutput out;
  out.data = vz_cmd;
  pub_output_->publish(out);

  // RCLCPP_INFO(get_logger(),
  //   "[alt_ctrl_pid] tgt=%.3f agl=%.3f vz=%.3f dt=%.4f cmd=%.3f",
  //   target_agl_, current_agl, current_vz, dt_s, vz_cmd);
}


// ============================================================================
// onSetPidGains  — live-tuning service
// ============================================================================

void AltitudeControllerNode::onSetPidGains(
  const std::shared_ptr<SetPidGains::Request>  req,
  std::shared_ptr<SetPidGains::Response>       res)
{
  const bool kp_change = (req->kp >= 0.0f);
  const bool ki_change = (req->ki >= 0.0f);
  const bool kd_change = (req->kd >= 0.0f);

  if (!kp_change && req->kp != -1.0f) {
    res->success = false;
    res->message = "Invalid kp: use a non-negative value or -1.0 to leave unchanged.";
    RCLCPP_WARN(get_logger(), "set_pid_gains rejected: %s", res->message.c_str());
    return;
  }
  if (!ki_change && req->ki != -1.0f) {
    res->success = false;
    res->message = "Invalid ki: use a non-negative value or -1.0 to leave unchanged.";
    RCLCPP_WARN(get_logger(), "set_pid_gains rejected: %s", res->message.c_str());
    return;
  }
  if (!kd_change && req->kd != -1.0f) {
    res->success = false;
    res->message = "Invalid kd: use a non-negative value or -1.0 to leave unchanged.";
    RCLCPP_WARN(get_logger(), "set_pid_gains rejected: %s", res->message.c_str());
    return;
  }

  float old_kp, old_ki, old_kd;
  {
    std::lock_guard<std::mutex> lk(gains_mtx_);
    old_kp = kp_;  old_ki = ki_;  old_kd = kd_;
    if (kp_change) kp_ = req->kp;
    if (ki_change) ki_ = req->ki;
    if (kd_change) kd_ = req->kd;
  }

  if (kp_change || ki_change || kd_change) {
    integral_ = 0.0f;
  }

  res->success = true;
  res->message = "Gains updated.";

  RCLCPP_INFO(get_logger(),
    "[alt_ctrl_pid] Gains  kp: %.5f→%.5f  ki: %.5f→%.5f  kd: %.5f→%.5f%s",
    old_kp, kp_change ? req->kp : old_kp,
    old_ki, ki_change ? req->ki : old_ki,
    old_kd, kd_change ? req->kd : old_kd,
    (!kp_change || !ki_change || !kd_change) ? "  (some unchanged)" : "");
}


// ============================================================================
// main  (standalone fallback — normally composed by mavros_gate_compositor)
// ============================================================================

#ifndef BUILDING_COMPOSITOR
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AltitudeControllerNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
