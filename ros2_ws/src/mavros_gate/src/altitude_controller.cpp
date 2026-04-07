#include "mavros_gate/altitude_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <rclcpp/qos.hpp>


// ============================================================================
// constructor
// ============================================================================

AltitudeControllerNode::AltitudeControllerNode()
: rclcpp::Node("altitude_controller")
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
  topic_input_  = base_ns + topic_input_;
  topic_output_ = base_ns + topic_output_;

  RCLCPP_INFO(get_logger(), "PID gains — kp=%.3f  ki=%.3f  kd=%.3f", kp_, ki_, kd_);
  RCLCPP_INFO(get_logger(), "Output clamp  [%.2f, %.2f] m/s", output_min_, output_max_);
  RCLCPP_INFO(get_logger(), "Integral clamp[%.2f, %.2f]",      integral_min_, integral_max_);
  RCLCPP_INFO(get_logger(), "Motion model  accel=%.2f m/s²  cmd_hz=%.2f", max_accel_mps2_, command_hz_);
  RCLCPP_INFO(get_logger(), "Subscribing  input : %s", topic_input_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing   output: %s", topic_output_.c_str());

  // ── QoS ──────────────────────────────────────────────────────────────────
  //  input:  reliable — we must not miss active/inactive transitions
  //  output: sensor QoS (best-effort) — matches control_gate subscription
  const auto qos_input  = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto qos_output = rclcpp::SensorDataQoS();

  // ── subscriber ──────────────────────────────────────────────────────────
  sub_input_ = this->create_subscription<AltCtrlInput>(
    topic_input_, qos_input,
    std::bind(&AltitudeControllerNode::onInput, this, std::placeholders::_1));

  // ── publisher ────────────────────────────────────────────────────────────
  pub_output_ = this->create_publisher<AltCtrlOutput>(topic_output_, qos_output);

  // ── on-air tuning service ────────────────────────────────────────────────
  const std::string srv_name = base_ns + "/altitude_controller/set_pid_gains";
  srv_set_gains_ = this->create_service<SetPidGains>(
    srv_name,
    std::bind(&AltitudeControllerNode::onSetPidGains, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(),
    "PID gain tuning service ready at: %s", srv_name.c_str());
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

  // ── topic paths (reuse custom_topics block used by control_gate) ──────────
  bool ok = true;
  auto get_topic = [&](const char* key, std::string& out) {
    try {
      out = root["custom_topics"][key].as<std::string>();
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(get_logger(), "YAML missing custom_topics.%s: %s", key, e.what());
      ok = false;
    }
  };
  get_topic("alt_ctrl_input",  topic_input_);
  get_topic("alt_ctrl_output", topic_output_);
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
      RCLCPP_WARN(get_logger(), "YAML altitude_controller.%s missing, using default %.3f — %s",
                  key, field, e.what());
    }
  };

  load("kp",             kp_);
  load("ki",             ki_);
  load("kd",             kd_);
  load("output_min",     output_min_);
  load("output_max",     output_max_);
  load("integral_min",   integral_min_);
  load("integral_max",   integral_max_);
  load("max_accel_mps2", max_accel_mps2_);
  load("command_hz",     command_hz_);

  if (command_hz_ <= 0.0f) {
    RCLCPP_WARN(get_logger(),
      "altitude_controller.command_hz must be > 0. Falling back to 20 Hz.");
    command_hz_ = 20.0f;
  }
  if (max_accel_mps2_ <= 0.0f) {
    RCLCPP_WARN(get_logger(),
      "altitude_controller.max_accel_mps2 must be > 0. Falling back to 2.0 m/s².");
    max_accel_mps2_ = 2.0f;
  }

  return true;
}


// ============================================================================
// PID / motion-model helpers
// ============================================================================

void AltitudeControllerNode::resetPid()
{
  integral_        = 0.0f;
  pid_initialized_ = false;
  RCLCPP_DEBUG(get_logger(), "[alt_ctrl_pid] state reset.");
}

float AltitudeControllerNode::computeMotionAwareAltitude(
  float current_agl, float current_vz) const
{
  const float stopping_distance =
    (current_vz * std::fabs(current_vz)) / (2.0f * max_accel_mps2_);
  return current_agl + stopping_distance;
}

float AltitudeControllerNode::computeCommandDt(double measured_dt_s) const
{
  const float nominal_dt_s = 1.0f / command_hz_;
  if (measured_dt_s > 0.0 && measured_dt_s <= 1.0) {
    return static_cast<float>(measured_dt_s);
  }
  return nominal_dt_s;
}

float AltitudeControllerNode::slewVelocityCommand(
  float current_vz, float target_vz_cmd, double dt_s) const
{
  const float cmd_dt_s = computeCommandDt(dt_s);
  const float max_delta = max_accel_mps2_ * cmd_dt_s;
  const float delta = std::clamp(target_vz_cmd - current_vz, -max_delta, max_delta);
  return std::clamp(current_vz + delta, output_min_, output_max_);
}

float AltitudeControllerNode::computeDesiredVelocity(
  float target_agl, float motion_aware_agl, float current_vz, double dt_s)
{
  float kp, ki, kd;
  {
    std::lock_guard<std::mutex> lk(gains_mtx_);
    kp = kp_;
    ki = ki_;
    kd = kd_;
  }

  const float error = target_agl - motion_aware_agl;

  const float p_term = kp * error;

  integral_ += ki * error * static_cast<float>(dt_s);
  integral_  = std::clamp(integral_, integral_min_, integral_max_);
  const float i_term = integral_;

  const float d_term = -kd * current_vz;

  const float raw = p_term + i_term + d_term;
  const float out = std::clamp(raw, output_min_, output_max_);

  RCLCPP_DEBUG(get_logger(),
    "[pid] tgt=%.3f motion_agl=%.3f err=%.3f vz=%.3f P=%.3f I=%.3f D=%.3f → %.3f (%.3f clamped)",
    target_agl, motion_aware_agl, error, current_vz, p_term, i_term, d_term, raw, out);

  return out;
}


// ============================================================================
// onInput  — main control callback
// ============================================================================

void AltitudeControllerNode::onInput(const AltCtrlInput::SharedPtr msg)
{
  if (!msg->active) {
    if (pid_initialized_) {
      RCLCPP_INFO(get_logger(), "[alt_ctrl_pid] Deactivated. Resetting state.");
      resetPid();
    }
    return;
  }

  if (msg->reset_integral && pid_initialized_) {
    integral_ = 0.0f;
    RCLCPP_INFO(get_logger(),
      "[alt_ctrl_pid] Integral reset (new target: %.3f m)", msg->target_agl_m);
  }

  const auto now = std::chrono::steady_clock::now();
  double dt_s = 1.0 / static_cast<double>(command_hz_);

  if (!pid_initialized_) {
    RCLCPP_INFO(get_logger(),
      "[alt_ctrl_pid] Activated. target_agl=%.3f m, current_agl=%.3f m, current_vz=%.3f m/s",
      msg->target_agl_m, msg->current_agl_m, msg->current_vz_mps);
    integral_        = 0.0f;
    pid_initialized_ = true;
  } else {
    dt_s = std::chrono::duration<double>(now - last_stamp_).count();
    if (dt_s <= 0.0 || dt_s > 1.0) {
      RCLCPP_WARN(get_logger(),
        "[alt_ctrl_pid] Abnormal dt=%.4f s — using nominal command period.", dt_s);
      dt_s = 1.0 / static_cast<double>(command_hz_);
    }
  }
  last_stamp_ = now;

  const float motion_aware_agl =
    computeMotionAwareAltitude(msg->current_agl_m, msg->current_vz_mps);

  const float desired_vz_steady = computeDesiredVelocity(
    msg->target_agl_m, motion_aware_agl, msg->current_vz_mps, dt_s);

  const float vz_cmd = slewVelocityCommand(
    msg->current_vz_mps, desired_vz_steady, dt_s);

  AltCtrlOutput out;
  out.data = vz_cmd;
  pub_output_->publish(out);

  RCLCPP_DEBUG(get_logger(),
    "[alt_ctrl_pid] tgt=%.3f cur=%.3f motion_agl=%.3f vz=%.3f dt=%.4f desired=%.3f cmd=%.3f",
    msg->target_agl_m, msg->current_agl_m, motion_aware_agl,
    msg->current_vz_mps, dt_s, desired_vz_steady, vz_cmd);
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
    "[alt_ctrl_pid] Gains  kp: %.4f→%.4f  ki: %.4f→%.4f  kd: %.4f→%.4f%s",
    old_kp, kp_change ? req->kp : old_kp,
    old_ki, ki_change ? req->ki : old_ki,
    old_kd, kd_change ? req->kd : old_kd,
    (!kp_change || !ki_change || !kd_change) ? "  (some unchanged)" : "");
}


// ============================================================================
// main
// ============================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AltitudeControllerNode>());
  rclcpp::shutdown();
  return 0;
}