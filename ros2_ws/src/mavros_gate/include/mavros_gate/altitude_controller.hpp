#ifndef MAVROS_GATE__ALTITUDE_CONTROLLER_HPP_
#define MAVROS_GATE__ALTITUDE_CONTROLLER_HPP_

/*
 * altitude_controller.hpp
 *
 * Motion-aware altitude controller node.
 *
 * ── Subscriptions ────────────────────────────────────────────────────────────
 *
 *  <base_ns>/altitude_controller/input   (drone_msgs/AltitudeControllerInput)
 *    COMMAND topic — reliable QoS.
 *    Carries only control commands from control_gate:
 *      active          – true  → run controller
 *                        false → stop & reset
 *      target_agl_m    – desired hold altitude [m]
 *      reset_integral  – edge-triggered: zero the integral before next tick
 *
 *  <base_ns>/<mcu_bridge_topic>          (drone_msgs/McuVerticalEstimate)
 *    MEASUREMENT topic — sensor QoS (best-effort, keep-last 1).
 *    Published directly by mcu_bridge at ~20 Hz:
 *      z_world_m      (height above origin, m)
 *      vz_world_mps   (vertical velocity,   m/s)
 *      agl_m          (above-ground-level,  m)
 *    The controller reads agl_m and vz_world_mps from this topic.
 *    Measurements are IGNORED while active == false.
 *
 * ── Publications ─────────────────────────────────────────────────────────────
 *
 *  <base_ns>/altitude_controller/output  (std_msgs/Float32)
 *    Commanded vertical velocity [m/s] (positive = up).
 *    Published at MCU rate while active == true.
 *
 * ── Services ─────────────────────────────────────────────────────────────────
 *
 *  <base_ns>/altitude_controller/set_pid_gains  (drone_msgs/srv/SetPidGains)
 *    On-air gain tuning: { float32 kp, ki, kd }
 *    Pass -1.0 for any gain to leave it unchanged.
 *
 * ── Config (read once at startup from mavros_config/config/control_params.yaml) ──
 *
 *  altitude_controller:
 *    kp:              1.2
 *    ki:              0.05
 *    kd:              0.3
 *    d_lpf_tau_s:     0.1   # time constant for LPF on vz used by D term
 *    output_min:     -2.0   # m/s clamp
 *    output_max:      2.0
 *    integral_min:   -0.5   # anti-windup clamp on integral term
 *    integral_max:    0.5
 *
 * ── PID design ───────────────────────────────────────────────────────────────
 *
 *  error  = target_agl - current_agl
 *
 *  P term = kp * error
 *
 *  I term = integral of error × dt, clamped to [integral_min, integral_max]
 *           Reset to 0 whenever active goes false OR reset_integral arrives.
 *
 *  D term = -kd * current_vz   (derivative-on-measurement: avoids derivative
 *           kick when the target changes; negative because positive vz already
 *           reduces the error). current_vz is low-pass filtered before use
 *           when d_lpf_tau_s > 0.
 *
 *  output = clamp(P + I + D, output_min, output_max)
 *
 *  The node is reactive: it runs once per MCU measurement while active.
 *  dt for the integral comes from wall-clock time between active measurements.
 */

#include <chrono>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <drone_msgs/msg/altitude_controller_input.hpp>
#include <drone_msgs/msg/mcu_vertical_estimate.hpp>
#include <drone_msgs/srv/set_pid_gains.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>


class AltitudeControllerNode : public rclcpp::Node
{
public:
  explicit AltitudeControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using AltCtrlInput    = drone_msgs::msg::AltitudeControllerInput;
  using AltCtrlOutput   = std_msgs::msg::Float32;
  using VerticalEst     = drone_msgs::msg::McuVerticalEstimate;
  using SetPidGains     = drone_msgs::srv::SetPidGains;

  // ── Config ────────────────────────────────────────────────────────────────
  bool loadConfig();

  // ── PID helpers ───────────────────────────────────────────────────────────
  void  resetPid();
  float computeDesiredVelocity(float target_agl, float current_agl,
                               float current_vz, double dt_s);

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void onCommand(const AltCtrlInput::SharedPtr msg);          // control_gate commands
  void onMcuEstimate(const VerticalEst::SharedPtr msg);       // mcu_bridge measurements
  void onSetPidGains(
    const std::shared_ptr<SetPidGains::Request>  req,
    std::shared_ptr<SetPidGains::Response>       res);

  // ── Topic paths ───────────────────────────────────────────────────────────
  std::string topic_cmd_;       // alt_ctrl_input  (command, from control_gate)
  std::string topic_output_;    // alt_ctrl_output
  std::string topic_mcu_;       // mcu_bridge topic (measurements)

  // ── PID gains (mutex-protected for live tuning) ───────────────────────────
  mutable std::mutex gains_mtx_;
  float kp_{1.2f};
  float ki_{0.05f};
  float kd_{0.3f};
  float d_lpf_tau_s_{0.0f};

  // ── PID output params ─────────────────────────────────────────────────────
  float output_min_{-2.0f};
  float output_max_{ 2.0f};
  float integral_min_{-0.5f};
  float integral_max_{ 0.5f};

  // ── Controller state ──────────────────────────────────────────────────────
  bool   active_{false};           // mirrors last command's active flag
  float  target_agl_{0.0f};       // current hold target [m]

  bool   pid_initialized_{false};
  float  integral_{0.0f};
  std::optional<float> filtered_vz_;
  std::chrono::steady_clock::time_point last_stamp_;

  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::Subscription<AltCtrlInput>::SharedPtr  sub_cmd_;
  rclcpp::Subscription<VerticalEst>::SharedPtr   sub_mcu_;
  rclcpp::Publisher<AltCtrlOutput>::SharedPtr    pub_output_;
  rclcpp::Service<SetPidGains>::SharedPtr        srv_set_gains_;
};

#endif  // MAVROS_GATE__ALTITUDE_CONTROLLER_HPP_
