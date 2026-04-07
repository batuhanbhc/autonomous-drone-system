#ifndef MAVROS_GATE__ALTITUDE_CONTROLLER_HPP_
#define MAVROS_GATE__ALTITUDE_CONTROLLER_HPP_

/*
 * altitude_controller.hpp
 *
 * Motion-aware altitude controller node.
 *
 * Subscribes to:
 *   <base_ns>/altitude_controller/input   (drone_msgs/AltitudeControllerInput)
 *     active          – true  → run controller and publish output
 *                       false → reset & stop
 *     target_agl_m    – desired altitude above ground [m]
 *     current_agl_m   – measured altitude above ground [m]
 *     current_vz_mps  – measured vertical velocity    [m/s]
 *
 * Publishes to:
 *   <base_ns>/altitude_controller/output  (std_msgs/Float32)
 *     Commanded vertical velocity [m/s] (positive = up)
 *
 * Services (for on-air tuning):
 *   <base_ns>/altitude_controller/set_pid_gains
 *     (drone_msgs/srv/SetPidGains)  { float32 kp, ki, kd }
 *
 * Config (read once at startup from mavros_config/config/control_params.yaml):
 *   altitude_controller:
 *     kp:              1.2
 *     ki:              0.05
 *     kd:              0.3
 *     output_min:     -2.0   # m/s clamp
 *     output_max:      2.0
 *     integral_min:   -0.5   # anti-windup clamp on integral term
 *     integral_max:    0.5
 *     max_accel_mps2:  2.0   # achievable vertical acceleration magnitude
 *     command_hz:     20.0   # expected rate of incoming controller updates
 *
 * Motion model
 * ────────────
 * The controller does not assume commanded velocity is achieved instantly.
 * Instead it models two limits:
 *   1. The vehicle cannot stop instantly, so the altitude error is evaluated
 *      against a motion-aware altitude = current_agl + stopping_distance,
 *      where stopping_distance = sign(vz) * vz² / (2 * max_accel).
 *   2. The next published velocity command is limited to what can be reached
 *      within one command period (1 / command_hz) at max_accel.
 *
 * Soft-PID design
 * ───────────────
 *  error     = target_agl - motion_aware_agl
 *
 *  P term    = kp * error
 *
 *  I term    = integral of error × dt, clamped to [integral_min, integral_max]
 *              Reset to 0 whenever active goes false.
 *
 *  D term    = -kd * current_vz   (derivative-on-measurement: avoids derivative
 *              kick when the target changes; negative because positive vz already
 *              reduces the error)
 *
 *  desired_vz_steady = clamp(P + I + D, output_min, output_max)
 *  output            = current_vz slewed toward desired_vz_steady with
 *                      acceleration limit max_accel over one command period.
 *
 * The node is reactive: it runs once per incoming message. dt for the integral
 * comes from wall-clock time between active messages, while command_hz defines
 * the nominal actuation horizon used by the motion model.
 */

#include <chrono>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <drone_msgs/msg/altitude_controller_input.hpp>
#include <drone_msgs/srv/set_pid_gains.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>


class AltitudeControllerNode : public rclcpp::Node
{
public:
  AltitudeControllerNode();

private:
  using AltCtrlInput = drone_msgs::msg::AltitudeControllerInput;
  using AltCtrlOutput = std_msgs::msg::Float32;
  using SetPidGains = drone_msgs::srv::SetPidGains;

  bool loadConfig();

  void resetPid();
  float computeDesiredVelocity(float target_agl, float motion_aware_agl,
                               float current_vz, double dt_s);
  float computeMotionAwareAltitude(float current_agl, float current_vz) const;
  float computeCommandDt(double measured_dt_s) const;
  float slewVelocityCommand(float current_vz, float target_vz_cmd, double dt_s) const;

  void onInput(const AltCtrlInput::SharedPtr msg);
  void onSetPidGains(
    const std::shared_ptr<SetPidGains::Request>  req,
    std::shared_ptr<SetPidGains::Response>       res);

  std::string topic_input_;
  std::string topic_output_;

  mutable std::mutex gains_mtx_;
  float kp_{1.2f};
  float ki_{0.05f};
  float kd_{0.3f};

  float output_min_{-2.0f};
  float output_max_{ 2.0f};
  float integral_min_{-0.5f};
  float integral_max_{ 0.5f};
  float max_accel_mps2_{2.0f};
  float command_hz_{20.0f};

  bool  pid_initialized_{false};
  float integral_{0.0f};
  std::chrono::steady_clock::time_point last_stamp_;

  rclcpp::Subscription<AltCtrlInput>::SharedPtr  sub_input_;
  rclcpp::Publisher<AltCtrlOutput>::SharedPtr    pub_output_;
  rclcpp::Service<SetPidGains>::SharedPtr        srv_set_gains_;
};

#endif  // MAVROS_GATE__ALTITUDE_CONTROLLER_HPP_