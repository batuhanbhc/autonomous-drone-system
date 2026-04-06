#ifndef MAVROS_GATE__COMMAND_GATE_HPP_
#define MAVROS_GATE__COMMAND_GATE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <sstream>
#include <iomanip>
#include <unordered_map>
#include <functional>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <vector>
#include <utility>
#include <atomic>

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <drone_msgs/msg/teleop_action.hpp>
#include <drone_msgs/msg/teleop_command.hpp>
#include <drone_msgs/msg/drone_state.hpp>
#include <drone_msgs/msg/drone_info.hpp>
#include <drone_msgs/msg/gcs_heartbeat.hpp>
#include <drone_msgs/msg/altitude_controller_input.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/message_interval.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>


class ControlGateNode : public rclcpp::Node {
public:
  ControlGateNode();

  struct CommandResult {
    bool success{false};
    std::string text;
  };

private:
  using TeleopCmd          = drone_msgs::msg::TeleopCommand;
  using TeleopAct          = drone_msgs::msg::TeleopAction;
  using DroneInfo          = drone_msgs::msg::DroneInfo;
  using DroneState         = drone_msgs::msg::DroneState;
  using GcsHeartbeat       = drone_msgs::msg::GcsHeartbeat;
  using MavrosExtendedState = mavros_msgs::msg::ExtendedState;
  using VerticalEstimate   = geometry_msgs::msg::Vector3Stamped;
  using AltCtrlInput       = drone_msgs::msg::AltitudeControllerInput;
  using AltCtrlOutput      = std_msgs::msg::Float32;  // vz m/s from PID

  using MonotonicTime = std::chrono::steady_clock::time_point;

  enum class ControlMode : std::uint8_t {
    Auto   = 0,
    Manual = 1
  };

  // ── Altitude controller state machine ──────────────────────────────────
  //
  //  GuidedTimeout  – mandatory, auto-triggered when no teleop action for
  //                   guided_timeout_s_. Sends vx=vy=yaw=0, vz=ctrl_output.
  //                   Cancelled by any incoming teleop action.
  //
  //  AltSupport     – optional, command-toggled (ALT_SUPPORT_TOGGLE).
  //                   Incoming teleop action with vz==0 → vz replaced by
  //                   ctrl_output. vz!=0 → bool=false, pass through.
  //                   Target agl snapshot on first vz==0 after each vz!=0.
  enum class AltCtrlMode : uint8_t {
    Off           = 0,
    GuidedTimeout = 1,
    AltSupport    = 2,
  };

  struct TopicPaths {
    std::string autonomous_action;
    std::string manual_action;
    std::string manual_command;
    std::string mcu_bridge;
    std::string alt_ctrl_input;   // publish target
    std::string alt_ctrl_output;  // subscribe source
  };

  struct VelocityLevel {
    double hv  = 1.0;
    double vv  = 0.5;
    double yaw = 0.5;  // rad/s
  };

  struct InternalState {
    ControlMode control_mode{ControlMode::Manual};
    VelocityLevel vel{0.0, 0.0};
    bool keyboard_on{true};
    bool safety_switch_on{true};
    bool connected{false};
    bool armed{false};
    bool guided{false};
    bool kill_switch_window{false};
    bool system_killed{false};
  };

  struct InternalStateUpdate {
    std::optional<ControlMode> control_mode;
    std::optional<VelocityLevel> vel;
    std::optional<bool> keyboard_on;
    std::optional<bool> safety_switch_on;
    std::optional<bool> connected;
    std::optional<bool> armed;
    std::optional<bool> guided;
    std::optional<bool> kill_switch_window;
    std::optional<bool> system_killed;
  };

  // Latest MCU vertical estimate — written from onVerticalEstimate,
  // read from timer callbacks and onTeleopAction (all on same executor thread
  // with single-threaded spin, but guarded by mutex for clarity / future safety)
  struct VerticalEstimateCache {
    float agl_m{0.0f};
    float z_world_m{0.0f};
    float vz_mps{0.0f};
    bool  valid{false};
  };

  template <typename... Args>
  static std::string stringf(const char* fmt, Args&&... args) {
    if (fmt == nullptr) return {};
    const int n = std::snprintf(nullptr, 0, fmt, std::forward<Args>(args)...);
    if (n <= 0) return {};
    std::string out;
    out.resize(static_cast<size_t>(n));
    std::snprintf(out.data(), static_cast<size_t>(n) + 1, fmt, std::forward<Args>(args)...);
    return out;
  }

  // command dispatch map
  std::unordered_map<uint8_t, std::function<CommandResult(const TeleopCmd&, const InternalState&)>> cmd_handlers_;
  static const char* commandName(int8_t);

  // ── Command handlers ──────────────────────────────────────────────────────
  CommandResult executeArm(const TeleopCmd&, const InternalState&);
  CommandResult executeDisarm(const TeleopCmd&, const InternalState&);
  CommandResult executeKillSwitch(const TeleopCmd&, const InternalState&);
  CommandResult executeKillConfirm(const TeleopCmd&, const InternalState&);
  CommandResult executeLand(const TeleopCmd&, const InternalState&);
  CommandResult executeRTL(const TeleopCmd&, const InternalState&);
  CommandResult executeGuided(const TeleopCmd&, const InternalState&);
  CommandResult executeTakeoff(const TeleopCmd&, const InternalState&);
  CommandResult executeControlToggle(const TeleopCmd&, const InternalState&);
  CommandResult executeKeyboardToggle(const TeleopCmd&, const InternalState&);
  CommandResult executeChangeSpeedHorizontal(const TeleopCmd&, const InternalState&);
  CommandResult executeChangeSpeedVertical(const TeleopCmd&, const InternalState&);
  CommandResult executeChangeSpeedYaw(const TeleopCmd&, const InternalState&);
  CommandResult executePressSafetySwitch(const TeleopCmd&, const InternalState&);
  CommandResult executeAltSupportToggle(const TeleopCmd&, const InternalState&);

  void initCommandHandlers();

  // ── Config / flight params ────────────────────────────────────────────────
  TopicPaths topics_;
  std::string base_ns_;
  float  takeoff_m_{1.5f};
  float  failsafe_watchdog_hz_{1.0f};
  double gcs_failsafe_s_{5.0};
  double guided_timeout_s_{3.0};
  float  guided_timeout_watchdog_hz_{10.0f};
  float  takeoff_reach_threshold_m_{0.2f};
  float  takeoff_timeout_s_{15.0f};

  // ── Runtime flags ─────────────────────────────────────────────────────────
  bool setpoint_blocked_{true};
  bool setpoint_blocked_initialized_{false};
  bool initialization_phase_{true};

  MonotonicTime last_action_t_;

  std::atomic<uint8_t> fcu_state_{0};
  std::atomic<bool>    critical_state_{false};

  std::atomic<int64_t> time_since_heartbeat_ns_{-1};
  std::atomic<int16_t> gcs_id_{-1};
  std::atomic<bool>    gcs_connected_{false};
  std::atomic<bool>    gcs_failsafe_{false};

  // ── Vertical estimate cache ───────────────────────────────────────────────
  mutable std::mutex    vert_est_mtx_;
  VerticalEstimateCache vert_est_;

  // ── Altitude controller runtime ───────────────────────────────────────────
  // All of these are only accessed on the ROS executor thread
  // (single-threaded spin), so no extra mutex is needed.
  AltCtrlMode alt_ctrl_mode_{AltCtrlMode::Off};

  float alt_ctrl_fixed_agl_{0.0f};    // target snapshot when mode activates
  float alt_ctrl_fixed_z_{0.0f};

  float alt_ctrl_vz_output_{0.0f};    // latest vz from PID (m/s)
  bool  alt_ctrl_output_fresh_{false};

  // AltSupport: remember last vz direction to detect nonzero→zero transition
  bool alt_support_last_vz_nonzero_{false};

  // Takeoff monitor
  bool          takeoff_monitor_active_{false};
  MonotonicTime takeoff_start_t_;

  // ── General helpers ───────────────────────────────────────────────────────
  void updateInternalStateAtomic(const InternalStateUpdate &);
  bool loadConfig();
  bool initializationRoutine();
  bool inInitializationPhase() const;
  float getTakeoffMeters() const;
  InternalState snapshotState() const;
  bool isCommandAlwaysEnabled(int8_t);
  MonotonicTime readLastAct() const;
  void updateLastAct(const MonotonicTime&);
  bool isSetpointBlocked(const InternalState& st) const;
  VerticalEstimateCache snapshotVertEst() const;
  bool isAnyFailsafeActive() const;

  // Altitude controller helpers
  void enterGuidedTimeout(const VerticalEstimateCache& ve);
  void exitGuidedTimeout();   // transitions to AltSupport or Off
  void enterAltSupport(float agl, float vz_mps);
  void exitAltSupport();
  void publishAltCtrlInput(bool active, float agl, float vz_mps);
  void publishSetpoint(float vx, float vy, float vz, float yaw_rate);

  // ── Subscriber callbacks ──────────────────────────────────────────────────
  void onTeleopCommand(const TeleopCmd::SharedPtr);
  void onTeleopAction(const TeleopAct::SharedPtr);
  void onMavrosState(const mavros_msgs::msg::State::SharedPtr);
  void onPublishStateTimer();
  void onGcsHeartbeat(const GcsHeartbeat::SharedPtr);
  void onFailsafeWatchdog();
  void onVerticalEstimate(const VerticalEstimate::SharedPtr);
  void onAltCtrlOutput(const AltCtrlOutput::SharedPtr);

  // Timer callbacks
  void onGuidedTimeoutWatchdog();
  void onTakeoffMonitorTick();

  // ── Publisher helpers ─────────────────────────────────────────────────────
  void publishInfo(uint8_t level, const std::string& text);
  void updateSetpointBlockStateAndMaybePublish(bool blocked, bool initial_publish);

  // ── Mutexes ───────────────────────────────────────────────────────────────
  mutable std::mutex state_mtx_;
  mutable std::mutex last_act_mtx_;

  // ── Internal state ────────────────────────────────────────────────────────
  InternalState state_;

  // ── Subscriptions ─────────────────────────────────────────────────────────
  rclcpp::Subscription<TeleopCmd>::SharedPtr        sub_teleop_command_;
  rclcpp::Subscription<TeleopAct>::SharedPtr        sub_teleop_action_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_mavros_state_;
  rclcpp::Subscription<GcsHeartbeat>::SharedPtr     sub_gcs_heartbeat_;
  rclcpp::Subscription<VerticalEstimate>::SharedPtr sub_vertical_estimate_;
  rclcpp::Subscription<AltCtrlOutput>::SharedPtr    sub_alt_ctrl_output_;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_setpoint_raw_local_;
  rclcpp::Publisher<DroneState>::SharedPtr    pub_control_state_;
  rclcpp::Publisher<DroneInfo>::SharedPtr     pub_drone_info_;
  rclcpp::Publisher<AltCtrlInput>::SharedPtr  pub_alt_ctrl_input_;

  // ── Services ──────────────────────────────────────────────────────────────
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr     arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr     command_long_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr         set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr      takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr msg_interval_client_;

  // ── Timers ────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr kill_switch_timer_;
  rclcpp::TimerBase::SharedPtr state_pub_timer_;
  rclcpp::TimerBase::SharedPtr gcs_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr guided_timeout_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr takeoff_monitor_timer_;
};

#endif  // MAVROS_GATE__COMMAND_GATE_HPP_