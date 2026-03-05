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

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <drone_msgs/msg/teleop_action.hpp>
#include <drone_msgs/msg/teleop_command.hpp>
#include <drone_msgs/msg/drone_state.hpp>
#include <drone_msgs/msg/drone_info.hpp>
#include <drone_msgs/msg/gcs_heartbeat.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
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
  using TeleopCmd = drone_msgs::msg::TeleopCommand;
  using TeleopAct = drone_msgs::msg::TeleopAction;
  using DroneInfo = drone_msgs::msg::DroneInfo;
  using DroneState = drone_msgs::msg::DroneState;
  using GcsHeartbeat = drone_msgs::msg::GcsHeartbeat;
  using MavrosExtendedState = mavros_msgs::msg::ExtendedState;

  using MonotonicTime = std::chrono::steady_clock::time_point;

  enum class ControlMode : std::uint8_t {
      Auto   = 0,
      Manual = 1
  };

  struct TopicPaths {
    std::string autonomous_action;
    std::string manual_action;
    std::string manual_command;
  };

  struct VelocityLevel {
    double hv;  // horizontal veloctiy
    double vv;  // vertical velocity
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

  // map from command ID to function pointers
  std::unordered_map<uint8_t, std::function<CommandResult(const TeleopCmd&, const InternalState&)>> cmd_handlers_;

  // map from command ID to command name
  static const char* commandName(int8_t);
  

  // command handlers
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
  CommandResult executeChangeSpeed(const TeleopCmd&, const InternalState&);
  CommandResult executePressSafetySwitch(const TeleopCmd&, const InternalState&);

  // function that initializes assigning command IDs to function pointers
  void initCommandHandlers();

  // members
  TopicPaths topics_;
  std::string base_ns_;
  float takeoff_m_;
  float failsafe_watchdog_hz_{1.0};
  bool setpoint_blocked_{true};
  bool setpoint_blocked_initialized_{false};  
  bool initialization_phase_{true};
  MonotonicTime last_action_t_;

  std::atomic<uint8_t> fcu_state_{0};
  std::atomic<bool> critical_state_{false};

  double gcs_failsafe_s_{5.0};
  std::atomic<int64_t> time_since_heartbeat_ns_{-1};
  std::atomic<int16_t> gcs_id_{-1};
  std::atomic<bool> gcs_connected_{false};
  std::atomic<bool> gcs_failsafe_{false};


  // helpers
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

  // subscriber callbacks
  void onTeleopCommand(const TeleopCmd::SharedPtr);
  void onTeleopAction(const TeleopAct::SharedPtr);
  void onMavrosState(const mavros_msgs::msg::State::SharedPtr);
  void onPublishStateTimer();
  void onGcsHeartbeat(const GcsHeartbeat::SharedPtr msg);
  void onFailsafeWatchdog();

  // publisher helpers
  void publishInfo(uint8_t level, const std::string& text);
  void updateSetpointBlockStateAndMaybePublish(bool blocked, bool initial_publish);

  // mutexes
  mutable std::mutex state_mtx_;
  mutable std::mutex last_act_mtx_;

  // internal state object
  InternalState state_;  

  // subscriptions
  rclcpp::Subscription<TeleopCmd>::SharedPtr sub_teleop_command_;
  rclcpp::Subscription<TeleopAct>::SharedPtr sub_teleop_action_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_mavros_state_;
  rclcpp::Subscription<GcsHeartbeat>::SharedPtr sub_gcs_heartbeat_;

  // publishers
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_setpoint_raw_local_;
  rclcpp::Publisher<DroneState>::SharedPtr pub_control_state_;
  rclcpp::Publisher<DroneInfo>::SharedPtr pub_drone_info_;

  // services
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_long_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr msg_interval_client_;

  // timers
  rclcpp::TimerBase::SharedPtr kill_switch_timer_;
  rclcpp::TimerBase::SharedPtr state_pub_timer_;
  rclcpp::TimerBase::SharedPtr gcs_watchdog_timer_;
};

#endif  // MAVROS_GATE__COMMAND_GATE_HPP_