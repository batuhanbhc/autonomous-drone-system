#ifndef MAVROS_GATE__COMMAND_GATE_HPP_
#define MAVROS_GATE__COMMAND_GATE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <teleop_msgs/msg/teleop_action.hpp>
#include <teleop_msgs/msg/teleop_command.hpp>

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

class CommandGateNode : public rclcpp::Node {
public:
  CommandGateNode();

  struct CommandResult {
    bool success{false};
    std::string text;
  };

private:
  struct TopicPaths {
    std::string autonomous_action;
    std::string manual_action;
    std::string manual_command;
  };

  struct InternalState {
    bool connected{false};
    bool armed{false};
    bool guided{false};
    int8_t last_command_id{-1};
    bool kill_switch_window{false};
    bool system_killed{false};
  };

  struct InternalStateUpdate {
    std::optional<bool> connected;
    std::optional<bool> armed;
    std::optional<bool> guided;
    std::optional<int8_t> last_command_id;
    std::optional<bool> kill_switch_window;
    std::optional<bool> system_killed;
  };

  using TeleopCmd = teleop_msgs::msg::TeleopCommand;

  // map from command ID to function pointers
  std::unordered_map<uint8_t, std::function<CommandResult(const TeleopCmd&, const InternalState&)>> cmd_handlers_;

  // map from command ID to command name
  static const char* commandName(int8_t id);

  // command handlers
  CommandResult executeArm(const TeleopCmd& msg, const InternalState&);
  CommandResult executeDisarm(const TeleopCmd& msg, const InternalState&);
  CommandResult executeKillSwitch(const TeleopCmd& msg, const InternalState&);
  CommandResult executeKillConfirm(const TeleopCmd& msg, const InternalState&);
  
  // function that initializes assigning command IDs to function pointers
  void initCommandHandlers();

  // helpers
  void updateInternalStateAtomic(const InternalStateUpdate & update);
  bool loadTopics();
  bool initializationRoutine();
  bool inInitializationPhase() const;
  InternalState snapshotState() const;

  // callbacks
  void onTeleopCommand(const teleop_msgs::msg::TeleopCommand::SharedPtr msg);
  void onTeleopAction(const teleop_msgs::msg::TeleopAction::SharedPtr msg);
  void onMavrosState(const mavros_msgs::msg::State::SharedPtr msg);

  // members
  TopicPaths topics_;

  // mutex lock
  mutable std::mutex state_mtx_;

  // internal state object
  InternalState state_;

  // flag variable set to "true" during initialization
  bool initialization_phase_{true};

  // subscriptions
  rclcpp::Subscription<teleop_msgs::msg::TeleopCommand>::SharedPtr sub_teleop_command_;
  rclcpp::Subscription<teleop_msgs::msg::TeleopAction>::SharedPtr sub_teleop_action_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_mavros_state_;

  // publishers
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_setpoint_raw_local_;

  // services
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_long_client_;

  // timers
  rclcpp::TimerBase::SharedPtr kill_switch_timer_;
};

#endif  // MAVROS_GATE__COMMAND_GATE_HPP_