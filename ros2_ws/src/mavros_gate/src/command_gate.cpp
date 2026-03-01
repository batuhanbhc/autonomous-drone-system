#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <teleop_msgs/msg/teleop_action.hpp>
#include <teleop_msgs/msg/teleop_command.hpp>
#include <mavros_msgs/msg/position_target.hpp>

#include <unordered_map>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

class CommandGateNode : public rclcpp::Node {
public:
  CommandGateNode() : Node("command_gate") {
    RCLCPP_INFO(get_logger(), "command_gate started");

    initialization_phase_ = true;

    // QoS profiles
    const auto qos_command =
      rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();

    const auto qos_action =
      rclcpp::QoS(rclcpp::KeepLast(20)).best_effort().durability_volatile();

    const auto qos_state =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    // Subscriptions
    sub_teleop_command_ = this->create_subscription<teleop_msgs::msg::TeleopCommand>(
      "/teleop/command", qos_command,
      std::bind(&CommandGateNode::onTeleopCommand, this, std::placeholders::_1));

    sub_teleop_action_ = this->create_subscription<teleop_msgs::msg::TeleopAction>(
      "/teleop/action", qos_action,
      std::bind(&CommandGateNode::onTeleopAction, this, std::placeholders::_1));

    sub_mavros_state_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", qos_state,
      std::bind(&CommandGateNode::onMavrosState, this, std::placeholders::_1));
    
    // Publishers
    pub_setpoint_raw_local_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
      "/mavros/setpoint_raw/local", qos_command);

    /* /autonomous/command does not exist yet -> keep commented out for now
    sub_autonomous_command_ = this->create_subscription<autonomous_msgs::msg::AutonomousCommand>(
      "/autonomous/command", 10,
      std::bind(&CommandGateNode::onAutonomousCommand, this, std::placeholders::_1));
    */

    // Initialization routine
    initializationRoutine();

    // Non-blocking wait 3 seconds, then report success (one-shot timer).
    init_success_timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      [this]() {
        {
          std::lock_guard<std::mutex> lk(state_mtx_);
          initialization_phase_ = false;
        }

        RCLCPP_INFO(this->get_logger(), "initialization routine successful");

        if (init_success_timer_) {
          init_success_timer_->cancel();  // one-shot behavior
        }
      });
  }

private:
  // Map: Teleop command IDs -> command names
  std::unordered_map<int32_t, std::string> command_id_to_name_;

  // Internal shared state (guarded by a mutex)
  struct InternalState {
    bool connected{false};
    bool armed{false};
    bool guided{false};
    bool manual_input{false};
    int32_t last_command_id{-1};
  };

  // Partial update object (fields not set will not be modified)
  struct InternalStateUpdate {
    std::optional<bool> connected;
    std::optional<bool> armed;
    std::optional<bool> guided;
    std::optional<bool> manual_input;
    std::optional<int32_t> last_command_id;
  };

  void updateInternalState(const InternalStateUpdate & update) {
    std::lock_guard<std::mutex> lk(state_mtx_);

    if (update.connected.has_value()) {
      state_.connected = update.connected.value();
    }
    if (update.armed.has_value()) {
      state_.armed = update.armed.value();
    }
    if (update.guided.has_value()) {
      state_.guided = update.guided.value();
    }
    if (update.manual_input.has_value()) {
      state_.manual_input = update.manual_input.value();
    }
    if (update.last_command_id.has_value()) {
      state_.last_command_id = update.last_command_id.value();
    }
  }

  std::unordered_map<int32_t, std::string> loadTeleopCommandMap() {
    std::unordered_map<int32_t, std::string> out;

    const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("mavros_config");
    const std::string yaml_path = pkg_share + "/config/teleop_params.yaml";

    YAML::Node root = YAML::LoadFile(yaml_path);

    YAML::Node params = root["teleop"];
    if (!params) {
      RCLCPP_WARN(get_logger(), "YAML missing 'teleop' root: %s", yaml_path.c_str());
      return out;
    }

    YAML::Node commands = params["commands"];
    if (!commands) {
      RCLCPP_WARN(get_logger(), "YAML missing 'teleop.commands': %s", yaml_path.c_str());
      return out;
    }

    YAML::Node cmd_params = commands["params"];
    if (!cmd_params || !cmd_params.IsMap()) {
      RCLCPP_WARN(get_logger(), "YAML missing/invalid 'teleop.commands.params': %s", yaml_path.c_str());
      return out;
    }

    for (auto it = cmd_params.begin(); it != cmd_params.end(); ++it) {
      const std::string name = it->first.as<std::string>();
      const YAML::Node node = it->second;

      if (!node || !node["command_id"]) {
        RCLCPP_WARN(get_logger(), "Command '%s' missing command_id (skipping)", name.c_str());
        continue;
      }

      const int32_t id = node["command_id"].as<int32_t>();

      // If duplicates exist, last one wins (or you can warn + skip)
      auto existing = out.find(id);
      if (existing != out.end()) {
        RCLCPP_WARN(get_logger(),
          "Duplicate command_id=%d for '%s' and '%s' (overwriting with '%s')",
          id, existing->second.c_str(), name.c_str(), name.c_str());
      }

      out[id] = name;
    }

    return out;
  }

  bool inInitializationPhase() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return initialization_phase_;
  }

  InternalState snapshotState() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return state_;
  }

  // initialization routine
  void initializationRoutine() {
    // --- load teleop command id - command name pairs
    command_id_to_name_ = loadTeleopCommandMap();
    RCLCPP_INFO(get_logger(), "%s", command_id_to_name_[1].c_str());
  }

  // --- Callbacks ---
  void onTeleopCommand(const teleop_msgs::msg::TeleopCommand::SharedPtr msg) {
    if (inInitializationPhase()) {
      return;
    }

    InternalStateUpdate update;
    update.last_command_id = msg->command_id;
    updateInternalState(update);
  }


  void onTeleopAction(const teleop_msgs::msg::TeleopAction::SharedPtr msg) {
    if (inInitializationPhase()) {
      return;
    }

    mavros_msgs::msg::PositionTarget sp;
    sp.header.stamp = msg->stamp;
    sp.header.frame_id = "base_link";  // informational; MAVLink uses coordinate_frame below

    // Body frame, NED
    sp.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;

    // Enable only: velocity (vx,vy,vz) and yaw_rate
    // Ignore: position, acceleration/force, yaw
    sp.type_mask =
      mavros_msgs::msg::PositionTarget::IGNORE_PX |
      mavros_msgs::msg::PositionTarget::IGNORE_PY |
      mavros_msgs::msg::PositionTarget::IGNORE_PZ |
      mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY |
      mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW;

    // Body-frame NED velocities
    sp.velocity.x = static_cast<double>(msg->float_1);  // vx
    sp.velocity.y = static_cast<double>(msg->float_2);  // vy
    sp.velocity.z = static_cast<double>(msg->float_3);  // vz

    // Yaw rate (rad/s)
    sp.yaw_rate = msg->float_4;

    // Unused fields
    sp.position.x = 0.0;
    sp.position.y = 0.0;
    sp.position.z = 0.0;
    sp.acceleration_or_force.x = 0.0;
    sp.acceleration_or_force.y = 0.0;
    sp.acceleration_or_force.z = 0.0;
    sp.yaw = 0.0f;

    pub_setpoint_raw_local_->publish(sp);
  }

  void onMavrosState(const mavros_msgs::msg::State::SharedPtr msg) {
    if (inInitializationPhase()) {
      return;
    }

    InternalStateUpdate update;
    update.connected = msg->connected;
    update.armed = msg->armed;
    update.guided = msg->guided;
    update.manual_input = msg->manual_input;

    updateInternalState(update);
  }

  // --- Members ---
  mutable std::mutex state_mtx_;
  InternalState state_;
  bool initialization_phase_{true};

  // subscriptions
  rclcpp::Subscription<teleop_msgs::msg::TeleopCommand>::SharedPtr sub_teleop_command_;
  rclcpp::Subscription<teleop_msgs::msg::TeleopAction>::SharedPtr sub_teleop_action_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_mavros_state_;

  // publishers
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_setpoint_raw_local_;

  // One-shot init success timer
  rclcpp::TimerBase::SharedPtr init_success_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandGateNode>());
  rclcpp::shutdown();
  return 0;
}