#include "mavros_gate/control_gate.hpp"



const char* ControlGateNode::commandName(int8_t id) {
  using Cmd = teleop_msgs::msg::TeleopCommand;
  switch (id) {
    case Cmd::KILL_SWITCH:    return "KILL_SWITCH";
    case Cmd::KILL_CONFIRM:   return "KILL_CONFIRM";
    case Cmd::CONTROL_TOGGLE: return "CONTROL_TOGGLE";
    case Cmd::KEYBOARD_TOGGLE: return "KEYBOARD_TOGGLE";
    case Cmd::ARM:            return "ARM";
    case Cmd::DISARM:         return "DISARM";
    case Cmd::LAND:           return "LAND";
    case Cmd::RTL:            return "RTL";
    case Cmd::LOITER:         return "LOITER";
    case Cmd::TAKEOFF:        return "TAKEOFF";
    case Cmd::SPEED_UP:       return "SPEED_UP";
    case Cmd::SPEED_DOWN:     return "SPEED_DOWN";
    case Cmd::VEL_YAW:        return "VEL_YAW";
    case Cmd::HOVER:          return "HOVER";
    default:                  return "<unknown>";
  }
}


// Updates the internal state in thread-safe manner, where multiple different subscription callbacks
// may want to update the internal state
void ControlGateNode::updateInternalStateAtomic(const InternalStateUpdate & update) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (update.control_mode)    state_.control_mode = *update.control_mode;
  if (update.vel)             state_.vel = *update.vel;
  if (update.keyboard_on)     state_.keyboard_on = *update.keyboard_on;
  if (update.connected)       state_.connected = *update.connected;
  if (update.armed)           state_.armed = *update.armed;
  if (update.guided)          state_.guided = *update.guided;
  if (update.kill_switch_window) state_.kill_switch_window = *update.kill_switch_window;
  if (update.system_killed) state_.system_killed = *update.system_killed;
}


bool ControlGateNode::loadConfig() {
  std::unordered_map<int32_t, std::string> out;

  const std::string pkg_share = ament_index_cpp::get_package_share_directory("mavros_config");
  const std::string yaml_path = pkg_share + "/config/control_params.yaml";

  YAML::Node root = YAML::LoadFile(yaml_path);

  YAML::Node t = root["topics"];
  if (!t || !t.IsMap()) {
    RCLCPP_WARN(get_logger(), "YAML missing/invalid 'topics'");
    return false;
  }

  auto get_str = [&](const char * key, std::string & out) -> bool {
    auto n = t[key];
    if (!n || !n.IsScalar()) {
      RCLCPP_WARN(get_logger(), "YAML missing/invalid topics.%s", key);
      return false;
    }
    out = n.as<std::string>();
    return true;
  };

  bool ok = true;
  ok &= get_str("autonomous_action", topics_.autonomous_action);
  ok &= get_str("manual_action", topics_.manual_action);
  ok &= get_str("manual_command", topics_.manual_command);
  try { takeoff_m_ = root["flight_params"]["takeoff_m"].as<float>(); }
  catch (const YAML::Exception&) { ok = false; }
  return ok;
}


// Used by subscription callback functions to ignore messages until node initialization is fully complete
bool ControlGateNode::inInitializationPhase() const {
  return initialization_phase_;
}

float ControlGateNode::getTakeoffMeters() const {
  return takeoff_m_;
}

// Returns a snapshot of the current internal state of command_gate node
ControlGateNode::InternalState ControlGateNode::snapshotState() const {
  std::lock_guard<std::mutex> lk(state_mtx_);
  return state_;
}


void ControlGateNode::initCommandHandlers() {
  cmd_handlers_[TeleopCmd::ARM] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeArm(cmd, st);
  };

  cmd_handlers_[TeleopCmd::DISARM] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeDisarm(cmd, st);
  };

  cmd_handlers_[TeleopCmd::KILL_SWITCH] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeKillSwitch(cmd, st);
  };

  cmd_handlers_[TeleopCmd::KILL_CONFIRM] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeKillConfirm(cmd, st);
  };

  cmd_handlers_[TeleopCmd::LAND] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeLand(cmd, st);
  };

  cmd_handlers_[TeleopCmd::RTL] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeRTL(cmd, st);
  };

  cmd_handlers_[TeleopCmd::LOITER] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeLoiter(cmd, st);
  };

  cmd_handlers_[TeleopCmd::TAKEOFF] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeTakeoff(cmd, st);
  };

  cmd_handlers_[TeleopCmd::CONTROL_TOGGLE] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeControlToggle(cmd, st);
  };

  cmd_handlers_[TeleopCmd::KEYBOARD_TOGGLE] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeKeyboardToggle(cmd, st);
  };

  cmd_handlers_[TeleopCmd::SPEED_UP] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeChangeSpeed(cmd, st);
  };

  cmd_handlers_[TeleopCmd::SPEED_DOWN] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeChangeSpeed(cmd, st);
  };
}


// Calls all necessary ros2 services/topics to initialize command_gate properly
bool ControlGateNode::initializationRoutine() {
  // Initialize internal state
  InternalStateUpdate st;
  st.control_mode = ControlMode::Manual;
  st.connected = false;
  st.armed = false;
  st.guided = false;
  st.kill_switch_window = false;
  st.system_killed = false;
  updateInternalStateAtomic(st);

  // Initialize clients
  arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  command_long_client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
  set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

  // Initialize command handlers
  initCommandHandlers();

  return true;
}