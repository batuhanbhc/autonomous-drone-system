#include "mavros_gate/control_gate.hpp"


using Cmd = drone_msgs::msg::TeleopCommand;
using DroneInfo = drone_msgs::msg::DroneInfo;
using MonotonicTime = std::chrono::steady_clock::time_point;

const char* ControlGateNode::commandName(int8_t id) {
  switch (id) {
    case Cmd::KILL_SWITCH:    return "KILL_SWITCH";
    case Cmd::KILL_CONFIRM:   return "KILL_CONFIRM";
    case Cmd::CONTROL_TOGGLE: return "CONTROL_TOGGLE";
    case Cmd::KEYBOARD_TOGGLE: return "KEYBOARD_TOGGLE";
    case Cmd::ARM:            return "ARM";
    case Cmd::DISARM:         return "DISARM";
    case Cmd::LAND:           return "LAND";
    case Cmd::RTL:            return "RTL";
    case Cmd::TAKEOFF:        return "TAKEOFF";
    case Cmd::GUIDED:         return "GUIDED";
    case Cmd::SPEED_UP:       return "SPEED_UP";
    case Cmd::SPEED_DOWN:     return "SPEED_DOWN";
    case Cmd::PRESS_SAFETY_SWITCH:  return "PRESS_SAFETY_SWITCH";
    case Cmd::VEL_YAW:        return "VEL_YAW";
    case Cmd::HOVER:          return "HOVER";
    default:                  return "<unknown>";
  }
}


bool ControlGateNode::isCommandAlwaysEnabled(int8_t id) {
  switch (id) {
    case Cmd::KILL_SWITCH:
    case Cmd::KILL_CONFIRM:
    case Cmd::CONTROL_TOGGLE:
    case Cmd::KEYBOARD_TOGGLE:
    case Cmd::LAND:
    case Cmd::RTL:
      return true;
    default:
      return false;
  }
}

// Updates the internal state in thread-safe manner, where multiple different subscription callbacks
// may want to update the internal state
void ControlGateNode::updateInternalStateAtomic(const InternalStateUpdate & update) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (update.control_mode)    state_.control_mode = *update.control_mode;
  if (update.vel)             state_.vel = *update.vel;
  if (update.keyboard_on)     state_.keyboard_on = *update.keyboard_on;
  if (update.safety_switch_on)  state_.safety_switch_on = *update.safety_switch_on;
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

  YAML::Node t = root["custom_topics"];
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
  try { gcs_failsafe_s_ = root["flight_params"]["gcs_failsafe_s"].as<double>(); }
  catch (const YAML::Exception&) { ok = false; }
  try { failsafe_watchdog_hz_ = root["flight_params"]["failsafe_watchdog_hz"].as<float>(); }
  catch (const YAML::Exception&) { ok = false; }
  return ok;
}


// Used by subscription callback functions to ignore messages until node initialization is fully complete
bool ControlGateNode::inInitializationPhase() const {
  // will only be used during initialization, no need for mutex
  return initialization_phase_;
}

float ControlGateNode::getTakeoffMeters() const {
  // read-only, no need for mutex
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

  cmd_handlers_[TeleopCmd::TAKEOFF] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeTakeoff(cmd, st);
  };

  cmd_handlers_[TeleopCmd::GUIDED] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executeGuided(cmd, st);
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

  cmd_handlers_[TeleopCmd::PRESS_SAFETY_SWITCH] = [this](const TeleopCmd& cmd, const InternalState& st) {
    return this->executePressSafetySwitch(cmd, st);
  };
}


// Calls all necessary ros2 services/topics to initialize command_gate properly
bool ControlGateNode::initializationRoutine() {
  updateLastAct(std::chrono::steady_clock::now());

  // Initialize client paths
  std::string arming_path = base_ns_ + "/mavros/cmd/arming";
  std::string command_long_path = base_ns_ + "/mavros/cmd/command";
  std::string set_mode_path = base_ns_ + "/mavros/set_mode";
  std::string takeoff_path = base_ns_ + "/mavros/cmd/takeoff";
  std::string msg_interval_path = base_ns_ + "/mavros/set_message_interval";

  // -----------------------------------
  // Wait for mavros to come online before doing anything else
  // Use arming service as a proxy for mavros being ready
  RCLCPP_INFO(get_logger(), "Waiting for mavros to come online...");
  
  auto arming_probe = this->create_client<mavros_msgs::srv::CommandBool>(arming_path);
  
  if (!arming_probe->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_FATAL(get_logger(), "mavros arming service not available after 60s — is mavros running?");
    return false;
  }
  RCLCPP_INFO(get_logger(), "mavros is online.");
  arming_probe.reset(); // discard probe client; real one is created below
  // -----------------------------------
  
  // Create clients
  arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(arming_path);
  command_long_client_ = this->create_client<mavros_msgs::srv::CommandLong>(command_long_path);
  set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(set_mode_path);
  takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(takeoff_path);
  msg_interval_client_ = this->create_client<mavros_msgs::srv::MessageInterval>(msg_interval_path);

  
// Helper to request a MAVLink message stream
  auto requestMsgInterval = [&](int id, float rate_hz, const char* label) -> bool {
    if (!msg_interval_client_->wait_for_service(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(), "/mavros/set_message_interval not available (%s)", label);
      return false;
    }
    auto req = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
    req->message_id = id;
    req->message_rate = rate_hz;
    int retries = 5;
    while (retries-- > 0) {
        auto result = msg_interval_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(
          this->get_node_base_interface(), result, std::chrono::seconds(3)) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(get_logger(), "Requested %s stream: success=%s",
             label, result.get()->success ? "true" : "false");
            break;
        }
        RCLCPP_WARN(get_logger(), "set_message_interval timed out, retrying... (%d left)", retries);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    if (retries <= 0) {
        RCLCPP_FATAL(get_logger(), "set_message_interval failed after all retries: %s", label);
        throw std::runtime_error("control_gate init failed");
    }
    
    return true;
  };

  if (!requestMsgInterval(245, 2.0f,  "EXTENDED_SYS_STATE"))  return false;
  if (!requestMsgInterval(147, 1.0f,  "BATTERY_STATUS"))       return false;
  if (!requestMsgInterval(32,  10.0f, "LOCAL_POSITION_NED"))   return false;
  if (!requestMsgInterval(24,  5.0f,  "GPS_RAW_INT"))          return false;

  // Initialize command handlers
  initCommandHandlers();

  return true;
}


bool ControlGateNode::isSetpointBlocked(const InternalState& st) const {
  // "Enabled" means: connected, manual, armed, not killed, keyboard on, not in critical state
  const bool enabled = st.connected && !st.system_killed && st.armed &&
      (st.control_mode == ControlMode::Manual) && st.keyboard_on &&
      !critical_state_.load(std::memory_order_relaxed) && !gcs_failsafe_.load(std::memory_order_relaxed);

  return !enabled;
}


void ControlGateNode::updateSetpointBlockStateAndMaybePublish(bool blocked, bool initial_publish) {
  // First time initialization message
  if (!setpoint_blocked_initialized_) {
    setpoint_blocked_ = blocked;
    setpoint_blocked_initialized_ = true;

    if (initial_publish) {
      if (blocked) {
        publishInfo(DroneInfo::LEVEL_WARN, "Setpoint commands are blocked.");
      } else {
        publishInfo(DroneInfo::LEVEL_INFO, "Setpoint commands are enabled.");
      }
    }
    return;
  }

  // Transition message
  if (blocked != setpoint_blocked_) {
    setpoint_blocked_ = blocked;

    if (blocked) {
      publishInfo(DroneInfo::LEVEL_WARN, "Setpoint commands are now blocked.");
    } else {
      publishInfo(DroneInfo::LEVEL_INFO, "Setpoint commands are now enabled.");
    }
  }
}


void ControlGateNode::updateLastAct(const MonotonicTime& t) {
    std::lock_guard<std::mutex> lk(last_act_mtx_);
    last_action_t_ = t;
}

MonotonicTime ControlGateNode::readLastAct(void) const {
    std::lock_guard<std::mutex> lk(last_act_mtx_);
    return last_action_t_;
}
