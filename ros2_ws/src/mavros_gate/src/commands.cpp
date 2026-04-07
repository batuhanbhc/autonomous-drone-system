#include "mavros_gate/control_gate.hpp"
#include <chrono>

using namespace std::chrono_literals;
using DroneInfo = drone_msgs::msg::DroneInfo;

ControlGateNode::CommandResult 
ControlGateNode::executeArm(const TeleopCmd&, const InternalState&) {
  /*
  This function sends asynchronous request to FCU via mavros to send "Arm" command
  */
  if (!arming_client_) {
    return {false, "Arming client not initialized."};
  }

  if (!arming_client_->service_is_ready()) {
    return {false, "Service /mavros/cmd/arming not available."};
  }

  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = true; // ARM

  (void)arming_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->success) {
        publishInfo(DroneInfo::LEVEL_INFO, "Arm accepted by FCU.");
        //RCLCPP_INFO(get_logger(), "Arm accepted by FCU.");

        InternalStateUpdate update;
        update.armed = true;
        updateInternalStateAtomic(update);
      } else {
        publishInfo(DroneInfo::LEVEL_WARN, stringf("Arm rejected by FCU (result=%u).", resp->result));
        //RCLCPP_WARN(get_logger(), "Arm rejected by FCU (result=%u).", resp->result);
      }
    }
  );

  return {true, "Arm request sent."};
}


ControlGateNode::CommandResult 
ControlGateNode::executeDisarm(const TeleopCmd&, const InternalState&) {
  /*
  This function sends asynchronous request to FCU via mavros to send "Disarm" command
  */
  if (!arming_client_) {
    return {false, "Arming client not initialized."};
  }

  if (!arming_client_->service_is_ready()) {
    return {false, "Service /mavros/cmd/arming not available."};
  }

  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = false; // DISARM

  (void)arming_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->success) {
        publishInfo(DroneInfo::LEVEL_INFO, "Disarm accepted by FCU.");
        //RCLCPP_INFO(get_logger(), "Disarm accepted by FCU");

        InternalStateUpdate update;
        update.armed = false;
        updateInternalStateAtomic(update);
        
      } else {
        publishInfo(DroneInfo::LEVEL_WARN, stringf("Disarm rejected by FCU (result=%u).", resp->result));
        //RCLCPP_WARN(get_logger(), "Disarm rejected by FCU (result=%u).", resp->result);
      }
    }
  );

  return {true, "Disarm request sent."};
}


ControlGateNode::CommandResult 
ControlGateNode::executeKillSwitch(const TeleopCmd&, const InternalState&) {
  /*
  This function starts a timer that enables Kill Confirm (a.k.a flight termination) command to be executed within a limited window
  */
  // RCLCPP_WARN(get_logger(), "-----------------------------------------------");
  // RCLCPP_WARN(get_logger(), "-----     ENABLING KILL-SWITCH WINDOW     -----");
  // RCLCPP_WARN(get_logger(), "-----------------------------------------------");

  publishInfo(DroneInfo::LEVEL_WARN, "--- ENABLING KILL-SWITCH WINDOW ---");

  auto new_timer = this->create_wall_timer(2s, [this]() {
    std::lock_guard<std::mutex> lk(state_mtx_);

    if (state_.kill_switch_window) {
      state_.kill_switch_window = false;
      // RCLCPP_INFO(get_logger(), "Kill-switch window ended.");
      publishInfo(DroneInfo::LEVEL_WARN, "Kill-switch window ended.");
    }
    
    // one-shot: stop future firings
    if (kill_switch_timer_) {
      kill_switch_timer_->cancel();
      kill_switch_timer_.reset();
    }
  });

  bool extended = false;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    extended = state_.kill_switch_window;
    state_.kill_switch_window = true;

    if (kill_switch_timer_) {
      kill_switch_timer_->cancel();
    }
    kill_switch_timer_ = std::move(new_timer);
  }

  return {true, extended ? "Kill-switch window extended (2s)." : "Kill-switch window started (2s)."};
}


ControlGateNode::CommandResult 
ControlGateNode::executeKillConfirm(const TeleopCmd&, const InternalState& int_state) {
  /*
  This function sends asynchronous request to FCU via mavros to send "Flight Termination" command.
  This command immediately stops the motors, hence copter will fall if it is not landed.
  */

  if (!command_long_client_) {
    return {false, "CommandLong client not initialized."};
  }

  // gate on the "kill switch window"
  if (!int_state.kill_switch_window) {
    return {false, "Kill-switch window not active."};
  }

  if (!command_long_client_->service_is_ready()) {
    return {false, "Service /mavros/cmd/command not available."};
  }

  // Build MAVLink COMMAND_LONG: MAV_CMD_DO_FLIGHTTERMINATION (185)
  //    param1 > 0.5 activates termination 
  auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  req->broadcast = false;
  req->command = 185;       // MAV_CMD_DO_FLIGHTTERMINATION
  req->confirmation = 0;
  req->param1 = 1.0f;       // Terminate
  req->param2 = 0.0f;
  req->param3 = 0.0f;
  req->param4 = 0.0f;
  req->param5 = 0.0f;
  req->param6 = 0.0f;
  req->param7 = 0.0f;

  (void)command_long_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->success) {
        // RCLCPP_ERROR(get_logger(), "-------------------------------------------------------");
        // RCLCPP_ERROR(get_logger(), "---       Flight termination accepted by FCU.       ---");
        // RCLCPP_ERROR(get_logger(), "---   System is killed, power cycle is required.    ---");
        // RCLCPP_ERROR(get_logger(), "-------------------------------------------------------");

        publishInfo(DroneInfo::LEVEL_ERROR, "! FLIGHT TERMINATION ! ");

        // update state to close kill switch window, and set system as killed
        InternalStateUpdate update;
        update.system_killed = true;
        update.kill_switch_window = false;
        updateInternalStateAtomic(update);

      } else {
        // RCLCPP_WARN(get_logger(), "Flight termination rejected by FCU (result=%u).", resp->result);
        publishInfo(DroneInfo::LEVEL_WARN, stringf("Flight termination rejected by FCU (result=%u).", resp->result));
      }
    }
  );  

  return {true, "Flight termination requested."};
}


ControlGateNode::CommandResult
ControlGateNode::executeLand(const TeleopCmd&, const InternalState&)
{
  /*
    Land straight down by switching FCU flight mode to LAND.
    Uses /mavros/set_mode (mavros_msgs/srv/SetMode).
  */

  if (!set_mode_client_) {
    return {false, "SetMode client not initialized."};
  }

  if (!set_mode_client_->service_is_ready()) {
    return {false, "Service /mavros/set_mode not available."};
  }

  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();

  req->base_mode = 0;
  req->custom_mode = "LAND";

  (void)set_mode_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->mode_sent) {
        publishInfo(DroneInfo::LEVEL_INFO, "Land mode request reached FCU.");
        // RCLCPP_INFO(get_logger(), "Land mode request reached FCU.");
      } else {
        publishInfo(DroneInfo::LEVEL_WARN, "Land mode request could not reach FCU.");
        // RCLCPP_WARN(get_logger(), "Land mode request could not reach FCU.");
      }
    }
  );

  return {true, "LAND mode request sent."};
}

ControlGateNode::CommandResult
ControlGateNode::executeRTL(const TeleopCmd&, const InternalState&)
{
  /*
    Sends MAV_CMD_NAV_RETURN_TO_LAUNCH (20) via /mavros/cmd/command (CommandLong).
    Params are unused (set to 0).
  */

  if (!command_long_client_) {
    return {false, "CommandLong client not initialized."};
  }

  if (!command_long_client_->service_is_ready()) {
    return {false, "Service /mavros/cmd/command not available."};
  }

  auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  req->broadcast = false;
  req->command = 20;          // MAV_CMD_NAV_RETURN_TO_LAUNCH
  req->confirmation = 0;
  req->param1 = 0.0f;
  req->param2 = 0.0f;
  req->param3 = 0.0f;
  req->param4 = 0.0f;
  req->param5 = 0.0f;
  req->param6 = 0.0f;
  req->param7 = 0.0f;

  (void)command_long_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->success) {
        publishInfo(DroneInfo::LEVEL_INFO, "RTL accepted by FCU.");
        // RCLCPP_INFO(get_logger(), "RTL accepted by FCU.");
      } else {
        publishInfo(DroneInfo::LEVEL_WARN, stringf("RTL rejected by FCU (result=%u).", resp->result));
        // RCLCPP_WARN(get_logger(), "RTL rejected by FCU (result=%u).", resp->result);
      }
    }
  );

  return {true, "RTL request sent."};
}

ControlGateNode::CommandResult
ControlGateNode::executeGuided(const TeleopCmd&, const InternalState&) {
  /*
    Set mode to GUIDED using MAVROS /mavros/set_mode (mavros_msgs/srv/SetMode).
    Note: SetMode response only indicates mode_sent (request forwarded), not FCU acceptance.
  */

  if (!set_mode_client_) {
    return {false, "SetMode client not initialized."};
  }

  if (!set_mode_client_->service_is_ready()) {
    return {false, "Service /mavros/set_mode not available."};
  }

  auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  mode_req->base_mode = 0;
  mode_req->custom_mode = "GUIDED";

  (void)set_mode_client_->async_send_request(
    mode_req,
    [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->mode_sent) {
        publishInfo(DroneInfo::LEVEL_INFO, "Guided mode request reached FCU.");
        // RCLCPP_INFO(get_logger(), "Guided mode request reached FCU.");
      } else {
        publishInfo(DroneInfo::LEVEL_WARN, "Guided mode request could not reach FCU.");
        // RCLCPP_WARN(get_logger(), "Guided mode request could not reach FCU.");
      }
    }
  );

  return {true, "Guided mode request sent."};
}

ControlGateNode::CommandResult
ControlGateNode::executeTakeoff(const TeleopCmd&, const InternalState&) {
  /*
    Takeoff using MAVROS: Call /mavros/cmd/takeoff (mavros_msgs/srv/CommandTOL).
  */

  if (!takeoff_client_) {
    return {false, "Takeoff client not initialized."};
  }

  // Send takeoff
  if (!takeoff_client_->service_is_ready()) {
    return {false, "Service /mavros/cmd/takeoff not available."};
  }

  auto to_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

  // Minimal common settings:
  to_req->min_pitch = 0.0f;
  to_req->yaw = 0.0f;

  // "Takeoff from here": leave lat/lon at 0
  to_req->latitude = 0.0f;
  to_req->longitude = 0.0f;

  // Target takeoff altitude (meters).
  float takeoff_m = getTakeoffMeters();
  to_req->altitude = takeoff_m;

  (void)takeoff_client_->async_send_request(
    to_req,
    [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->success) {
        publishInfo(DroneInfo::LEVEL_INFO, "Takeoff accepted by FCU.");
        takeoff_start_t_        = std::chrono::steady_clock::now();
        takeoff_monitor_active_ = true;
        updateLastAct(std::chrono::steady_clock::now());  // prevent immediate guided timeout
        takeoff_monitor_timer_->reset(); 
        RCLCPP_INFO(get_logger(), "Takeoff accepted by FCU.");
      } else {
        publishInfo(DroneInfo::LEVEL_WARN, stringf("Takeoff rejected by FCU (result=%u).", resp->result));
        RCLCPP_WARN(get_logger(), "Takeoff rejected by FCU (result=%u).", resp->result);
      }
    }
  );

  std::ostringstream oss;
  oss << "Takeoff (" << std::fixed << std::setprecision(2) << takeoff_m << "m) request sent.";
  return {true, oss.str()};
}


ControlGateNode::CommandResult
ControlGateNode::executeControlToggle(const TeleopCmd&, const InternalState& int_state) {
  /*
    Toggles control mode between ControlGateNode::ControlMode::Auto/Manual
  */

  bool tmp_flag = 0; // ControlMode::Auto
  if (int_state.control_mode == ControlMode::Manual) {
    tmp_flag = 1;
  }

  InternalStateUpdate upt;
  upt.control_mode = tmp_flag ? ControlMode::Auto : ControlMode::Manual;
  updateInternalStateAtomic(upt);

  const char* mode_old = tmp_flag ? "MANUAL" : "AUTO";
  const char* mode_new = tmp_flag ? "AUTO" : "MANUAL";
  std::string tmp = std::string("Control mode changed: ") + mode_old + " -> " + mode_new + ".";
  publishInfo(DroneInfo::LEVEL_INFO, tmp);
  return {true, tmp};
}


ControlGateNode::CommandResult
ControlGateNode::executeKeyboardToggle(const TeleopCmd&, const InternalState& state) {
  /*
    Updates internal state according the incoming keyboard state
  */
  bool keyboard_on_old = state.keyboard_on;

  InternalStateUpdate upt;
  upt.keyboard_on = keyboard_on_old ? false: true;
  updateInternalStateAtomic(upt);
  
  std::string keyboard_state = keyboard_on_old ? "OFF": "ON";
  std::string out_msg = std::string("Keyboard state: ") + keyboard_state;
  publishInfo(DroneInfo::LEVEL_INFO, out_msg);
  return {true, out_msg};
}


ControlGateNode::CommandResult
ControlGateNode::executeChangeSpeedHorizontal(const TeleopCmd& msg, const InternalState& st) {
    double hv = msg.float_1;

    InternalStateUpdate upt;
    upt.vel = VelocityLevel{hv, st.vel.vv, st.vel.yaw};
    updateInternalStateAtomic(upt);

    std::ostringstream oss;
    oss << "Horizontal velocity: " << std::fixed << std::setprecision(2) << hv << " m/s.";
    publishInfo(DroneInfo::LEVEL_INFO, oss.str());
    return {true, oss.str()};
}

ControlGateNode::CommandResult
ControlGateNode::executeChangeSpeedVertical(const TeleopCmd& msg, const InternalState& st) {
    double vv = msg.float_1;

    InternalStateUpdate upt;
    upt.vel = VelocityLevel{st.vel.hv, vv, st.vel.yaw};
    updateInternalStateAtomic(upt);

    std::ostringstream oss;
    oss << "Vertical velocity: " << std::fixed << std::setprecision(2) << vv << " m/s.";
    publishInfo(DroneInfo::LEVEL_INFO, oss.str());
    return {true, oss.str()};
}

ControlGateNode::CommandResult
ControlGateNode::executeChangeSpeedYaw(const TeleopCmd& msg, const InternalState& st) {
    double yaw = msg.float_1;

    InternalStateUpdate upt;
    upt.vel = VelocityLevel{st.vel.hv, st.vel.vv, yaw};
    updateInternalStateAtomic(upt);

    std::ostringstream oss;
    oss << "Yaw rate: " << std::fixed << std::setprecision(2) << yaw << " rad/s.";
    publishInfo(DroneInfo::LEVEL_INFO, oss.str());
    return {true, oss.str()};
}


ControlGateNode::CommandResult
ControlGateNode::executePressSafetySwitch(const TeleopCmd&, const InternalState& int_state)
{
  // MAV_CMD_DO_SET_SAFETY_SWITCH_STATE (5300)
  // param1: 0 = SAFE, 1 = UNSAFE (safety OFF / outputs enabled)

  if (!command_long_client_) {
    return {false, "CommandLong client not initialized."};
  }

  if (!command_long_client_->service_is_ready()) {
    return {false, "Service /mavros/cmd/command not available."};
  }

  // safety switch should not be pressed if copter is armed. Else, it may fall if in mid-air
  if (int_state.armed ) {
    publishInfo(DroneInfo::LEVEL_ERROR, "Safety switch rejected: Vehicle currently ARMED.");
    return {false, "Safety switch rejected: Vehicle currently ARMED."};
  }

  const bool currently_safe = int_state.safety_switch_on; 
  const float target = currently_safe ? 1.0f : 0.0f;      // 1->UNSAFE, 0->SAFE

  const std::string old_state = currently_safe ? "SAFE" : "UNSAFE";
  const std::string new_state = currently_safe ? "UNSAFE" : "SAFE";
  const bool new_safe_bool = (new_state == "SAFE");

  auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  req->broadcast = false;
  req->command = 5300;
  req->confirmation = 0;
  req->param1 = target;
  req->param2 = 0.0f;
  req->param3 = 0.0f;
  req->param4 = 0.0f;
  req->param5 = 0.0f;
  req->param6 = 0.0f;
  req->param7 = 0.0f;

  (void)command_long_client_->async_send_request(
    req,
    [this, old_state, new_state, new_safe_bool]
    (rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->success) {
        publishInfo(DroneInfo::LEVEL_INFO, stringf("Safety switch pressed: %s -> %s.", old_state.c_str(), new_state.c_str()));
        // RCLCPP_INFO(get_logger(), "Safety switch pressed: %s -> %s.", old_state.c_str(), new_state.c_str());

        InternalStateUpdate upt;
        upt.safety_switch_on = new_safe_bool;
        updateInternalStateAtomic(upt);

      } else {
        publishInfo(DroneInfo::LEVEL_WARN, stringf("Safety switch command rejected by FCU (result=%u).", resp->result));
        // RCLCPP_WARN(get_logger(), "Safety switch command rejected by FCU (result=%u).", resp->result);
      }
    }
  );

  return {true, "Safety switch command request sent."};
}

ControlGateNode::CommandResult
ControlGateNode::executeAltSupportToggle(const TeleopCmd&, const InternalState& st)
{
  if (!st.armed || !st.guided) {
    return {false, "AltHold toggle rejected: must be armed and guided."};
  }

  if (!has_taken_off_) {
    return {false, "AltHold toggle rejected: takeoff not complete."};
  }

  if (alt_ctrl_mode_ == AltCtrlMode::Off) {
    const VerticalEstimateCache ve = snapshotVertEst();
    if (!ve.valid) {
      return {false, "AltHold toggle rejected: no valid vertical estimate."};
    }
    enterAltHold(ve.agl_m, ve.agl_m, ve.vz_mps);
    return {true, "AltHold enabled."};
  } else {
    exitAltHold();
    return {true, "AltHold disabled."};
  }
}