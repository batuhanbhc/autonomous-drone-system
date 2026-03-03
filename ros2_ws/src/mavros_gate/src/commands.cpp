#include "mavros_gate/control_gate.hpp"
#include <chrono>

using namespace std::chrono_literals;

ControlGateNode::CommandResult 
ControlGateNode::executeArm(const TeleopCmd&, const InternalState&) {
  /*
  This function sends asynchronous request to FCU via mavros to send "Arm" command
  */
  if (!arming_client_) {
    return {false, "Arming client not initialized."};
  }

  if (!arming_client_->service_is_ready()) {
    // Avoid blocking a callback thread for long; just fail fast (or short wait).
    if (!arming_client_->wait_for_service(200ms)) {
      return {false, "Service /mavros/cmd/arming not available."};
    }
  }

  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = true; // ARM

  (void)arming_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->success) {
        RCLCPP_INFO(get_logger(), "Arm accepted by FCU.");

        InternalStateUpdate update;
        update.armed = true;
        updateInternalStateAtomic(update);
      } else {
        RCLCPP_WARN(get_logger(), "Arm rejected by FCU (result=%u).", resp->result);
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
    if (!arming_client_->wait_for_service(200ms)) {
      return {false, "Service /mavros/cmd/arming not available."};
    }
  }

  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = false; // DISARM

  (void)arming_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->success) {
        RCLCPP_INFO(get_logger(), "Disarm accepted by FCU");

        InternalStateUpdate update;
        update.armed = false;
        updateInternalStateAtomic(update);
        
      } else {
        RCLCPP_WARN(get_logger(), "Disarm rejected by FCU (result=%u).", resp->result);
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
  RCLCPP_WARN(get_logger(), "-----------------------------------------------");
  RCLCPP_WARN(get_logger(), "-----     ENABLING KILL-SWITCH WINDOW     -----");
  RCLCPP_WARN(get_logger(), "-----------------------------------------------");

  auto new_timer = this->create_wall_timer(2s, [this]() {
    std::lock_guard<std::mutex> lk(state_mtx_);

    if (state_.kill_switch_window) {
      state_.kill_switch_window = false;
      RCLCPP_INFO(get_logger(), "Kill-switch window ended.");
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
ControlGateNode::executeKillConfirm(const TeleopCmd&, const InternalState& state) {
  /*
  This function sends asynchronous request to FCU via mavros to send "Flight Termination" command.
  This command immediately stops the motors, hence copter will fall if it is not landed.
  */

  if (!command_long_client_) {
    return {false, "CommandLong client not initialized."};
  }

  // gate on the "kill switch window"
  if (!state.kill_switch_window) {
    return {false, "Kill-switch window not active."};
  }

  if (!command_long_client_->service_is_ready()) {
    if (!command_long_client_->wait_for_service(200ms)) {
      return {false, "Service /mavros/cmd/command not available."};
    }
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
        RCLCPP_ERROR(get_logger(), "-------------------------------------------------------");
        RCLCPP_ERROR(get_logger(), "-----     Flight termination accepted by FCU.     -----");
        RCLCPP_ERROR(get_logger(), "-----    System is killed, reboot is required.    -----");
        RCLCPP_ERROR(get_logger(), "-------------------------------------------------------");

        // update state to close kill switch window, and set system as killed
        InternalStateUpdate update;
        update.system_killed = true;
        update.kill_switch_window = false;
        updateInternalStateAtomic(update);

      } else {
        RCLCPP_ERROR(get_logger(), "Flight termination rejected by FCU (result=%u).", resp->result);
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
    if (!set_mode_client_->wait_for_service(200ms)) {
      return {false, "Service /mavros/set_mode not available."};
    }
  }

  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();

  req->base_mode = 0;
  req->custom_mode = "LAND";

  (void)set_mode_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->mode_sent) {
        RCLCPP_INFO(get_logger(), "Land mode request reached FCU.");
      } else {
        RCLCPP_WARN(get_logger(), "Land mode request could not reach FCU.");
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
    if (!command_long_client_->wait_for_service(200ms)) {
      return {false, "Service /mavros/cmd/command not available."};
    }
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
        RCLCPP_INFO(get_logger(), "RTL accepted by FCU.");
      } else {
        RCLCPP_WARN(get_logger(), "RTL rejected by FCU (result=%u).", resp->result);
      }
    }
  );

  return {true, "RTL request sent."};
}



ControlGateNode::CommandResult
ControlGateNode::executeLoiter(const TeleopCmd&, const InternalState&) {
  /*
    Uses /mavros/set_mode (mavros_msgs/srv/SetMode).
  */

  if (!set_mode_client_) {
    return {false, "SetMode client not initialized."};
  }

  if (!set_mode_client_->service_is_ready()) {
    if (!set_mode_client_->wait_for_service(200ms)) {
      return {false, "Service /mavros/set_mode not available."};
    }
  }

  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();

  req->base_mode = 0;                 // leave autopilot to interpret custom_mode
  req->custom_mode = "LOITER";   

  (void)set_mode_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->mode_sent) {
        RCLCPP_INFO(get_logger(), "Loiter mode request reached FCU.");
      } else {
        RCLCPP_WARN(get_logger(), "Loiter mode request could not reach FCU.");
      }
    }
  );

  return {true, "Loiter mode request sent."};
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
    if (!set_mode_client_->wait_for_service(200ms)) {
      return {false, "Service /mavros/set_mode not available."};
    }
  }

  auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  mode_req->base_mode = 0;
  mode_req->custom_mode = "GUIDED";

  (void)set_mode_client_->async_send_request(
    mode_req,
    [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->mode_sent) {
        RCLCPP_INFO(get_logger(), "Guided mode request reached FCU.");
      } else {
        RCLCPP_WARN(get_logger(), "Guided mode request could not reach FCU.");
      }
    }
  );

  return {true, "Guided mode request sent."};
}

ControlGateNode::CommandResult
ControlGateNode::executeTakeoff(const TeleopCmd& msg, const InternalState& state) {
  /*
    Takeoff using MAVROS:
      1) Set mode to GUIDED (via executeGuided)
      2) Call /mavros/cmd/takeoff (mavros_msgs/srv/CommandTOL).
  */

  if (!takeoff_client_) {
    return {false, "Takeoff client not initialized."};
  }

  
  // 1) Set GUIDED mode (separate command)
  const auto guided_res = executeGuided(msg, state);
  if (!guided_res.success) {
    return guided_res;
  }
  
  // 2) Send takeoff
  if (!takeoff_client_->service_is_ready()) {
    if (!takeoff_client_->wait_for_service(200ms)) {
      return {false, "Service /mavros/cmd/takeoff not available."};
    }
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
        RCLCPP_INFO(get_logger(), "Takeoff accepted by FCU.");
      } else {
        RCLCPP_WARN(get_logger(), "Takeoff rejected by FCU (result=%u).", resp->result);
      }
    }
  );

  std::ostringstream oss;
  oss << "Takeoff (" << std::fixed << std::setprecision(2) << takeoff_m << "m) request sent.";
  return {true, oss.str()};
}


ControlGateNode::CommandResult
ControlGateNode::executeControlToggle(const TeleopCmd&, const InternalState& state) {
  /*
    Toggles control mode between ControlGateNode::ControlMode::Auto/Manual
  */

  bool tmp_flag = 0; // ControlMode::Auto
  if (state.control_mode == ControlMode::Manual) {
    tmp_flag = 1;
  }

  InternalStateUpdate upt;
  upt.control_mode = tmp_flag ? ControlMode::Auto : ControlMode::Manual;
  updateInternalStateAtomic(upt);

  const char* mode_old = tmp_flag ? "MANUAL" : "AUTO";
  const char* mode_new = tmp_flag ? "AUTO" : "MANUAL";
  return {true, std::string("Control mode changed: ") + mode_old + " -> " + mode_new + "."};
}


ControlGateNode::CommandResult
ControlGateNode::executeKeyboardToggle(const TeleopCmd& msg, const InternalState&) {
  /*
    Updates internal state according the incoming keyboard state
  */
  
  InternalStateUpdate upt;
  upt.keyboard_on = msg.bool_1;
  updateInternalStateAtomic(upt);
  
  std::string keyboard_state = msg.bool_1 ? "ON": "OFF";
  std::string out_msg = std::string("Keyboard state: ") + keyboard_state;
  return {true, out_msg};
}


ControlGateNode::CommandResult
ControlGateNode::executeChangeSpeed(const TeleopCmd& msg, const InternalState&) {
  /*
    Updates internal state according to current ControlGateNode::ControlMode::/Manual mode velocity settings
  */
  
  double hv = msg.float_1, vv = msg.float_2;

  InternalStateUpdate upt;
  upt.vel = VelocityLevel{hv, vv};
  updateInternalStateAtomic(upt);

  std::ostringstream oss;
  oss << "Velocity levels: (Horizontal: " << std::fixed << std::setprecision(2) << hv << " m/s, " \
      << "Vertical: " << std::fixed << std::setprecision(2) << vv << " m/s).";
  return {true, oss.str()};
}