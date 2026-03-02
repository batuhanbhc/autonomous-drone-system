#include "mavros_gate/command_gate.hpp"
#include <chrono>

using namespace std::chrono_literals;

// Sends ARM command
CommandGateNode::CommandResult CommandGateNode::executeArm(const TeleopCmd&, const InternalState&) {
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

  RCLCPP_INFO(get_logger(), "Sending ARM request");
  (void)arming_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->success) {
        RCLCPP_INFO(get_logger(), "ARM accepted by FCU (result=%u).", resp->result);
      } else {
        RCLCPP_WARN(get_logger(), "ARM rejected by FCU (result=%u).", resp->result);
      }
    }
  );

  return {true, "ARM request sent."};
}


// Send DISARM command
CommandGateNode::CommandResult CommandGateNode::executeDisarm(const TeleopCmd&, const InternalState&) {
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

  RCLCPP_INFO(get_logger(), "Sending DISARM request");
  (void)arming_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->success) {
        RCLCPP_INFO(get_logger(), "DISARM accepted by FCU (result=%u).", resp->result);
        
      } else {
        RCLCPP_WARN(get_logger(), "DISARM rejected by FCU (result=%u).", resp->result);
      }
    }
  );

  return {true, "DISARM request sent."};
}


// Activate kill switch window
CommandGateNode::CommandResult CommandGateNode::executeKillSwitch(const TeleopCmd&, const InternalState&) {
  auto new_timer = this->create_wall_timer(2s, [this]() {
    std::lock_guard<std::mutex> lk(state_mtx_);

    state_.kill_switch_window = false;
    RCLCPP_INFO(get_logger(), "KILL-SWITCH window ended.");

    // one-shot: stop future firings
    if (kill_switch_timer_) {
      kill_switch_timer_->cancel();
      kill_switch_timer_.reset();  // optional: release it
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

  return {true, extended ? "KILL-SWITCH window extended (2s)." : "KILL-SWITCH window started (2s)."};
}


CommandGateNode::CommandResult CommandGateNode::executeKillConfirm(const TeleopCmd&, const InternalState& state) {
  // gate on the "kill switch window"
  if (!state.kill_switch_window) {
    return {false, "KILL-CONFIRM rejected: kill-switch window not active."};
  }

  // prevent duplicate kills
  if (state.system_killed) {
    return {false, "KILL-CONFIRM rejected: system already killed."};
  }

  // 2) Need CommandLong client to /mavros/cmd/command
  if (!command_long_client_) {
    return {false, "CommandLong client not initialized."};
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

  RCLCPP_ERROR(get_logger(), "KILL-CONFIRM: sending FLIGHT TERMINATION (CMD=185, param1=1.0)");

  (void)command_long_client_->async_send_request(
    req,
    [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture fut) {
      const auto resp = fut.get();
      if (resp->success) {
        RCLCPP_ERROR(get_logger(), "KILL-CONFIRM accepted by FCU (result=%u).", resp->result);
      } else {
        RCLCPP_ERROR(get_logger(), "KILL-CONFIRM rejected by FCU (result=%u).", resp->result);
      }
    }
  );

  // 4) Update internal state atomically
  InternalStateUpdate update;
  update.system_killed = true;
  update.kill_switch_window = false;
  updateInternalStateAtomic(update);

  return {true, "KILL-CONFIRM: flight termination command sent."};
}