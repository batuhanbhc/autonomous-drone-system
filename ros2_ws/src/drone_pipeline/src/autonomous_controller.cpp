#include "drone_pipeline/autonomous_controller.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "yaml-cpp/yaml.h"

namespace drone_pipeline
{

// ─────────────────────────────────────────────────────────────────────────────
//  Config
// ─────────────────────────────────────────────────────────────────────────────

AutonomousController::Config AutonomousController::loadConfig()
{
  const std::string share_dir =
    ament_index_cpp::get_package_share_directory("mavros_config");
  const std::string config_path = share_dir + "/config/control_params.yaml";

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to parse YAML: %s", e.what());
    throw;
  }

  Config cfg;
  const std::string ns = "/drone_" + std::to_string(root["drone_id"].as<int>());
  cfg.drone_id     = static_cast<uint8_t>(root["drone_id"].as<int>());
  cfg.scene_topic  = ns + root["custom_topics"]["scene"].as<std::string>();
  cfg.enable_topic = ns + root["custom_topics"]["autonomous_enable"].as<std::string>();
  cfg.output_topic = ns + root["custom_topics"]["autonomous_action"].as<std::string>();
  cfg.control_hz   = root["autonomous_controller"]["hz"].as<double>();
  return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────

AutonomousController::AutonomousController(const rclcpp::NodeOptions & options)
: Node("autonomous_controller", options)
{
  config_ = loadConfig();

  const auto sensor_qos  = rclcpp::SensorDataQoS();
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  scene_cb_group_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  enable_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions scene_opts;
  scene_opts.callback_group = scene_cb_group_;
  scene_sub_ = create_subscription<drone_msgs::msg::SceneState>(
    config_.scene_topic, sensor_qos,
    [this](drone_msgs::msg::SceneState::ConstSharedPtr msg) { onScene(msg); },
    scene_opts);

  rclcpp::SubscriptionOptions enable_opts;
  enable_opts.callback_group = enable_cb_group_;
  enable_sub_ = create_subscription<drone_msgs::msg::Toggle>(
    config_.enable_topic, reliable_qos,
    [this](drone_msgs::msg::Toggle::ConstSharedPtr msg) { onEnable(msg); },
    enable_opts);

  output_pub_ = create_publisher<drone_msgs::msg::AutonomousAction>(
    config_.output_topic, reliable_qos);

  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(std::round(1000.0 / config_.control_hz)));
  control_timer_ = create_wall_timer(
    period_ms,
    [this]() { onControlTimer(); },
    timer_cb_group_);

  RCLCPP_INFO(get_logger(),
    "autonomous_controller ready — scene=%s  enable=%s  output=%s  hz=%.1f",
    config_.scene_topic.c_str(), config_.enable_topic.c_str(),
    config_.output_topic.c_str(), config_.control_hz);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Enable toggle
// ─────────────────────────────────────────────────────────────────────────────

void AutonomousController::onEnable(drone_msgs::msg::Toggle::ConstSharedPtr msg)
{
  const bool was = autonomous_enabled_.exchange(msg->state);
  if (was != msg->state) {
    RCLCPP_INFO(get_logger(), "Autonomous control: %s", msg->state ? "ON" : "OFF");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Scene subscription
// ─────────────────────────────────────────────────────────────────────────────

void AutonomousController::onScene(drone_msgs::msg::SceneState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(scene_mtx_);
  last_scene_ = *msg;
  has_scene_  = true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Control timer
// ─────────────────────────────────────────────────────────────────────────────

void AutonomousController::onControlTimer()
{
  if (!autonomous_enabled_.load()) return;

  drone_msgs::msg::SceneState scene;
  {
    std::lock_guard<std::mutex> lk(scene_mtx_);
    if (has_scene_) scene = last_scene_;
  }

  // TODO: autonomous control logic — compute vx, vy, yaw_rate from scene

  drone_msgs::msg::AutonomousAction cmd;
  cmd.stamp    = has_scene_ ? scene.stamp : static_cast<builtin_interfaces::msg::Time>(this->now());
  cmd.vx       = 1.0f;
  cmd.vy       = 0.0f;
  cmd.yaw_rate = 0.0f;
  output_pub_->publish(cmd);
}

}  // namespace drone_pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(drone_pipeline::AutonomousController)
