#pragma once

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "drone_msgs/msg/scene_state.hpp"
#include "drone_msgs/msg/toggle.hpp"
#include "drone_msgs/msg/autonomous_action.hpp"

namespace drone_pipeline
{

class AutonomousController : public rclcpp::Node
{
public:
  explicit AutonomousController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AutonomousController() = default;

private:
  struct Config
  {
    uint8_t     drone_id{};
    std::string scene_topic;
    std::string enable_topic;
    std::string output_topic;
    double      control_hz{4.0};
  };

  Config loadConfig();

  void onScene(drone_msgs::msg::SceneState::ConstSharedPtr msg);
  void onEnable(drone_msgs::msg::Toggle::ConstSharedPtr msg);
  void onControlTimer();

  Config config_;

  rclcpp::CallbackGroup::SharedPtr              scene_cb_group_;
  rclcpp::CallbackGroup::SharedPtr              enable_cb_group_;
  rclcpp::CallbackGroup::SharedPtr              timer_cb_group_;

  rclcpp::Subscription<drone_msgs::msg::SceneState>::SharedPtr scene_sub_;
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr     enable_sub_;
  rclcpp::Publisher<drone_msgs::msg::AutonomousAction>::SharedPtr output_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  drone_msgs::msg::SceneState last_scene_;
  std::mutex                  scene_mtx_;
  bool                        has_scene_{false};

  std::atomic<bool> autonomous_enabled_{false};
};

}  // namespace drone_pipeline
