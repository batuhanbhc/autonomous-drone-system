#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "drone_msgs/msg/autonomous_action.hpp"
#include "drone_msgs/msg/scene_state.hpp"
#include "drone_msgs/msg/toggle.hpp"
#include "rclcpp/rclcpp.hpp"

namespace drone_pipeline
{

class AutonomousController : public rclcpp::Node
{
public:
  explicit AutonomousController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AutonomousController() override;

private:
  struct Config
  {
    uint8_t drone_id{};
    std::string scene_topic;
    std::string enable_topic;
    std::string output_topic;
    std::string logs_path;
    std::string model_path{"/marl_agent.onnx"};
    double control_hz{4.0};
    double x_min{-15.0};
    double x_max{15.0};
    double y_min{-15.0};
    double y_max{15.0};
    double z_min{0.3};
    double z_max{8.0};
    double max_range{15.0};
    double horizontal_fov_deg{55.8};
    double vertical_fov_deg{43.3};
    double camera_tilt_deg{45.0};
    double recent_half_life_seconds{10.0};
    double historic_half_life_seconds{60.0};
    double coverage_half_life_seconds{60.0};
    double recent_hit_gain{0.6};
    double recent_miss_penalty{0.25};
    double blob_sigma{1.0};
    double ego_sigma{2.5};
    double max_horizontal_velocity{1.0};
    double horizontal_bin_interval{1.0};
    double max_yaw_rate{0.5};
    double yaw_bin_interval{0.5};
    int grid_h{80};
    int grid_w{80};
    int max_agents{2};
    int cmd_history_len{4};
  };

  struct ObservationState
  {
    std::vector<float> people_belief_recent;
    std::vector<float> people_belief_historic;
    std::vector<float> coverage_map;
    std::vector<float> own_coverage_map;
    std::vector<float> shared_drone_map;
    std::vector<float> own_ego_map;
    std::vector<float> footprint_map;
  };

  struct InferenceInputs
  {
    std::vector<float> grid;
    std::vector<float> local_base;
    std::vector<float> move_mask;
    std::size_t visible_count{0};
    float centroid_present{0.0f};
    float centroid_forward_offset{0.0f};
    float centroid_lateral_offset{0.0f};
  };

  struct CsvRow
  {
    std::string line;
  };

  Config loadConfig();
  std::string resolveSessionDir(const std::string & logs_path);
  void initOnnx();
  void initLogging();
  void resetObservationState();
  void flushLogBuffer();

  void onScene(drone_msgs::msg::SceneState::ConstSharedPtr msg);
  void onEnable(drone_msgs::msg::Toggle::ConstSharedPtr msg);
  void onControlTimer();

  InferenceInputs buildInferenceInputs(const drone_msgs::msg::SceneState & scene);
  std::vector<float> runInference(const InferenceInputs & inputs, double & inference_ms);
  void publishCommand(
    const builtin_interfaces::msg::Time & stamp,
    float vx,
    float vy,
    float yaw_rate);
  void logEvent(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & event,
    const drone_msgs::msg::SceneState * scene,
    const InferenceInputs * inputs,
    const std::vector<float> * action,
    double inference_ms);

  Config config_;
  ObservationState obs_state_;
  std::deque<std::array<float, 3>> cmd_history_;
  std::uint64_t controller_step_{0};

  rclcpp::CallbackGroup::SharedPtr scene_cb_group_;
  rclcpp::CallbackGroup::SharedPtr enable_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr flush_cb_group_;

  rclcpp::Subscription<drone_msgs::msg::SceneState>::SharedPtr scene_sub_;
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr enable_sub_;
  rclcpp::Publisher<drone_msgs::msg::AutonomousAction>::SharedPtr output_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr flush_timer_;

  drone_msgs::msg::SceneState last_scene_;
  std::mutex scene_mtx_;
  bool has_scene_{false};

  std::atomic<bool> autonomous_enabled_{false};

  Ort::Env ort_env_{ORT_LOGGING_LEVEL_WARNING, "autonomous_controller"};
  Ort::SessionOptions ort_session_options_;
  std::unique_ptr<Ort::Session> ort_session_;
  Ort::MemoryInfo ort_memory_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
  std::vector<int64_t> grid_shape_;
  std::vector<int64_t> local_shape_;
  std::vector<int64_t> move_mask_shape_;
  std::string grid_input_name_;
  std::string local_input_name_;
  std::string move_mask_input_name_;
  std::string action_output_name_;
  std::vector<const char *> input_names_;
  std::vector<const char *> output_names_;
  std::vector<float> vx_bins_;
  std::vector<float> vy_bins_;
  std::vector<float> yaw_rate_bins_;

  std::string session_dir_;
  std::ofstream log_file_;
  std::mutex log_mtx_;
  std::vector<CsvRow> log_buffer_;
  static constexpr std::size_t kCsvFlushSize = 256;
};

}  // namespace drone_pipeline
