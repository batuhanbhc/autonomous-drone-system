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
    double coverage_half_life_seconds{20.0};
    double search_phase_seconds{30.0};
    double blob_sigma{1.5};
    double ego_sigma{2.0};
    double people_count_normalizer{30.0};
    double count_map_compression_scale{3.0};
    double count_memory_historic_miss_penalty{0.2};
    double max_horizontal_velocity{1.0};
    double horizontal_bin_interval{1.0};
    double max_yaw_rate{0.35};
    double yaw_bin_interval{0.35};
    int grid_h{60};
    int grid_w{60};
    int max_agents{2};
    int cmd_history_len{5};
    int status_history_seconds{5};
    int hotspot_top_k{2};
    double hotspot_min_density{1.5};
    double hotspot_suppression_radius_scale{5.0};
    int hotspot_suppression_radius_min_cells{2};
    std::string local_people_map_mode{"count_density"};
    bool include_local_recent_count_memory_channel{true};
    bool include_shared_count_density_channel{false};
    bool include_instant_fov_channels{true};
    bool include_persistent_coverage_channel{true};
    bool hide_person_features_during_search{true};
    bool save_actor_inputs{true};
    double actor_input_snapshot_interval_seconds{1.0};
  };

  struct ObservationState
  {
    std::vector<float> people_belief_recent;
    std::vector<float> people_belief_historic;
    std::vector<float> people_count_density;
    std::vector<float> people_count_memory_recent;
    std::vector<float> people_count_memory_historic;
    std::vector<float> coverage_map;
    std::vector<float> persistent_coverage_map;
    std::vector<float> own_coverage_map;
    std::vector<float> shared_drone_map;
    std::vector<float> own_ego_map;
    std::vector<float> footprint_map;
    std::vector<int32_t> people_count_last_observed_step;
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

  struct Hotspot
  {
    float x{0.0f};
    float y{0.0f};
    float density{0.0f};
    float age{0.0f};
  };

  struct CsvRow
  {
    std::string line;
  };

  struct StatusHistoryAnchor
  {
    bool valid{false};
    float x{0.0f};
    float y{0.0f};
    float yaw{0.0f};
    double time_s{0.0};
  };

  Config loadConfig();
  std::string resolveSessionDir(const std::string & logs_path);
  void initOnnx();
  void initLogging();
  void resetObservationState();
  void flushLogBuffer();
  std::vector<Hotspot> extractHotspots() const;
  void updateStatusHistory(const drone_msgs::msg::SceneState & scene, std::size_t num_visible);
  float compressCountValue(float value) const;
  float visitedFraction() const;
  bool actorHidesPersonFeatures(bool is_search_phase) const;
  void saveActorInputSnapshot(
    const builtin_interfaces::msg::Time & stamp,
    const drone_msgs::msg::SceneState & scene,
    const InferenceInputs & inputs,
    const std::vector<float> * action);
  std::vector<std::string> actorChannelNames() const;
  std::vector<std::string> localFeatureNames() const;

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
  std::deque<std::array<float, 5>> status_history_;
  StatusHistoryAnchor status_history_anchor_;
  std::uint64_t controller_step_{0};
  int prev_visible_count_{0};

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
  int actor_grid_channels_{0};
  int shared_people_channels_{0};
  int teammate_slots_{0};
  int hotspot_top_k_inferred_{0};
  int hotspot_suppression_radius_cells_{0};
  int status_history_seconds_inferred_{0};
  std::uint64_t actor_input_snapshot_interval_steps_{0};
  bool include_persistent_coverage_channel_{true};
  bool include_recent_count_memory_channel_{true};
  bool include_shared_count_density_channel_{false};
  bool include_instant_fov_channels_{true};
  bool hide_person_features_during_search_{true};
  bool exposes_spatial_memory_channels_{false};
  double historic_half_life_steps_{1.0};
  std::uint64_t search_phase_steps_{0};
  std::string actor_input_snapshot_dir_;
  std::uint64_t actor_input_snapshot_count_{0};

  std::string session_dir_;
  std::ofstream log_file_;
  std::mutex log_mtx_;
  std::vector<CsvRow> log_buffer_;
  static constexpr std::size_t kCsvFlushSize = 256;
};

}  // namespace drone_pipeline
