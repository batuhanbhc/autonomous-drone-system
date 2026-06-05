#include "drone_pipeline/autonomous_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "yaml-cpp/yaml.h"

namespace fs = std::filesystem;

namespace drone_pipeline
{

namespace
{

template<typename T>
T yamlOr(const YAML::Node & node, const char * key, const T & default_value)
{
  return node[key] ? node[key].as<T>() : default_value;
}

template<typename T>
T yamlNestedOr(
  const YAML::Node & parent,
  const char * section,
  const char * key,
  const T & default_value)
{
  const YAML::Node subsection = parent[section];
  if (subsection && subsection[key]) {
    return subsection[key].as<T>();
  }
  return yamlOr<T>(parent, key, default_value);
}

double clampDouble(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double angleWrap(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::vector<float> buildSymmetricBins(double max_abs, double interval)
{
  if (max_abs <= 0.0 || interval <= 0.0) {
    throw std::runtime_error("Invalid action-bin configuration");
  }
  const double steps = max_abs / interval;
  const auto rounded_steps = static_cast<int>(std::llround(steps));
  if (std::abs(steps - static_cast<double>(rounded_steps)) > 1e-6) {
    throw std::runtime_error("Action-bin max must be divisible by interval");
  }

  std::vector<float> bins;
  bins.reserve(static_cast<std::size_t>(2 * rounded_steps + 1));
  for (int i = -rounded_steps; i <= rounded_steps; ++i) {
    bins.push_back(static_cast<float>(i * interval));
  }
  return bins;
}

std::vector<int64_t> resolveShape(const std::vector<int64_t> & raw_shape)
{
  std::vector<int64_t> shape = raw_shape;
  for (auto & dim : shape) {
    if (dim <= 0) {
      dim = 1;
    }
  }
  return shape;
}

std::size_t gridIndex(int /*h*/, int w, int v, int u)
{
  return static_cast<std::size_t>(v) * static_cast<std::size_t>(w) +
    static_cast<std::size_t>(u);
}

std::size_t channelIndex(int h, int w, int c, int v, int u)
{
  return static_cast<std::size_t>(c) * static_cast<std::size_t>(h) * static_cast<std::size_t>(w) +
    static_cast<std::size_t>(v) * static_cast<std::size_t>(w) +
    static_cast<std::size_t>(u);
}

std::pair<int, int> worldToGrid(
  double x,
  double y,
  double x_min,
  double x_max,
  double y_min,
  double y_max,
  int grid_h,
  int grid_w)
{
  const double x_clamped = clampDouble(x, x_min, x_max);
  const double y_clamped = clampDouble(y, y_min, y_max);
  const int u = static_cast<int>(
    (x_clamped - x_min) / std::max(1e-6, x_max - x_min) * static_cast<double>(grid_w - 1));
  const int v = static_cast<int>(
    (y_clamped - y_min) / std::max(1e-6, y_max - y_min) * static_cast<double>(grid_h - 1));
  return {
    std::max(0, std::min(grid_h - 1, v)),
    std::max(0, std::min(grid_w - 1, u))
  };
}

std::pair<double, double> gridToWorld(
  int v,
  int u,
  double x_min,
  double x_max,
  double y_min,
  double y_max,
  int grid_h,
  int grid_w)
{
  const double x = x_min +
    (static_cast<double>(u) / std::max(1, grid_w - 1)) * (x_max - x_min);
  const double y = y_min +
    (static_cast<double>(v) / std::max(1, grid_h - 1)) * (y_max - y_min);
  return {x, y};
}

double clampPitchFromDown(double angle_rad)
{
  constexpr double kHorizonEps = 1e-3;
  return clampDouble(
    angle_rad,
    -0.5 * M_PI + kHorizonEps,
    0.5 * M_PI - kHorizonEps);
}

std::pair<double, double> geometricMedian(
  const std::vector<std::pair<double, double>> & points,
  int max_iter = 50,
  double tol = 1e-6)
{
  if (points.size() == 1) return points[0];
  double px = 0.0, py = 0.0;
  for (const auto & [x, y] : points) { px += x; py += y; }
  px /= static_cast<double>(points.size());
  py /= static_cast<double>(points.size());
  for (int iter = 0; iter < max_iter; ++iter) {
    double num_x = 0.0, num_y = 0.0, denom = 0.0;
    for (const auto & [x, y] : points) {
      const double dist = std::hypot(px - x, py - y);
      if (dist < 1e-10) continue;
      const double w = 1.0 / dist;
      num_x += w * x; num_y += w * y; denom += w;
    }
    if (denom < 1e-10) break;
    const double new_px = num_x / denom;
    const double new_py = num_y / denom;
    if (std::hypot(new_px - px, new_py - py) < tol) { px = new_px; py = new_py; break; }
    px = new_px; py = new_py;
  }
  return {px, py};
}

std::pair<double, double> footprintForwardExtents(
  double z,
  double camera_tilt_deg,
  double vertical_fov_deg)
{
  const double tilt = clampPitchFromDown(0.5 * M_PI - camera_tilt_deg * M_PI / 180.0);
  const double half_v = 0.5 * vertical_fov_deg * M_PI / 180.0;
  const double lower_angle = clampPitchFromDown(tilt - half_v);
  const double upper_angle = clampPitchFromDown(tilt + half_v);
  const double lower_forward = z * std::tan(lower_angle);
  const double upper_forward = z * std::tan(upper_angle);
  return {std::min(lower_forward, upper_forward), std::max(lower_forward, upper_forward)};
}

double lateralHalfWidthAtForwardDistance(
  double forward,
  double z,
  double horizontal_fov_deg)
{
  const double half_h = 0.5 * horizontal_fov_deg * M_PI / 180.0;
  const double slant = std::hypot(z, forward);
  return slant * std::tan(half_h);
}

std::pair<double, double> worldToDroneLocal(
  double px,
  double py,
  double drone_x,
  double drone_y,
  double yaw)
{
  const double dx = px - drone_x;
  const double dy = py - drone_y;
  const double forward = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double lateral = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  return {forward, lateral};
}

bool pointInFootprintLocal(
  double forward,
  double lateral,
  double z,
  double camera_tilt_deg,
  double horizontal_fov_deg,
  double vertical_fov_deg)
{
  const auto [min_forward, max_forward] =
    footprintForwardExtents(z, camera_tilt_deg, vertical_fov_deg);
  if (forward < min_forward || forward > max_forward) {
    return false;
  }
  const double tilt = clampPitchFromDown(0.5 * M_PI - camera_tilt_deg * M_PI / 180.0);
  const double half_v = 0.5 * vertical_fov_deg * M_PI / 180.0;
  const double pitch_from_down = std::atan2(forward, z);
  if (std::abs(pitch_from_down - tilt) > half_v + 1e-9) {
    return false;
  }
  const double half_width = lateralHalfWidthAtForwardDistance(forward, z, horizontal_fov_deg);
  return std::abs(lateral) <= half_width;
}

void splatGaussianMax(
  std::vector<float> & grid,
  int grid_h,
  int grid_w,
  int center_v,
  int center_u,
  double sigma)
{
  const double denom = 2.0 * sigma * sigma;
  for (int v = 0; v < grid_h; ++v) {
    for (int u = 0; u < grid_w; ++u) {
      const double dv = static_cast<double>(v - center_v);
      const double du = static_cast<double>(u - center_u);
      const float value = static_cast<float>(std::exp(-(dv * dv + du * du) / denom));
      const auto idx = gridIndex(grid_h, grid_w, v, u);
      grid[idx] = std::max(grid[idx], value);
    }
  }
}

void splatGaussianAdd(
  std::vector<float> & grid,
  int grid_h,
  int grid_w,
  int center_v,
  int center_u,
  double sigma)
{
  const double denom = 2.0 * sigma * sigma;
  for (int v = 0; v < grid_h; ++v) {
    for (int u = 0; u < grid_w; ++u) {
      const double dv = static_cast<double>(v - center_v);
      const double du = static_cast<double>(u - center_u);
      grid[gridIndex(grid_h, grid_w, v, u)] +=
        static_cast<float>(std::exp(-(dv * dv + du * du) / denom));
    }
  }
}

std::string maskToString(const std::vector<float> & move_mask)
{
  std::string mask;
  mask.reserve(move_mask.size());
  for (float value : move_mask) {
    mask.push_back(value > 0.5f ? '1' : '0');
  }
  return mask;
}

builtin_interfaces::msg::Time nowAsBuiltinTime(rclcpp::Node & node)
{
  const auto now = node.now();
  const auto total_ns = now.nanoseconds();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(total_ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);
  return stamp;
}

}  // namespace

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
  const int drone_id = root["drone_id"].as<int>();
  const std::string ns = "/drone_" + std::to_string(drone_id);

  cfg.drone_id = static_cast<uint8_t>(drone_id);
  cfg.scene_topic = ns + root["custom_topics"]["scene"].as<std::string>();
  cfg.enable_topic = ns + root["custom_topics"]["autonomous_enable"].as<std::string>();
  cfg.output_topic = ns + root["custom_topics"]["autonomous_action"].as<std::string>();
  cfg.logs_path = root["flight_params"]["logs_path"].as<std::string>();
  cfg.control_hz = root["autonomous_controller"]["hz"].as<double>();
  cfg.camera_tilt_deg = root["camera"]["camera_mount_angle"].as<double>();

  const auto controller = root["autonomous_controller"];
  cfg.model_path = yamlOr<std::string>(controller, "model_path", cfg.model_path);
  cfg.x_min = yamlOr<double>(controller, "x_min", cfg.x_min);
  cfg.x_max = yamlOr<double>(controller, "x_max", cfg.x_max);
  cfg.y_min = yamlOr<double>(controller, "y_min", cfg.y_min);
  cfg.y_max = yamlOr<double>(controller, "y_max", cfg.y_max);
  cfg.max_range = yamlOr<double>(controller, "max_range", cfg.max_range);
  cfg.horizontal_fov_deg =
    yamlOr<double>(controller, "horizontal_fov_deg", cfg.horizontal_fov_deg);
  cfg.vertical_fov_deg =
    yamlOr<double>(controller, "vertical_fov_deg", cfg.vertical_fov_deg);
  cfg.search_phase_seconds =
    yamlOr<double>(controller, "search_phase_seconds", cfg.search_phase_seconds);
  cfg.recent_half_life_seconds = yamlNestedOr<double>(
    controller, "observation_update", "recent_half_life_seconds", cfg.recent_half_life_seconds);
  cfg.historic_half_life_seconds = yamlNestedOr<double>(
    controller, "observation_update", "historic_half_life_seconds", cfg.historic_half_life_seconds);
  cfg.coverage_half_life_seconds = yamlNestedOr<double>(
    controller, "observation_update", "coverage_half_life_seconds", cfg.coverage_half_life_seconds);
  cfg.blob_sigma =
    yamlNestedOr<double>(controller, "observation_update", "blob_sigma", cfg.blob_sigma);
  cfg.ego_sigma =
    yamlNestedOr<double>(controller, "observation_update", "ego_sigma", cfg.ego_sigma);
  cfg.people_count_normalizer = yamlNestedOr<double>(
    controller, "observation_update", "people_count_normalizer", cfg.people_count_normalizer);
  cfg.count_map_compression_scale = yamlNestedOr<double>(
    controller,
    "observation_update",
    "count_map_compression_scale",
    cfg.count_map_compression_scale);
  cfg.count_memory_historic_miss_penalty = yamlNestedOr<double>(
    controller,
    "observation_update",
    "count_memory_historic_miss_penalty",
    cfg.count_memory_historic_miss_penalty);
  cfg.count_memory_decay_grace_period_seconds = yamlNestedOr<double>(
    controller,
    "observation_update",
    "count_memory_decay_grace_period_seconds",
    cfg.count_memory_decay_grace_period_seconds);
  cfg.max_horizontal_velocity = yamlNestedOr<double>(
    controller, "action_bins", "max_horizontal_velocity", cfg.max_horizontal_velocity);
  cfg.horizontal_bin_interval = yamlNestedOr<double>(
    controller, "action_bins", "horizontal_bin_interval", cfg.horizontal_bin_interval);
  cfg.max_yaw_rate =
    yamlNestedOr<double>(controller, "action_bins", "max_yaw_rate", cfg.max_yaw_rate);
  cfg.yaw_bin_interval =
    yamlNestedOr<double>(controller, "action_bins", "yaw_bin_interval", cfg.yaw_bin_interval);
  cfg.max_agents = yamlOr<int>(controller, "max_agents", cfg.max_agents);
  cfg.cmd_history_len =
    yamlNestedOr<int>(controller, "network", "cmd_history_len", cfg.cmd_history_len);
  cfg.status_history_seconds = yamlNestedOr<int>(
    controller, "network", "status_history_seconds", cfg.status_history_seconds);
  cfg.hotspot_top_k =
    yamlNestedOr<int>(controller, "network", "hotspot_top_k", cfg.hotspot_top_k);
  cfg.hotspot_min_density = yamlNestedOr<double>(
    controller, "observation_update", "hotspot_min_density", cfg.hotspot_min_density);
  cfg.hotspot_suppression_radius_scale = yamlNestedOr<double>(
    controller,
    "observation_update",
    "hotspot_suppression_radius_scale",
    cfg.hotspot_suppression_radius_scale);
  cfg.hotspot_suppression_radius_min_cells = yamlNestedOr<int>(
    controller,
    "observation_update",
    "hotspot_suppression_radius_min_cells",
    cfg.hotspot_suppression_radius_min_cells);
  cfg.local_people_map_mode = yamlNestedOr<std::string>(
    controller, "network", "local_people_map_mode", cfg.local_people_map_mode);
  cfg.include_local_recent_count_memory_channel = yamlNestedOr<bool>(
    controller,
    "network",
    "include_local_recent_count_memory_channel",
    cfg.include_local_recent_count_memory_channel);
  cfg.include_shared_count_density_channel = yamlNestedOr<bool>(
    controller,
    "network",
    "include_shared_count_density_channel",
    cfg.include_shared_count_density_channel);
  cfg.include_shared_count_memory_staleness_channel = yamlNestedOr<bool>(
    controller,
    "network",
    "include_shared_count_memory_staleness_channel",
    cfg.include_shared_count_memory_staleness_channel);
  cfg.include_instant_fov_channels = yamlNestedOr<bool>(
    controller,
    "network",
    "include_instant_fov_channels",
    cfg.include_instant_fov_channels);
  cfg.include_persistent_coverage_channel = yamlNestedOr<bool>(
    controller,
    "network",
    "include_persistent_coverage_channel",
    cfg.include_persistent_coverage_channel);
  cfg.hide_person_features_during_search = yamlNestedOr<bool>(
    controller,
    "network",
    "hide_person_features_during_search",
    cfg.hide_person_features_during_search);
  cfg.enable_agent_ids = yamlNestedOr<bool>(
    controller,
    "network",
    "enable_agent_ids",
    cfg.enable_agent_ids);
  cfg.save_actor_inputs = yamlNestedOr<bool>(
    controller,
    "input_snapshot",
    "enabled",
    cfg.save_actor_inputs);
  cfg.actor_input_snapshot_interval_seconds = yamlNestedOr<double>(
    controller,
    "input_snapshot",
    "interval_seconds",
    cfg.actor_input_snapshot_interval_seconds);

  if (cfg.local_people_map_mode != "instant" && cfg.local_people_map_mode != "count_density") {
    throw std::runtime_error("Unsupported local_people_map_mode in autonomous_controller config");
  }
  if (cfg.people_count_normalizer <= 0.0) {
    throw std::runtime_error("people_count_normalizer must be > 0");
  }
  if (cfg.count_map_compression_scale <= 0.0) {
    throw std::runtime_error("count_map_compression_scale must be > 0");
  }
  if (cfg.count_memory_decay_grace_period_seconds < 0.0) {
    throw std::runtime_error("count_memory_decay_grace_period_seconds must be >= 0");
  }
  if (cfg.status_history_seconds < 0) {
    throw std::runtime_error("status_history_seconds must be >= 0");
  }
  if (cfg.actor_input_snapshot_interval_seconds < 0.0) {
    throw std::runtime_error("actor_input_snapshot_interval_seconds must be >= 0");
  }

  return cfg;
}

std::string AutonomousController::resolveSessionDir(const std::string & logs_path)
{
  const std::string configured_session_dir = get_parameter("session_dir").as_string();
  if (!configured_session_dir.empty()) {
    fs::create_directories(configured_session_dir);
    return configured_session_dir;
  }

  fs::create_directories(logs_path);
  std::size_t dir_count = 0;
  for (const auto & entry : fs::directory_iterator(logs_path)) {
    if (entry.is_directory()) {
      ++dir_count;
    }
  }

  std::ostringstream oss;
  oss << std::setw(4) << std::setfill('0') << (dir_count + 1);
  const std::string candidate = logs_path + "/" + oss.str();
  fs::create_directory(candidate);
  return candidate;
}

void AutonomousController::initOnnx()
{
  ort_session_options_.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
  ort_session_options_.SetIntraOpNumThreads(4);
  ort_session_options_.SetInterOpNumThreads(1);
  ort_session_options_.AddConfigEntry("session.intra_op.allow_spinning", "0");
  ort_session_options_.AddConfigEntry("session.inter_op.allow_spinning", "0");
  ort_session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

  ort_session_ = std::make_unique<Ort::Session>(
    ort_env_,
    config_.model_path.c_str(),
    ort_session_options_);

  Ort::AllocatorWithDefaultOptions allocator;

  if (ort_session_->GetInputCount() != 3) {
    throw std::runtime_error("Expected 3 ONNX inputs: grid, local_base, move_mask");
  }
  if (ort_session_->GetOutputCount() < 1) {
    throw std::runtime_error("ONNX model has no outputs");
  }

  for (std::size_t i = 0; i < ort_session_->GetInputCount(); ++i) {
    auto name = ort_session_->GetInputNameAllocated(i, allocator);
    const std::string input_name = name.get();
    const auto shape = resolveShape(
      ort_session_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());

    if (input_name == "grid") {
      grid_input_name_ = input_name;
      grid_shape_ = shape;
    } else if (input_name == "local_base") {
      local_input_name_ = input_name;
      local_shape_ = shape;
    } else if (input_name == "move_mask") {
      move_mask_input_name_ = input_name;
      move_mask_shape_ = shape;
    }
  }

  if (grid_input_name_.empty() || local_input_name_.empty() || move_mask_input_name_.empty()) {
    throw std::runtime_error("Failed to resolve required ONNX input names");
  }

  auto output_name = ort_session_->GetOutputNameAllocated(0, allocator);
  action_output_name_ = output_name.get();

  if (grid_shape_.size() != 4 || local_shape_.size() != 2 || move_mask_shape_.size() != 2) {
    throw std::runtime_error("Unexpected ONNX tensor ranks");
  }

  actor_grid_channels_ = static_cast<int>(grid_shape_[1]);
  config_.grid_h = static_cast<int>(grid_shape_[2]);
  config_.grid_w = static_cast<int>(grid_shape_[3]);
  include_persistent_coverage_channel_ = config_.include_persistent_coverage_channel;
  include_recent_count_memory_channel_ = config_.include_local_recent_count_memory_channel;
  include_shared_count_density_channel_ = config_.include_shared_count_density_channel;
  include_shared_count_memory_staleness_channel_ =
    config_.include_shared_count_memory_staleness_channel;
  include_instant_fov_channels_ = config_.include_instant_fov_channels;
  enable_agent_ids_ = config_.enable_agent_ids;
  hide_person_features_during_search_ = config_.hide_person_features_during_search;
  const int base_actor_grid_channels =
    6 +
    (include_recent_count_memory_channel_ ? 1 : 0) +
    (include_instant_fov_channels_ ? 2 : 0) +
    (include_persistent_coverage_channel_ ? 1 : 0) +
    (include_shared_count_density_channel_ ? 1 : 0) +
    (include_shared_count_memory_staleness_channel_ ? 1 : 0);
  const int actor_grid_delta = actor_grid_channels_ - base_actor_grid_channels;
  exposes_spatial_memory_channels_ = (actor_grid_delta == 2);
  shared_people_channels_ =
    1 +
    (exposes_spatial_memory_channels_ ? 2 : 0) +
    (include_shared_count_density_channel_ ? 1 : 0) +
    (include_shared_count_memory_staleness_channel_ ? 1 : 0) +
    (include_persistent_coverage_channel_ ? 1 : 0);
  historic_half_life_steps_ = std::max(1.0, config_.historic_half_life_seconds * config_.control_hz);
  count_memory_decay_grace_steps_ = static_cast<std::uint64_t>(std::max<long long>(
    0LL,
    std::llround(std::ceil(
      config_.count_memory_decay_grace_period_seconds * config_.control_hz))));
  hotspot_suppression_radius_cells_ = std::max(
    config_.hotspot_suppression_radius_min_cells,
    static_cast<int>(std::lround(config_.hotspot_suppression_radius_scale * config_.blob_sigma)));
  if (config_.search_phase_seconds > 0.0) {
    search_phase_steps_ = static_cast<std::uint64_t>(std::ceil(
      config_.search_phase_seconds * config_.control_hz - 1e-9));
  } else {
    search_phase_steps_ = 0;
  }
  actor_input_snapshot_interval_steps_ =
    config_.save_actor_inputs && config_.actor_input_snapshot_interval_seconds > 0.0 ?
    static_cast<std::uint64_t>(std::max<long long>(
      1LL,
      std::llround(config_.actor_input_snapshot_interval_seconds * config_.control_hz))) :
    0U;

  input_names_ = {
    grid_input_name_.c_str(),
    local_input_name_.c_str(),
    move_mask_input_name_.c_str(),
  };
  output_names_ = {action_output_name_.c_str()};

  vx_bins_ = buildSymmetricBins(config_.max_horizontal_velocity, config_.horizontal_bin_interval);
  vy_bins_ = vx_bins_;
  yaw_rate_bins_ = buildSymmetricBins(config_.max_yaw_rate, config_.yaw_bin_interval);

  if (actor_grid_delta != 0 && actor_grid_delta != 2) {
    throw std::runtime_error("Unsupported ONNX actor grid channel count");
  }

  const std::size_t base_local_dim = static_cast<std::size_t>(local_shape_[1]);
  bool found_layout = false;
  for (int teammate_slots_candidate = 0;
    teammate_slots_candidate <= std::max(0, config_.max_agents - 1);
    ++teammate_slots_candidate)
  {
    const std::size_t static_base_dim = static_cast<std::size_t>(
      13 +
      (enable_agent_ids_ ? 1 : 0) +
      6 * teammate_slots_candidate +
      5 * std::max(0, config_.status_history_seconds) +
      3 * config_.cmd_history_len);
    if (base_local_dim < static_base_dim) {
      continue;
    }
    const std::size_t extra_dim = base_local_dim - static_base_dim;
    if (extra_dim % 5 != 0) {
      continue;
    }
    teammate_slots_ = teammate_slots_candidate;
    hotspot_top_k_inferred_ = static_cast<int>(extra_dim / 5);
    status_history_seconds_inferred_ = std::max(0, config_.status_history_seconds);
    found_layout = true;
  }
  if (!found_layout) {
    throw std::runtime_error(
      "ONNX local_base dimension does not match the current PyBullet feature layout");
  }
  if (config_.hotspot_top_k != hotspot_top_k_inferred_) {
    throw std::runtime_error("Configured hotspot_top_k does not match ONNX local_base layout");
  }
  if (teammate_slots_ > 0) {
    RCLCPP_WARN(
      get_logger(),
      "ONNX model expects %d teammate slot(s), but ROS2 autonomous_controller still "
      "zero-fills teammate state and teammate coverage inputs. The model will load, "
      "but multi-drone observation parity with training is not implemented.",
      teammate_slots_);
  }
  const std::size_t expected_move_mask_dim = vx_bins_.size() * vy_bins_.size();
  if (static_cast<std::size_t>(move_mask_shape_[1]) != expected_move_mask_dim) {
    throw std::runtime_error("ONNX move_mask dimension does not match configured action bins");
  }
}

void AutonomousController::initLogging()
{
  session_dir_ = resolveSessionDir(config_.logs_path);
  const std::string log_path = session_dir_ + "/autonomous_controller.csv";
  actor_input_snapshot_dir_ = session_dir_ + "/actor_input_snapshots";
  if (config_.save_actor_inputs && actor_input_snapshot_interval_steps_ > 0) {
    fs::create_directories(actor_input_snapshot_dir_);
  }

  log_file_.open(log_path, std::ios::out | std::ios::app);
  if (!log_file_.is_open()) {
    throw std::runtime_error("Cannot open autonomous controller CSV: " + log_path);
  }

  log_file_.seekp(0, std::ios::end);
  if (log_file_.tellp() == 0) {
    log_file_
      << "timestamp_sec,timestamp_nanosec,step,event,enabled,odom_valid,track_count,"
      << "visible_count,centroid_present,centroid_forward_offset,centroid_lateral_offset,"
      << "inference_ms,cmd_vx,cmd_vy,cmd_yaw_rate,drone_x,drone_y,drone_z,drone_yaw,"
      << "principal_x,principal_y,move_mask\n";
    log_file_.flush();
  }
}

void AutonomousController::resetObservationState()
{
  const std::size_t cell_count =
    static_cast<std::size_t>(config_.grid_h) * static_cast<std::size_t>(config_.grid_w);
  obs_state_.people_belief_recent.assign(cell_count, 0.0f);
  obs_state_.people_belief_historic.assign(cell_count, 0.0f);
  obs_state_.people_count_density.assign(cell_count, 0.0f);
  obs_state_.people_count_memory_recent.assign(cell_count, 0.0f);
  obs_state_.people_count_memory_historic.assign(cell_count, 0.0f);
  obs_state_.coverage_map.assign(cell_count, 0.0f);
  obs_state_.persistent_coverage_map.assign(cell_count, 0.0f);
  obs_state_.own_coverage_map.assign(cell_count, 0.0f);
  obs_state_.shared_count_memory_staleness.assign(cell_count, 0.0f);
  obs_state_.shared_drone_map.assign(cell_count, 0.0f);
  obs_state_.own_ego_map.assign(cell_count, 0.0f);
  obs_state_.footprint_map.assign(cell_count, 0.0f);
  obs_state_.people_count_last_observed_step.assign(cell_count, -1);
  obs_state_.people_count_last_visible_step.assign(cell_count, -1);
  obs_state_.people_count_recent_last_visible_step.assign(cell_count, -1);
  controller_step_ = 0;
  prev_visible_count_ = 0;
  status_history_anchor_ = StatusHistoryAnchor{};
  actor_input_snapshot_count_ = 0;

  cmd_history_.clear();
  for (int i = 0; i < config_.cmd_history_len; ++i) {
    cmd_history_.push_back({0.0f, 0.0f, 0.0f});
  }

  status_history_.clear();
  for (int i = 0; i < config_.status_history_seconds; ++i) {
    status_history_.push_back({0.0f, 0.0f, 0.0f, 1.0f, 0.0f});
  }
}

std::vector<AutonomousController::Hotspot> AutonomousController::extractHotspots() const
{
  std::vector<Hotspot> hotspots;
  hotspots.reserve(static_cast<std::size_t>(hotspot_top_k_inferred_));
  if (hotspot_top_k_inferred_ <= 0) {
    return hotspots;
  }

  std::vector<float> working = obs_state_.people_count_memory_historic;
  const int radius = hotspot_suppression_radius_cells_;

  for (int hotspot_idx = 0; hotspot_idx < hotspot_top_k_inferred_; ++hotspot_idx) {
    const auto peak_it = std::max_element(working.begin(), working.end());
    if (peak_it == working.end() || *peak_it < static_cast<float>(config_.hotspot_min_density)) {
      break;
    }
    const std::size_t flat_idx = static_cast<std::size_t>(std::distance(working.begin(), peak_it));
    const int v = static_cast<int>(flat_idx / static_cast<std::size_t>(config_.grid_w));
    const int u = static_cast<int>(flat_idx % static_cast<std::size_t>(config_.grid_w));
    const auto [wx, wy] = gridToWorld(
      v,
      u,
      config_.x_min,
      config_.x_max,
      config_.y_min,
      config_.y_max,
      config_.grid_h,
      config_.grid_w);
    const int last_seen_step = obs_state_.people_count_last_observed_step[flat_idx];
    const float age = (last_seen_step < 0) ? 1.0f : static_cast<float>(clampDouble(
      (static_cast<double>(controller_step_) - static_cast<double>(last_seen_step)) /
      historic_half_life_steps_,
      0.0,
      1.0));
    hotspots.push_back(Hotspot{
      static_cast<float>(wx),
      static_cast<float>(wy),
      *peak_it,
      age,
    });

    const int v0 = std::max(0, v - radius);
    const int v1 = std::min(config_.grid_h, v + radius + 1);
    const int u0 = std::max(0, u - radius);
    const int u1 = std::min(config_.grid_w, u + radius + 1);
    for (int sv = v0; sv < v1; ++sv) {
      for (int su = u0; su < u1; ++su) {
        const int dv = sv - v;
        const int du = su - u;
        if (dv * dv + du * du <= radius * radius) {
          working[gridIndex(config_.grid_h, config_.grid_w, sv, su)] =
            -std::numeric_limits<float>::infinity();
        }
      }
    }
  }

  return hotspots;
}

void AutonomousController::updateStatusHistory(
  const drone_msgs::msg::SceneState & scene,
  std::size_t num_visible)
{
  if (config_.status_history_seconds <= 0) {
    return;
  }

  const double current_time_s = static_cast<double>(controller_step_) / config_.control_hz;
  if (!status_history_anchor_.valid) {
    status_history_anchor_.valid = true;
    status_history_anchor_.x = static_cast<float>(scene.drone_x);
    status_history_anchor_.y = static_cast<float>(scene.drone_y);
    status_history_anchor_.yaw = static_cast<float>(scene.drone_yaw);
    status_history_anchor_.time_s = current_time_s;
    return;
  }

  const double elapsed_s = current_time_s - status_history_anchor_.time_s;
  if (elapsed_s + 1e-9 < 1.0) {
    return;
  }

  const double max_disp = config_.max_horizontal_velocity * std::max(elapsed_s, 1e-6);
  const float dx_norm = max_disp <= 0.0 ? 0.0f : static_cast<float>(clampDouble(
      (scene.drone_x - static_cast<double>(status_history_anchor_.x)) / max_disp,
      -1.0,
      1.0));
  const float dy_norm = max_disp <= 0.0 ? 0.0f : static_cast<float>(clampDouble(
      (scene.drone_y - static_cast<double>(status_history_anchor_.y)) / max_disp,
      -1.0,
      1.0));
  const double delta_yaw = angleWrap(scene.drone_yaw - static_cast<double>(status_history_anchor_.yaw));
  const float visible_norm = static_cast<float>(num_visible) /
    static_cast<float>(config_.people_count_normalizer);

  status_history_.push_back({
    dx_norm,
    dy_norm,
    static_cast<float>(std::sin(delta_yaw)),
    static_cast<float>(std::cos(delta_yaw)),
    visible_norm,
  });
  while (static_cast<int>(status_history_.size()) > config_.status_history_seconds) {
    status_history_.pop_front();
  }

  status_history_anchor_.x = static_cast<float>(scene.drone_x);
  status_history_anchor_.y = static_cast<float>(scene.drone_y);
  status_history_anchor_.yaw = static_cast<float>(scene.drone_yaw);
  status_history_anchor_.time_s = current_time_s;
}

float AutonomousController::compressCountValue(float value) const
{
  const float scale = static_cast<float>(std::max(config_.count_map_compression_scale, 1e-6));
  return value <= 0.0f ? 0.0f : value / (value + scale);
}

float AutonomousController::visitedFraction() const
{
  if (obs_state_.persistent_coverage_map.empty()) {
    return 0.0f;
  }
  const auto visited = static_cast<double>(std::count_if(
    obs_state_.persistent_coverage_map.begin(),
    obs_state_.persistent_coverage_map.end(),
    [](float value) { return value > 0.0f; }));
  return static_cast<float>(visited / static_cast<double>(obs_state_.persistent_coverage_map.size()));
}

bool AutonomousController::actorHidesPersonFeatures(bool is_search_phase) const
{
  return hide_person_features_during_search_ && is_search_phase;
}

std::vector<std::string> AutonomousController::actorChannelNames() const
{
  std::vector<std::string> names;
  names.reserve(static_cast<std::size_t>(actor_grid_channels_));
  names.push_back(
    config_.local_people_map_mode == "instant" ?
    "Local instant spatial support" :
    "Local count density");
  if (include_recent_count_memory_channel_) {
    names.push_back("Local recent count memory");
  }
  if (exposes_spatial_memory_channels_) {
    names.push_back("Shared recent spatial support");
    names.push_back("Shared historic spatial support");
  }
  if (include_shared_count_density_channel_) {
    names.push_back("Shared count density");
  }
  names.push_back("Shared historic count memory");
  if (include_shared_count_memory_staleness_channel_) {
    names.push_back("Shared count-memory staleness");
  }
  if (include_persistent_coverage_channel_) {
    names.push_back("Shared permanent coverage");
  }
  if (include_instant_fov_channels_) {
    names.push_back("Own instant FOV footprint");
    names.push_back("Teammate instant FOV footprint");
  }
  names.push_back("Own FOV coverage");
  names.push_back("Teammate FOV coverage");
  names.push_back("Shared drone map");
  names.push_back("Own ego map");
  return names;
}

std::vector<std::string> AutonomousController::localFeatureNames() const
{
  std::vector<std::string> names = {
    "x",
    "y",
    "sin_yaw",
    "cos_yaw",
    "num_visible",
    "delta_visible",
    "visited_fraction",
    "search_phase_progress",
    "is_search_phase",
    "is_coverage_phase",
    "centroid_present",
    "centroid_forward_offset",
    "centroid_lateral_offset",
  };
  names.reserve(static_cast<std::size_t>(local_shape_[1]));
  for (int idx = 0; idx < hotspot_top_k_inferred_; ++idx) {
    const std::string prefix = "hotspot_" + std::to_string(idx) + "_";
    names.push_back(prefix + "valid");
    names.push_back(prefix + "forward_offset");
    names.push_back(prefix + "lateral_offset");
    names.push_back(prefix + "density");
    names.push_back(prefix + "age");
  }
  if (enable_agent_ids_) {
    names.push_back("agent_id");
  }
  for (int idx = 0; idx < teammate_slots_; ++idx) {
    const std::string prefix = "teammate_" + std::to_string(idx) + "_";
    names.push_back(prefix + "mask");
    names.push_back(prefix + "rel_x");
    names.push_back(prefix + "rel_y");
    names.push_back(prefix + "rel_z");
    names.push_back(prefix + "sin_yaw");
    names.push_back(prefix + "cos_yaw");
  }
  for (int idx = 0; idx < status_history_seconds_inferred_; ++idx) {
    const std::string prefix = "status_hist_" + std::to_string(idx) + "_";
    names.push_back(prefix + "delta_x");
    names.push_back(prefix + "delta_y");
    names.push_back(prefix + "sin_delta_yaw");
    names.push_back(prefix + "cos_delta_yaw");
    names.push_back(prefix + "num_visible");
  }
  for (int idx = 0; idx < config_.cmd_history_len; ++idx) {
    const std::string prefix = "cmd_hist_" + std::to_string(idx) + "_";
    names.push_back(prefix + "vx");
    names.push_back(prefix + "vy");
    names.push_back(prefix + "yaw_rate");
  }
  return names;
}

void AutonomousController::saveActorInputSnapshot(
  const builtin_interfaces::msg::Time & stamp,
  const drone_msgs::msg::SceneState & scene,
  const InferenceInputs & inputs,
  const std::vector<float> * action)
{
  if (!config_.save_actor_inputs || actor_input_snapshot_interval_steps_ == 0 ||
    actor_input_snapshot_dir_.empty())
  {
    return;
  }

  std::ostringstream filename;
  filename << actor_input_snapshot_dir_
           << "/snapshot_"
           << std::setw(6) << std::setfill('0') << actor_input_snapshot_count_
           << "_step_"
           << std::setw(8) << std::setfill('0') << controller_step_
           << ".json";
  std::ofstream out(filename.str(), std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open actor input snapshot file: %s", filename.str().c_str());
    return;
  }

  const auto channel_names = actorChannelNames();
  const auto local_feature_names = localFeatureNames();
  const std::size_t cell_count =
    static_cast<std::size_t>(config_.grid_h) * static_cast<std::size_t>(config_.grid_w);
  const bool is_search_phase = inputs.local_base.size() > 8 && inputs.local_base[8] > 0.5f;
  const bool person_features_hidden = actorHidesPersonFeatures(is_search_phase);
  double principal_x = scene.principal_x;
  double principal_y = scene.principal_y;
  if (!std::isfinite(principal_x) || !std::isfinite(principal_y)) {
    const double principal_forward =
      scene.drone_z * std::tan(0.5 * M_PI - config_.camera_tilt_deg * M_PI / 180.0);
    principal_x = scene.drone_x + principal_forward * std::cos(scene.drone_yaw);
    principal_y = scene.drone_y + principal_forward * std::sin(scene.drone_yaw);
  }
  const auto [min_forward, max_forward] =
    footprintForwardExtents(scene.drone_z, config_.camera_tilt_deg, config_.vertical_fov_deg);
  const auto [principal_forward, principal_lateral] =
    worldToDroneLocal(principal_x, principal_y, scene.drone_x, scene.drone_y, scene.drone_yaw);
  const double max_lateral_scale = std::max({
    lateralHalfWidthAtForwardDistance(min_forward, scene.drone_z, config_.horizontal_fov_deg),
    lateralHalfWidthAtForwardDistance(principal_forward, scene.drone_z, config_.horizontal_fov_deg),
    lateralHalfWidthAtForwardDistance(max_forward, scene.drone_z, config_.horizontal_fov_deg),
    1e-6
  });
  const auto hotspots = person_features_hidden ? std::vector<Hotspot>{} : extractHotspots();

  if (channel_names.size() != static_cast<std::size_t>(actor_grid_channels_)) {
    RCLCPP_ERROR(
      get_logger(),
      "Actor input snapshot skipped because channel name count (%zu) does not match channel count (%d)",
      channel_names.size(),
      actor_grid_channels_);
    return;
  }

  auto writeStringArray = [&out](const std::vector<std::string> & values) {
    out << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
        out << ',';
      }
      out << '"' << values[i] << '"';
    }
    out << ']';
  };

  auto writeFloatVector = [&out](const std::vector<float> & values) {
    out << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
        out << ',';
      }
      out << values[i];
    }
    out << ']';
  };

  out << std::setprecision(7);
  out << "{\n";
  out << "  \"snapshot_index\": " << actor_input_snapshot_count_ << ",\n";
  out << "  \"step\": " << controller_step_ << ",\n";
  out << "  \"timestamp\": {\"sec\": " << stamp.sec << ", \"nanosec\": " << stamp.nanosec << "},\n";
  out << "  \"schema_version\": 1,\n";
  out << "  \"grid_shape\": [" << actor_grid_channels_ << ',' << config_.grid_h << ',' << config_.grid_w << "],\n";
  out << "  \"local_base_dim\": " << inputs.local_base.size() << ",\n";
  out << "  \"move_mask_dim\": " << inputs.move_mask.size() << ",\n";
  out << "  \"channel_names\": ";
  writeStringArray(channel_names);
  out << ",\n";
  out << "  \"local_feature_names\": ";
  writeStringArray(local_feature_names);
  out << ",\n";
  out << "  \"observation_context\": {\n";
  out << "    \"camera_tilt_deg\": " << config_.camera_tilt_deg << ",\n";
  out << "    \"horizontal_fov_deg\": " << config_.horizontal_fov_deg << ",\n";
  out << "    \"vertical_fov_deg\": " << config_.vertical_fov_deg << ",\n";
  out << "    \"max_range\": " << config_.max_range << ",\n";
  out << "    \"search_phase_active\": " << (is_search_phase ? "true" : "false") << ",\n";
  out << "    \"search_phase_progress\": " <<
    (inputs.local_base.size() > 7 ? inputs.local_base[7] : 1.0f) << ",\n";
  out << "    \"person_features_hidden\": " << (person_features_hidden ? "true" : "false") << ",\n";
  out << "    \"camera_pose\": {\n";
  out << "      \"x\": " << scene.drone_x << ",\n";
  out << "      \"y\": " << scene.drone_y << ",\n";
  out << "      \"z\": " << scene.drone_z << ",\n";
  out << "      \"yaw\": " << scene.drone_yaw << ",\n";
  out << "      \"tilt_deg\": " << config_.camera_tilt_deg << "\n";
  out << "    },\n";
  out << "    \"footprint_local\": {\n";
  out << "      \"min_forward\": " << min_forward << ",\n";
  out << "      \"max_forward\": " << max_forward << ",\n";
  out << "      \"principal_forward\": " << principal_forward << ",\n";
  out << "      \"principal_lateral\": " << principal_lateral << ",\n";
  out << "      \"max_lateral_scale\": " << max_lateral_scale << "\n";
  out << "    }\n";
  out << "  },\n";
  out << "  \"scene\": {\n";
  out << "    \"odom_valid\": " << (scene.odom_valid ? "true" : "false") << ",\n";
  out << "    \"drone_x\": " << scene.drone_x << ",\n";
  out << "    \"drone_y\": " << scene.drone_y << ",\n";
  out << "    \"drone_z\": " << scene.drone_z << ",\n";
  out << "    \"drone_yaw\": " << scene.drone_yaw << ",\n";
  out << "    \"principal_x\": " << principal_x << ",\n";
  out << "    \"principal_y\": " << principal_y << ",\n";
  out << "    \"tracks\": [";
  for (std::size_t i = 0; i < scene.tracks.size(); ++i) {
    if (i > 0) {
      out << ',';
    }
    const auto & track = scene.tracks[i];
    const auto [forward, lateral] =
      worldToDroneLocal(track.x, track.y, scene.drone_x, scene.drone_y, scene.drone_yaw);
    const auto [track_v, track_u] = worldToGrid(
      track.x,
      track.y,
      config_.x_min,
      config_.x_max,
      config_.y_min,
      config_.y_max,
      config_.grid_h,
      config_.grid_w);
    out << "{"
        << "\"track_id\": " << track.track_id << ','
        << "\"x\": " << track.x << ','
        << "\"y\": " << track.y << ','
        << "\"vx\": " << track.vx << ','
        << "\"vy\": " << track.vy << ','
        << "\"forward\": " << forward << ','
        << "\"lateral\": " << lateral << ','
        << "\"grid_v\": " << track_v << ','
        << "\"grid_u\": " << track_u
        << "}";
  }
  out << "],\n";
  out << "    \"hotspots\": [";
  for (std::size_t i = 0; i < hotspots.size(); ++i) {
    if (i > 0) {
      out << ',';
    }
    const auto & hotspot = hotspots[i];
    const auto [forward, lateral] =
      worldToDroneLocal(hotspot.x, hotspot.y, scene.drone_x, scene.drone_y, scene.drone_yaw);
    out << "{"
        << "\"rank\": " << i << ','
        << "\"x\": " << hotspot.x << ','
        << "\"y\": " << hotspot.y << ','
        << "\"forward\": " << forward << ','
        << "\"lateral\": " << lateral << ','
        << "\"density\": " << hotspot.density << ','
        << "\"age\": " << hotspot.age
        << "}";
  }
  out << "]\n";
  out << "  },\n";
  out << "  \"actor_input\": {\n";
  out << "    \"visible_count\": " << inputs.visible_count << ",\n";
  out << "    \"centroid_present\": " << inputs.centroid_present << ",\n";
  out << "    \"centroid_forward_offset\": " << inputs.centroid_forward_offset << ",\n";
  out << "    \"centroid_lateral_offset\": " << inputs.centroid_lateral_offset << ",\n";
  out << "    \"local_base\": ";
  writeFloatVector(inputs.local_base);
  out << ",\n";
  out << "    \"move_mask\": ";
  writeFloatVector(inputs.move_mask);
  out << ",\n";
  out << "    \"local_base_named\": [";
  for (std::size_t i = 0; i < inputs.local_base.size(); ++i) {
    if (i > 0) {
      out << ',';
    }
    const std::string feature_name =
      i < local_feature_names.size() ? local_feature_names[i] : "feature_" + std::to_string(i);
    out << "{"
        << "\"index\": " << i << ','
        << "\"name\": \"" << feature_name << "\","
        << "\"value\": " << inputs.local_base[i]
        << "}";
  }
  out << "],\n";
  out << "    \"local_full\": [";
  for (std::size_t i = 0; i < inputs.local_base.size(); ++i) {
    if (i > 0) {
      out << ',';
    }
    out << inputs.local_base[i];
  }
  for (std::size_t i = 0; i < inputs.move_mask.size(); ++i) {
    out << ',' << inputs.move_mask[i];
  }
  out << "],\n";
  out << "    \"grid\": [\n";
  for (int c = 0; c < actor_grid_channels_; ++c) {
    out << "      {\n";
    out << "        \"channel_index\": " << c << ",\n";
    out << "        \"name\": \"" << channel_names[static_cast<std::size_t>(c)] << "\",\n";
    out << "        \"values\": [";
    for (int v = 0; v < config_.grid_h; ++v) {
      if (v > 0) {
        out << ',';
      }
      out << '[';
      for (int u = 0; u < config_.grid_w; ++u) {
        if (u > 0) {
          out << ',';
        }
        out << inputs.grid[static_cast<std::size_t>(c) * cell_count +
          static_cast<std::size_t>(v) * static_cast<std::size_t>(config_.grid_w) +
          static_cast<std::size_t>(u)];
      }
      out << ']';
    }
    out << "]\n";
    out << "      }";
    if (c + 1 < actor_grid_channels_) {
      out << ',';
    }
    out << '\n';
  }
  out << "    ]\n";
  out << "  }";
  if (action != nullptr && action->size() >= 3) {
    out << ",\n  \"predicted_action\": {"
        << "\"vx\": " << (*action)[0] << ','
        << "\"vy\": " << (*action)[1] << ','
        << "\"yaw_rate\": " << (*action)[2]
        << "}\n";
  } else {
    out << '\n';
  }
  out << "}\n";
  ++actor_input_snapshot_count_;
}

void AutonomousController::flushLogBuffer()
{
  std::lock_guard<std::mutex> lk(log_mtx_);
  if (!log_file_.is_open() || log_buffer_.empty()) {
    return;
  }
  for (const auto & row : log_buffer_) {
    log_file_ << row.line;
  }
  log_buffer_.clear();
  log_file_.flush();
}

AutonomousController::AutonomousController(const rclcpp::NodeOptions & options)
: Node("autonomous_controller", options)
{
  declare_parameter<std::string>("session_dir", "");
  config_ = loadConfig();
  initOnnx();
  resetObservationState();
  initLogging();

  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto reliable_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  scene_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  enable_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  flush_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions scene_opts;
  scene_opts.callback_group = scene_cb_group_;
  scene_sub_ = create_subscription<drone_msgs::msg::SceneState>(
    config_.scene_topic,
    sensor_qos,
    [this](drone_msgs::msg::SceneState::ConstSharedPtr msg) { onScene(msg); },
    scene_opts);

  rclcpp::SubscriptionOptions enable_opts;
  enable_opts.callback_group = enable_cb_group_;
  enable_sub_ = create_subscription<drone_msgs::msg::Toggle>(
    config_.enable_topic,
    reliable_qos,
    [this](drone_msgs::msg::Toggle::ConstSharedPtr msg) { onEnable(msg); },
    enable_opts);

  output_pub_ = create_publisher<drone_msgs::msg::AutonomousAction>(
    config_.output_topic,
    reliable_qos);

  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(std::round(1000.0 / config_.control_hz)));
  control_timer_ = create_wall_timer(
    period_ms,
    [this]() { onControlTimer(); },
    timer_cb_group_);

  flush_timer_ = create_wall_timer(
    std::chrono::seconds(3),
    [this]() { flushLogBuffer(); },
    flush_cb_group_);

  RCLCPP_INFO(
    get_logger(),
    "autonomous_controller ready — scene=%s enable=%s output=%s hz=%.1f model=%s session=%s",
    config_.scene_topic.c_str(),
    config_.enable_topic.c_str(),
    config_.output_topic.c_str(),
    config_.control_hz,
    config_.model_path.c_str(),
    session_dir_.c_str());
}

AutonomousController::~AutonomousController()
{
  flushLogBuffer();
  if (log_file_.is_open()) {
    log_file_.flush();
    log_file_.close();
  }
}

void AutonomousController::onScene(drone_msgs::msg::SceneState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(scene_mtx_);
  last_scene_ = *msg;
  has_scene_ = true;
}

void AutonomousController::onEnable(drone_msgs::msg::Toggle::ConstSharedPtr msg)
{
  const bool was_enabled = autonomous_enabled_.exchange(msg->state);
  if (was_enabled == msg->state) {
    return;
  }

  resetObservationState();

  drone_msgs::msg::SceneState scene_copy;
  const drone_msgs::msg::SceneState * scene_ptr = nullptr;
  {
    std::lock_guard<std::mutex> lk(scene_mtx_);
    if (has_scene_) {
      scene_copy = last_scene_;
      scene_ptr = &scene_copy;
    }
  }

  const auto stamp = nowAsBuiltinTime(*this);
  logEvent(stamp, msg->state ? "enabled" : "disabled", scene_ptr, nullptr, nullptr, 0.0);
  RCLCPP_INFO(get_logger(), "Autonomous control: %s", msg->state ? "ON" : "OFF");
}

AutonomousController::InferenceInputs AutonomousController::buildInferenceInputs(
  const drone_msgs::msg::SceneState & scene)
{
  const std::size_t cell_count =
    static_cast<std::size_t>(config_.grid_h) * static_cast<std::size_t>(config_.grid_w);

  InferenceInputs inputs;
  inputs.grid.assign(static_cast<std::size_t>(actor_grid_channels_) * cell_count, 0.0f);
  inputs.local_base.assign(static_cast<std::size_t>(local_shape_[1]), 0.0f);
  inputs.move_mask.assign(static_cast<std::size_t>(move_mask_shape_[1]), 0.0f);

  std::vector<float> instant_map(cell_count, 0.0f);
  std::vector<float> step_density(cell_count, 0.0f);
  std::vector<float> shared_count_density_obs(cell_count, 0.0f);
  std::vector<float> local_people_map_obs(cell_count, 0.0f);
  std::vector<float> local_recent_count_memory_obs(cell_count, 0.0f);
  std::vector<float> shared_historic_count_memory_obs(cell_count, 0.0f);
  std::vector<float> teammate_instant_coverage_map(cell_count, 0.0f);
  std::vector<float> teammate_coverage_map(cell_count, 0.0f);

  const double dt = 1.0 / config_.control_hz;
  const double decay_recent = std::pow(0.5, dt / config_.recent_half_life_seconds);
  const double decay_historic = std::pow(0.5, dt / config_.historic_half_life_seconds);
  const double decay_coverage = std::pow(0.5, dt / config_.coverage_half_life_seconds);

  if (exposes_spatial_memory_channels_) {
    for (float & value : obs_state_.people_belief_recent) {
      value *= static_cast<float>(decay_recent);
    }
    for (float & value : obs_state_.people_belief_historic) {
      value *= static_cast<float>(decay_historic);
    }
  }
  for (float & value : obs_state_.own_coverage_map) {
    value *= static_cast<float>(decay_coverage);
  }
  for (float & value : obs_state_.coverage_map) {
    value *= static_cast<float>(decay_coverage);
  }

  obs_state_.footprint_map.assign(cell_count, 0.0f);
  for (int v = 0; v < config_.grid_h; ++v) {
    for (int u = 0; u < config_.grid_w; ++u) {
      const auto [wx, wy] = gridToWorld(
        v,
        u,
        config_.x_min,
        config_.x_max,
        config_.y_min,
        config_.y_max,
        config_.grid_h,
        config_.grid_w);
      const auto [forward, lateral] =
        worldToDroneLocal(wx, wy, scene.drone_x, scene.drone_y, scene.drone_yaw);
      if (std::hypot(wx - scene.drone_x, wy - scene.drone_y) > config_.max_range) {
        continue;
      }
      if (pointInFootprintLocal(
          forward,
          lateral,
          scene.drone_z,
          config_.camera_tilt_deg,
          config_.horizontal_fov_deg,
          config_.vertical_fov_deg))
      {
        obs_state_.footprint_map[gridIndex(config_.grid_h, config_.grid_w, v, u)] = 1.0f;
      }
    }
  }

  for (std::size_t i = 0; i < cell_count; ++i) {
    obs_state_.own_coverage_map[i] = std::max(
      obs_state_.own_coverage_map[i],
      obs_state_.footprint_map[i]);
    obs_state_.coverage_map[i] = obs_state_.own_coverage_map[i];
    obs_state_.persistent_coverage_map[i] = std::max(
      obs_state_.persistent_coverage_map[i],
      obs_state_.footprint_map[i]);
  }

  inputs.visible_count = scene.tracks.size();
  const bool is_search_phase = search_phase_steps_ > 0 && controller_step_ <= search_phase_steps_;
  const double search_phase_progress = [&]() {
      if (search_phase_steps_ == 0) {
        return 1.0;
      }
      const auto completed_search_steps = std::min<std::uint64_t>(
        controller_step_ > 0 ? controller_step_ - 1 : 0,
        search_phase_steps_);
      return static_cast<double>(completed_search_steps) /
        static_cast<double>(search_phase_steps_);
    }();
  double centroid_x = 0.0;
  double centroid_y = 0.0;
  if (!scene.tracks.empty()) {
    std::vector<std::pair<double, double>> track_positions;
    track_positions.reserve(scene.tracks.size());
    for (const auto & track : scene.tracks) {
      track_positions.emplace_back(track.x, track.y);
      const auto [det_v, det_u] = worldToGrid(
        track.x,
        track.y,
        config_.x_min,
        config_.x_max,
        config_.y_min,
        config_.y_max,
        config_.grid_h,
        config_.grid_w);
      splatGaussianMax(instant_map, config_.grid_h, config_.grid_w, det_v, det_u, config_.blob_sigma);
      splatGaussianAdd(step_density, config_.grid_h, config_.grid_w, det_v, det_u, config_.blob_sigma);
      if (exposes_spatial_memory_channels_) {
        splatGaussianMax(
          obs_state_.people_belief_recent,
          config_.grid_h,
          config_.grid_w,
          det_v,
          det_u,
          config_.blob_sigma);
        splatGaussianMax(
          obs_state_.people_belief_historic,
          config_.grid_h,
          config_.grid_w,
          det_v,
          det_u,
          config_.blob_sigma);
      }
    }
    std::tie(centroid_x, centroid_y) = geometricMedian(track_positions);
  }

  obs_state_.people_count_density = step_density;
  const bool hide_person_features = actorHidesPersonFeatures(is_search_phase);

  for (std::size_t i = 0; i < cell_count; ++i) {
    if (obs_state_.people_count_density[i] >= static_cast<float>(config_.hotspot_min_density)) {
      obs_state_.people_count_last_observed_step[i] = static_cast<int32_t>(controller_step_);
    }
  }

  for (std::size_t i = 0; i < cell_count; ++i) {
    if (obs_state_.footprint_map[i] > 0.0f) {
      obs_state_.people_count_last_visible_step[i] = static_cast<int32_t>(controller_step_);
      obs_state_.people_count_recent_last_visible_step[i] = static_cast<int32_t>(controller_step_);
    }
  }

  for (std::size_t i = 0; i < cell_count; ++i) {
    const int last_visible = obs_state_.people_count_last_visible_step[i];
    if (
      last_visible >= 0 &&
      (static_cast<std::uint64_t>(controller_step_) - static_cast<std::uint64_t>(last_visible)) >
      count_memory_decay_grace_steps_)
    {
      obs_state_.people_count_memory_historic[i] *= static_cast<float>(decay_historic);
    }

    const int local_last_visible = obs_state_.people_count_recent_last_visible_step[i];
    if (
      local_last_visible >= 0 &&
      (static_cast<std::uint64_t>(controller_step_) - static_cast<std::uint64_t>(local_last_visible)) >
      count_memory_decay_grace_steps_)
    {
      obs_state_.people_count_memory_recent[i] *= static_cast<float>(decay_recent);
    }

    if (
      obs_state_.footprint_map[i] > 0.0f &&
      obs_state_.people_count_density[i] < static_cast<float>(config_.hotspot_min_density))
    {
      obs_state_.people_count_memory_historic[i] *= static_cast<float>(
        1.0 - config_.count_memory_historic_miss_penalty);
      obs_state_.people_count_memory_recent[i] *= static_cast<float>(
        1.0 - config_.count_memory_historic_miss_penalty);
    }
  }
  for (std::size_t i = 0; i < cell_count; ++i) {
    obs_state_.people_count_memory_recent[i] = std::max(
      obs_state_.people_count_memory_recent[i],
      obs_state_.people_count_density[i]);
    obs_state_.people_count_memory_historic[i] = std::max(
      obs_state_.people_count_memory_historic[i],
      obs_state_.people_count_density[i]);
    shared_count_density_obs[i] = compressCountValue(obs_state_.people_count_density[i]);
    local_recent_count_memory_obs[i] = compressCountValue(obs_state_.people_count_memory_recent[i]);
    shared_historic_count_memory_obs[i] = compressCountValue(obs_state_.people_count_memory_historic[i]);
    if (
      obs_state_.people_count_last_visible_step[i] < 0 ||
      obs_state_.people_count_memory_historic[i] <= 1e-6f)
    {
      obs_state_.shared_count_memory_staleness[i] = 0.0f;
    } else {
      const auto age_steps = static_cast<double>(
        static_cast<std::uint64_t>(controller_step_) -
        static_cast<std::uint64_t>(obs_state_.people_count_last_visible_step[i]));
      const auto post_grace_age = std::max(
        age_steps - static_cast<double>(count_memory_decay_grace_steps_),
        0.0);
      const auto staleness = 1.0 - std::pow(
        0.5,
        post_grace_age / std::max(historic_half_life_steps_, 1.0));
      obs_state_.shared_count_memory_staleness[i] = static_cast<float>(
        clampDouble(staleness, 0.0, 1.0));
    }
  }

  if (config_.local_people_map_mode == "instant") {
    local_people_map_obs = instant_map;
  } else {
    local_people_map_obs = shared_count_density_obs;
  }

  obs_state_.shared_drone_map.assign(cell_count, 0.0f);
  obs_state_.own_ego_map.assign(cell_count, 0.0f);
  const auto [drone_v, drone_u] = worldToGrid(
    scene.drone_x,
    scene.drone_y,
    config_.x_min,
    config_.x_max,
    config_.y_min,
    config_.y_max,
    config_.grid_h,
    config_.grid_w);
  splatGaussianMax(
    obs_state_.shared_drone_map,
    config_.grid_h,
    config_.grid_w,
    drone_v,
    drone_u,
    config_.ego_sigma);
  splatGaussianMax(
    obs_state_.own_ego_map,
    config_.grid_h,
    config_.grid_w,
    drone_v,
    drone_u,
    config_.ego_sigma);

  const int local_people_channel = 0;
  int next_channel = 1;
  const int local_recent_count_channel = include_recent_count_memory_channel_ ? next_channel++ : -1;
  const int shared_recent_spatial_channel = exposes_spatial_memory_channels_ ? next_channel++ : -1;
  const int shared_historic_spatial_channel = exposes_spatial_memory_channels_ ? next_channel++ : -1;
  const int shared_count_density_channel = include_shared_count_density_channel_ ? next_channel++ : -1;
  const int shared_historic_count_channel = next_channel++;
  const int shared_count_memory_staleness_channel =
    include_shared_count_memory_staleness_channel_ ? next_channel++ : -1;
  const int shared_persistent_coverage_channel = include_persistent_coverage_channel_ ? next_channel++ : -1;
  const int own_instant_coverage_channel = include_instant_fov_channels_ ? next_channel++ : -1;
  const int teammate_instant_coverage_channel = include_instant_fov_channels_ ? next_channel++ : -1;
  const int own_coverage_channel = next_channel++;
  const int teammate_coverage_channel = next_channel++;
  const int shared_drone_channel = next_channel++;
  const int own_ego_channel = next_channel++;
  if (next_channel != actor_grid_channels_) {
    throw std::runtime_error("Configured actor grid layout does not match ONNX channel count");
  }

  for (int v = 0; v < config_.grid_h; ++v) {
    for (int u = 0; u < config_.grid_w; ++u) {
      const auto idx = gridIndex(config_.grid_h, config_.grid_w, v, u);
      const float actor_local_people = hide_person_features ? 0.0f : local_people_map_obs[idx];
      const float actor_local_recent = hide_person_features ? 0.0f : local_recent_count_memory_obs[idx];
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, local_people_channel, v, u)] =
        actor_local_people;
      if (local_recent_count_channel >= 0) {
        inputs.grid[channelIndex(config_.grid_h, config_.grid_w, local_recent_count_channel, v, u)] =
          actor_local_recent;
      }
      if (exposes_spatial_memory_channels_) {
        inputs.grid[channelIndex(config_.grid_h, config_.grid_w, shared_recent_spatial_channel, v, u)] =
          hide_person_features ? 0.0f : obs_state_.people_belief_recent[idx];
        inputs.grid[channelIndex(config_.grid_h, config_.grid_w, shared_historic_spatial_channel, v, u)] =
          hide_person_features ? 0.0f : obs_state_.people_belief_historic[idx];
      }
      if (shared_count_density_channel >= 0) {
        inputs.grid[channelIndex(config_.grid_h, config_.grid_w, shared_count_density_channel, v, u)] =
          hide_person_features ? 0.0f : shared_count_density_obs[idx];
      }
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, shared_historic_count_channel, v, u)] =
        hide_person_features ? 0.0f : shared_historic_count_memory_obs[idx];
      if (shared_count_memory_staleness_channel >= 0) {
        inputs.grid[channelIndex(
          config_.grid_h, config_.grid_w, shared_count_memory_staleness_channel, v, u)] =
          hide_person_features ? 0.0f : obs_state_.shared_count_memory_staleness[idx];
      }
      if (shared_persistent_coverage_channel >= 0) {
        inputs.grid[channelIndex(config_.grid_h, config_.grid_w, shared_persistent_coverage_channel, v, u)] =
          obs_state_.persistent_coverage_map[idx];
      }
      if (own_instant_coverage_channel >= 0) {
        inputs.grid[channelIndex(config_.grid_h, config_.grid_w, own_instant_coverage_channel, v, u)] =
          obs_state_.footprint_map[idx];
        inputs.grid[channelIndex(config_.grid_h, config_.grid_w, teammate_instant_coverage_channel, v, u)] =
          teammate_instant_coverage_map[idx];
      }
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, own_coverage_channel, v, u)] =
        obs_state_.own_coverage_map[idx];
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, teammate_coverage_channel, v, u)] =
        teammate_coverage_map[idx];
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, shared_drone_channel, v, u)] =
        obs_state_.shared_drone_map[idx];
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, own_ego_channel, v, u)] =
        obs_state_.own_ego_map[idx];
    }
  }

  double principal_x = scene.principal_x;
  double principal_y = scene.principal_y;
  if (!std::isfinite(principal_x) || !std::isfinite(principal_y)) {
    const double principal_forward = scene.drone_z * std::tan(0.5 * M_PI - config_.camera_tilt_deg * M_PI / 180.0);
    principal_x = scene.drone_x + principal_forward * std::cos(scene.drone_yaw);
    principal_y = scene.drone_y + principal_forward * std::sin(scene.drone_yaw);
  }

  const auto [min_forward, max_forward] =
    footprintForwardExtents(scene.drone_z, config_.camera_tilt_deg, config_.vertical_fov_deg);
  const auto [principal_forward, unused_principal_lateral] =
    worldToDroneLocal(principal_x, principal_y, scene.drone_x, scene.drone_y, scene.drone_yaw);
  (void)unused_principal_lateral;

  updateStatusHistory(scene, scene.tracks.size());

  if (!scene.tracks.empty() && !hide_person_features) {
    inputs.centroid_present = 1.0f;
    const auto [centroid_forward, centroid_lateral] =
      worldToDroneLocal(centroid_x, centroid_y, scene.drone_x, scene.drone_y, scene.drone_yaw);
    const double forward_norm_scale = std::max({
      std::abs(min_forward - principal_forward),
      std::abs(max_forward - principal_forward),
      1e-6
    });
    const double max_lateral_scale = std::max({
      lateralHalfWidthAtForwardDistance(min_forward, scene.drone_z, config_.horizontal_fov_deg),
      lateralHalfWidthAtForwardDistance(principal_forward, scene.drone_z, config_.horizontal_fov_deg),
      lateralHalfWidthAtForwardDistance(max_forward, scene.drone_z, config_.horizontal_fov_deg),
      1e-6
    });
    inputs.centroid_forward_offset = static_cast<float>(clampDouble(
      (centroid_forward - principal_forward) / forward_norm_scale, -1.0, 1.0));
    inputs.centroid_lateral_offset = static_cast<float>(clampDouble(
      centroid_lateral / max_lateral_scale, -1.0, 1.0));
  }

  const float delta_visible = static_cast<float>(clampDouble(
      static_cast<double>(static_cast<int>(scene.tracks.size()) - prev_visible_count_) /
      config_.people_count_normalizer,
      -1.0,
      1.0));
  const float visible_count_norm = hide_person_features ? 0.0f :
    static_cast<float>(scene.tracks.size()) / static_cast<float>(config_.people_count_normalizer);

  inputs.local_base[0] = static_cast<float>(
    2.0 * (scene.drone_x - config_.x_min) / (config_.x_max - config_.x_min) - 1.0);
  inputs.local_base[1] = static_cast<float>(
    2.0 * (scene.drone_y - config_.y_min) / (config_.y_max - config_.y_min) - 1.0);
  inputs.local_base[2] = static_cast<float>(std::sin(scene.drone_yaw));
  inputs.local_base[3] = static_cast<float>(std::cos(scene.drone_yaw));
  inputs.local_base[4] = visible_count_norm;
  inputs.local_base[5] = hide_person_features ? 0.0f : delta_visible;
  inputs.local_base[6] = visitedFraction();
  inputs.local_base[7] = static_cast<float>(clampDouble(search_phase_progress, 0.0, 1.0));
  inputs.local_base[8] = is_search_phase ? 1.0f : 0.0f;
  inputs.local_base[9] = is_search_phase ? 0.0f : 1.0f;
  inputs.local_base[10] = inputs.centroid_present;
  inputs.local_base[11] = inputs.centroid_forward_offset;
  inputs.local_base[12] = inputs.centroid_lateral_offset;

  const double forward_norm_scale = std::max({
    std::abs(min_forward - principal_forward),
    std::abs(max_forward - principal_forward),
    1e-6
  });
  const double max_lateral_scale = std::max({
    lateralHalfWidthAtForwardDistance(min_forward, scene.drone_z, config_.horizontal_fov_deg),
    lateralHalfWidthAtForwardDistance(principal_forward, scene.drone_z, config_.horizontal_fov_deg),
    lateralHalfWidthAtForwardDistance(max_forward, scene.drone_z, config_.horizontal_fov_deg),
    1e-6
  });
  const auto hotspots = hide_person_features ? std::vector<Hotspot>{} : extractHotspots();
  std::size_t local_idx = 13;
  for (int hotspot_idx = 0; hotspot_idx < hotspot_top_k_inferred_; ++hotspot_idx) {
    if (hotspot_idx < static_cast<int>(hotspots.size())) {
      const auto & hotspot = hotspots[static_cast<std::size_t>(hotspot_idx)];
      const auto [hotspot_forward, hotspot_lateral] =
        worldToDroneLocal(hotspot.x, hotspot.y, scene.drone_x, scene.drone_y, scene.drone_yaw);
      inputs.local_base[local_idx++] = 1.0f;
      inputs.local_base[local_idx++] = static_cast<float>(clampDouble(
        (hotspot_forward - principal_forward) / forward_norm_scale,
        -1.0,
        1.0));
      inputs.local_base[local_idx++] = static_cast<float>(clampDouble(
        hotspot_lateral / max_lateral_scale,
        -1.0,
        1.0));
      inputs.local_base[local_idx++] = hotspot.density;
      inputs.local_base[local_idx++] = hotspot.age;
    } else {
      local_idx += 5;
    }
  }

  if (enable_agent_ids_) {
    if (config_.max_agents > 1) {
      inputs.local_base[local_idx++] = static_cast<float>(
        clampDouble(
          (2.0 * static_cast<double>(config_.drone_id) /
          static_cast<double>(config_.max_agents - 1)) - 1.0,
          -1.0,
          1.0));
    } else {
      inputs.local_base[local_idx++] = 0.0f;
    }
  }

  for (int teammate_idx = 0; teammate_idx < teammate_slots_; ++teammate_idx) {
    inputs.local_base[local_idx++] = 0.0f;
    inputs.local_base[local_idx++] = 0.0f;
    inputs.local_base[local_idx++] = 0.0f;
    inputs.local_base[local_idx++] = 0.0f;
    inputs.local_base[local_idx++] = 0.0f;
    inputs.local_base[local_idx++] = 0.0f;
  }

  if (status_history_seconds_inferred_ > 0) {
    for (const auto & hist : status_history_) {
      inputs.local_base[local_idx++] = hist[0];
      inputs.local_base[local_idx++] = hist[1];
      inputs.local_base[local_idx++] = hist[2];
      inputs.local_base[local_idx++] = hist[3];
      inputs.local_base[local_idx++] = hide_person_features ? 0.0f : hist[4];
    }
  }

  if (config_.cmd_history_len > 0) {
    for (const auto & cmd : cmd_history_) {
      inputs.local_base[local_idx++] = config_.max_horizontal_velocity > 0.0
        ? cmd[0] / static_cast<float>(config_.max_horizontal_velocity) : 0.0f;
      inputs.local_base[local_idx++] = config_.max_horizontal_velocity > 0.0
        ? cmd[1] / static_cast<float>(config_.max_horizontal_velocity) : 0.0f;
      inputs.local_base[local_idx++] = config_.max_yaw_rate > 0.0
        ? cmd[2] / static_cast<float>(config_.max_yaw_rate) : 0.0f;
    }
  }
  if (local_idx != inputs.local_base.size()) {
    throw std::runtime_error("Configured local_base layout does not match ONNX local dimension");
  }

  std::size_t mask_idx = 0;
  for (float vx : vx_bins_) {
    for (float vy : vy_bins_) {
      const double new_x = scene.drone_x + static_cast<double>(vx) * dt;
      const double new_y = scene.drone_y + static_cast<double>(vy) * dt;
      inputs.move_mask[mask_idx++] = (
        new_x >= config_.x_min &&
        new_x <= config_.x_max &&
        new_y >= config_.y_min &&
        new_y <= config_.y_max) ? 1.0f : 0.0f;
    }
  }
  if (std::none_of(inputs.move_mask.begin(), inputs.move_mask.end(), [](float value) { return value > 0.5f; })) {
    const std::size_t zero_vel_idx = (vx_bins_.size() / 2) * vy_bins_.size() + (vy_bins_.size() / 2);
    inputs.move_mask[zero_vel_idx] = 1.0f;
  }

  prev_visible_count_ = static_cast<int>(scene.tracks.size());
  return inputs;
}

std::vector<float> AutonomousController::runInference(
  const InferenceInputs & inputs,
  double & inference_ms)
{
  auto start = std::chrono::steady_clock::now();

  std::vector<Ort::Value> input_tensors;
  input_tensors.reserve(3);
  input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
    ort_memory_info_,
    const_cast<float *>(inputs.grid.data()),
    inputs.grid.size(),
    grid_shape_.data(),
    grid_shape_.size()));
  input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
    ort_memory_info_,
    const_cast<float *>(inputs.local_base.data()),
    inputs.local_base.size(),
    local_shape_.data(),
    local_shape_.size()));
  input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
    ort_memory_info_,
    const_cast<float *>(inputs.move_mask.data()),
    inputs.move_mask.size(),
    move_mask_shape_.data(),
    move_mask_shape_.size()));

  auto output_tensors = ort_session_->Run(
    Ort::RunOptions{nullptr},
    input_names_.data(),
    input_tensors.data(),
    input_tensors.size(),
    output_names_.data(),
    output_names_.size());

  auto end = std::chrono::steady_clock::now();
  inference_ms = std::chrono::duration<double, std::milli>(end - start).count();

  if (output_tensors.empty()) {
    throw std::runtime_error("ONNX inference returned no outputs");
  }

  const auto & output_tensor = output_tensors.front();
  const auto output_info = output_tensor.GetTensorTypeAndShapeInfo();
  const std::size_t element_count = output_info.GetElementCount();
  if (element_count < 3) {
    throw std::runtime_error("ONNX output action tensor is smaller than expected");
  }

  const float * action_ptr = output_tensor.GetTensorData<float>();
  return {action_ptr[0], action_ptr[1], action_ptr[2]};
}

void AutonomousController::publishCommand(
  const builtin_interfaces::msg::Time & stamp,
  float vx,
  float vy,
  float yaw_rate)
{
  drone_msgs::msg::AutonomousAction cmd;
  cmd.stamp = stamp;
  cmd.vx = vx;
  cmd.vy = vy;
  cmd.yaw_rate = yaw_rate;
  output_pub_->publish(cmd);
}

void AutonomousController::logEvent(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & event,
  const drone_msgs::msg::SceneState * scene,
  const InferenceInputs * inputs,
  const std::vector<float> * action,
  double inference_ms)
{
  const bool enabled = autonomous_enabled_.load();
  const bool odom_valid = scene ? scene->odom_valid : false;
  const std::size_t track_count = scene ? scene->tracks.size() : 0U;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  std::ostringstream oss;
  oss << stamp.sec << ','
      << stamp.nanosec << ','
      << controller_step_ << ','
      << event << ','
      << (enabled ? 1 : 0) << ','
      << (odom_valid ? 1 : 0) << ','
      << track_count << ','
      << (inputs ? inputs->visible_count : 0U) << ','
      << (inputs ? inputs->centroid_present : 0.0f) << ','
      << (inputs ? inputs->centroid_forward_offset : 0.0f) << ','
      << (inputs ? inputs->centroid_lateral_offset : 0.0f) << ','
      << inference_ms << ','
      << (action ? (*action)[0] : nan) << ','
      << (action ? (*action)[1] : nan) << ','
      << (action ? (*action)[2] : nan) << ','
      << (scene ? scene->drone_x : nan) << ','
      << (scene ? scene->drone_y : nan) << ','
      << (scene ? scene->drone_z : nan) << ','
      << (scene ? scene->drone_yaw : nan) << ','
      << (scene ? scene->principal_x : nan) << ','
      << (scene ? scene->principal_y : nan) << ','
      << (inputs ? maskToString(inputs->move_mask) : "") << '\n';

  std::lock_guard<std::mutex> lk(log_mtx_);
  log_buffer_.push_back({oss.str()});
  if (log_buffer_.size() >= kCsvFlushSize) {
    for (const auto & row : log_buffer_) {
      log_file_ << row.line;
    }
    log_buffer_.clear();
    log_file_.flush();
  }
}

void AutonomousController::onControlTimer()
{
  if (!autonomous_enabled_.load()) {
    return;
  }

  drone_msgs::msg::SceneState scene;
  {
    std::lock_guard<std::mutex> lk(scene_mtx_);
    if (!has_scene_) {
      const auto now_msg = nowAsBuiltinTime(*this);
      publishCommand(now_msg, 0.0f, 0.0f, 0.0f);
      logEvent(now_msg, "no_scene", nullptr, nullptr, nullptr, 0.0);
      return;
    }
    scene = last_scene_;
  }

  try {
    ++controller_step_;
    const bool should_save_actor_inputs =
      actor_input_snapshot_interval_steps_ > 0 &&
      (controller_step_ % actor_input_snapshot_interval_steps_ == 0);
    InferenceInputs inputs = buildInferenceInputs(scene);

    if (!scene.odom_valid) {
      if (should_save_actor_inputs) {
        saveActorInputSnapshot(scene.stamp, scene, inputs, nullptr);
      }
      publishCommand(scene.stamp, 0.0f, 0.0f, 0.0f);
      logEvent(scene.stamp, "odom_invalid", &scene, nullptr, nullptr, 0.0);
      return;
    }

    double inference_ms = 0.0;
    std::vector<float> action;
    try {
      action = runInference(inputs, inference_ms);
      if (should_save_actor_inputs) {
        saveActorInputSnapshot(scene.stamp, scene, inputs, &action);
      }
    } catch (...) {
      if (should_save_actor_inputs) {
        saveActorInputSnapshot(scene.stamp, scene, inputs, nullptr);
      }
      throw;
    }
    publishCommand(scene.stamp, action[0], action[1], action[2]);
    logEvent(scene.stamp, "inference", &scene, &inputs, &action, inference_ms);
    if (config_.cmd_history_len > 0) {
      cmd_history_.push_back({action[0], action[1], action[2]});
      cmd_history_.pop_front();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Autonomous ONNX inference failed: %s",
      e.what());
    publishCommand(scene.stamp, 0.0f, 0.0f, 0.0f);
    logEvent(scene.stamp, "inference_error", &scene, nullptr, nullptr, 0.0);
  }
}

}  // namespace drone_pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(drone_pipeline::AutonomousController)
