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

double clampDouble(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
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
  const double tilt = clampPitchFromDown(camera_tilt_deg * M_PI / 180.0);
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
  const double tilt = clampPitchFromDown(camera_tilt_deg * M_PI / 180.0);
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
  cfg.recent_half_life_seconds =
    yamlOr<double>(controller, "recent_half_life_seconds", cfg.recent_half_life_seconds);
  cfg.historic_half_life_seconds =
    yamlOr<double>(controller, "historic_half_life_seconds", cfg.historic_half_life_seconds);
  cfg.coverage_half_life_seconds =
    yamlOr<double>(controller, "coverage_half_life_seconds", cfg.coverage_half_life_seconds);
  cfg.recent_hit_gain = yamlOr<double>(controller, "recent_hit_gain", cfg.recent_hit_gain);
  cfg.recent_miss_penalty =
    yamlOr<double>(controller, "recent_miss_penalty", cfg.recent_miss_penalty);
  cfg.blob_sigma = yamlOr<double>(controller, "blob_sigma", cfg.blob_sigma);
  cfg.ego_sigma = yamlOr<double>(controller, "ego_sigma", cfg.ego_sigma);
  cfg.max_horizontal_velocity =
    yamlOr<double>(controller, "max_horizontal_velocity", cfg.max_horizontal_velocity);
  cfg.horizontal_bin_interval =
    yamlOr<double>(controller, "horizontal_bin_interval", cfg.horizontal_bin_interval);
  cfg.max_yaw_rate = yamlOr<double>(controller, "max_yaw_rate", cfg.max_yaw_rate);
  cfg.yaw_bin_interval =
    yamlOr<double>(controller, "yaw_bin_interval", cfg.yaw_bin_interval);
  cfg.max_agents = yamlOr<int>(controller, "max_agents", cfg.max_agents);

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

  config_.grid_h = static_cast<int>(grid_shape_[2]);
  config_.grid_w = static_cast<int>(grid_shape_[3]);

  input_names_ = {
    grid_input_name_.c_str(),
    local_input_name_.c_str(),
    move_mask_input_name_.c_str(),
  };
  output_names_ = {action_output_name_.c_str()};

  vx_bins_ = buildSymmetricBins(config_.max_horizontal_velocity, config_.horizontal_bin_interval);
  vy_bins_ = vx_bins_;
  yaw_rate_bins_ = buildSymmetricBins(config_.max_yaw_rate, config_.yaw_bin_interval);

  const std::size_t expected_local_dim = static_cast<std::size_t>(8 + 6 * (config_.max_agents - 1));
  const std::size_t expected_move_mask_dim = vx_bins_.size() * vy_bins_.size();
  if (static_cast<std::size_t>(local_shape_[1]) != expected_local_dim) {
    throw std::runtime_error("ONNX local_base dimension does not match max_agents=2 controller");
  }
  if (static_cast<std::size_t>(move_mask_shape_[1]) != expected_move_mask_dim) {
    throw std::runtime_error("ONNX move_mask dimension does not match configured action bins");
  }
}

void AutonomousController::initLogging()
{
  session_dir_ = resolveSessionDir(config_.logs_path);
  const std::string log_path = session_dir_ + "/autonomous_controller.csv";

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
  obs_state_.coverage_map.assign(cell_count, 0.0f);
  obs_state_.own_coverage_map.assign(cell_count, 0.0f);
  obs_state_.shared_drone_map.assign(cell_count, 0.0f);
  obs_state_.own_ego_map.assign(cell_count, 0.0f);
  obs_state_.footprint_map.assign(cell_count, 0.0f);
  controller_step_ = 0;
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
  inputs.grid.assign(8 * cell_count, 0.0f);
  inputs.local_base.assign(static_cast<std::size_t>(local_shape_[1]), 0.0f);
  inputs.move_mask.assign(static_cast<std::size_t>(move_mask_shape_[1]), 0.0f);

  std::vector<float> instant_map(cell_count, 0.0f);
  std::vector<float> step_density(cell_count, 0.0f);
  std::vector<float> recent_hits(cell_count, 0.0f);
  std::vector<float> team_detection_support(cell_count, 0.0f);
  std::vector<float> teammate_coverage_map(cell_count, 0.0f);

  const double dt = 1.0 / config_.control_hz;
  const double decay_recent = std::pow(0.5, dt / config_.recent_half_life_seconds);
  const double decay_historic = std::pow(0.5, dt / config_.historic_half_life_seconds);
  const double decay_coverage = std::pow(0.5, dt / config_.coverage_half_life_seconds);

  for (float & value : obs_state_.people_belief_recent) {
    value *= static_cast<float>(decay_recent);
  }
  for (float & value : obs_state_.people_belief_historic) {
    value *= static_cast<float>(decay_historic);
  }
  for (float & value : obs_state_.own_coverage_map) {
    value *= static_cast<float>(decay_coverage);
  }

  obs_state_.footprint_map.assign(cell_count, 0.0f);
  for (int v = 0; v < config_.grid_h; ++v) {
    for (int u = 0; u < config_.grid_w; ++u) {
      const double wx = config_.x_min +
        static_cast<double>(u) / static_cast<double>(config_.grid_w - 1) *
        (config_.x_max - config_.x_min);
      const double wy = config_.y_min +
        static_cast<double>(v) / static_cast<double>(config_.grid_h - 1) *
        (config_.y_max - config_.y_min);
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
  }

  inputs.visible_count = scene.tracks.size();
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
      splatGaussianMax(recent_hits, config_.grid_h, config_.grid_w, det_v, det_u, config_.blob_sigma);
      splatGaussianMax(
        team_detection_support,
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
    std::tie(centroid_x, centroid_y) = geometricMedian(track_positions);
  }

  if (config_.recent_miss_penalty > 0.0) {
    for (std::size_t i = 0; i < cell_count; ++i) {
      if (obs_state_.footprint_map[i] > 0.5f && team_detection_support[i] < 0.1f) {
        obs_state_.people_belief_recent[i] *= static_cast<float>(1.0 - config_.recent_miss_penalty);
      }
    }
  }
  if (config_.recent_hit_gain > 0.0) {
    for (std::size_t i = 0; i < cell_count; ++i) {
      obs_state_.people_belief_recent[i] = std::min(
        1.0f,
        obs_state_.people_belief_recent[i] +
        static_cast<float>(config_.recent_hit_gain) * recent_hits[i]);
    }
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

  for (int v = 0; v < config_.grid_h; ++v) {
    for (int u = 0; u < config_.grid_w; ++u) {
      const auto idx = gridIndex(config_.grid_h, config_.grid_w, v, u);
      const float count_density_obs = std::tanh(0.1f * step_density[idx]);
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, 0, v, u)] = instant_map[idx];
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, 1, v, u)] =
        obs_state_.people_belief_recent[idx];
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, 2, v, u)] =
        obs_state_.people_belief_historic[idx];
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, 3, v, u)] = count_density_obs;
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, 4, v, u)] =
        obs_state_.own_coverage_map[idx];
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, 5, v, u)] =
        teammate_coverage_map[idx];
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, 6, v, u)] =
        obs_state_.shared_drone_map[idx];
      inputs.grid[channelIndex(config_.grid_h, config_.grid_w, 7, v, u)] =
        obs_state_.own_ego_map[idx];
    }
  }

  double principal_x = scene.principal_x;
  double principal_y = scene.principal_y;
  if (!std::isfinite(principal_x) || !std::isfinite(principal_y)) {
    const double principal_forward = scene.drone_z * std::tan(config_.camera_tilt_deg * M_PI / 180.0);
    principal_x = scene.drone_x + principal_forward * std::cos(scene.drone_yaw);
    principal_y = scene.drone_y + principal_forward * std::sin(scene.drone_yaw);
  }

  const auto [min_forward, max_forward] =
    footprintForwardExtents(scene.drone_z, config_.camera_tilt_deg, config_.vertical_fov_deg);
  const auto [principal_forward, unused_principal_lateral] =
    worldToDroneLocal(principal_x, principal_y, scene.drone_x, scene.drone_y, scene.drone_yaw);
  (void)unused_principal_lateral;

  if (!scene.tracks.empty()) {
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

  inputs.local_base[0] = static_cast<float>(
    2.0 * (scene.drone_x - config_.x_min) / (config_.x_max - config_.x_min) - 1.0);
  inputs.local_base[1] = static_cast<float>(
    2.0 * (scene.drone_y - config_.y_min) / (config_.y_max - config_.y_min) - 1.0);
  inputs.local_base[2] = static_cast<float>(std::sin(scene.drone_yaw));
  inputs.local_base[3] = static_cast<float>(std::cos(scene.drone_yaw));
  inputs.local_base[4] = static_cast<float>(scene.tracks.size()) / 30.0f;
  inputs.local_base[5] = inputs.centroid_present;
  inputs.local_base[6] = inputs.centroid_forward_offset;
  inputs.local_base[7] = inputs.centroid_lateral_offset;

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

  if (!scene.odom_valid) {
    publishCommand(scene.stamp, 0.0f, 0.0f, 0.0f);
    logEvent(scene.stamp, "odom_invalid", &scene, nullptr, nullptr, 0.0);
    return;
  }

  try {
    ++controller_step_;
    InferenceInputs inputs = buildInferenceInputs(scene);
    double inference_ms = 0.0;
    const auto action = runInference(inputs, inference_ms);
    publishCommand(scene.stamp, action[0], action[1], action[2]);
    logEvent(scene.stamp, "inference", &scene, &inputs, &action, inference_ms);
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
