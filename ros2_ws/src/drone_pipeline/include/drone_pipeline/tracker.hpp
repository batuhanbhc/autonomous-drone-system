#pragma once

#include <array>
#include <vector>

namespace drone_pipeline
{

struct TrackerConfig
{
  double max_association_distance_m{12.0};
  double mahalanobis_gate{25.0};
  int    max_missed_frames{20};
  int    min_hits_to_confirm{2};
  double process_noise_std_pos{1.0};
  double process_noise_std_vel{3.0};
  double base_meas_noise_m{0.75};
  double pixel_std_x{2.0};
  double pixel_std_y{4.0};
  double height_ratio_normal_threshold{0.8};
  double height_ratio_suspicious_threshold{0.2};
  double height_ratio_r_inflation{9.0};
  double trusted_height_ema_alpha{0.5};
  double min_valid_box_height_px{10.0};
  double angular_vel_low_deg_s{5.0};
  double angular_vel_high_deg_s{30.0};
  double angular_vel_max_scale{12.0};
  double new_track_min_dist_calm_m{0.5};
  double new_track_min_dist_noisy_m{2.0};
};

struct DetectionMeasurement
{
  std::array<double, 2> world_xy{};
  std::array<double, 4> meas_cov{};
  std::array<float, 4>  image_box{};
  double                score{0.0};
};

struct TrackEstimate
{
  int                   track_id{0};
  std::array<double, 4> state{};
  std::array<double, 2> raw_world_xy{};
  std::array<float, 4>  image_box{};
  double                score{0.0};
  int                   age{0};
  int                   hits{0};
  int                   missed{0};
  bool                  confirmed{false};
  bool                  matched_in_frame{false};
};

class MultiObjectTracker
{
public:
  explicit MultiObjectTracker(const TrackerConfig & config = TrackerConfig());

  void reset();
  std::vector<TrackEstimate> step(
    const std::vector<DetectionMeasurement> & detections,
    double dt,
    double noise_scale);

private:
  struct Track
  {
    int                   track_id{0};
    std::array<double, 4> state{};
    std::array<double, 16> covariance{};
    int                   age{0};
    int                   hits{0};
    int                   missed{0};
    bool                  confirmed{false};
    std::array<double, 2> raw_world_xy{};
    bool                  matched_in_frame{false};
    std::array<float, 4>  image_box{};
    double                score{0.0};
    double                trusted_height_px{0.0};
    bool                  has_trusted_height{false};
  };

  TrackerConfig config_;
  std::vector<Track> tracks_;
  int next_id_{1};

  void predictTrack(Track & track, double dt) const;
  double mahalanobisDistance(
    const Track & track,
    const DetectionMeasurement & detection,
    const std::array<double, 4> & measurement_covariance) const;
  double euclideanDistance(
    const Track & track,
    const DetectionMeasurement & detection) const;
  void updateTrack(
    Track & track,
    const DetectionMeasurement & detection,
    const std::array<double, 4> & measurement_covariance,
    bool update_trusted_height) const;
  void spawnTrack(const DetectionMeasurement & detection, double min_dist);
  bool nearConfirmedTrack(const std::array<double, 2> & world_xy, double min_dist) const;
  void pruneTracks();
  std::vector<TrackEstimate> confirmedTracks() const;
};

}  // namespace drone_pipeline
