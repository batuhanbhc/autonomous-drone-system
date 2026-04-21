#include "drone_pipeline/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_pipeline
{

namespace
{

constexpr double kInvalidAssignmentCost = 1e6;
constexpr double kAcceptedAssignmentCost = 1e5;

std::array<double, 16> makeIdentity4()
{
  return {1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0};
}

std::array<double, 16> makeDiagonal4(double d0, double d1, double d2, double d3)
{
  return {d0,  0.0, 0.0, 0.0,
          0.0, d1,  0.0, 0.0,
          0.0, 0.0, d2,  0.0,
          0.0, 0.0, 0.0, d3};
}

std::array<double, 16> transpose4(const std::array<double, 16> & a)
{
  std::array<double, 16> out{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out[r * 4 + c] = a[c * 4 + r];
    }
  }
  return out;
}

std::array<double, 16> multiply4(
  const std::array<double, 16> & a,
  const std::array<double, 16> & b)
{
  std::array<double, 16> out{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) {
        sum += a[r * 4 + k] * b[k * 4 + c];
      }
      out[r * 4 + c] = sum;
    }
  }
  return out;
}

std::array<double, 16> add4(
  const std::array<double, 16> & a,
  const std::array<double, 16> & b)
{
  std::array<double, 16> out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = a[i] + b[i];
  }
  return out;
}

std::array<double, 4> invert2x2(const std::array<double, 4> & a, bool & ok)
{
  const double det = a[0] * a[3] - a[1] * a[2];
  if (std::abs(det) < 1e-9) {
    ok = false;
    return {};
  }

  ok = true;
  const double inv_det = 1.0 / det;
  return { a[3] * inv_det, -a[1] * inv_det,
          -a[2] * inv_det,  a[0] * inv_det };
}

std::vector<int> solveAssignment(const std::vector<std::vector<double>> & cost)
{
  const int rows = static_cast<int>(cost.size());
  if (rows == 0) {
    return {};
  }

  const int cols = static_cast<int>(cost.front().size());
  const int dim = std::max(rows, cols);
  const double inf = std::numeric_limits<double>::infinity();

  std::vector<std::vector<double>> padded(
    dim + 1, std::vector<double>(dim + 1, kInvalidAssignmentCost));
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      padded[r + 1][c + 1] = cost[r][c];
    }
  }

  std::vector<double> u(dim + 1, 0.0);
  std::vector<double> v(dim + 1, 0.0);
  std::vector<int> p(dim + 1, 0);
  std::vector<int> way(dim + 1, 0);

  for (int i = 1; i <= dim; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(dim + 1, inf);
    std::vector<char> used(dim + 1, false);

    do {
      used[j0] = true;
      const int i0 = p[j0];
      double delta = inf;
      int j1 = 0;

      for (int j = 1; j <= dim; ++j) {
        if (used[j]) {
          continue;
        }

        const double cur = padded[i0][j] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }

      for (int j = 0; j <= dim; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> assignment(rows, -1);
  for (int j = 1; j <= dim; ++j) {
    if (p[j] >= 1 && p[j] <= rows && j <= cols) {
      assignment[p[j] - 1] = j - 1;
    }
  }
  return assignment;
}

}  // namespace

MultiObjectTracker::MultiObjectTracker(const TrackerConfig & config)
: config_(config)
{
}

void MultiObjectTracker::reset()
{
  tracks_.clear();
  next_id_ = 1;
}

std::vector<TrackEstimate> MultiObjectTracker::step(
  const std::vector<DetectionMeasurement> & detections,
  double dt,
  double noise_scale)
{
  for (auto & track : tracks_) {
    predictTrack(track, dt, noise_scale);
  }

  const double duplicate_track_min_dist =
    (noise_scale > 1.5) ? config_.new_track_min_dist_noisy_m :
    config_.new_track_min_dist_calm_m;

  if (tracks_.empty()) {
    for (const auto & detection : detections) {
      spawnTrack(detection, 0.0);
    }
    return confirmedTracks();
  }

  if (detections.empty()) {
    for (auto & track : tracks_) {
      ++track.missed;
    }
    pruneTracks();
    return confirmedTracks();
  }

  std::vector<std::vector<double>> cost(
    tracks_.size(), std::vector<double>(detections.size(), kInvalidAssignmentCost));

  for (std::size_t track_idx = 0; track_idx < tracks_.size(); ++track_idx) {
    for (std::size_t det_idx = 0; det_idx < detections.size(); ++det_idx) {
      const double md = mahalanobisDistance(tracks_[track_idx], detections[det_idx]);
      const double ed = euclideanDistance(tracks_[track_idx], detections[det_idx]);
      if (md <= config_.mahalanobis_gate && ed <= config_.max_association_distance_m) {
        cost[track_idx][det_idx] = md;
      }
    }
  }

  const std::vector<int> assignment = solveAssignment(cost);
  std::vector<bool> matched_tracks(tracks_.size(), false);
  std::vector<bool> matched_detections(detections.size(), false);

  for (std::size_t track_idx = 0; track_idx < assignment.size(); ++track_idx) {
    const int det_idx = assignment[track_idx];
    if (det_idx < 0) {
      continue;
    }
    if (cost[track_idx][det_idx] >= kAcceptedAssignmentCost) {
      continue;
    }

    updateTrack(tracks_[track_idx], detections[det_idx]);
    matched_tracks[track_idx] = true;
    matched_detections[det_idx] = true;
  }

  for (std::size_t track_idx = 0; track_idx < tracks_.size(); ++track_idx) {
    if (!matched_tracks[track_idx]) {
      ++tracks_[track_idx].missed;
    }
  }

  for (std::size_t det_idx = 0; det_idx < detections.size(); ++det_idx) {
    if (!matched_detections[det_idx]) {
      spawnTrack(detections[det_idx], duplicate_track_min_dist);
    }
  }

  pruneTracks();
  return confirmedTracks();
}

void MultiObjectTracker::predictTrack(Track & track, double dt, double noise_scale) const
{
  track.state[0] += dt * track.state[2];
  track.state[1] += dt * track.state[3];

  const std::array<double, 16> f = {
    1.0, 0.0, dt,  0.0,
    0.0, 1.0, 0.0, dt,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0
  };

  const double q_pos = config_.process_noise_std_pos * config_.process_noise_std_pos * noise_scale;
  const double q_vel = config_.process_noise_std_vel * config_.process_noise_std_vel * noise_scale;
  const std::array<double, 16> q = makeDiagonal4(
    q_pos * dt * dt,
    q_pos * dt * dt,
    q_vel * dt,
    q_vel * dt);

  const std::array<double, 16> fp = multiply4(f, track.covariance);
  track.covariance = add4(multiply4(fp, transpose4(f)), q);
  ++track.age;
}

double MultiObjectTracker::mahalanobisDistance(
  const Track & track,
  const DetectionMeasurement & detection) const
{
  const double y0 = detection.world_xy[0] - track.state[0];
  const double y1 = detection.world_xy[1] - track.state[1];

  std::array<double, 4> s = {
    track.covariance[0] + detection.meas_cov[0],
    track.covariance[1] + detection.meas_cov[1],
    track.covariance[4] + detection.meas_cov[2],
    track.covariance[5] + detection.meas_cov[3]
  };

  bool ok = false;
  const std::array<double, 4> s_inv = invert2x2(s, ok);
  if (!ok) {
    return kInvalidAssignmentCost;
  }

  const double sy0 = s_inv[0] * y0 + s_inv[1] * y1;
  const double sy1 = s_inv[2] * y0 + s_inv[3] * y1;
  return y0 * sy0 + y1 * sy1;
}

double MultiObjectTracker::euclideanDistance(
  const Track & track,
  const DetectionMeasurement & detection) const
{
  const double dx = track.state[0] - detection.world_xy[0];
  const double dy = track.state[1] - detection.world_xy[1];
  return std::sqrt(dx * dx + dy * dy);
}

void MultiObjectTracker::updateTrack(Track & track, const DetectionMeasurement & detection) const
{
  const double y0 = detection.world_xy[0] - track.state[0];
  const double y1 = detection.world_xy[1] - track.state[1];

  std::array<double, 4> s = {
    track.covariance[0] + detection.meas_cov[0],
    track.covariance[1] + detection.meas_cov[1],
    track.covariance[4] + detection.meas_cov[2],
    track.covariance[5] + detection.meas_cov[3]
  };

  bool ok = false;
  const std::array<double, 4> s_inv = invert2x2(s, ok);
  if (!ok) {
    return;
  }

  std::array<double, 8> k{};
  for (int row = 0; row < 4; ++row) {
    const double p0 = track.covariance[row * 4 + 0];
    const double p1 = track.covariance[row * 4 + 1];
    k[row * 2 + 0] = p0 * s_inv[0] + p1 * s_inv[2];
    k[row * 2 + 1] = p0 * s_inv[1] + p1 * s_inv[3];
  }

  track.state[0] += k[0] * y0 + k[1] * y1;
  track.state[1] += k[2] * y0 + k[3] * y1;
  track.state[2] += k[4] * y0 + k[5] * y1;
  track.state[3] += k[6] * y0 + k[7] * y1;

  std::array<double, 16> kh{};
  for (int row = 0; row < 4; ++row) {
    kh[row * 4 + 0] = k[row * 2 + 0];
    kh[row * 4 + 1] = k[row * 2 + 1];
  }

  std::array<double, 16> a = makeIdentity4();
  for (std::size_t i = 0; i < a.size(); ++i) {
    a[i] -= kh[i];
  }

  std::array<double, 8> kr{};
  for (int row = 0; row < 4; ++row) {
    kr[row * 2 + 0] =
      k[row * 2 + 0] * detection.meas_cov[0] + k[row * 2 + 1] * detection.meas_cov[2];
    kr[row * 2 + 1] =
      k[row * 2 + 0] * detection.meas_cov[1] + k[row * 2 + 1] * detection.meas_cov[3];
  }

  std::array<double, 16> krkt{};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      krkt[row * 4 + col] =
        kr[row * 2 + 0] * k[col * 2 + 0] +
        kr[row * 2 + 1] * k[col * 2 + 1];
    }
  }

  track.covariance = add4(
    multiply4(multiply4(a, track.covariance), transpose4(a)),
    krkt);

  ++track.hits;
  track.missed = 0;
  if (track.hits >= config_.min_hits_to_confirm) {
    track.confirmed = true;
  }
  track.image_box = detection.image_box;
  track.score = detection.score;
}

void MultiObjectTracker::spawnTrack(const DetectionMeasurement & detection, double min_dist)
{
  if (nearConfirmedTrack(detection.world_xy, min_dist)) {
    return;
  }

  Track track;
  track.track_id = next_id_++;
  track.state = {detection.world_xy[0], detection.world_xy[1], 0.0, 0.0};
  track.covariance = makeDiagonal4(4.0, 4.0, 25.0, 25.0);
  track.age = 1;
  track.hits = 1;
  track.confirmed = (config_.min_hits_to_confirm <= 1);
  track.image_box = detection.image_box;
  track.score = detection.score;
  tracks_.push_back(track);
}

bool MultiObjectTracker::nearConfirmedTrack(
  const std::array<double, 2> & world_xy,
  double min_dist) const
{
  for (const auto & track : tracks_) {
    if (!track.confirmed) {
      continue;
    }

    const double dx = track.state[0] - world_xy[0];
    const double dy = track.state[1] - world_xy[1];
    if (std::sqrt(dx * dx + dy * dy) < min_dist) {
      return true;
    }
  }
  return false;
}

void MultiObjectTracker::pruneTracks()
{
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this](const Track & track) {
        return track.missed > config_.max_missed_frames;
      }),
    tracks_.end());
}

std::vector<TrackEstimate> MultiObjectTracker::confirmedTracks() const
{
  std::vector<TrackEstimate> out;
  out.reserve(tracks_.size());
  for (const auto & track : tracks_) {
    if (!track.confirmed) {
      continue;
    }

    TrackEstimate estimate;
    estimate.track_id = track.track_id;
    estimate.state = track.state;
    estimate.image_box = track.image_box;
    estimate.score = track.score;
    estimate.age = track.age;
    estimate.hits = track.hits;
    estimate.missed = track.missed;
    estimate.confirmed = track.confirmed;
    out.push_back(estimate);
  }

  std::sort(
    out.begin(), out.end(),
    [](const TrackEstimate & a, const TrackEstimate & b) {
      return a.track_id < b.track_id;
    });
  return out;
}

}  // namespace drone_pipeline
