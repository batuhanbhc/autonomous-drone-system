import math
import random
from collections import deque
from typing import List, Dict, Tuple, Optional

import numpy as np

from sim.camera_geometry import (
    footprint_corners_world,
    footprint_forward_extents,
    lateral_half_width_at_forward_distance,
    point_in_footprint_local,
    point_in_footprint_world,
    principal_point_world,
    world_to_drone_local,
)


def geometric_median(points, max_iter=50, tol=1e-6):
    if len(points) == 1:
        return points[0][0], points[0][1]
    px = sum(p[0] for p in points) / len(points)
    py = sum(p[1] for p in points) / len(points)
    for _ in range(max_iter):
        num_x = num_y = denom = 0.0
        for x, y in points:
            dist = math.hypot(px - x, py - y)
            if dist < 1e-10:
                continue
            w = 1.0 / dist
            num_x += w * x
            num_y += w * y
            denom += w
        if denom < 1e-10:
            break
        new_px, new_py = num_x / denom, num_y / denom
        if math.hypot(new_px - px, new_py - py) < tol:
            px, py = new_px, new_py
            break
        px, py = new_px, new_py
    return px, py


class ObservationBuilder:
    """
    Builds an actor grid observation each step.

      Channel 0 — people spatial support INSTANTANEOUS, LOCAL TO THIS DRONE
                  Only detections made by THIS drone on the current step.
                  No decay. Bright = this drone sees spatial person support here now.

      Shared count density
                  Summed Gaussian blobs from all detections on the current step,
                  then tanh-compressed. Bright and wide means more people now.

      Shared recent count memory
                  A bounded recent density estimate updated from currently visible
                  cells only. This map is still computed, but is not exposed in
                  the current default actor/critic layout.

      Shared historic count memory
                  A slower decayed peak-memory over observed count density.
                  Preserves "the strongest crowd density seen here recently."

      Own FOV coverage
                  (simulation-time half-life: coverage_half_life_seconds)
                  Bright = this drone's camera was aimed here recently.

      Teammate-only FOV coverage
                  Bright = at least one other drone's camera was aimed here recently.

      Shared drone map
                  Gaussian blobs at all active drone positions.
                  Gives the CNN a shared spatial view of teammate placement.

      Ego map for this drone
                  Gaussian blob at this drone's own position.
                  Lets the actor distinguish "me" from the shared drone layer.

    The current/default actor layout uses 7 channels total:
      [instant spatial support] + [count density, historic count]
      + [own coverage, teammate coverage, shared drone, ego].

    The recent count memory map is still computed internally, but its exposure
    to the actor/critic is currently disabled in the default layout.

    Legacy layouts are still supported for old checkpoints:
      8 channels: current count-only layout + recent count memory
      9 channels: spatial-memory layout + count density + historic count memory
     10 channels: spatial-memory layout + count density + recent count + historic count

    Legacy 9/10-channel layouts insert shared recent/historic spatial-support
    maps ahead of the count-density channels. New runs should use the 7-channel
    default unless you intentionally want the recent count memory channel back.

    The centralised critic receives a ((shared_people_channels + 3) + 3 * num_agents, H, W)
    grid built by build_global_state() in state_utils.py:
      Channel 0   — shared instantaneous spatial support union
      Channel ... — shared people channels from the actor layout
      Channel ... — shared FOV coverage union
      Channel ... — shared drone map
      Channel ... — per-drone instantaneous maps (one per agent, critic-only)
      Channel ...  — per-drone own coverage maps (one per agent, critic-only)
      Channel ...  — ego map per drone (one per agent, critic-only)

    All decay constants are derived from simulation-time half-lives.
    Formula: decay_per_step = 0.5 ^ (dt / half_life_seconds)
    """

    def __init__(
        self,
        x_min: float = -15.0,
        x_max: float = 15.0,
        y_min: float = -15.0,
        y_max: float = 15.0,
        grid_h: int = 60,
        grid_w: int = 60,
        max_range: float = 15.0,
        horizontal_fov_deg: float = 55.8,
        vertical_fov_deg: float = 43.3,
        detection_prob: float = 1.0,
        position_noise_std: float = 0.1,
        camera_tilt_deg: float = 45.0,
        num_drones: int = 2,
        sim_hz: float = 4.0,
        blob_sigma: float = 1.0,
        ego_sigma: float = 2.5,
        recent_half_life_seconds: float = 10.0,    # team short-memory people spatial support
        historic_half_life_seconds: float = 30.0,  # slower people spatial support memory
        coverage_half_life_seconds: float = 10.0,  # FOV visited-recently window
        recent_hit_gain: float = 0.6,
        recent_miss_penalty: float = 0.25,
        z_min: float = 0.3,
        z_max: float = 8.0,
        cmd_history_len: int = 4,
        hotspot_top_k: int = 3,
        hotspot_min_density: float = 0.25,
        hotspot_suppression_radius_scale: float = 4.0,
        hotspot_suppression_radius_min_cells: int = 2,
        people_count_normalizer: float = 30.0,
        count_density_gain: float = 0.35,
        count_memory_recent_alpha: float = 0.7,
        count_memory_historic_miss_penalty: float = 0.1,
        max_horizontal_velocity: float = 1.0,
        max_yaw_rate: float = 0.7,
        actor_grid_channels: int = 7,
    ):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.num_drones = num_drones

        self.max_range = max_range
        self.horizontal_fov_deg = horizontal_fov_deg
        self.vertical_fov_deg = vertical_fov_deg

        self.detection_prob = detection_prob
        self.position_noise_std = position_noise_std

        self.camera_tilt_deg = camera_tilt_deg

        self.blob_sigma = blob_sigma
        self.ego_sigma  = ego_sigma
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.people_count_normalizer = float(people_count_normalizer)
        self.count_density_gain = float(count_density_gain)
        self.count_memory_recent_alpha = float(count_memory_recent_alpha)
        self.count_memory_historic_miss_penalty = float(count_memory_historic_miss_penalty)
        self.recent_hit_gain = float(recent_hit_gain)
        self.recent_miss_penalty = float(recent_miss_penalty)
        self.cmd_history_len = int(cmd_history_len)
        self.hotspot_top_k = max(0, int(hotspot_top_k))
        self.hotspot_min_density = float(hotspot_min_density)
        self.hotspot_suppression_radius_scale = float(hotspot_suppression_radius_scale)
        self.hotspot_suppression_radius_min_cells = int(hotspot_suppression_radius_min_cells)
        self.hotspot_suppression_radius_cells = max(
            self.hotspot_suppression_radius_min_cells,
            int(round(self.hotspot_suppression_radius_scale * self.blob_sigma)),
        )
        self._max_hvel = float(max_horizontal_velocity)
        self._max_yaw_rate = float(max_yaw_rate)
        self.actor_grid_channels = int(actor_grid_channels)
        if self.actor_grid_channels not in {7, 8, 9, 10}:
            raise ValueError(
                "actor_grid_channels must be 7, 8, 9, or 10: "
                "7=count-only without recent count memory, "
                "8=count-only with recent count memory, "
                "9=spatial+count without recent count memory, "
                "10=spatial+count with recent count memory; "
                f"got {self.actor_grid_channels}"
            )
        self.include_recent_count_memory_channel = self.actor_grid_channels in {8, 10}
        self.exposes_spatial_memory_channels = self.actor_grid_channels in {9, 10}
        self.shared_people_channels = (
            2
            + (2 if self.exposes_spatial_memory_channels else 0)
            + (1 if self.include_recent_count_memory_channel else 0)
        )
        self.actor_own_coverage_channel = 1 + self.shared_people_channels
        self.actor_teammate_coverage_channel = self.actor_own_coverage_channel + 1
        self.actor_shared_drone_channel = self.actor_teammate_coverage_channel + 1
        self.actor_own_ego_channel = self.actor_shared_drone_channel + 1
        self.actor_channel_names = [
            "Instant spatial support",
        ]
        if self.exposes_spatial_memory_channels:
            self.actor_channel_names += [
                "Shared recent spatial support",
                "Shared historic spatial support",
            ]
        self.actor_channel_names += [
            "Shared count density",
            "Shared historic count memory",
        ]
        if self.include_recent_count_memory_channel:
            self.actor_channel_names.insert(
                len(self.actor_channel_names) - 1,
                "Shared recent count memory",
            )
        self.actor_channel_names += [
            "Own FOV coverage",
            "Teammate FOV coverage",
            "Shared drone map",
            "Own ego map",
        ]
        self.critic_shared_channel_names = [
            "Shared instant spatial union",
        ]
        if self.exposes_spatial_memory_channels:
            self.critic_shared_channel_names += [
                "Shared recent spatial support",
                "Shared historic spatial support",
            ]
        self.critic_shared_channel_names += [
            "Shared count density",
            "Shared historic count memory",
        ]
        if self.include_recent_count_memory_channel:
            self.critic_shared_channel_names.insert(
                len(self.critic_shared_channel_names) - 1,
                "Shared recent count memory",
            )
        self.critic_shared_channel_names += [
            "Shared FOV coverage",
            "Shared drone map",
        ]
        self._cmd_histories: List[deque] = [
            deque([(0.0, 0.0, 0.0)] * self.cmd_history_len, maxlen=self.cmd_history_len)
            for _ in range(self.num_drones)
        ]
        if self.recent_hit_gain < 0.0:
            raise ValueError(
                f"recent_hit_gain must be >= 0, got {self.recent_hit_gain}"
            )
        if not (0.0 <= self.recent_miss_penalty <= 1.0):
            raise ValueError(
                "recent_miss_penalty must be within [0, 1], got "
                f"{self.recent_miss_penalty}"
            )
        if self.z_max <= self.z_min:
            raise ValueError(
                f"z_max must be > z_min, got z_min={self.z_min}, z_max={self.z_max}"
            )
        if self.people_count_normalizer <= 0.0:
            raise ValueError(
                "people_count_normalizer must be > 0, got "
                f"{self.people_count_normalizer}"
            )
        if self.count_density_gain <= 0.0:
            raise ValueError(
                f"count_density_gain must be > 0, got {self.count_density_gain}"
            )
        if not (0.0 <= self.count_memory_recent_alpha <= 1.0):
            raise ValueError(
                "count_memory_recent_alpha must be within [0, 1], got "
                f"{self.count_memory_recent_alpha}"
            )
        if not (0.0 <= self.count_memory_historic_miss_penalty <= 1.0):
            raise ValueError(
                "count_memory_historic_miss_penalty must be within [0, 1], got "
                f"{self.count_memory_historic_miss_penalty}"
            )
        if self.hotspot_min_density < 0.0:
            raise ValueError(
                f"hotspot_min_density must be >= 0, got {self.hotspot_min_density}"
            )
        if self.hotspot_suppression_radius_scale <= 0.0:
            raise ValueError(
                "hotspot_suppression_radius_scale must be > 0, got "
                f"{self.hotspot_suppression_radius_scale}"
            )
        if self.hotspot_suppression_radius_min_cells < 0:
            raise ValueError(
                "hotspot_suppression_radius_min_cells must be >= 0, got "
                f"{self.hotspot_suppression_radius_min_cells}"
            )

        dt = 1.0 / sim_hz
        self.historic_half_life_steps = max(1.0, historic_half_life_seconds * sim_hz)

        # All three decay constants are derived from simulation-time half-lives.
        # decay_per_step = 0.5 ^ (dt / half_life_seconds)
        self.decay_recent   = 0.5 ** (dt / recent_half_life_seconds)
        self.decay_historic = 0.5 ** (dt / historic_half_life_seconds)
        self.decay_coverage = 0.5 ** (dt / coverage_half_life_seconds)

        # Sanity-check print so you can verify at startup
        steps_per_episode = 300
        print(
            f"[ObsBuilder] sim_hz={sim_hz}  dt={dt:.3f}s\n"
            f"  Ch1 recent   half-life: {recent_half_life_seconds}s "
            f"= {recent_half_life_seconds * sim_hz:.1f} steps  "
            f"| decay/step={self.decay_recent:.4f} "
            f"| value after episode: {self.decay_recent**steps_per_episode:.4f}\n"
            f"  Ch2 historic half-life: {historic_half_life_seconds}s "
            f"= {historic_half_life_seconds * sim_hz:.1f} steps  "
            f"| decay/step={self.decay_historic:.4f} "
            f"| value after episode: {self.decay_historic**steps_per_episode:.4f}\n"
            f"  Coverage half-life: {coverage_half_life_seconds}s "
            f"= {coverage_half_life_seconds * sim_hz:.1f} steps  "
            f"| decay/step={self.decay_coverage:.4f} "
            f"| value after episode: {self.decay_coverage**steps_per_episode:.4f}\n"
            f"  Legacy Ch1 params unused by spatial-support maps: "
            f"recent_hit_gain={self.recent_hit_gain:.3f}  "
            f"recent_miss_penalty={self.recent_miss_penalty:.3f}\n"
            f"  Exposed spatial-memory channels: {'enabled' if self.exposes_spatial_memory_channels else 'disabled'}\n"
            f"  Recent count memory channel: {'enabled' if self.include_recent_count_memory_channel else 'disabled (computed internally only)'}\n"
            f"  Historic count memory channel: enabled\n"
            f"  Critic shared channels: {', '.join(self.critic_shared_channel_names)}"
        )

        # Ch0: instantaneous spatial support — per-drone current-step observations only.
        self.people_detect_instant = np.zeros((num_drones, grid_h, grid_w), dtype=np.float32)
        # Ch1: fast-decay recent spatial support — shared across all drones.
        self.people_belief_recent = np.zeros((grid_h, grid_w), dtype=np.float32)
        # Ch2: slower-decay historic spatial support — shared across all drones.
        self.people_belief_historic = np.zeros((grid_h, grid_w), dtype=np.float32)
        # Ch3: count density — rebuilt every step from current detections.
        self.people_count_density = np.zeros((grid_h, grid_w), dtype=np.float32)
        self.people_count_memory_recent = np.zeros((grid_h, grid_w), dtype=np.float32)
        self.people_count_memory_historic = np.zeros((grid_h, grid_w), dtype=np.float32)
        self.people_count_last_observed_step = np.full((grid_h, grid_w), -1, dtype=np.int32)
        # Ch4: FOV coverage — "camera was aimed here recently" — shared across all drones.
        self.coverage_map = np.zeros((grid_h, grid_w), dtype=np.float32)
        self.coverage_maps_per_drone = np.zeros((num_drones, grid_h, grid_w), dtype=np.float32)
        self.footprint_maps_snapshot = np.zeros((num_drones, grid_h, grid_w), dtype=np.float32)
        self.instant_maps_snapshot = self.people_detect_instant.copy()

        # Precompute meshgrid for fast Gaussian splatting
        vs = np.arange(grid_h, dtype=np.float32)
        us = np.arange(grid_w, dtype=np.float32)
        self._uu, self._vv = np.meshgrid(us, vs)

        self.prev_drone_positions: Dict[int, Tuple[float, float]] = {}

    def reset(self):
        self.people_detect_instant.fill(0.0)
        self.people_belief_recent.fill(0.0)
        self.people_belief_historic.fill(0.0)
        self.people_count_density.fill(0.0)
        self.people_count_memory_recent.fill(0.0)
        self.people_count_memory_historic.fill(0.0)
        self.people_count_last_observed_step.fill(-1)
        self.coverage_map.fill(0.0)
        self.coverage_maps_per_drone.fill(0.0)
        self.footprint_maps_snapshot.fill(0.0)
        self.instant_maps_snapshot = self.people_detect_instant.copy()
        self.prev_drone_positions = {}
        for h in self._cmd_histories:
            h.clear()
            h.extend([(0.0, 0.0, 0.0)] * self.cmd_history_len)

    def update_cmd_history(self, drone_idx: int, vx: float, vy: float, yaw_rate: float) -> None:
        if self.cmd_history_len == 0:
            return
        self._cmd_histories[drone_idx].append((vx, vy, yaw_rate))

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        x_clamped = min(max(x, self.x_min), self.x_max)
        y_clamped = min(max(y, self.y_min), self.y_max)
        u = int((x_clamped - self.x_min) / (self.x_max - self.x_min) * (self.grid_w - 1))
        v = int((y_clamped - self.y_min) / (self.y_max - self.y_min) * (self.grid_h - 1))
        return v, u

    def grid_to_world(self, v: int, u: int) -> Tuple[float, float]:
        x = self.x_min + (float(u) / max(self.grid_w - 1, 1)) * (self.x_max - self.x_min)
        y = self.y_min + (float(v) / max(self.grid_h - 1, 1)) * (self.y_max - self.y_min)
        return x, y

    def _extract_hotspots(self, current_step: int) -> list[tuple[float, float, float, float]]:
        if self.hotspot_top_k <= 0:
            return []
        score_map = np.asarray(self.people_count_memory_historic, dtype=np.float32)
        working = score_map.copy()
        hotspots: list[tuple[float, float, float, float]] = []
        radius = self.hotspot_suppression_radius_cells
        threshold = self.hotspot_min_density
        age_scale = self.historic_half_life_steps

        for _ in range(self.hotspot_top_k):
            flat_idx = int(np.argmax(working))
            peak_value = float(working.flat[flat_idx])
            if peak_value < threshold:
                break
            v, u = np.unravel_index(flat_idx, working.shape)
            wx, wy = self.grid_to_world(v, u)
            last_seen_step = int(self.people_count_last_observed_step[v, u])
            if last_seen_step < 0:
                age_norm = 1.0
            else:
                age_norm = min(max((current_step - last_seen_step) / age_scale, 0.0), 1.0)
            hotspots.append((wx, wy, peak_value, age_norm))

            v0 = max(0, v - radius)
            v1 = min(self.grid_h, v + radius + 1)
            u0 = max(0, u - radius)
            u1 = min(self.grid_w, u + radius + 1)
            vv, uu = np.ogrid[v0:v1, u0:u1]
            suppress_mask = (vv - v) ** 2 + (uu - u) ** 2 <= radius * radius
            working[v0:v1, u0:u1][suppress_mask] = -np.inf

        return hotspots

    def _angle_wrap(self, angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _splat_gaussian(self, grid: np.ndarray, v: int, u: int, sigma: float):
        """
        Paint a Gaussian blob centred on (v, u) onto grid in-place.
        Uses np.maximum so overlapping detections don't cancel each other.
        """
        blob = self._gaussian_blob(v, u, sigma)
        np.maximum(grid, blob, out=grid)

    def _gaussian_blob(self, v: int, u: int, sigma: float) -> np.ndarray:
        return np.exp(-((self._vv - v) ** 2 + (self._uu - u) ** 2) / (2.0 * sigma ** 2))

    def _make_ego_map(self, drone_state: Dict) -> np.ndarray:
        """
        Return a (H, W) float32 array with a Gaussian blob at the drone's
        current grid position. Each drone gets its own ego map.
        """
        x, y, _ = drone_state["position"]
        v, u = self.world_to_grid(x, y)
        ego = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        self._splat_gaussian(ego, v, u, self.ego_sigma)
        return ego

    def _make_shared_drone_map(self, drone_states: List[Dict]) -> np.ndarray:
        shared = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        for drone_state in drone_states:
            x, y, _ = drone_state["position"]
            v, u = self.world_to_grid(x, y)
            self._splat_gaussian(shared, v, u, self.ego_sigma)
        return shared

    def get_noisy_detection(self, person_position):
        if random.random() > self.detection_prob:
            return None
        px, py, pz = person_position
        noisy_x = px + random.gauss(0.0, self.position_noise_std)
        noisy_y = py + random.gauss(0.0, self.position_noise_std)
        return (noisy_x, noisy_y, pz)

    def get_visible_people(self, drone_state, people_positions):
        visible_ids = []
        detections = []
        for idx, pos in enumerate(people_positions):
            if self.is_in_camera_footprint(drone_state, pos):
                noisy_det = self.get_noisy_detection(pos)
                if noisy_det is not None:
                    visible_ids.append(idx)
                    detections.append(noisy_det)
        return visible_ids, detections

    def _get_footprint_cells(self, drone_state: Dict) -> List[Tuple[int, int]]:
        """
        Return all grid cells (v, u) whose centers fall inside the camera
        footprint trapezoid of this drone.
        """
        x, y, z = drone_state["position"]
        yaw = drone_state["yaw"]

        corners = footprint_corners_world(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            camera_tilt_deg=self.camera_tilt_deg,
            horizontal_fov_deg=self.horizontal_fov_deg,
            vertical_fov_deg=self.vertical_fov_deg,
        )
        xs = [corner[0] for corner in corners]
        ys = [corner[1] for corner in corners]
        cell_w = (self.x_max - self.x_min) / (self.grid_w - 1)
        cell_h = (self.y_max - self.y_min) / (self.grid_h - 1)

        x0 = max(self.x_min, min(xs) - cell_w)
        x1 = min(self.x_max, max(xs) + cell_w)
        y0 = max(self.y_min, min(ys) - cell_h)
        y1 = min(self.y_max, max(ys) + cell_h)

        u_min = max(0, int((x0 - self.x_min) / (self.x_max - self.x_min) * (self.grid_w - 1)))
        u_max = min(self.grid_w - 1, int((x1 - self.x_min) / (self.x_max - self.x_min) * (self.grid_w - 1)) + 1)
        v_min = max(0, int((y0 - self.y_min) / (self.y_max - self.y_min) * (self.grid_h - 1)))
        v_max = min(self.grid_h - 1, int((y1 - self.y_min) / (self.y_max - self.y_min) * (self.grid_h - 1)) + 1)

        cells = []
        for v in range(v_min, v_max + 1):
            for u in range(u_min, u_max + 1):
                wx = self.x_min + u / (self.grid_w - 1) * (self.x_max - self.x_min)
                wy = self.y_min + v / (self.grid_h - 1) * (self.y_max - self.y_min)
                forward, lateral = world_to_drone_local(wx, wy, x, y, yaw)
                if point_in_footprint_local(
                    forward=forward,
                    lateral=lateral,
                    z=z,
                    camera_tilt_deg=self.camera_tilt_deg,
                    horizontal_fov_deg=self.horizontal_fov_deg,
                    vertical_fov_deg=self.vertical_fov_deg,
                ):
                    cells.append((v, u))

        return cells

    def _build_current_footprint_maps(self, drone_states: List[Dict]) -> np.ndarray:
        current_footprint_maps = np.zeros(
            (len(drone_states), self.grid_h, self.grid_w),
            dtype=np.float32,
        )
        for drone_idx, drone_state in enumerate(drone_states):
            for (v, u) in self._get_footprint_cells(drone_state):
                current_footprint_maps[drone_idx, v, u] = 1.0
        return current_footprint_maps

    def update_coverage_map(self, current_footprint_maps: np.ndarray):
        """
        Decay per-drone and shared coverage maps at the configured half-life,
        then mark all cells currently inside each active drone's camera FOV
        footprint.
        """
        active_drones = int(current_footprint_maps.shape[0])
        self.coverage_map *= self.decay_coverage
        self.coverage_maps_per_drone *= self.decay_coverage
        if active_drones < self.num_drones:
            self.coverage_maps_per_drone[active_drones:].fill(0.0)
        if active_drones == 0:
            self.coverage_map.fill(0.0)
            return

        np.maximum(
            self.coverage_maps_per_drone[:active_drones],
            current_footprint_maps,
            out=self.coverage_maps_per_drone[:active_drones],
        )
        self.coverage_map[...] = self.coverage_maps_per_drone[:active_drones].max(axis=0)

    def build_people_maps(
        self,
        detections_per_drone,
        current_footprint_maps: np.ndarray,
        current_step: int,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Update and return belief maps.

        Ch0 (instantaneous): per-drone — each drone's spatial support from only the
        current step. Shape: (num_drones, H, W).

        Legacy Ch1-Ch2 (recent/historic shared spatial support) are updated only
        when the 10-channel legacy layout is requested.

        Count-density channels are:
          - current-step count density
          - recent count memory (always computed, optionally exposed)
          - historic count memory

        Historic count memory also gets weak negative evidence: if a cell is
        currently inside any drone's footprint but its current observed density
        stays below hotspot_min_density, that cell is additionally attenuated
        before the peak update.
        """
        self.people_detect_instant.fill(0.0)
        if self.exposes_spatial_memory_channels:
            self.people_belief_recent *= self.decay_recent
            self.people_belief_historic *= self.decay_historic
        step_density = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)

        for drone_idx, detections in enumerate(detections_per_drone):
            for det in detections:
                x, y, _ = det
                v, u = self.world_to_grid(x, y)
                blob = self._gaussian_blob(v, u, self.blob_sigma)
                # Ch0: only THIS drone's current-step spatial support
                np.maximum(self.people_detect_instant[drone_idx], blob, out=self.people_detect_instant[drone_idx])
                if self.exposes_spatial_memory_channels:
                    np.maximum(self.people_belief_recent, blob, out=self.people_belief_recent)
                    np.maximum(self.people_belief_historic, blob, out=self.people_belief_historic)
                step_density += blob

        self.people_count_density[...] = step_density
        count_density_obs = np.tanh(
            self.count_density_gain * self.people_count_density
        ).astype(np.float32)
        observed_density_mask = count_density_obs >= self.hotspot_min_density
        self.people_count_last_observed_step[observed_density_mask] = int(current_step)
        self.people_count_memory_recent *= self.decay_recent
        self.people_count_memory_historic *= self.decay_historic

        if current_footprint_maps.size > 0:
            team_visible_mask = current_footprint_maps.max(axis=0) > 0.0
            alpha = self.count_memory_recent_alpha
            self.people_count_memory_recent[team_visible_mask] = (
                (1.0 - alpha) * self.people_count_memory_recent[team_visible_mask]
                + alpha * count_density_obs[team_visible_mask]
            )
            historic_miss_mask = team_visible_mask & (~observed_density_mask)
            self.people_count_memory_historic[historic_miss_mask] *= (
                1.0 - self.count_memory_historic_miss_penalty
            )

        np.maximum(
            self.people_count_memory_historic,
            count_density_obs,
            out=self.people_count_memory_historic,
        )
        recent_count_memory_obs = self.people_count_memory_recent.copy()
        historic_count_memory_obs = self.people_count_memory_historic.copy()

        return (
            self.people_detect_instant.copy(),
            self.people_belief_recent.copy(),
            self.people_belief_historic.copy(),
            count_density_obs,
            recent_count_memory_obs,
            historic_count_memory_obs,
        )

    def build_shared_grid(
        self,
        drone_states,
        detections_per_drone,
        current_step: int,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns:
          shared_people: (shared_people_channels, H, W) — channels shared
                         across all drones.
          instant_maps: (num_drones, H, W) — per-drone instantaneous spatial support
          own_coverage_maps: (num_drones, H, W) — per-drone recent FOV coverage
          teammate_coverage_maps: (num_drones, H, W) — union of all other
                                 drones' recent FOV coverage
        """
        current_footprint_maps = self._build_current_footprint_maps(drone_states)
        self.footprint_maps_snapshot.fill(0.0)
        active_drones = len(drone_states)
        if active_drones > 0:
            self.footprint_maps_snapshot[:active_drones] = current_footprint_maps
        self.update_coverage_map(current_footprint_maps)
        (
            instant_maps,
            recent,
            historic,
            count_density,
            recent_count_memory,
            historic_count_memory,
        ) = self.build_people_maps(
            detections_per_drone,
            current_footprint_maps,
            current_step,
        )
        own_coverage_maps = self.coverage_maps_per_drone[:active_drones].copy()
        teammate_coverage_maps = np.zeros_like(own_coverage_maps)
        for drone_idx in range(active_drones):
            teammate_indices = [idx for idx in range(active_drones) if idx != drone_idx]
            if teammate_indices:
                teammate_coverage_maps[drone_idx] = self.coverage_maps_per_drone[
                    teammate_indices
                ].max(axis=0)
        shared_people = []
        if self.exposes_spatial_memory_channels:
            shared_people.extend([recent, historic])
        shared_people.append(count_density)
        if self.include_recent_count_memory_channel:
            shared_people.append(recent_count_memory)
        # Keep this computation available for ablations, but do not expose it
        # in the default actor/critic layout.
        # shared_people.append(recent_count_memory)
        shared_people.append(historic_count_memory)
        return np.stack(shared_people, axis=0), instant_maps, own_coverage_maps, teammate_coverage_maps

    def build_local_vector(
        self,
        drone_state: Dict,
        drone_id: int,
        num_visible: int = 0,
        search_phase_progress: float = 1.0,
        is_search_phase: float = 0.0,
        is_coverage_phase: float = 1.0,
        detections: Optional[List[Tuple[float, float, float]]] = None,
        other_drone_states: Optional[List[Dict]] = None,
        hotspots: Optional[list[tuple[float, float, float, float]]] = None,
        cmd_history: Optional[deque] = None,
    ) -> np.ndarray:
        """
        Local feature vector for one drone.

        Components:
          [0] x           — normalised to [-1, 1]
          [1] y           — normalised to [-1, 1]
          [2] sin(yaw)
          [3] cos(yaw)
          [4] num_visible — normalised (count / 30)
          [5] search_phase_progress — completed fraction in [0, 1]
          [6] is_search_phase
          [7] is_coverage_phase
          [8] detection_centroid_present
          [9] centroid_forward_offset_from_principal — normalised to [-1, 1]
          [10] centroid_lateral_offset_from_principal — normalised to [-1, 1]
          [11...] hotspot slots: [valid, rel_dx_world, rel_dy_world, density, age] * hotspot_top_k
          [...] teammate blocks: [mask, rel_x, rel_y, rel_z, sin(yaw), cos(yaw)]
          [...] command history (oldest→newest): [vx, vy, yaw_rate] * cmd_history_len

        Total: 11 + 5*hotspot_top_k + 6*(num_drones-1) + 3*cmd_history_len values.
        """
        x, y, z = drone_state["position"]
        yaw = drone_state["yaw"]

        x_min, x_max = self.x_min, self.x_max
        y_min, y_max = self.y_min, self.y_max
        area_diag = math.hypot(x_max - x_min, y_max - y_min)

        centroid_present = 0.0
        centroid_forward_offset = 0.0
        centroid_lateral_offset = 0.0
        if detections and float(is_search_phase) <= 0.0:
            centroid_present = 1.0
            centroid_x, centroid_y = geometric_median([(det[0], det[1]) for det in detections])
            principal_x, principal_y = principal_point_world(
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                camera_tilt_deg=self.camera_tilt_deg,
            )
            min_forward, max_forward = footprint_forward_extents(
                z=z,
                camera_tilt_deg=self.camera_tilt_deg,
                vertical_fov_deg=self.vertical_fov_deg,
            )
            principal_forward, _ = world_to_drone_local(
                principal_x,
                principal_y,
                x,
                y,
                yaw,
            )
            centroid_forward, centroid_lateral = world_to_drone_local(
                centroid_x,
                centroid_y,
                x,
                y,
                yaw,
            )
            forward_norm_scale = max(
                abs(min_forward - principal_forward),
                abs(max_forward - principal_forward),
                1e-6,
            )
            max_lateral_scale = max(
                lateral_half_width_at_forward_distance(
                    forward=min_forward,
                    z=z,
                    horizontal_fov_deg=self.horizontal_fov_deg,
                ),
                lateral_half_width_at_forward_distance(
                    forward=principal_forward,
                    z=z,
                    horizontal_fov_deg=self.horizontal_fov_deg,
                ),
                lateral_half_width_at_forward_distance(
                    forward=max_forward,
                    z=z,
                    horizontal_fov_deg=self.horizontal_fov_deg,
                ),
                1e-6,
            )
            centroid_forward_offset = max(
                -1.0,
                min(1.0, (centroid_forward - principal_forward) / forward_norm_scale),
            )
            centroid_lateral_offset = max(
                -1.0,
                min(1.0, centroid_lateral / max_lateral_scale),
            )

        vec = [
            2.0 * (x - x_min) / (x_max - x_min) - 1.0,
            2.0 * (y - y_min) / (y_max - y_min) - 1.0,
            math.sin(yaw),
            math.cos(yaw),
            float(num_visible) / self.people_count_normalizer,
            min(max(float(search_phase_progress), 0.0), 1.0),
            float(is_search_phase),
            float(is_coverage_phase),
            centroid_present,
            centroid_forward_offset,
            centroid_lateral_offset,
        ]

        hotspot_items = [] if float(is_search_phase) > 0.0 else (hotspots or [])
        for idx in range(self.hotspot_top_k):
            if idx < len(hotspot_items):
                hx, hy, hdensity, hage = hotspot_items[idx]
                rel_dx = hx - x
                rel_dy = hy - y
                vec += [
                    1.0,
                    max(-1.0, min(1.0, rel_dx / area_diag)),
                    max(-1.0, min(1.0, rel_dy / area_diag)),
                    float(hdensity),
                    float(hage),
                ]
            else:
                vec += [0.0, 0.0, 0.0, 0.0, 0.0]

        other_states = other_drone_states or []
        for other in other_states:
            ox, oy, oz = other["position"]
            oyaw = other["yaw"]
            vec += [
                1.0,
                (ox - x) / area_diag,
                (oy - y) / area_diag,
                (oz - z) / (self.z_max - self.z_min),
                math.sin(oyaw),
                math.cos(oyaw),
            ]

        missing_teammates = max(0, self.num_drones - 1 - len(other_states))
        for _ in range(missing_teammates):
            vec += [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if cmd_history is not None and self.cmd_history_len > 0:
            for vx_c, vy_c, yr_c in cmd_history:
                vec += [
                    vx_c / self._max_hvel if self._max_hvel > 0 else 0.0,
                    vy_c / self._max_hvel if self._max_hvel > 0 else 0.0,
                    yr_c / self._max_yaw_rate if self._max_yaw_rate > 0 else 0.0,
                ]

        return np.array(vec, dtype=np.float32)

    def build_observations(
        self,
        drone_states,
        people_positions,
        phase_context: Optional[Dict[str, float]] = None,
        current_step: int = 0,
    ):
        visible_ids_per_drone = []
        detections_per_drone = []
        phase_context = phase_context or {
            "search_phase_progress": 1.0,
            "is_search_phase": 0.0,
            "is_coverage_phase": 1.0,
        }

        for drone_state in drone_states:
            visible_ids, detections = self.get_visible_people(drone_state, people_positions)
            visible_ids_per_drone.append(visible_ids)
            detections_per_drone.append(detections)

        # shared_people : (shared_people_channels, H, W) — same for all drones
        # instant_maps: (num_drones, H, W) — per-drone current-step spatial support
        shared_people, instant_maps, own_coverage_maps, teammate_coverage_maps = self.build_shared_grid(
            drone_states,
            detections_per_drone,
            current_step=current_step,
        )
        hotspots = self._extract_hotspots(current_step=current_step)

        self.instant_maps_snapshot = instant_maps
        shared_drone_map = self._make_shared_drone_map(drone_states)
        observations = []
        for i, drone_state in enumerate(drone_states):
            other_states = [s for j, s in enumerate(drone_states) if j != i]

            local_vec = self.build_local_vector(
                drone_state,
                drone_id=i,
                num_visible=len(visible_ids_per_drone[i]),
                search_phase_progress=phase_context["search_phase_progress"],
                is_search_phase=phase_context["is_search_phase"],
                is_coverage_phase=phase_context["is_coverage_phase"],
                detections=detections_per_drone[i],
                other_drone_states=other_states,
                hotspots=hotspots,
                cmd_history=self._cmd_histories[i] if self.cmd_history_len > 0 else None,
            )

            # Critic keeps one ego map per drone, but the actor sees a shared
            # drone-position map containing all active drones.
            own_ego_map = self._make_ego_map(drone_state)

            # Actor grid (C, H, W), where C is 7 for the current default
            # count-only layout, 8 for the legacy count-only + recent-count
            # layout, 9 for spatial+count without recent count, and 10 for
            # the legacy spatial+count layout.
            #   ch0 — instantaneous spatial support LOCAL to this drone  (per-drone)
            #   ch1.. — shared people memory/count channels         (shared)
            #   ...  — own FOV coverage                             (per-drone)
            #   ...  — teammate-only FOV coverage                   (per-drone)
            #   ...  — shared drone map                             (shared)
            #   ...  — own ego map                                  (per-drone)
            local_instant = instant_maps[i][np.newaxis, :, :]   # (1, H, W)
            grid = np.concatenate(
                [
                    local_instant,
                    shared_people,
                    own_coverage_maps[i][np.newaxis, :, :],
                    teammate_coverage_maps[i][np.newaxis, :, :],
                    shared_drone_map[np.newaxis, :, :],
                    own_ego_map[np.newaxis, :, :],
                ],
                axis=0,
            ).astype(np.float32)
            if grid.shape[0] != self.actor_grid_channels:
                raise ValueError(
                    "Actor grid channel count does not match the configured layout: "
                    f"expected {self.actor_grid_channels}, got {grid.shape[0]}"
                )

            observations.append({
                "grid": grid,
                "local": local_vec,      # (11 + 6*(N-1) + 3*cmd_history_len,) — unique per drone
                "own_ego_map": own_ego_map,
            })

        return observations, visible_ids_per_drone, detections_per_drone

    def is_in_camera_footprint(self, drone_state, person_position):
        drone_x, drone_y, drone_z = drone_state["position"]
        yaw = drone_state["yaw"]
        person_x, person_y, _ = person_position

        planar_dist = math.hypot(person_x - drone_x, person_y - drone_y)
        if planar_dist > self.max_range:
            return False

        return point_in_footprint_world(
            px=person_x,
            py=person_y,
            drone_x=drone_x,
            drone_y=drone_y,
            z=drone_z,
            yaw=yaw,
            camera_tilt_deg=self.camera_tilt_deg,
            horizontal_fov_deg=self.horizontal_fov_deg,
            vertical_fov_deg=self.vertical_fov_deg,
        )
