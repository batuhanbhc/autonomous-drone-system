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

      Channel 0 — LOCAL people map for THIS drone
                  Current default: raw local count density from only this
                  drone's current-step detections.
                  Legacy layout: instantaneous spatial support from only this
                  drone's current-step detections.

      Shared count density
                  Raw summed Gaussian blobs from all detections on the current
                  step. This is kept only for legacy layouts.

      Shared historic count memory
                  A slower decayed peak-memory over observed raw count density.
                  Preserves "the strongest crowd density seen here recently."

      Local recent count memory
                  Per-drone recent raw-density memory updated with the same
                  miss-penalty and max-hit rule as historic count memory, but
                  with a faster natural decay.

      Shared permanent coverage
                  Shared across all drones. If any drone ever visits a cell
                  during the episode, that cell becomes 1 and never decays.

      Own instant FOV footprint
                  Bright = this drone's current-step camera footprint.

      Teammate-only instant FOV footprint
                  Bright = at least one other drone's current-step footprint.

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

    The current/default actor layout uses 10 channels total:
      [local count density, local recent count memory]
      + [historic count, permanent coverage]
      + [own instant footprint, teammate instant footprint,
         own coverage, teammate coverage, shared drone, ego].

    The instant spatial-support map and the shared count-density map are still
    computed internally for legacy layouts and diagnostics.

    Legacy layouts are still supported for old checkpoints. Relative to the
    current default, they can additionally expose:
      shared count density (+1 channel)
      shared recent/historic spatial-support memory (+2 channels)

      The centralised critic receives a grid built by build_global_state() in
      state_utils.py:
      Legacy instant-map layout:
        ((shared_people_channels + 6) + 4 * num_agents, H, W)
      Current local-count layout:
        ((shared_people_channels + 5) + 5 * num_agents, H, W)

      Shared channels include the actor's shared people channels, plus shared
      current-step FOV footprint, shared recent FOV coverage, the shared
      drone map, and critic-only privileged GT maps for whole-map people
      occupancy and whole-map people density. Legacy
      instant-map checkpoints additionally prepend a shared instantaneous
      spatial-support union.

      Per-drone channels always contain:
        local people map, local recent count memory, own instant footprint,
        own recent coverage, own ego map

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
        detection_forward_decay_start_norm: float = 1.0,
        camera_tilt_deg: float = 45.0,
        num_drones: int = 2,
        sim_hz: float = 4.0,
        blob_sigma: float = 1.0,
        ego_sigma: float = 2.5,
        recent_half_life_seconds: float = 10.0,    # team short-memory people spatial support
        historic_half_life_seconds: float = 30.0,  # slower people spatial support memory
        coverage_half_life_seconds: float = 10.0,  # FOV visited-recently window
        z_min: float = 0.3,
        z_max: float = 8.0,
        cmd_history_len: int = 0,
        status_history_seconds: int = 4,
        hotspot_top_k: int = 3,
        hotspot_min_density: float = 1.5,
        hotspot_suppression_radius_scale: float = 4.0,
        hotspot_suppression_radius_min_cells: int = 2,
        people_count_normalizer: float = 30.0,
        local_people_map_mode: str = "count_density",
        include_local_recent_count_memory_channel: bool = True,
        include_shared_count_density_channel: bool = False,
        include_instant_fov_channels: bool = True,
        hide_person_features_during_search: bool = False,
        count_memory_historic_miss_penalty: float = 0.35,
        max_horizontal_velocity: float = 1.0,
        max_yaw_rate: float = 0.7,
        actor_grid_channels: int = 10,
        include_persistent_coverage_channel: bool = True,
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
        self.detection_forward_decay_start_norm = float(
            detection_forward_decay_start_norm
        )

        self.camera_tilt_deg = camera_tilt_deg

        self.blob_sigma = blob_sigma
        self.ego_sigma  = ego_sigma
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.dt = 1.0 / float(sim_hz)
        self.people_count_normalizer = float(people_count_normalizer)
        self.local_people_map_mode = str(local_people_map_mode)
        if self.local_people_map_mode not in {"instant", "count_density"}:
            raise ValueError(
                "local_people_map_mode must be 'instant' or 'count_density', got "
                f"{self.local_people_map_mode!r}"
            )
        self.include_local_recent_count_memory_channel = bool(
            include_local_recent_count_memory_channel
        )
        self.include_shared_count_density_channel = bool(
            include_shared_count_density_channel
        )
        self.include_instant_fov_channels = bool(include_instant_fov_channels)
        self.hide_person_features_during_search = bool(
            hide_person_features_during_search
        )
        self.count_memory_historic_miss_penalty = float(count_memory_historic_miss_penalty)
        self.cmd_history_len = int(cmd_history_len)
        self.status_history_seconds = int(status_history_seconds)
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
        self.include_persistent_coverage_channel = bool(
            include_persistent_coverage_channel
        )
        if not (0.0 <= self.detection_forward_decay_start_norm <= 1.0):
            raise ValueError(
                "detection_forward_decay_start_norm must be within [0, 1], got "
                f"{self.detection_forward_decay_start_norm}"
            )
        base_actor_grid_channels = (
            6
            + int(self.include_local_recent_count_memory_channel)
            + (2 if self.include_instant_fov_channels else 0)
            + int(self.include_persistent_coverage_channel)
            + int(self.include_shared_count_density_channel)
        )
        actor_grid_delta = self.actor_grid_channels - base_actor_grid_channels
        if actor_grid_delta not in {0, 2}:
            raise ValueError(
                "actor_grid_channels does not match the configured actor layout: "
                f"include_local_recent_count_memory_channel="
                f"{self.include_local_recent_count_memory_channel}, "
                f"include_persistent_coverage_channel={self.include_persistent_coverage_channel}, "
                f"include_shared_count_density_channel={self.include_shared_count_density_channel}, "
                f"include_instant_fov_channels={self.include_instant_fov_channels}, "
                f"got actor_grid_channels={self.actor_grid_channels}"
            )
        self.exposes_spatial_memory_channels = actor_grid_delta == 2
        self.shared_people_channels = (
            1
            + (2 if self.exposes_spatial_memory_channels else 0)
            + (1 if self.include_shared_count_density_channel else 0)
            + (1 if self.include_persistent_coverage_channel else 0)
        )
        shared_idx = 0
        self.actor_shared_person_channel_indices: list[int] = []
        if self.exposes_spatial_memory_channels:
            self.actor_shared_person_channel_indices.extend([shared_idx, shared_idx + 1])
            shared_idx += 2
        if self.include_shared_count_density_channel:
            self.actor_shared_person_channel_indices.append(shared_idx)
            shared_idx += 1
        self.actor_shared_historic_count_memory_channel = shared_idx
        self.actor_shared_person_channel_indices.append(shared_idx)
        shared_idx += 1
        self.actor_shared_persistent_coverage_channel = (
            shared_idx if self.include_persistent_coverage_channel else None
        )
        self.actor_local_recent_count_memory_channel = (
            1 if self.include_local_recent_count_memory_channel else None
        )
        actor_shared_offset = 1 + int(self.include_local_recent_count_memory_channel)
        self.actor_own_instant_coverage_channel = (
            actor_shared_offset + self.shared_people_channels
            if self.include_instant_fov_channels
            else None
        )
        self.actor_teammate_instant_coverage_channel = (
            self.actor_own_instant_coverage_channel + 1
            if self.actor_own_instant_coverage_channel is not None
            else None
        )
        self.actor_own_coverage_channel = (
            (self.actor_teammate_instant_coverage_channel + 1)
            if self.actor_teammate_instant_coverage_channel is not None
            else (actor_shared_offset + self.shared_people_channels)
        )
        self.actor_teammate_coverage_channel = self.actor_own_coverage_channel + 1
        self.actor_shared_drone_channel = self.actor_teammate_coverage_channel + 1
        self.actor_own_ego_channel = self.actor_shared_drone_channel + 1
        self.actor_channel_names = [
            (
                "Local instant spatial support"
                if self.local_people_map_mode == "instant"
                else "Local count density"
            ),
        ]
        if self.include_local_recent_count_memory_channel:
            self.actor_channel_names.append("Local recent count memory")
        if self.exposes_spatial_memory_channels:
            self.actor_channel_names += [
                "Shared recent spatial support",
                "Shared historic spatial support",
            ]
        if self.include_shared_count_density_channel:
            self.actor_channel_names.append("Shared count density")
        self.actor_channel_names.append("Shared historic count memory")
        if self.include_persistent_coverage_channel:
            self.actor_channel_names.append("Shared permanent coverage")
        if self.include_instant_fov_channels:
            self.actor_channel_names += [
                "Own instant FOV footprint",
                "Teammate instant FOV footprint",
            ]
        self.actor_channel_names += [
            "Own FOV coverage",
            "Teammate FOV coverage",
            "Shared drone map",
            "Own ego map",
        ]
        self.critic_has_shared_local_people_union = (
            self.local_people_map_mode == "instant"
        )
        self.critic_local_people_channel_name = (
            "Instant map" if self.local_people_map_mode == "instant" else "Count density"
        )
        self.critic_has_local_recent_count_memory_channel = (
            self.include_local_recent_count_memory_channel
        )
        self.critic_shared_channel_names = []
        if self.critic_has_shared_local_people_union:
            self.critic_shared_channel_names.append("Shared instant spatial union")
        if self.exposes_spatial_memory_channels:
            self.critic_shared_channel_names += [
                "Shared recent spatial support",
                "Shared historic spatial support",
            ]
        if self.include_shared_count_density_channel:
            self.critic_shared_channel_names.append("Shared count density")
        self.critic_shared_channel_names.append("Shared historic count memory")
        if self.include_persistent_coverage_channel:
            self.critic_shared_channel_names.append("Shared permanent coverage")
        if self.include_instant_fov_channels:
            self.critic_shared_channel_names.append("Shared instant FOV footprint")
        self.critic_shared_channel_names += [
            "Shared FOV coverage",
            "Shared drone map",
            "GT people occupancy",
            "GT people density",
        ]
        self._cmd_histories: List[deque] = [
            deque([(0.0, 0.0, 0.0)] * self.cmd_history_len, maxlen=self.cmd_history_len)
            for _ in range(self.num_drones)
        ]
        self._status_histories: List[deque] = [
            deque(
                [(0.0, 0.0, 0.0, 1.0, 0.0)] * self.status_history_seconds,
                maxlen=self.status_history_seconds,
            )
            for _ in range(self.num_drones)
        ]
        self._status_history_anchor_states: List[Optional[dict[str, float]]] = [
            None for _ in range(self.num_drones)
        ]
        self._prev_visible_counts: List[int] = [0 for _ in range(self.num_drones)]
        if self.z_max <= self.z_min:
            raise ValueError(
                f"z_max must be > z_min, got z_min={self.z_min}, z_max={self.z_max}"
            )
        if self.people_count_normalizer <= 0.0:
            raise ValueError(
                "people_count_normalizer must be > 0, got "
                f"{self.people_count_normalizer}"
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
        if self.status_history_seconds < 0:
            raise ValueError(
                f"status_history_seconds must be >= 0, got {self.status_history_seconds}"
            )

        dt = self.dt
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
            f"  Local people map mode: {self.local_people_map_mode}\n"
            f"  Local recent count memory channel: "
            f"{'enabled' if self.include_local_recent_count_memory_channel else 'disabled'}\n"
            f"  Shared count density channel: {'enabled' if self.include_shared_count_density_channel else 'disabled'}\n"
            f"  Instant FOV channels: {'enabled' if self.include_instant_fov_channels else 'disabled'}\n"
            f"  Exposed spatial-memory channels: {'enabled' if self.exposes_spatial_memory_channels else 'disabled'}\n"
            f"  Persistent coverage channel: {'enabled' if self.include_persistent_coverage_channel else 'disabled'}\n"
            f"  Historic count memory channel: enabled\n"
            f"  Critic shared channels: {', '.join(self.critic_shared_channel_names)}"
        )

        # Ch0: instantaneous spatial support — per-drone current-step observations only.
        self.people_detect_instant = np.zeros((num_drones, grid_h, grid_w), dtype=np.float32)
        # Ch1: fast-decay recent spatial support — shared across all drones.
        self.people_belief_recent = np.zeros((grid_h, grid_w), dtype=np.float32)
        # Ch2: slower-decay historic spatial support — shared across all drones.
        self.people_belief_historic = np.zeros((grid_h, grid_w), dtype=np.float32)
        # Ch3: raw count density — rebuilt every step from current detections.
        self.people_count_density = np.zeros((grid_h, grid_w), dtype=np.float32)
        self.people_count_memory_recent = np.zeros((num_drones, grid_h, grid_w), dtype=np.float32)
        self.people_count_memory_historic = np.zeros((grid_h, grid_w), dtype=np.float32)
        self.people_count_last_observed_step = np.full((grid_h, grid_w), -1, dtype=np.int32)
        # Shared recent FOV coverage — "camera was aimed here recently".
        self.coverage_map = np.zeros((grid_h, grid_w), dtype=np.float32)
        self.coverage_maps_per_drone = np.zeros((num_drones, grid_h, grid_w), dtype=np.float32)
        self.persistent_coverage_map = np.zeros((grid_h, grid_w), dtype=np.float32)
        self.footprint_maps_snapshot = np.zeros((num_drones, grid_h, grid_w), dtype=np.float32)
        self.instant_maps_snapshot = self.people_detect_instant.copy()
        self.local_people_maps_snapshot = self.people_detect_instant.copy()
        self.local_recent_count_memory_maps_snapshot = np.zeros(
            (num_drones, grid_h, grid_w), dtype=np.float32
        )
        self.gt_people_binary_snapshot = np.zeros((grid_h, grid_w), dtype=np.float32)
        self.gt_people_density_snapshot = np.zeros((grid_h, grid_w), dtype=np.float32)

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
        self.persistent_coverage_map.fill(0.0)
        self.footprint_maps_snapshot.fill(0.0)
        self.instant_maps_snapshot = self.people_detect_instant.copy()
        self.local_people_maps_snapshot = self.people_detect_instant.copy()
        self.local_recent_count_memory_maps_snapshot.fill(0.0)
        self.gt_people_binary_snapshot.fill(0.0)
        self.gt_people_density_snapshot.fill(0.0)
        self.prev_drone_positions = {}
        for h in self._cmd_histories:
            h.clear()
            h.extend([(0.0, 0.0, 0.0)] * self.cmd_history_len)
        for h in self._status_histories:
            h.clear()
            h.extend([(0.0, 0.0, 0.0, 1.0, 0.0)] * self.status_history_seconds)
        for drone_idx in range(self.num_drones):
            self._status_history_anchor_states[drone_idx] = None
        self._prev_visible_counts = [0 for _ in range(self.num_drones)]

    def update_cmd_history(self, drone_idx: int, vx: float, vy: float, yaw_rate: float) -> None:
        if self.cmd_history_len == 0:
            return
        self._cmd_histories[drone_idx].append((vx, vy, yaw_rate))

    def update_status_history(
        self,
        drone_idx: int,
        drone_state: Dict,
        num_visible: int,
        current_step: int,
    ) -> None:
        if self.status_history_seconds == 0:
            return

        x, y, _ = drone_state["position"]
        yaw = float(drone_state["yaw"])
        current_time_s = float(current_step) * self.dt
        anchor = self._status_history_anchor_states[drone_idx]
        if anchor is None:
            self._status_history_anchor_states[drone_idx] = {
                "x": float(x),
                "y": float(y),
                "yaw": yaw,
                "time_s": current_time_s,
            }
            return

        elapsed_s = current_time_s - float(anchor["time_s"])
        if elapsed_s + 1e-9 < 1.0:
            return

        max_disp = self._max_hvel * max(elapsed_s, 1e-6)
        dx_norm = 0.0 if max_disp <= 0.0 else max(-1.0, min(1.0, (float(x) - anchor["x"]) / max_disp))
        dy_norm = 0.0 if max_disp <= 0.0 else max(-1.0, min(1.0, (float(y) - anchor["y"]) / max_disp))
        delta_yaw = self._angle_wrap(yaw - float(anchor["yaw"]))
        visible_norm = float(num_visible) / self.people_count_normalizer
        self._status_histories[drone_idx].append(
            (
                dx_norm,
                dy_norm,
                math.sin(delta_yaw),
                math.cos(delta_yaw),
                visible_norm,
            )
        )
        self._status_history_anchor_states[drone_idx] = {
            "x": float(x),
            "y": float(y),
            "yaw": yaw,
            "time_s": current_time_s,
        }

    def _actor_hides_person_features(self, is_search_phase: float) -> bool:
        return self.hide_person_features_during_search and float(is_search_phase) > 0.5

    def _visited_fraction(self) -> float:
        if self.persistent_coverage_map.size == 0:
            return 0.0
        return float(
            np.mean(np.asarray(self.persistent_coverage_map, dtype=np.float32) > 0.0)
        )

    def _mask_actor_shared_people(
        self,
        shared_people: np.ndarray,
        hide_person_features: bool,
    ) -> np.ndarray:
        if not hide_person_features or shared_people.size == 0:
            return shared_people
        masked = np.array(shared_people, copy=True)
        if self.actor_shared_person_channel_indices:
            masked[self.actor_shared_person_channel_indices] = 0.0
        return masked

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

    def _build_gt_people_maps(self, people_positions) -> None:
        binary = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        density = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        for x, y, _ in people_positions:
            v, u = self.world_to_grid(x, y)
            binary[v, u] = 1.0
            density += self._gaussian_blob(v, u, self.blob_sigma)
        self.gt_people_binary_snapshot[...] = binary
        self.gt_people_density_snapshot[...] = density.astype(np.float32)

    def _forward_norm_for_person(self, drone_state: Dict, person_position) -> float:
        drone_x, drone_y, drone_z = drone_state["position"]
        yaw = drone_state["yaw"]
        person_x, person_y, _ = person_position
        forward, _ = world_to_drone_local(
            person_x,
            person_y,
            drone_x,
            drone_y,
            yaw,
        )
        min_forward, max_forward = footprint_forward_extents(
            z=drone_z,
            camera_tilt_deg=self.camera_tilt_deg,
            vertical_fov_deg=self.vertical_fov_deg,
        )
        span = max(max_forward - min_forward, 1e-6)
        return min(max((forward - min_forward) / span, 0.0), 1.0)

    def _forward_detection_scale(self, forward_norm: float) -> float:
        cutoff = self.detection_forward_decay_start_norm
        if cutoff >= 1.0 or forward_norm <= cutoff:
            return 1.0
        return max(0.0, (1.0 - forward_norm) / max(1.0 - cutoff, 1e-6))

    def get_noisy_detection(self, person_position, forward_norm: float):
        effective_detection_prob = self.detection_prob * self._forward_detection_scale(
            forward_norm
        )
        if random.random() > effective_detection_prob:
            return None
        px, py, pz = person_position
        noise_std = self.position_noise_std * (1.0 + 2.0 * forward_norm)
        noisy_x = px + random.gauss(0.0, noise_std)
        noisy_y = py + random.gauss(0.0, noise_std)
        return (noisy_x, noisy_y, pz)

    def get_visible_people(self, drone_state, people_positions):
        visible_ids = []
        detections = []
        for idx, pos in enumerate(people_positions):
            if self.is_in_camera_footprint(drone_state, pos):
                forward_norm = self._forward_norm_for_person(drone_state, pos)
                noisy_det = self.get_noisy_detection(pos, forward_norm)
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
        Decay the recent per-drone/shared coverage maps, and accumulate a
        separate shared permanent coverage map with no decay.
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
        np.maximum(
            self.persistent_coverage_map,
            current_footprint_maps[:active_drones].max(axis=0),
            out=self.persistent_coverage_map,
        )

    def build_people_maps(
        self,
        detections_per_drone,
        current_footprint_maps: np.ndarray,
        current_step: int,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Update and return belief maps.

        Ch0 (instantaneous): per-drone — each drone's spatial support from only the
        current step. Shape: (num_drones, H, W).

        Legacy Ch1-Ch2 (recent/historic shared spatial support) are updated only
        when the 10-channel legacy layout is requested.

        Count-density channels are raw densities:
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
        per_drone_step_density = np.zeros((self.num_drones, self.grid_h, self.grid_w), dtype=np.float32)

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
                per_drone_step_density[drone_idx] += blob

        self.people_count_density[...] = step_density
        count_density_obs = self.people_count_density.astype(np.float32)
        per_drone_count_density_obs = per_drone_step_density.astype(np.float32)
        observed_density_mask = count_density_obs >= self.hotspot_min_density
        self.people_count_last_observed_step[observed_density_mask] = int(current_step)
        self.people_count_memory_recent *= self.decay_recent
        self.people_count_memory_historic *= self.decay_historic

        if current_footprint_maps.size > 0:
            team_visible_mask = current_footprint_maps.max(axis=0) > 0.0
            historic_miss_mask = team_visible_mask & (~observed_density_mask)
            self.people_count_memory_historic[historic_miss_mask] *= (
                1.0 - self.count_memory_historic_miss_penalty
            )
            for drone_idx in range(min(current_footprint_maps.shape[0], self.num_drones)):
                local_visible_mask = current_footprint_maps[drone_idx] > 0.0
                local_observed_density_mask = (
                    per_drone_count_density_obs[drone_idx] >= self.hotspot_min_density
                )
                local_miss_mask = local_visible_mask & (~local_observed_density_mask)
                self.people_count_memory_recent[drone_idx][local_miss_mask] *= (
                    1.0 - self.count_memory_historic_miss_penalty
                )

        np.maximum(
            self.people_count_memory_recent,
            per_drone_count_density_obs,
            out=self.people_count_memory_recent,
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
            per_drone_count_density_obs,
            recent_count_memory_obs,
            historic_count_memory_obs,
        )

    def build_shared_grid(
        self,
        drone_states,
        detections_per_drone,
        current_step: int,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns:
          shared_people: (shared_people_channels, H, W) — channels shared
                         across all drones.
          local_people_maps: (num_drones, H, W) — actor/critic per-drone
                             people maps for the current layout
          local_recent_count_memory_maps: (num_drones, H, W) — actor/critic
                                 per-drone recent count-memory maps
          instant_maps: (num_drones, H, W) — per-drone instantaneous spatial support
          own_instant_coverage_maps: (num_drones, H, W) — per-drone current-step
                                 FOV footprint
          teammate_instant_coverage_maps: (num_drones, H, W) — union of all other
                                 drones' current-step FOV footprints
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
            per_drone_count_density,
            recent_count_memory,
            historic_count_memory,
        ) = self.build_people_maps(
            detections_per_drone,
            current_footprint_maps,
            current_step,
        )
        own_instant_coverage_maps = current_footprint_maps[:active_drones].copy()
        teammate_instant_coverage_maps = np.zeros_like(own_instant_coverage_maps)
        for drone_idx in range(active_drones):
            teammate_indices = [idx for idx in range(active_drones) if idx != drone_idx]
            if teammate_indices:
                teammate_instant_coverage_maps[drone_idx] = current_footprint_maps[
                    teammate_indices
                ].max(axis=0)
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
        if self.include_shared_count_density_channel:
            shared_people.append(count_density)
        shared_people.append(historic_count_memory)
        if self.include_persistent_coverage_channel:
            shared_people.append(self.persistent_coverage_map.copy())
        local_people_maps = (
            instant_maps
            if self.local_people_map_mode == "instant"
            else per_drone_count_density[:active_drones]
        )
        return (
            np.stack(shared_people, axis=0),
            local_people_maps,
            recent_count_memory[:active_drones].copy(),
            instant_maps,
            own_instant_coverage_maps,
            teammate_instant_coverage_maps,
            own_coverage_maps,
            teammate_coverage_maps,
        )

    def build_local_vector(
        self,
        drone_state: Dict,
        drone_id: int,
        num_visible: int = 0,
        delta_visible: float = 0.0,
        visited_fraction: float = 0.0,
        search_phase_progress: float = 1.0,
        is_search_phase: float = 0.0,
        is_coverage_phase: float = 1.0,
        detections: Optional[List[Tuple[float, float, float]]] = None,
        other_drone_states: Optional[List[Dict]] = None,
        hotspots: Optional[list[tuple[float, float, float, float]]] = None,
        status_history: Optional[deque] = None,
        cmd_history: Optional[deque] = None,
        hide_person_features: bool = False,
    ) -> np.ndarray:
        """
        Local feature vector for one drone.

        Components:
          [0] x           — normalised to [-1, 1]
          [1] y           — normalised to [-1, 1]
          [2] sin(yaw)
          [3] cos(yaw)
          [4] num_visible — normalised (count / 30)
          [5] delta_visible — normalised change vs previous step
          [6] visited_fraction — team permanent-FOV coverage fraction in [0, 1]
          [7] search_phase_progress — completed fraction in [0, 1]
          [8] is_search_phase
          [9] is_coverage_phase
          [10] detection_centroid_present
          [11] centroid_forward_offset_from_principal — normalised to [-1, 1]
          [12] centroid_lateral_offset_from_principal — normalised to [-1, 1]
          [13...] hotspot slots: [valid, rel_dx_world, rel_dy_world, density, age] * hotspot_top_k
          [...] teammate blocks: [mask, rel_x, rel_y, rel_z, sin(yaw), cos(yaw)]
          [...] status history (oldest→newest): [delta_x, delta_y, sin(delta_yaw),
                                                  cos(delta_yaw), num_visible_norm]
                                                  * status_history_seconds
          [...] command history (oldest→newest): [vx, vy, yaw_rate] * cmd_history_len

        Total:
          13 + 5*hotspot_top_k + 6*(num_drones-1)
          + 5*status_history_seconds + 3*cmd_history_len values.
        """
        x, y, z = drone_state["position"]
        yaw = drone_state["yaw"]

        x_min, x_max = self.x_min, self.x_max
        y_min, y_max = self.y_min, self.y_max
        area_diag = math.hypot(x_max - x_min, y_max - y_min)

        centroid_present = 0.0
        centroid_forward_offset = 0.0
        centroid_lateral_offset = 0.0
        visible_count_norm = float(num_visible) / self.people_count_normalizer
        delta_visible_value = float(delta_visible)
        centroid_detections = [] if hide_person_features else (detections or [])
        if centroid_detections:
            centroid_present = 1.0
            centroid_x, centroid_y = geometric_median(
                [(det[0], det[1]) for det in centroid_detections]
            )
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
            0.0 if hide_person_features else visible_count_norm,
            0.0 if hide_person_features else delta_visible_value,
            min(max(float(visited_fraction), 0.0), 1.0),
            min(max(float(search_phase_progress), 0.0), 1.0),
            float(is_search_phase),
            float(is_coverage_phase),
            centroid_present,
            centroid_forward_offset,
            centroid_lateral_offset,
        ]

        hotspot_items = [] if hide_person_features else (hotspots or [])
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

        if status_history is not None and self.status_history_seconds > 0:
            for dx_h, dy_h, sin_dyaw_h, cos_dyaw_h, num_visible_h in status_history:
                vec += [
                    dx_h,
                    dy_h,
                    sin_dyaw_h,
                    cos_dyaw_h,
                    0.0 if hide_person_features else num_visible_h,
                ]

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
        self._build_gt_people_maps(people_positions)

        for drone_state in drone_states:
            visible_ids, detections = self.get_visible_people(drone_state, people_positions)
            visible_ids_per_drone.append(visible_ids)
            detections_per_drone.append(detections)

        # shared_people : (shared_people_channels, H, W) — same for all drones
        # local_people_maps: (num_drones, H, W) — per-drone current-layout people maps
        # local_recent_count_memory_maps: (num_drones, H, W) — per-drone recent count memory
        (
            shared_people,
            local_people_maps,
            local_recent_count_memory_maps,
            instant_maps,
            own_instant_coverage_maps,
            teammate_instant_coverage_maps,
            own_coverage_maps,
            teammate_coverage_maps,
        ) = self.build_shared_grid(
            drone_states,
            detections_per_drone,
            current_step=current_step,
        )
        hotspots = self._extract_hotspots(current_step=current_step)
        for drone_idx, drone_state in enumerate(drone_states):
            self.update_status_history(
                drone_idx=drone_idx,
                drone_state=drone_state,
                num_visible=len(visible_ids_per_drone[drone_idx]),
                current_step=current_step,
            )

        self.instant_maps_snapshot = instant_maps
        self.local_people_maps_snapshot.fill(0.0)
        if len(local_people_maps) > 0:
            self.local_people_maps_snapshot[:len(local_people_maps)] = local_people_maps
        self.local_recent_count_memory_maps_snapshot.fill(0.0)
        if len(local_recent_count_memory_maps) > 0:
            self.local_recent_count_memory_maps_snapshot[
                :len(local_recent_count_memory_maps)
            ] = local_recent_count_memory_maps
        shared_drone_map = self._make_shared_drone_map(drone_states)
        visited_fraction = self._visited_fraction()
        hide_person_features = self._actor_hides_person_features(
            phase_context["is_search_phase"]
        )
        actor_shared_people = self._mask_actor_shared_people(
            shared_people,
            hide_person_features=hide_person_features,
        )
        visible_count_deltas = [
            max(
                -1.0,
                min(
                    1.0,
                    (len(visible_ids_per_drone[i]) - self._prev_visible_counts[i])
                    / self.people_count_normalizer,
                ),
            )
            for i in range(len(drone_states))
        ]
        observations = []
        for i, drone_state in enumerate(drone_states):
            other_states = [s for j, s in enumerate(drone_states) if j != i]

            critic_local_vec = self.build_local_vector(
                drone_state,
                drone_id=i,
                num_visible=len(visible_ids_per_drone[i]),
                delta_visible=visible_count_deltas[i],
                visited_fraction=visited_fraction,
                search_phase_progress=phase_context["search_phase_progress"],
                is_search_phase=phase_context["is_search_phase"],
                is_coverage_phase=phase_context["is_coverage_phase"],
                detections=detections_per_drone[i],
                other_drone_states=other_states,
                hotspots=hotspots,
                status_history=self._status_histories[i] if self.status_history_seconds > 0 else None,
                cmd_history=self._cmd_histories[i] if self.cmd_history_len > 0 else None,
                hide_person_features=False,
            )
            local_vec = self.build_local_vector(
                drone_state,
                drone_id=i,
                num_visible=len(visible_ids_per_drone[i]),
                delta_visible=visible_count_deltas[i],
                visited_fraction=visited_fraction,
                search_phase_progress=phase_context["search_phase_progress"],
                is_search_phase=phase_context["is_search_phase"],
                is_coverage_phase=phase_context["is_coverage_phase"],
                detections=detections_per_drone[i],
                other_drone_states=other_states,
                hotspots=hotspots,
                status_history=self._status_histories[i] if self.status_history_seconds > 0 else None,
                cmd_history=self._cmd_histories[i] if self.cmd_history_len > 0 else None,
                hide_person_features=hide_person_features,
            )

            # Critic keeps one ego map per drone, but the actor sees a shared
            # drone-position map containing all active drones.
            own_ego_map = self._make_ego_map(drone_state)

            # Actor grid (C, H, W), where C is 10 for the current default
            # local-count + local-recent-count + permanent-coverage
            # + instant-FOV layout. Legacy
            # layouts without the extra footprint channels, the permanent
            # shared coverage channel, or with local instant maps are still
            # loadable.
            #   ch0 — per-drone local people map                         (per-drone)
            #   ch1 — per-drone local recent count memory               (per-drone)
            #   ch2.. — shared memory/count/coverage channels       (shared)
            #   ...  — own instant FOV footprint                     (per-drone)
            #   ...  — teammate-only instant FOV footprint           (per-drone)
            #   ...  — own FOV coverage                             (per-drone)
            #   ...  — teammate-only FOV coverage                   (per-drone)
            #   ...  — shared drone map                             (shared)
            #   ...  — own ego map                                  (per-drone)
            actor_local_people_map = (
                np.zeros_like(local_people_maps[i], dtype=np.float32)
                if hide_person_features
                else local_people_maps[i]
            )
            actor_local_recent_count_memory_map = (
                np.zeros_like(local_recent_count_memory_maps[i], dtype=np.float32)
                if hide_person_features
                else local_recent_count_memory_maps[i]
            )
            local_people_map = actor_local_people_map[np.newaxis, :, :]   # (1, H, W)
            grid_parts = [local_people_map]
            if self.include_local_recent_count_memory_channel:
                grid_parts.append(actor_local_recent_count_memory_map[np.newaxis, :, :])
            grid_parts.append(actor_shared_people)
            if self.include_instant_fov_channels:
                grid_parts.extend(
                    [
                        own_instant_coverage_maps[i][np.newaxis, :, :],
                        teammate_instant_coverage_maps[i][np.newaxis, :, :],
                    ]
                )
            grid_parts.extend(
                [
                    own_coverage_maps[i][np.newaxis, :, :],
                    teammate_coverage_maps[i][np.newaxis, :, :],
                    shared_drone_map[np.newaxis, :, :],
                    own_ego_map[np.newaxis, :, :],
                ]
            )
            grid = np.concatenate(grid_parts, axis=0).astype(np.float32)
            if grid.shape[0] != self.actor_grid_channels:
                raise ValueError(
                    "Actor grid channel count does not match the configured layout: "
                    f"expected {self.actor_grid_channels}, got {grid.shape[0]}"
                )

            observations.append({
                "grid": grid,
                "local": local_vec,      # core local features + hotspot/teammate/history slots
                "critic_local": critic_local_vec,
                "own_ego_map": own_ego_map,
            })

        for i in range(len(drone_states)):
            self._prev_visible_counts[i] = len(visible_ids_per_drone[i])
        for i in range(len(drone_states), self.num_drones):
            self._prev_visible_counts[i] = 0

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
