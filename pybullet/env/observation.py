import math
import random
from typing import List, Dict, Tuple, Optional

import numpy as np

from sim.camera_geometry import (
    footprint_corners_world,
    point_in_footprint_local,
    point_in_footprint_world,
    world_to_drone_local,
)


class ObservationBuilder:
    """
    Builds a (7, H, W) actor grid observation each step:

      Channel 0 — people detections INSTANTANEOUS, LOCAL TO THIS DRONE
                  Only detections made by THIS drone on the current step.
                  No decay. Bright = this drone sees someone here right now.

      Channel 1 — people belief RECENT presence (shared across all drones)
                  (simulation-time half-life: recent_half_life_seconds)
                  Splatted from all drones' detections with short memory.
                  Bright = someone was seen here recently by the team.

      Channel 2 — people belief HISTORIC presence (shared across all drones)
                  (simulation-time half-life: historic_half_life_seconds)
                  Splatted from all drones' detections — combined memory.
                  Bright = someone was in this area this episode.
                  Tells the network "this area has people, worth revisiting."

      Channel 3 — people COUNT density for THIS STEP (shared across all drones)
                  Summed Gaussian blobs from all detections on the current step,
                  then tanh-compressed. Bright and wide means more people now.

      Channel 4 — FOV coverage (shared across all drones)
                  (simulation-time half-life: coverage_half_life_seconds)
                  Bright = camera was aimed here recently.
                  Compare with historic:
                    historic high + coverage high  -> watching people
                    historic high + coverage low   -> people here, not checked recently
                    historic low  + coverage high  -> looked here, nobody found
                    both low                       -> unexplored

      Channel 5 — Shared drone map
                  Gaussian blobs at all active drone positions.
                  Gives the CNN a shared spatial view of teammate placement.

      Channel 6 — Ego map for this drone
                  Gaussian blob at this drone's own position.
                  Lets the actor distinguish "me" from the shared drone layer.

    The centralised critic receives a (6 + 2 * num_agents, H, W) grid
    built by build_global_state() in state_utils.py:
      Channel 0   — shared instantaneous detections union
      Channel 1   — shared recent presence
      Channel 2   — shared historic presence
      Channel 3   — shared current-step count density
      Channel 4   — shared FOV coverage
      Channel 5   — shared drone map
      Channel 6..  — per-drone instantaneous maps (one per agent, critic-only)
      Channel ...  — ego map per drone (one per agent, critic-only)

    All decay constants are derived from simulation-time half-lives.
    Formula: decay_per_step = 0.5 ^ (dt / half_life_seconds)
    """

    def __init__(
        self,
        x_min: float = -10.0,
        x_max: float = 10.0,
        y_min: float = -10.0,
        y_max: float = 10.0,
        grid_h: int = 32,
        grid_w: int = 32,
        max_range: float = 6.0,
        horizontal_fov_deg: float = 60.0,
        vertical_fov_deg: float = 45.0,
        detection_prob: float = 0.95,
        position_noise_std: float = 0.1,
        camera_tilt_deg: float = 45.0,
        num_drones: int = 1,
        sim_hz: float = 1.0,
        blob_sigma: float = 1.5,
        ego_sigma: float = 2.5,
        recent_half_life_seconds: float = 3.0,    # Ch1: team short-memory people belief
        historic_half_life_seconds: float = 15.0,  # Ch2: episodic people memory
        coverage_half_life_seconds: float = 5.0,  # Ch4: FOV visited-recently window
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
        self.count_density_gain = 0.35

        dt = 1.0 / sim_hz

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
            f"  Ch4 coverage half-life: {coverage_half_life_seconds}s "
            f"= {coverage_half_life_seconds * sim_hz:.1f} steps  "
            f"| decay/step={self.decay_coverage:.4f} "
            f"| value after episode: {self.decay_coverage**steps_per_episode:.4f}\n"
            f"  Critic shared channels: instant union, recent presence, historic presence, "
            f"count density, coverage, drone map"
        )

        # Ch0: instantaneous detections — per-drone current-step observations only.
        self.people_detect_instant = np.zeros((num_drones, grid_h, grid_w), dtype=np.float32)
        # Ch1: fast decay — "seen recently" — shared across all drones.
        self.people_belief_recent = np.zeros((grid_h, grid_w), dtype=np.float32)
        # Ch2: slow decay — "seen somewhere in this episode" — shared across all drones.
        self.people_belief_historic = np.zeros((grid_h, grid_w), dtype=np.float32)
        # Ch3: count density — rebuilt every step from current detections.
        self.people_count_density = np.zeros((grid_h, grid_w), dtype=np.float32)
        # Ch4: FOV coverage — "camera was aimed here recently" — shared across all drones.
        self.coverage_map = np.zeros((grid_h, grid_w), dtype=np.float32)
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
        self.coverage_map.fill(0.0)
        self.instant_maps_snapshot = self.people_detect_instant.copy()
        self.prev_drone_positions = {}

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        x_clamped = min(max(x, self.x_min), self.x_max)
        y_clamped = min(max(y, self.y_min), self.y_max)
        u = int((x_clamped - self.x_min) / (self.x_max - self.x_min) * (self.grid_w - 1))
        v = int((y_clamped - self.y_min) / (self.y_max - self.y_min) * (self.grid_h - 1))
        return v, u

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

    def update_coverage_map(self, drone_states: List[Dict]):
        """
        Decay coverage map at coverage_half_life_seconds simulation-time rate,
        then mark all cells inside each drone's camera FOV footprint.
        """
        self.coverage_map *= self.decay_coverage
        for drone_state in drone_states:
            for (v, u) in self._get_footprint_cells(drone_state):
                self.coverage_map[v, u] = 1.0

    def build_people_maps(
        self,
        detections_per_drone,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Update and return belief maps.

        Ch0 (instantaneous): per-drone — each drone's detections from only the
        current step. Shape: (num_drones, H, W).

        Ch1 (recent): shared recent presence.

        Ch2 (historic): shared historic presence.

        Ch3: current-step count density, built from this step only.

        np.maximum is used so that multiple detections in the same area
        reinforce rather than interfere with each other.
        """
        self.people_detect_instant.fill(0.0)
        self.people_belief_recent *= self.decay_recent
        self.people_belief_historic *= self.decay_historic

        step_density = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)

        for drone_idx, detections in enumerate(detections_per_drone):
            for det in detections:
                x, y, _ = det
                v, u = self.world_to_grid(x, y)
                blob = self._gaussian_blob(v, u, self.blob_sigma)
                # Ch0: only THIS drone's current-step detections
                np.maximum(self.people_detect_instant[drone_idx], blob, out=self.people_detect_instant[drone_idx])
                # Ch1: all drones contribute to the short shared recent map
                np.maximum(self.people_belief_recent, blob, out=self.people_belief_recent)
                # Ch2: all drones contribute to the long shared historic map
                np.maximum(self.people_belief_historic, blob, out=self.people_belief_historic)
                step_density += blob

        self.people_count_density[...] = step_density
        count_density_obs = np.tanh(
            self.count_density_gain * self.people_count_density
        ).astype(np.float32)

        return (
            self.people_detect_instant.copy(),
            self.people_belief_recent.copy(),
            self.people_belief_historic.copy(),
            count_density_obs,
        )

    def build_shared_grid(self, drone_states, detections_per_drone) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns:
          shared_4ch  : (4, H, W) — channels shared across all drones:
                          [0] recent presence  (ch1 in actor grid)
                          [1] historic presence(ch2 in actor grid)
                          [2] count density    (ch3 in actor grid)
                          [3] FOV coverage     (ch4 in actor grid)
          instant_maps: (num_drones, H, W) — per-drone instantaneous detections
        """
        self.update_coverage_map(drone_states)
        instant_maps, recent, historic, count_density = self.build_people_maps(detections_per_drone)
        shared_4ch = np.stack(
            [recent, historic, count_density, self.coverage_map],
            axis=0,
        ).astype(np.float32)
        return shared_4ch, instant_maps

    def build_local_vector(
        self,
        drone_state: Dict,
        drone_id: int,
        num_visible: int = 0,
        other_drone_states: Optional[List[Dict]] = None,
    ) -> np.ndarray:
        """
        Local feature vector for one drone.

        Components:
          [0] x           — normalised to [-1, 1]
          [1] y           — normalised to [-1, 1]
          [2] z           — normalised to [-1, 1]
          [3] sin(yaw)
          [4] cos(yaw)
          [5] num_visible — normalised (count / 30)
          [6+] teammate blocks: [mask, rel_x, rel_y, rel_z, sin(yaw), cos(yaw)]

        Total: 6 + 6*(num_drones-1) values.
        """
        x, y, z = drone_state["position"]
        yaw = drone_state["yaw"]

        x_min, x_max = self.x_min, self.x_max
        y_min, y_max = self.y_min, self.y_max
        z_min, z_max = 0.3, 8.0
        area_diag = math.hypot(x_max - x_min, y_max - y_min)

        vec = [
            2.0 * (x - x_min) / (x_max - x_min) - 1.0,
            2.0 * (y - y_min) / (y_max - y_min) - 1.0,
            2.0 * (z - z_min) / (z_max - z_min) - 1.0,
            math.sin(yaw),
            math.cos(yaw),
            float(num_visible) / 30.0,
        ]

        other_states = other_drone_states or []
        for other in other_states:
            ox, oy, oz = other["position"]
            oyaw = other["yaw"]
            vec += [
                1.0,
                (ox - x) / area_diag,
                (oy - y) / area_diag,
                (oz - z) / (z_max - z_min),
                math.sin(oyaw),
                math.cos(oyaw),
            ]

        missing_teammates = max(0, self.num_drones - 1 - len(other_states))
        for _ in range(missing_teammates):
            vec += [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        return np.array(vec, dtype=np.float32)

    def build_observations(self, drone_states, people_positions):
        visible_ids_per_drone = []
        detections_per_drone = []

        for drone_state in drone_states:
            visible_ids, detections = self.get_visible_people(drone_state, people_positions)
            visible_ids_per_drone.append(visible_ids)
            detections_per_drone.append(detections)

        # shared_4ch : (4, H, W) — [recent presence, historic presence,
        #                           count density, coverage]
        #                           — same for all drones
        # instant_maps: (num_drones, H, W) — per-drone current-step detections
        shared_4ch, instant_maps = self.build_shared_grid(drone_states, detections_per_drone)

        self.instant_maps_snapshot = instant_maps
        shared_drone_map = self._make_shared_drone_map(drone_states)
        observations = []
        for i, drone_state in enumerate(drone_states):
            other_states = [s for j, s in enumerate(drone_states) if j != i]

            local_vec = self.build_local_vector(
                drone_state,
                drone_id=i,
                num_visible=len(visible_ids_per_drone[i]),
                other_drone_states=other_states,
            )

            # Critic keeps one ego map per drone, but the actor sees a shared
            # drone-position map containing all active drones.
            own_ego_map = self._make_ego_map(drone_state)

            # Actor grid (7, H, W):
            #   ch0 — instantaneous detections LOCAL to this drone  (per-drone)
            #   ch1 — recent presence                               (shared)
            #   ch2 — historic presence                             (shared)
            #   ch3 — current-step count density                    (shared)
            #   ch4 — FOV coverage                                  (shared)
            #   ch5 — shared drone map                              (shared)
            #   ch6 — own ego map                                   (per-drone)
            local_instant = instant_maps[i][np.newaxis, :, :]   # (1, H, W)
            grid = np.concatenate(
                [
                    local_instant,
                    shared_4ch,
                    shared_drone_map[np.newaxis, :, :],
                    own_ego_map[np.newaxis, :, :],
                ],
                axis=0,
            ).astype(np.float32)

            observations.append({
                "grid": grid,            # (7, H, W)
                "local": local_vec,      # (6 + 6*(N-1),) — unique per drone
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
