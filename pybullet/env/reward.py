import math
from typing import List, Dict, Tuple, Optional
import numpy as np

from sim.camera_geometry import (
    principal_point_world,
    footprint_center,
    footprint_corners_world,
    footprint_forward_extents,
    lateral_half_width_at_forward_distance,
    point_in_footprint_world,
    ray_polygon_intersection_distance,
    world_to_drone_local,
)


class RewardCalculator:
    def __init__(
        self,
        x_min: float = -5.0,
        x_max: float = 5.0,
        y_min: float = -5.0,
        y_max: float = 5.0,
        wc: float = 1.0,      # pure coverage weight (visible / num_people)
        wqual: float = 0.0,   # FOV quality weight (quality-weighted visible / num_people)
        wd: float = 5.0,      # discovery bonus
        wo: float = 2.0,      # overlap penalty
        wx: float = 0.3,      # exploration bonus
        ws: float = 1.0,      # boundary penalty
        wclose: float = 0.0,  # proximity penalty before collision
        wfov_overlap: float = 0.0,  # current-step FOV IoU overlap penalty
        wcoll: float = 1.0,   # collision penalty
        we: float = 0.0,      # energy penalty (disabled)
        wi: float = 0.5,      # idle penalty — strong to force both drones near people
        wfov: float = 0.5,    # FOV boundary penalty — penalise looking outside area
        coverage_edge_quality: float = 0.2,  # quality floor for worst visible framing
        reward_quality_mode: str = "principal_linear",
        boundary_margin: float = 0.2,
        drone_closeness_margin: float = 1.0,
        tilt_deg: float = 45.0,
        horizontal_fov_deg: float = 60.0,
        vertical_fov_deg: float = 45.0,
        fov_margin: float = 1.0,  # soft margin for FOV boundary penalty
    ):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

        self.wc    = wc
        self.wqual = wqual
        self.wd    = wd
        self.wo    = wo
        self.wx    = wx
        self.ws    = ws
        self.wclose = wclose
        self.wfov_overlap = wfov_overlap
        self.wcoll = wcoll
        self.we    = we
        self.wi    = wi
        self.wfov  = wfov
        self.coverage_edge_quality = coverage_edge_quality
        self.coverage_lateral_exponent = 2.0
        self.reward_quality_mode = str(reward_quality_mode)

        self.boundary_margin = boundary_margin
        self.drone_closeness_margin = drone_closeness_margin
        self.tilt_deg        = tilt_deg
        self.horizontal_fov_deg = horizontal_fov_deg
        self.vertical_fov_deg = vertical_fov_deg
        self.fov_margin      = fov_margin
        if self.drone_closeness_margin < 0.0:
            raise ValueError(
                "drone_closeness_margin must be >= 0, got "
                f"{self.drone_closeness_margin}"
            )
        if not (0.0 <= self.coverage_edge_quality <= 1.0):
            raise ValueError(
                "coverage_edge_quality must be within [0, 1], got "
                f"{self.coverage_edge_quality}"
            )
        if self.reward_quality_mode not in {
            "legacy",
            "principal_linear",
            "principal_squared",
        }:
            raise ValueError(
                "reward_quality_mode must be one of "
                "{'legacy', 'principal_linear', 'principal_squared'}, got "
                f"{self.reward_quality_mode}"
            )

    def coverage_quality_from_components(
        self,
        forward_norm: float,
        lateral_norm: float,
    ) -> float:
        forward_norm = min(max(float(forward_norm), 0.0), 1.0)
        lateral_norm = min(max(float(lateral_norm), 0.0), 1.0)
        q_forward = 1.0 - forward_norm
        q_lateral = 1.0 - lateral_norm ** self.coverage_lateral_exponent
        base_quality = q_forward * q_lateral
        return self.coverage_edge_quality + (
            1.0 - self.coverage_edge_quality
        ) * base_quality

    def coverage_quality_for_point(
        self,
        drone_state: Dict,
        point: Tuple[float, float],
        eps: float = 1e-6,
    ) -> float:
        if self.reward_quality_mode != "legacy":
            radial_norm = self._principal_radial_norm_for_point(
                drone_state=drone_state,
                point=point,
                eps=eps,
            )
            if self.reward_quality_mode == "principal_squared":
                base_quality = 1.0 - radial_norm ** 2
            else:
                base_quality = 1.0 - radial_norm
            return self.coverage_edge_quality + (
                1.0 - self.coverage_edge_quality
            ) * base_quality

        x, y, z = drone_state["position"]
        yaw = drone_state["yaw"]
        px, py = point
        forward, lateral = world_to_drone_local(px, py, x, y, yaw)
        min_forward, max_forward = footprint_forward_extents(
            z=z,
            camera_tilt_deg=self.tilt_deg,
            vertical_fov_deg=self.vertical_fov_deg,
        )
        forward_span = max(max_forward - min_forward, eps)
        forward_norm = min(max((forward - min_forward) / forward_span, 0.0), 1.0)
        half_width = lateral_half_width_at_forward_distance(
            forward=forward,
            z=z,
            horizontal_fov_deg=self.horizontal_fov_deg,
        )
        if half_width <= eps:
            lateral_norm = 1.0
        else:
            lateral_norm = min(abs(lateral) / half_width, 1.0)
        return self.coverage_quality_from_components(forward_norm, lateral_norm)

    def _principal_radial_norm_for_point(
        self,
        drone_state: Dict,
        point: Tuple[float, float],
        eps: float = 1e-6,
    ) -> float:
        x, y, z = drone_state["position"]
        yaw = drone_state["yaw"]
        px, py = point
        principal_x, principal_y = principal_point_world(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            camera_tilt_deg=self.tilt_deg,
        )
        direction = (px - principal_x, py - principal_y)
        radial_distance = math.hypot(direction[0], direction[1])
        if radial_distance <= eps:
            return 0.0

        footprint = footprint_corners_world(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            camera_tilt_deg=self.tilt_deg,
            horizontal_fov_deg=self.horizontal_fov_deg,
            vertical_fov_deg=self.vertical_fov_deg,
        )
        boundary_distance = ray_polygon_intersection_distance(
            origin=(principal_x, principal_y),
            direction=direction,
            polygon=footprint,
            eps=eps,
        )
        if boundary_distance is None or boundary_distance <= eps:
            return 1.0
        return min(max(radial_distance / boundary_distance, 0.0), 1.0)

    def point_in_footprint_world_for_reward(
        self,
        drone_state: Dict,
        point: Tuple[float, float],
    ) -> bool:
        x, y, z = drone_state["position"]
        yaw = drone_state["yaw"]
        px, py = point
        return point_in_footprint_world(
            px=px,
            py=py,
            drone_x=x,
            drone_y=y,
            z=z,
            yaw=yaw,
            camera_tilt_deg=self.tilt_deg,
            horizontal_fov_deg=self.horizontal_fov_deg,
            vertical_fov_deg=self.vertical_fov_deg,
        )

    @staticmethod
    def _discovery_time_weight(current_step: int, episode_steps: int) -> float:
        if episode_steps <= 1:
            return 1.0
        progress = min(max(current_step - 1, 0), episode_steps - 1) / (episode_steps - 1)
        return 1.0 - progress

    # ------------------------------------------------------------------ #
    #  Coverage — quality-weighted unique coverage using camera-local
    #  near/far and lateral framing quality
    # ------------------------------------------------------------------ #

    def compute_coverage_reward(
        self,
        visible_ids_per_drone: List[List[int]],
        num_people: int,
        eps: float = 1e-6,
    ) -> Tuple[float, int, set]:
        visible_union = set()
        for ids in visible_ids_per_drone:
            visible_union.update(ids)
        c_t = len(visible_union)
        if num_people <= 0:
            return 0.0, c_t, visible_union
        r_cov = (c_t / (num_people + eps))**2
        return r_cov, c_t, visible_union

    def compute_fov_quality_reward(
        self,
        visible_ids_per_drone: List[List[int]],
        drone_states: List[Dict],
        people_positions: List[Tuple[float, float, float]],
        num_people: int,
        eps: float = 1e-6,
    ) -> float:
        if num_people <= 0:
            return 0.0
        per_person_quality: Dict[int, float] = {}
        for drone_idx, ids in enumerate(visible_ids_per_drone):
            if drone_idx >= len(drone_states):
                break
            drone_state = drone_states[drone_idx]
            for person_id in ids:
                if person_id >= len(people_positions):
                    continue
                px, py, _ = people_positions[person_id]
                quality = self.coverage_quality_for_point(
                    drone_state=drone_state,
                    point=(px, py),
                    eps=eps,
                )
                if quality > per_person_quality.get(person_id, 0.0):
                    per_person_quality[person_id] = quality
        return (sum(per_person_quality.values())/ (num_people + eps))**2

    # ------------------------------------------------------------------ #
    #  Discovery — reward finding NEW people
    # ------------------------------------------------------------------ #

    def compute_discovery_reward(
        self,
        visible_union: set,
        ever_seen: set,
        num_people: int,
        current_step: int,
        episode_steps: int,
        eps: float = 1e-6,
    ) -> Tuple[float, set]:
        new_discoveries = visible_union - ever_seen
        time_weight = self._discovery_time_weight(current_step, episode_steps)
        r_disc = time_weight * len(new_discoveries) / (num_people + eps)
        return r_disc, new_discoveries

    # ------------------------------------------------------------------ #
    #  Overlap — penalise both drones watching the same person
    # ------------------------------------------------------------------ #

    def compute_overlap_penalty(
        self,
        visible_ids_per_drone: List[List[int]],
        num_people: int,
        eps: float = 1e-6,
    ) -> float:
        if num_people <= 0:
            return 0.0
        visible_union = set()
        for ids in visible_ids_per_drone:
            visible_union.update(ids)
        overlap = 0
        for person_id in visible_union:
            k = sum(person_id in ids for ids in visible_ids_per_drone)
            overlap += max(0, k - 1)
        return overlap / (num_people + eps)

    # ------------------------------------------------------------------ #
    #  Idle — penalise drones that see nobody
    # ------------------------------------------------------------------ #

    def compute_idle_penalty(
        self,
        visible_ids_per_drone: List[List[int]],
    ) -> float:
        """
        Strong penalty per drone seeing nobody.
        wi=0.5 means both drones idle = -1.0/step = -500 over 500-step episode.
        """
        penalty = 0.0
        for ids in visible_ids_per_drone:
            if len(ids) == 0:
                penalty += 1.0
        return penalty

    # ------------------------------------------------------------------ #
    #  FOV boundary — penalise looking outside the environment
    # ------------------------------------------------------------------ #

    def compute_fov_boundary_penalty(
        self,
        drone_states: List[Dict],
    ) -> float:
        """
        Penalise drones whose camera footprint center falls outside the
        environment boundaries. Prevents wasting FOV on empty space outside
        the area (caused by camera tilt at edges).

        Hard penalty if footprint center is outside bounds.
        Soft penalty if footprint center is within fov_margin of bounds.
        """
        penalty = 0.0
        for drone_state in drone_states:
            x, y, z   = drone_state["position"]
            yaw       = drone_state["yaw"]
            cx, cy = footprint_center(
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                camera_tilt_deg=self.tilt_deg,
                vertical_fov_deg=self.vertical_fov_deg,
            )
            corners = footprint_corners_world(
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                camera_tilt_deg=self.tilt_deg,
                horizontal_fov_deg=self.horizontal_fov_deg,
                vertical_fov_deg=self.vertical_fov_deg,
            )

            if any(
                corner_x < self.x_min or corner_x > self.x_max
                or corner_y < self.y_min or corner_y > self.y_max
                for corner_x, corner_y in corners
            ):
                penalty += 1.0
            elif (
                cx < self.x_min + self.fov_margin
                or cx > self.x_max - self.fov_margin
                or cy < self.y_min + self.fov_margin
                or cy > self.y_max - self.fov_margin
            ):
                penalty += 0.3
        return penalty

    # ------------------------------------------------------------------ #
    #  Safety
    # ------------------------------------------------------------------ #

    def compute_boundary_penalty(self, drone_states: List[Dict]) -> float:
        penalty = 0.0
        for drone_state in drone_states:
            x, y, _ = drone_state["position"]
            dist = min(
                x - self.x_min,
                self.x_max - x,
                y - self.y_min,
                self.y_max - y,
            )
            if dist < 0:
                # outside bounds: hard penalty proportional to overshoot
                penalty += 1.0 + (-dist)
            elif dist < self.boundary_margin:
                # inside margin: linear ramp from 0 (at margin edge) to 1 (at wall)
                penalty += 1.0 - dist / self.boundary_margin
        return penalty

    def compute_collision_penalty(
        self,
        drone_states: List[Dict],
        min_separation: float = 0.4,
    ) -> float:
        penalty = 0.0
        for i in range(len(drone_states)):
            xi, yi, zi = drone_states[i]["position"]
            for j in range(i + 1, len(drone_states)):
                xj, yj, zj = drone_states[j]["position"]
                dx = xi - xj
                dy = yi - yj
                dz = zi - zj
                if dx*dx + dy*dy + dz*dz < min_separation * min_separation:
                    penalty += 1.0
        return penalty

    def compute_drone_closeness_penalty(
        self,
        drone_states: List[Dict],
    ) -> float:
        if self.drone_closeness_margin <= 0.0:
            return 0.0
        penalty = 0.0
        for i in range(len(drone_states)):
            xi, yi, zi = drone_states[i]["position"]
            for j in range(i + 1, len(drone_states)):
                xj, yj, zj = drone_states[j]["position"]
                dist = np.linalg.norm(
                    np.array([xi - xj, yi - yj, zi - zj], dtype=np.float32)
                )
                if dist < self.drone_closeness_margin:
                    penalty += 1.0 - (dist / self.drone_closeness_margin)
        return penalty

    def compute_energy_penalty(
        self,
        actions: Optional[List[Tuple[float, float, float, float]]] = None,
    ) -> float:
        if actions is None or self.we == 0.0:
            return 0.0
        penalty = 0.0
        for action in actions:
            vx, vy, vz, yaw_rate = action
            penalty += abs(vx) + abs(vy) + abs(vz) + abs(yaw_rate)
        return penalty

    def compute_exploration_reward(
        self,
        drone_states: List[Dict],
        coverage_map: np.ndarray,
    ) -> float:
        if coverage_map is None or coverage_map.size == 0 or self.wx == 0.0:
            return 0.0
        h, w = coverage_map.shape
        total_score = 0.0
        num_cells   = 0
        xs = np.linspace(self.x_min, self.x_max, w)
        ys = np.linspace(self.y_min, self.y_max, h)
        for drone_state in drone_states:
            x, y, z = drone_state["position"]
            yaw = drone_state["yaw"]
            for row in range(h):
                py = ys[row]
                for col in range(w):
                    px = xs[col]
                    if point_in_footprint_world(
                        px=px,
                        py=py,
                        drone_x=x,
                        drone_y=y,
                        z=z,
                        yaw=yaw,
                        camera_tilt_deg=self.tilt_deg,
                        horizontal_fov_deg=self.horizontal_fov_deg,
                        vertical_fov_deg=self.vertical_fov_deg,
                    ):
                        total_score += (1.0 - float(coverage_map[row, col]))
                        num_cells   += 1
        return total_score / num_cells if num_cells > 0 else 0.0

    def compute_fov_overlap_penalty(
        self,
        footprint_maps: Optional[np.ndarray],
        eps: float = 1e-6,
    ) -> float:
        if self.wfov_overlap == 0.0:
            return 0.0
        if footprint_maps is None or footprint_maps.size == 0:
            return 0.0

        active_maps = np.asarray(footprint_maps, dtype=np.float32)
        if active_maps.ndim != 3 or active_maps.shape[0] < 2:
            return 0.0

        penalty = 0.0
        pair_count = 0
        for i in range(active_maps.shape[0]):
            map_i = active_maps[i]
            if not np.any(map_i > 0.0):
                continue
            for j in range(i + 1, active_maps.shape[0]):
                map_j = active_maps[j]
                if not np.any(map_j > 0.0):
                    continue
                intersection = np.minimum(map_i, map_j)
                union = np.maximum(map_i, map_j)
                union_sum = float(union.sum())
                if union_sum <= eps:
                    continue
                penalty += float(intersection.sum()) / union_sum
                pair_count += 1

        if pair_count == 0:
            return 0.0
        return penalty / pair_count

    # ------------------------------------------------------------------ #
    #  Total reward
    # ------------------------------------------------------------------ #

    def compute_reward(
        self,
        visible_ids_per_drone: List[List[int]],
        drone_states: List[Dict],
        people_positions: List[Tuple[float, float, float]],
        num_people: int,
        ever_seen: set,
        current_step: int,
        episode_steps: int,
        actions: Optional[List[Tuple[float, float, float, float]]] = None,
        coverage_map: Optional[np.ndarray] = None,
        footprint_maps: Optional[np.ndarray] = None,
    ) -> Tuple[float, dict, set]:

        r_cov, c_t, visible_union = self.compute_coverage_reward(
            visible_ids_per_drone,
            num_people,
        )
        r_fovq = self.compute_fov_quality_reward(
            visible_ids_per_drone,
            drone_states,
            people_positions,
            num_people,
        )

        r_disc, new_discoveries = self.compute_discovery_reward(
            visible_union,
            ever_seen,
            num_people,
            current_step=current_step,
            episode_steps=episode_steps,
        )
        r_ov     = self.compute_overlap_penalty(visible_ids_per_drone, num_people)
        r_idle   = self.compute_idle_penalty(visible_ids_per_drone)
        r_fov    = self.compute_fov_boundary_penalty(drone_states)
        r_exp    = self.compute_exploration_reward(drone_states, coverage_map)
        r_fov_overlap = self.compute_fov_overlap_penalty(footprint_maps)
        r_bound  = self.compute_boundary_penalty(drone_states)
        r_close  = self.compute_drone_closeness_penalty(drone_states)
        r_coll   = self.compute_collision_penalty(drone_states)
        r_safe   = r_bound + r_close + r_coll
        r_energy = self.compute_energy_penalty(actions)

        total_reward = (
            self.wc    * r_cov
            + self.wqual * r_fovq
            + self.wd  * r_disc
            + self.wx  * r_exp
            - self.wo  * r_ov
            - self.wi  * r_idle
            - self.wfov * r_fov
            - self.wfov_overlap * r_fov_overlap
            - self.ws  * r_bound
            - self.wclose * r_close
            - self.wcoll * r_coll
            - self.we  * r_energy
        )

        reward_info = {
            "r_cov":          r_cov,
            "r_fovq":         r_fovq,
            "r_disc":         r_disc,
            "discovery_time_weight": self._discovery_time_weight(current_step, episode_steps),
            "new_discovered": len(new_discoveries),
            "coverage_count": c_t,
            "r_ov":           r_ov,
            "r_idle":         r_idle,
            "r_fov":          r_fov,
            "r_exp":          r_exp,
            "r_fov_overlap":  r_fov_overlap,
            "r_bound":        r_bound,
            "r_close":        r_close,
            "r_coll":         r_coll,
            "r_safe":         r_safe,
            "r_energy":       r_energy,
            "total_reward":   total_reward,
        }

        return total_reward, reward_info, new_discoveries
