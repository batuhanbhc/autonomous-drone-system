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
    world_to_drone_local,
)


class RewardCalculator:
    def __init__(
        self,
        x_min: float = -5.0,
        x_max: float = 5.0,
        y_min: float = -5.0,
        y_max: float = 5.0,
        wc: float = 1.0,      # pure coverage weight
        coverage_exponent: float = 1.5,
        wcoverage_contrib: float = 1.0,  # per-drone marginal contribution to team coverage
        wqual: float = 0.0,   # FOV quality weight over split-assigned people
        wd: float = 5.0,      # discovery bonus
        wo: float = 2.0,      # overlap penalty
        wx: float = 0.3,      # exploration bonus
        ws: float = 1.0,      # boundary penalty
        wclose: float = 0.0,  # proximity penalty before collision
        wfov_overlap: float = 0.0,  # current-step redundant-FOV penalty
        fov_overlap_threshold: float = 0.2,
        wcoll: float = 1.0,   # collision penalty
        we: float = 0.0,      # yaw-rate penalty (disabled)
        wi: float = 0.5,      # idle penalty — strong to force both drones near people
        wfov: float = 0.5,    # FOV boundary penalty — penalise looking outside area
        wcompletion: float = 0.0,  # search-phase completion-progress bonus
        coverage_edge_quality: float = 0.2,  # quality floor for worst visible framing
        reward_quality_mode: str = "principal_top_corner_linear",
        reward_quality_gamma: float = 1.5,
        reward_completion_power: float = 1.0,
        boundary_margin: float = 0.2,
        drone_closeness_margin: float = 1.0,
        tilt_deg: float = 45.0,
        horizontal_fov_deg: float = 60.0,
        vertical_fov_deg: float = 45.0,
        fov_margin: float = 1.0,  # soft margin for FOV boundary penalty
        search_phase_steps: int = 0,
    ):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

        self.wc    = wc
        self.coverage_exponent = float(coverage_exponent)
        self.wcoverage_contrib = wcoverage_contrib
        self.wqual = wqual
        self.wd    = wd
        self.wo    = wo
        self.wx    = wx
        self.ws    = ws
        self.wclose = wclose
        self.wfov_overlap = wfov_overlap
        self.fov_overlap_threshold = float(fov_overlap_threshold)
        self.wcoll = wcoll
        self.we    = we
        self.wi    = wi
        self.wfov  = wfov
        self.wcompletion = wcompletion
        self.coverage_edge_quality = coverage_edge_quality
        self.coverage_lateral_exponent = 2.0
        self.reward_quality_mode = str(reward_quality_mode)
        self.reward_quality_gamma = float(reward_quality_gamma)
        self.reward_completion_power = float(reward_completion_power)

        self.boundary_margin = boundary_margin
        self.drone_closeness_margin = drone_closeness_margin
        self.tilt_deg        = tilt_deg
        self.horizontal_fov_deg = horizontal_fov_deg
        self.vertical_fov_deg = vertical_fov_deg
        self.fov_margin      = fov_margin
        self.search_phase_steps = int(search_phase_steps)
        if self.drone_closeness_margin < 0.0:
            raise ValueError(
                "drone_closeness_margin must be >= 0, got "
                f"{self.drone_closeness_margin}"
            )
        if self.search_phase_steps < 0:
            raise ValueError(
                f"search_phase_steps must be >= 0, got {self.search_phase_steps}"
            )
        if not (0.0 <= self.coverage_edge_quality <= 1.0):
            raise ValueError(
                "coverage_edge_quality must be within [0, 1], got "
                f"{self.coverage_edge_quality}"
            )
        if self.coverage_exponent <= 0.0:
            raise ValueError(
                "coverage_exponent must be > 0, got "
                f"{self.coverage_exponent}"
            )
        if self.reward_quality_mode not in {
            "legacy",
            "principal_linear",
            "principal_squared",
            "principal_power",
            "principal_top_corner_linear",
        }:
            raise ValueError(
                "reward_quality_mode must be one of "
                "{'legacy', 'principal_linear', 'principal_squared', "
                "'principal_power', 'principal_top_corner_linear'}, got "
                f"{self.reward_quality_mode}"
            )
        if self.reward_quality_gamma <= 0.0:
            raise ValueError(
                "reward_quality_gamma must be > 0, got "
                f"{self.reward_quality_gamma}"
            )
        if self.reward_completion_power <= 0.0:
            raise ValueError(
                "reward_completion_power must be > 0, got "
                f"{self.reward_completion_power}"
            )
        if not (0.0 <= self.fov_overlap_threshold < 1.0):
            raise ValueError(
                "fov_overlap_threshold must be within [0, 1), got "
                f"{self.fov_overlap_threshold}"
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
            if self.reward_quality_mode == "principal_top_corner_linear":
                radial_norm = self._principal_top_corner_radial_norm_for_point(
                    drone_state=drone_state,
                    point=point,
                    eps=eps,
                )
                base_quality = (1.0 - radial_norm) ** self.reward_quality_gamma
            else:
                radial_norm = self._principal_radial_norm_for_point(
                    drone_state=drone_state,
                    point=point,
                    eps=eps,
                )
                if self.reward_quality_mode == "principal_squared":
                    base_quality = 1.0 - radial_norm ** 2
                elif self.reward_quality_mode == "principal_power":
                    base_quality = (1.0 - radial_norm) ** self.reward_quality_gamma
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
        radial_distance = math.hypot(px - principal_x, py - principal_y)
        if radial_distance <= eps:
            return 0.0

        principal_forward, _ = world_to_drone_local(
            principal_x,
            principal_y,
            x,
            y,
            yaw,
        )
        radius = lateral_half_width_at_forward_distance(
            forward=principal_forward,
            z=z,
            horizontal_fov_deg=self.horizontal_fov_deg,
        )
        if radius <= eps:
            return 1.0
        return min(max(radial_distance / radius, 0.0), 1.0)

    def _principal_top_corner_radial_norm_for_point(
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
        radial_distance = math.hypot(px - principal_x, py - principal_y)
        if radial_distance <= eps:
            return 0.0

        polygon = footprint_corners_world(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            camera_tilt_deg=self.tilt_deg,
            horizontal_fov_deg=self.horizontal_fov_deg,
            vertical_fov_deg=self.vertical_fov_deg,
        )
        top_corners = sorted(polygon, key=lambda corner: corner[1], reverse=True)[:2]
        max_radius = max(
            math.hypot(corner_x - principal_x, corner_y - principal_y)
            for corner_x, corner_y in top_corners
        )
        if max_radius <= eps:
            return 1.0
        return min(max(radial_distance / max_radius, 0.0), 1.0)

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

    def is_search_phase(self, current_step: int) -> bool:
        return self.search_phase_steps > 0 and current_step <= self.search_phase_steps

    def phase_reward_scales(self, current_step: int) -> Tuple[float, float, float]:
        if self.is_search_phase(current_step):
            return 0.0, 0.0, 1.0
        return 1.0, 1.0, 0.0

    # ------------------------------------------------------------------ #
    #  Coverage — quality-weighted unique coverage using camera-local
    #  near/far and lateral framing quality
    # ------------------------------------------------------------------ #

    def _coverage_stats_for_visible_union(
        self,
        visible_union: set,
        num_people: int,
        person_weights: Optional[List[float]] = None,
        eps: float = 1e-6,
    ) -> Tuple[float, float, float, int, float]:
        if num_people <= 0:
            return 0.0, 0.0, 0.0, 0, 0.0
        if person_weights is None:
            visible_weight = float(len(visible_union))
            total_weight = float(num_people)
            contributing_visible_count = len(visible_union)
        else:
            max_people = min(num_people, len(person_weights))
            contributing_visible_count = sum(
                1
                for person_id in visible_union
                if 0 <= person_id < max_people and float(person_weights[person_id]) > 0.0
            )
            visible_weight = sum(
                float(person_weights[person_id])
                for person_id in visible_union
                if 0 <= person_id < max_people
            )
            total_weight = max(
                sum(float(person_weights[idx]) for idx in range(max_people)),
                eps,
            )
        coverage_ratio = (visible_weight / (total_weight + eps))
        r_cov = coverage_ratio ** self.coverage_exponent
        return (
            r_cov,
            coverage_ratio,
            visible_weight,
            contributing_visible_count,
            total_weight,
        )

    def compute_coverage_reward(
        self,
        visible_ids_per_drone: List[List[int]],
        num_people: int,
        person_weights: Optional[List[float]] = None,
        eps: float = 1e-6,
    ) -> Tuple[float, float, int, set, float, int, float]:
        visible_union = set()
        for ids in visible_ids_per_drone:
            visible_union.update(ids)
        c_t = len(visible_union)
        (
            r_cov,
            coverage_ratio,
            visible_weight,
            contributing_visible_count,
            total_weight,
        ) = self._coverage_stats_for_visible_union(
            visible_union=visible_union,
            num_people=num_people,
            person_weights=person_weights,
            eps=eps,
        )
        return (
            r_cov,
            coverage_ratio,
            c_t,
            visible_union,
            visible_weight,
            contributing_visible_count,
            total_weight,
        )

    def compute_marginal_coverage_contributions(
        self,
        visible_ids_per_drone: List[List[int]],
        num_people: int,
        full_r_cov: float,
        full_coverage_ratio: float,
        person_weights: Optional[List[float]] = None,
        eps: float = 1e-6,
    ) -> Tuple[np.ndarray, np.ndarray]:
        active_drones = len(visible_ids_per_drone)
        per_agent_reward = np.zeros((active_drones,), dtype=np.float32)
        per_agent_ratio = np.zeros((active_drones,), dtype=np.float32)
        if num_people <= 0 or active_drones == 0:
            return per_agent_reward, per_agent_ratio

        for drone_idx in range(active_drones):
            visible_union_without_drone = set()
            for other_idx, ids in enumerate(visible_ids_per_drone):
                if other_idx == drone_idx:
                    continue
                visible_union_without_drone.update(ids)
            (
                r_cov_without_drone,
                coverage_ratio_without_drone,
                _visible_weight,
                _contributing_visible_count,
                _total_weight,
            ) = self._coverage_stats_for_visible_union(
                visible_union=visible_union_without_drone,
                num_people=num_people,
                person_weights=person_weights,
                eps=eps,
            )
            per_agent_reward[drone_idx] = max(0.0, full_r_cov - r_cov_without_drone)
            per_agent_ratio[drone_idx] = max(
                0.0,
                full_coverage_ratio - coverage_ratio_without_drone,
            )

        return per_agent_reward, per_agent_ratio

    def compute_fov_quality_rewards(
        self,
        visible_ids_per_drone: List[List[int]],
        drone_states: List[Dict],
        people_positions: List[Tuple[float, float, float]],
        num_people: int,
        person_weights: Optional[List[float]] = None,
        eps: float = 1e-6,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        active_drones = len(drone_states)
        per_agent_contribution_reward = np.zeros((active_drones,), dtype=np.float32)
        per_agent_quality_reward = np.zeros((active_drones,), dtype=np.float32)
        per_agent_mean_quality = np.zeros((active_drones,), dtype=np.float32)
        per_agent_split_visible_ratio = np.zeros((active_drones,), dtype=np.float32)
        per_agent_assigned_weight = np.zeros((active_drones,), dtype=np.float32)
        if num_people <= 0:
            return (
                per_agent_contribution_reward,
                per_agent_quality_reward,
                per_agent_mean_quality,
                per_agent_split_visible_ratio,
                per_agent_assigned_weight,
            )

        if person_weights is None:
            max_people = num_people
            total_weight = float(num_people)
        else:
            max_people = min(num_people, len(person_weights))
            total_weight = max(
                sum(float(person_weights[idx]) for idx in range(max_people)),
                eps,
            )
        total_weight = max(total_weight, eps)

        quality_numerators = np.zeros((active_drones,), dtype=np.float32)
        visible_union = set()
        for ids in visible_ids_per_drone:
            visible_union.update(ids)

        for person_id in visible_union:
            if person_id < 0 or person_id >= len(people_positions) or person_id >= max_people:
                continue
            person_weight = (
                1.0 if person_weights is None else float(person_weights[person_id])
            )
            if person_weight <= 0.0:
                continue
            px, py, _ = people_positions[person_id]
            visible_drones: list[tuple[int, float]] = []
            for drone_idx, ids in enumerate(visible_ids_per_drone[:active_drones]):
                if person_id not in ids:
                    continue
                quality = self.coverage_quality_for_point(
                    drone_state=drone_states[drone_idx],
                    point=(px, py),
                    eps=eps,
                )
                visible_drones.append((drone_idx, quality))
            if not visible_drones:
                continue

            quality_sum = sum(quality for _, quality in visible_drones)
            if quality_sum <= eps:
                split_scale = 1.0 / float(len(visible_drones))
                for drone_idx, _ in visible_drones:
                    assigned_weight = person_weight * split_scale
                    per_agent_assigned_weight[drone_idx] += assigned_weight
                    per_agent_split_visible_ratio[drone_idx] += (
                        split_scale / (num_people + eps)
                    )
                continue

            for drone_idx, quality in visible_drones:
                split_scale = quality / quality_sum
                assigned_weight = person_weight * split_scale
                per_agent_assigned_weight[drone_idx] += assigned_weight
                per_agent_split_visible_ratio[drone_idx] += (
                    split_scale / (num_people + eps)
                )
                quality_numerators[drone_idx] += assigned_weight * quality

        positive_assignment_mask = per_agent_assigned_weight > eps
        if np.any(positive_assignment_mask):
            per_agent_mean_quality[positive_assignment_mask] = (
                quality_numerators[positive_assignment_mask]
                / per_agent_assigned_weight[positive_assignment_mask]
            )
            per_agent_contribution_reward[positive_assignment_mask] = (
                per_agent_assigned_weight[positive_assignment_mask] / total_weight
            )
            # Keep framing quality distinct from coverage amount while still
            # preventing a single perfectly-centered person from dominating.
            per_agent_quality_reward[positive_assignment_mask] = (
                per_agent_mean_quality[positive_assignment_mask]
                * np.sqrt(per_agent_contribution_reward[positive_assignment_mask])
            )

        return (
            per_agent_contribution_reward,
            per_agent_quality_reward,
            per_agent_mean_quality,
            per_agent_split_visible_ratio,
            per_agent_assigned_weight,
        )

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

    def compute_boundary_penalties(self, drone_states: List[Dict]) -> np.ndarray:
        penalties = np.zeros((len(drone_states),), dtype=np.float32)
        for drone_idx, drone_state in enumerate(drone_states):
            x, y, _ = drone_state["position"]
            dist = min(
                x - self.x_min,
                self.x_max - x,
                y - self.y_min,
                self.y_max - y,
            )
            if dist < 0:
                # outside bounds: hard penalty proportional to overshoot
                penalties[drone_idx] = 1.0 + (-dist)
            elif dist < self.boundary_margin:
                # inside margin: linear ramp from 0 (at margin edge) to 1 (at wall)
                penalties[drone_idx] = 1.0 - dist / self.boundary_margin
        return penalties

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

    def compute_drone_closeness_penalties(
        self,
        drone_states: List[Dict],
    ) -> np.ndarray:
        penalties = np.zeros((len(drone_states),), dtype=np.float32)
        if self.drone_closeness_margin <= 0.0:
            return penalties
        for i in range(len(drone_states)):
            xi, yi, zi = drone_states[i]["position"]
            for j in range(len(drone_states)):
                if i == j:
                    continue
                xj, yj, zj = drone_states[j]["position"]
                dist = np.linalg.norm(
                    np.array([xi - xj, yi - yj, zi - zj], dtype=np.float32)
                )
                if dist < self.drone_closeness_margin:
                    penalties[i] += 1.0 - (dist / self.drone_closeness_margin)
        return penalties

    def compute_energy_penalty(
        self,
        actions: Optional[List[Tuple[float, float, float, float]]] = None,
    ) -> float:
        if actions is None or self.we == 0.0:
            return 0.0
        penalty = 0.0
        for action in actions:
            _, _, _, yaw_rate = action
            penalty += abs(yaw_rate)
        return penalty

    def compute_exploration_reward(
        self,
        coverage_map: np.ndarray,
        footprint_maps: Optional[np.ndarray],
    ) -> float:
        if (
            coverage_map is None
            or coverage_map.size == 0
            or footprint_maps is None
            or footprint_maps.size == 0
            or self.wx == 0.0
        ):
            return 0.0
        active_maps = np.asarray(footprint_maps, dtype=np.float32)
        if active_maps.ndim != 3:
            return 0.0
        weights = active_maps.sum(axis=0)
        total_weight = float(weights.sum())
        if total_weight <= 0.0:
            return 0.0

        # Binary exploration: cells already visited this episode have zero novelty,
        # unvisited cells have full novelty. Keep the old decaying path commented
        # out so it is easy to restore later.
        visited = (np.asarray(coverage_map, dtype=np.float32) > 0.0).astype(np.float32)
        novelty = 1.0 - visited

        # Old decaying exploration:
        # novelty = 1.0 - np.asarray(coverage_map, dtype=np.float32)
        return float((novelty * weights).sum() / total_weight)

    def compute_completion_reward(
        self,
        coverage_map_before_step: Optional[np.ndarray],
        coverage_map_after_step: Optional[np.ndarray],
    ) -> float:
        if (
            coverage_map_before_step is None
            or coverage_map_after_step is None
            or self.wcompletion == 0.0
        ):
            return 0.0
        before = np.asarray(coverage_map_before_step, dtype=np.float32)
        after = np.asarray(coverage_map_after_step, dtype=np.float32)
        if before.size == 0 or after.size == 0:
            return 0.0
        visited_before = float(np.mean(before > 0.0))
        visited_after = float(np.mean(after > 0.0))
        if visited_after <= visited_before:
            return 0.0
        power = self.reward_completion_power
        return (visited_after ** power) - (visited_before ** power)

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
        valid_drone_count = 0
        for i in range(active_maps.shape[0]):
            own_map = active_maps[i]
            own_area = float(own_map.sum())
            if own_area <= eps:
                continue
            teammate_indices = [j for j in range(active_maps.shape[0]) if j != i]
            if not teammate_indices:
                continue
            teammate_union = active_maps[teammate_indices].max(axis=0)
            redundant_area = float(np.minimum(own_map, teammate_union).sum())
            overlap_fraction = redundant_area / max(own_area, eps)
            thresholded_overlap = max(
                0.0,
                (overlap_fraction - self.fov_overlap_threshold)
                / max(1.0 - self.fov_overlap_threshold, eps),
            )
            penalty += thresholded_overlap
            valid_drone_count += 1

        if valid_drone_count == 0:
            return 0.0
        return penalty / valid_drone_count

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
        coverage_map_after_step: Optional[np.ndarray] = None,
        footprint_maps: Optional[np.ndarray] = None,
        person_weights: Optional[List[float]] = None,
    ) -> Tuple[float, dict, set]:

        (
            r_cov,
            coverage_ratio,
            c_t,
            visible_union,
            visible_weight,
            contributing_visible_count,
            total_weight,
        ) = self.compute_coverage_reward(
            visible_ids_per_drone,
            num_people,
            person_weights=person_weights,
        )
        (
            _per_agent_split_coverage_contrib,
            per_agent_r_fovq,
            per_agent_mean_visible_quality,
            per_agent_split_visible_ratio,
            per_agent_assigned_weight,
        ) = self.compute_fov_quality_rewards(
            visible_ids_per_drone,
            drone_states,
            people_positions,
            num_people,
            person_weights=person_weights,
        )
        (
            per_agent_r_cov_contrib,
            per_agent_marginal_coverage_ratio,
        ) = self.compute_marginal_coverage_contributions(
            visible_ids_per_drone=visible_ids_per_drone,
            num_people=num_people,
            full_r_cov=r_cov,
            full_coverage_ratio=coverage_ratio,
            person_weights=person_weights,
        )
        per_agent_visible_ratio = per_agent_split_visible_ratio.copy()
        mean_visible_quality = float(per_agent_mean_visible_quality.mean()) if len(
            per_agent_mean_visible_quality
        ) > 0 else 0.0
        r_fovq = float(per_agent_r_fovq.mean()) if len(per_agent_r_fovq) > 0 else 0.0
        r_cov_contrib = float(per_agent_r_cov_contrib.mean()) if len(
            per_agent_r_cov_contrib
        ) > 0 else 0.0

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
        r_exp    = self.compute_exploration_reward(coverage_map, footprint_maps)
        r_completion = self.compute_completion_reward(coverage_map, coverage_map_after_step)
        r_fov_overlap = self.compute_fov_overlap_penalty(footprint_maps)
        per_agent_r_bound = self.compute_boundary_penalties(drone_states)
        per_agent_r_close = self.compute_drone_closeness_penalties(drone_states)
        r_bound = float(per_agent_r_bound.sum())
        r_close = float(per_agent_r_close.sum())
        r_coll   = self.compute_collision_penalty(drone_states)
        r_safe   = r_bound + r_close + r_coll
        r_energy = self.compute_energy_penalty(actions)

        cov_scale, fovq_scale, exp_scale = self.phase_reward_scales(current_step)
        is_search_phase = float(self.is_search_phase(current_step))
        is_coverage_phase = 1.0 - is_search_phase
        # Keep discovery reward active for the full episode.
        discovery_scale = 1.0
        energy_scale = is_coverage_phase
        overlap_scale = is_coverage_phase
        fov_overlap_scale = is_coverage_phase

        shared_reward = (
            (self.wc * cov_scale) * r_cov
            + (self.wd * discovery_scale) * r_disc
            + (self.wx * exp_scale) * r_exp
            + (self.wcompletion * exp_scale) * r_completion
            - (self.wo * overlap_scale) * r_ov
            - self.wi  * r_idle
            - self.wfov * r_fov
            - (self.wfov_overlap * fov_overlap_scale) * r_fov_overlap
            - self.wcoll * r_coll
            - (self.we * energy_scale) * r_energy
        )
        per_agent_total_reward = (
            shared_reward
            + (self.wcoverage_contrib * cov_scale) * per_agent_r_cov_contrib
            + (self.wqual * fovq_scale) * per_agent_r_fovq
            - self.ws * per_agent_r_bound
            - self.wclose * per_agent_r_close
        ).astype(np.float32)
        total_reward = float(per_agent_total_reward.mean()) if len(per_agent_total_reward) > 0 else float(shared_reward)

        reward_info = {
            "r_cov":          r_cov,
            "coverage_ratio": coverage_ratio,
            "r_fovq":         r_fovq,
            "r_cov_contrib":  r_cov_contrib,
            "mean_visible_quality": mean_visible_quality,
            "per_agent_r_cov_contrib": per_agent_r_cov_contrib.tolist(),
            "per_agent_r_fovq": per_agent_r_fovq.tolist(),
            "per_agent_mean_visible_quality": per_agent_mean_visible_quality.tolist(),
            "per_agent_visible_ratio": per_agent_visible_ratio.tolist(),
            "per_agent_marginal_coverage_ratio": (
                per_agent_marginal_coverage_ratio.tolist()
            ),
            "per_agent_split_visible_ratio": per_agent_split_visible_ratio.tolist(),
            "per_agent_assigned_weight": per_agent_assigned_weight.tolist(),
            "r_disc":         r_disc,
            "discovery_time_weight": self._discovery_time_weight(current_step, episode_steps),
            "new_discovered": len(new_discoveries),
            "coverage_count": c_t,
            "coverage_contributing_count": contributing_visible_count,
            "coverage_weight": visible_weight,
            "coverage_total_weight": total_weight,
            "r_ov":           r_ov,
            "r_idle":         r_idle,
            "r_fov":          r_fov,
            "r_exp":          r_exp,
            "r_completion":   r_completion,
            "r_fov_overlap":  r_fov_overlap,
            "r_bound":        r_bound,
            "per_agent_r_bound": per_agent_r_bound.tolist(),
            "r_close":        r_close,
            "per_agent_r_close": per_agent_r_close.tolist(),
            "r_coll":         r_coll,
            "r_safe":         r_safe,
            "r_energy":       r_energy,
            "coverage_reward_scale": cov_scale,
            "fov_quality_reward_scale": fovq_scale,
            "discovery_reward_scale": discovery_scale,
            "overlap_penalty_scale": overlap_scale,
            "energy_penalty_scale": energy_scale,
            "exploration_reward_scale": exp_scale,
            "fov_overlap_penalty_scale": fov_overlap_scale,
            "is_search_phase": is_search_phase,
            "is_coverage_phase": is_coverage_phase,
            "shared_reward": shared_reward,
            "per_agent_total_reward": per_agent_total_reward.tolist(),
            "total_reward":   total_reward,
        }

        return total_reward, reward_info, new_discoveries
