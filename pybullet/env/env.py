import math
import random
from typing import List, Tuple

import pybullet as p

from sim.pybullet_world import PyBulletWorld
from sim.drone import Drone
from sim.person import Person
from sim.debug_draw import DebugDrawer
from env.observation import ObservationBuilder
from env.reward import RewardCalculator


class MultiUAVEnv:
    def __init__(
        self,
        gui: bool = False,
        sim_hz: float = 4.0,
        x_min: float = -15.0,
        x_max: float = 15.0,
        y_min: float = -15.0,
        y_max: float = 15.0,
        z_min: float = 0.3,
        z_max: float = 8.0,
        drone_height: float = 6.0,
        num_drones: int = 2,
        random_spawn: bool = False,
        drone_spawn_radius: float = 3.0,
        fixed_active_num_drones: int | None = None,
        num_people: int = 20,
        min_people: int = 10,
        max_people: int = 30,
        episode_steps: int = 480,
        search_phase_seconds: float = 20.0,
        max_groups: int = 4,
        num_group_regions: int = 4,
        drone_wall_margin: float = 0.0,
        person_spawn_margin: float = 0.5,
        group_spawn_margin: float = 0.3,
        min_person_spawn_dist: float = 0.35,
        group_center_speed_min: float = 0.05,
        group_center_speed_max: float = 0.25,
        group_center_turn_prob: float = 0.25,
        group_center_turn_std: float = 0.4,
        max_range: float = 15.0,
        horizontal_fov_deg: float = 55.8,
        vertical_fov_deg: float = 43.3,
        detection_prob: float = 1.0,
        position_noise_std: float = 0.1,
        camera_tilt_deg: float = 45.0,
        recent_half_life_seconds: float = 10.0,
        historic_half_life_seconds: float = 30.0,
        coverage_half_life_seconds: float = 10.0,
        recent_hit_gain: float = 0.6,
        recent_miss_penalty: float = 0.25,
        grid_h: int = 60,
        grid_w: int = 60,
        blob_sigma: float = 1.0,
        ego_sigma: float = 2.5,
        people_count_normalizer: float = 30.0,
        count_density_gain: float = 0.35,
        count_memory_recent_alpha: float = 0.7,
        count_memory_historic_miss_penalty: float = 0.1,
        reward_wc: float = 1.0,
        reward_wqual: float = 2.0,
        reward_wd: float = 0.0,
        reward_wo: float = 0.0,
        reward_wx: float = 1.0,
        reward_ws: float = 1.0,
        reward_wclose: float = 1.0,
        reward_wfov_overlap: float = 3.0,
        reward_wcoll: float = 0.0,
        reward_we: float = 0.0,
        reward_wi: float = 0.0,
        reward_wfov: float = 0.0,
        reward_coverage_edge_quality: float = 0.0,
        reward_quality_mode: str = "principal_linear",
        reward_boundary_margin: float = 5.0,
        reward_drone_closeness_margin: float = 5.0,
        reward_fov_margin: float = 1.0,
        vel_tau_s: float = 0.4,
        yaw_rate_tau_s: float = 0.4,
        cmd_history_len: int = 4,
        hotspot_top_k: int = 3,
        hotspot_min_density: float = 0.25,
        hotspot_suppression_radius_scale: float = 4.0,
        hotspot_suppression_radius_min_cells: int = 2,
        max_horizontal_velocity: float = 1.0,
        max_yaw_rate: float = 0.7,
        actor_grid_channels: int = 8,
        include_persistent_coverage_channel: bool = True,
        debug_observation_plots: bool = False,
        debug_observation_plot_every: int = 25,
        debug_reward_contours: bool = False,
    ):
        self.gui        = gui
        self.sim_hz     = sim_hz
        self.vel_tau_s         = float(vel_tau_s)
        self.yaw_rate_tau_s    = float(yaw_rate_tau_s)
        self.cmd_history_len   = int(cmd_history_len)
        self.max_horizontal_velocity = float(max_horizontal_velocity)
        self.max_yaw_rate      = float(max_yaw_rate)

        # dt is the simulated time between two consecutive control decisions.
        # Example: sim_hz=10 → dt=0.1 s, so a 1.0 m/s velocity command moves
        # the drone by 0.1 m in that decision step.
        self.dt = 1.0 / sim_hz

        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.drone_wall_margin = drone_wall_margin
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        if not (self.z_min <= drone_height <= self.z_max):
            raise ValueError(
                f"drone_height must be within [{self.z_min}, {self.z_max}], got {drone_height}"
            )
        self.drone_height = drone_height
        self.move_x_min = self.x_min + self.drone_wall_margin
        self.move_x_max = self.x_max - self.drone_wall_margin
        self.move_y_min = self.y_min + self.drone_wall_margin
        self.move_y_max = self.y_max - self.drone_wall_margin
        self.center_x = 0.5 * (self.x_min + self.x_max)
        self.center_y = 0.5 * (self.y_min + self.y_max)
        if self.move_x_min > self.move_x_max or self.move_y_min > self.move_y_max:
            raise ValueError(
                "drone_wall_margin leaves no movable arena: "
                f"x=[{self.move_x_min}, {self.move_x_max}], "
                f"y=[{self.move_y_min}, {self.move_y_max}]"
            )
        self.max_center_spawn_radius = min(
            self.center_x - self.move_x_min,
            self.move_x_max - self.center_x,
            self.center_y - self.move_y_min,
            self.move_y_max - self.center_y,
        )
        self.drone_spawn_radius = float(drone_spawn_radius)
        if self.drone_spawn_radius < 0.0:
            raise ValueError(
                f"drone_spawn_radius must be >= 0, got {self.drone_spawn_radius}"
            )
        if self.drone_spawn_radius > self.max_center_spawn_radius:
            raise ValueError(
                "drone_spawn_radius exceeds the largest center-circle that fits in "
                "the movable arena: "
                f"requested={self.drone_spawn_radius}, "
                f"max={self.max_center_spawn_radius}"
            )

        self.num_drones    = num_drones
        self.max_drones    = num_drones
        self.active_num_drones = num_drones
        self.random_spawn = bool(random_spawn)
        self.fixed_active_num_drones = fixed_active_num_drones
        if self.fixed_active_num_drones is not None:
            if not (1 <= self.fixed_active_num_drones <= self.max_drones):
                raise ValueError(
                    "fixed_active_num_drones must be within "
                    f"[1, {self.max_drones}], got {self.fixed_active_num_drones}"
                )
        self.num_people    = num_people
        self.min_people    = min_people
        self.max_people    = max_people
        self.episode_steps = episode_steps
        self.search_phase_seconds = float(search_phase_seconds)
        if self.search_phase_seconds < 0.0:
            raise ValueError(
                f"search_phase_seconds must be >= 0, got {self.search_phase_seconds}"
            )
        self.search_phase_steps = min(
            self.episode_steps,
            int(math.ceil(self.search_phase_seconds * self.sim_hz - 1e-9)),
        ) if self.search_phase_seconds > 0.0 else 0

        self.max_groups        = max_groups
        self.num_group_regions = num_group_regions
        
        self.world = PyBulletWorld(gui=self.gui, time_step=self.dt)

        self.drones: List[Drone]  = []
        self.people: List[Person] = []

        self.obs_builder = ObservationBuilder(
            x_min=self.x_min,
            x_max=self.x_max,
            y_min=self.y_min,
            y_max=self.y_max,
            grid_h=grid_h,
            grid_w=grid_w,
            max_range=max_range,
            horizontal_fov_deg=horizontal_fov_deg,
            vertical_fov_deg=vertical_fov_deg,
            detection_prob=detection_prob,
            position_noise_std=position_noise_std,
            camera_tilt_deg=camera_tilt_deg,
            num_drones=self.max_drones,
            sim_hz=self.sim_hz,
            blob_sigma=blob_sigma,
            ego_sigma=ego_sigma,
            recent_half_life_seconds=recent_half_life_seconds,
            historic_half_life_seconds=historic_half_life_seconds,
            coverage_half_life_seconds=coverage_half_life_seconds,
            recent_hit_gain=recent_hit_gain,
            recent_miss_penalty=recent_miss_penalty,
            z_min=self.z_min,
            z_max=self.z_max,
            cmd_history_len=self.cmd_history_len,
            hotspot_top_k=hotspot_top_k,
            hotspot_min_density=hotspot_min_density,
            hotspot_suppression_radius_scale=hotspot_suppression_radius_scale,
            hotspot_suppression_radius_min_cells=hotspot_suppression_radius_min_cells,
            people_count_normalizer=people_count_normalizer,
            count_density_gain=count_density_gain,
            count_memory_recent_alpha=count_memory_recent_alpha,
            count_memory_historic_miss_penalty=count_memory_historic_miss_penalty,
            max_horizontal_velocity=self.max_horizontal_velocity,
            max_yaw_rate=self.max_yaw_rate,
            actor_grid_channels=actor_grid_channels,
            include_persistent_coverage_channel=include_persistent_coverage_channel,
        )

        self.reward_calc = RewardCalculator(
            x_min=self.x_min,
            x_max=self.x_max,
            y_min=self.y_min,
            y_max=self.y_max,
            wc=reward_wc,
            wqual=reward_wqual,
            wd=reward_wd,
            wo=reward_wo,
            wx=reward_wx,
            ws=reward_ws,
            wclose=reward_wclose,
            wfov_overlap=reward_wfov_overlap,
            wcoll=reward_wcoll,
            we=reward_we,
            wi=reward_wi,
            wfov=reward_wfov,
            coverage_edge_quality=reward_coverage_edge_quality,
            reward_quality_mode=reward_quality_mode,
            boundary_margin=reward_boundary_margin,
            drone_closeness_margin=reward_drone_closeness_margin,
            tilt_deg=camera_tilt_deg,
            horizontal_fov_deg=horizontal_fov_deg,
            vertical_fov_deg=vertical_fov_deg,
            fov_margin=reward_fov_margin,
            search_phase_steps=self.search_phase_steps,
        )

        self.debug_drawer = DebugDrawer(
            tilt_deg=camera_tilt_deg,
            horizontal_fov_deg=horizontal_fov_deg,
            vertical_fov_deg=vertical_fov_deg,
            show_reward_contours=debug_reward_contours,
            coverage_edge_quality=reward_coverage_edge_quality,
        )
        self.debug_observation_plots = bool(debug_observation_plots)
        self.debug_observation_plot_every = max(1, int(debug_observation_plot_every))
        self._debug_obs_plot = None

        self.current_step = 0
        self.ever_seen: set = set()

        self.episode_group_info = {
            "num_groups": 0,
            "group_centers": [],
            "group_assignments": [],
            "group_sizes": {},
        }

        self.person_spawn_margin   = person_spawn_margin
        self.group_spawn_margin    = group_spawn_margin
        self.min_person_spawn_dist = min_person_spawn_dist
        self.group_center_speed_min = float(group_center_speed_min)
        self.group_center_speed_max = float(group_center_speed_max)
        self.group_center_turn_prob = float(group_center_turn_prob)
        self.group_center_turn_std = float(group_center_turn_std)
        if self.group_center_speed_min < 0.0 or self.group_center_speed_max < 0.0:
            raise ValueError("group_center speeds must be >= 0")
        if self.group_center_speed_min > self.group_center_speed_max:
            raise ValueError(
                "group_center_speed_min must be <= group_center_speed_max, got "
                f"{self.group_center_speed_min} > {self.group_center_speed_max}"
            )
        if self.group_center_turn_prob < 0.0:
            raise ValueError(
                f"group_center_turn_prob must be >= 0, got {self.group_center_turn_prob}"
            )
        if self.group_center_turn_std < 0.0:
            raise ValueError(
                f"group_center_turn_std must be >= 0, got {self.group_center_turn_std}"
            )
        self.group_center_motion_margin = max(self.group_spawn_margin + 1.0, 1.2)
        self.group_motion_state: dict[int, dict[str, float]] = {}

        if self.drone_wall_margin < 0.0:
            raise ValueError(
                f"drone_wall_margin must be >= 0, got {self.drone_wall_margin}"
            )

    # ------------------------------------------------------------------ #
    #  Spawn helpers
    # ------------------------------------------------------------------ #

    def _random_drone_pos(self):
        x = random.uniform(self.move_x_min, self.move_x_max)
        y = random.uniform(self.move_y_min, self.move_y_max)
        return (x, y, self.drone_height)

    def _center_circle_drone_pos(self):
        theta = random.uniform(0.0, 2.0 * math.pi)
        radius = self.drone_spawn_radius * math.sqrt(random.random())
        x = self.center_x + radius * math.cos(theta)
        y = self.center_y + radius * math.sin(theta)
        return (x, y, self.drone_height)

    def _random_person_pos(self):
        x = random.uniform(self.x_min + self.person_spawn_margin,
                           self.x_max - self.person_spawn_margin)
        y = random.uniform(self.y_min + self.person_spawn_margin,
                           self.y_max - self.person_spawn_margin)
        return (x, y, 0.35)

    def _sample_num_people(self):
        if self.min_people == self.max_people:
            return self.min_people
        return random.randint(self.min_people, self.max_people)

    def _build_candidate_regions(self):
        if self.num_group_regions == 4:
            x_mid = 0.5 * (self.x_min + self.x_max)
            y_mid = 0.5 * (self.y_min + self.y_max)
            return [
                (self.x_min, x_mid, self.y_min, y_mid),
                (x_mid, self.x_max, self.y_min, y_mid),
                (self.x_min, x_mid, y_mid, self.y_max),
                (x_mid, self.x_max, y_mid, self.y_max),
            ]
        regions = []
        width = (self.x_max - self.x_min) / self.num_group_regions
        for i in range(self.num_group_regions):
            rx0 = self.x_min + i * width
            regions.append((rx0, rx0 + width, self.y_min, self.y_max))
        return regions

    def _sample_point_in_region(self, region, margin=0.8, z=0.35):
        rx0, rx1, ry0, ry1 = region
        x = random.uniform(rx0 + margin, rx1 - margin)
        y = random.uniform(ry0 + margin, ry1 - margin)
        return (x, y, z)

    def _is_far_enough_from_existing(self, candidate_xy, existing_positions, min_dist):
        cx, cy = candidate_xy
        for ex, ey, _ in existing_positions:
            if math.hypot(cx - ex, cy - ey) < min_dist:
                return False
        return True

    def _sample_nonoverlapping_pos(self, existing_positions, region=None,
                                   min_dist=None, max_tries=80, z=0.35):
        if min_dist is None:
            min_dist = self.min_person_spawn_dist
        for _ in range(max_tries):
            if region is None:
                px, py, _ = self._random_person_pos()
            else:
                px, py, _ = self._sample_point_in_region(region, margin=0.5, z=z)
            if self._is_far_enough_from_existing((px, py), existing_positions, min_dist):
                return (px, py, z)
        return (self._random_person_pos() if region is None
                else self._sample_point_in_region(region, margin=0.5, z=z))

    def _sample_group_member_pos(self, group_center, group_radius,
                                  existing_positions, min_dist=None,
                                  max_tries=80, z=0.35):
        if min_dist is None:
            min_dist = self.min_person_spawn_dist
        gcx, gcy = group_center
        for _ in range(max_tries):
            angle = random.uniform(-math.pi, math.pi)
            r     = random.uniform(0.0, group_radius)
            px    = gcx + r * math.cos(angle)
            py    = gcy + r * math.sin(angle)
            px    = max(self.x_min + self.group_spawn_margin,
                        min(self.x_max - self.group_spawn_margin, px))
            py    = max(self.y_min + self.group_spawn_margin,
                        min(self.y_max - self.group_spawn_margin, py))
            if self._is_far_enough_from_existing((px, py), existing_positions, min_dist):
                return (px, py, z)
        return (gcx, gcy, z)

    def _sample_group_structure(self, num_people: int):
        max_possible_groups = min(self.max_groups, num_people)
        if max_possible_groups <= 0:
            num_groups = 0
        else:
            num_groups = random.randint(1, max_possible_groups)
        group_centers   = []
        group_sizes     = {}
        assignments     = [None] * num_people

        if num_groups > 0:
            regions          = self._build_candidate_regions()
            selected_regions = random.sample(regions, min(num_groups, len(regions)))

            for g_id, region in enumerate(selected_regions):
                center = self._sample_point_in_region(region, margin=0.8)
                group_centers.append((center[0], center[1]))

            people_indices = list(range(num_people))
            random.shuffle(people_indices)

            min_group_size = max(2, num_people // (num_groups * 3))
            max_group_size = max(min_group_size + 1, num_people // num_groups + 2)

            for g_id in range(num_groups):
                size = random.randint(min_group_size, max_group_size)
                group_sizes[g_id] = size

            assigned = 0
            for g_id in range(num_groups):
                for _ in range(group_sizes[g_id]):
                    if assigned < num_people:
                        assignments[people_indices[assigned]] = g_id
                        assigned += 1

            # Assign a portion of remaining people to groups; leave the rest standalone
            remaining = num_people - assigned
            num_to_group = int(remaining * 0.75)
            for i in range(num_to_group):
                assignments[people_indices[assigned + i]] = i % num_groups

        return {
            "num_groups":        num_groups,
            "group_centers":     group_centers,
            "group_assignments": assignments,
            "group_sizes":       group_sizes,
        }

    def _spawn_entities(self):
        self.drones = []
        self.people = []
        if self.fixed_active_num_drones is not None:
            self.active_num_drones = self.fixed_active_num_drones
        else:
            self.active_num_drones = random.randint(1, self.max_drones)

        for drone_idx in range(self.active_num_drones):
            if self.random_spawn:
                pos = self._random_drone_pos()
            else:
                pos = self._center_circle_drone_pos()
            drone = Drone(
                start_pos=pos,
                size=(0.08, 0.08, 0.03),
                vel_tau_s=self.vel_tau_s,
                yaw_rate_tau_s=self.yaw_rate_tau_s,
            )
            drone.spawn()
            yaw = random.uniform(-math.pi, math.pi)
            drone.reset(pos=pos, yaw=yaw)
            self.drones.append(drone)

        current_num_people    = self._sample_num_people()
        self.episode_group_info = self._sample_group_structure(current_num_people)
        assignments           = self.episode_group_info["group_assignments"]
        group_centers         = self.episode_group_info["group_centers"]
        self._initialize_group_motion(group_centers)
        spawned_positions     = []

        for person_idx in range(current_num_people):
            person   = Person(start_pos=self._random_person_pos())
            person.spawn()
            group_id = assignments[person_idx]
            if group_id is not None:
                gcx, gcy     = group_centers[group_id]
                group_radius = random.uniform(0.4, 0.9)
                pos = self._sample_group_member_pos(
                    group_center=(gcx, gcy),
                    group_radius=group_radius,
                    existing_positions=spawned_positions,
                    min_dist=self.min_person_spawn_dist,
                    z=0.35,
                )
                person.reset(pos=pos, grouped=True, group_id=group_id,
                             group_center=(gcx, gcy), group_radius=group_radius)
            else:
                pos = self._sample_nonoverlapping_pos(
                    existing_positions=spawned_positions,
                    min_dist=self.min_person_spawn_dist,
                    z=0.35,
                )
                person.reset(pos=pos, grouped=False)
            spawned_positions.append(pos)
            self.people.append(person)

        self._sync_people_group_centers()

    # ------------------------------------------------------------------ #

    def _initialize_group_motion(self, group_centers):
        self.group_motion_state = {}
        for group_id, center in enumerate(group_centers):
            cx, cy = center
            self.group_motion_state[group_id] = {
                "center_x": float(cx),
                "center_y": float(cy),
                "heading": random.uniform(-math.pi, math.pi),
                "speed": random.uniform(
                    self.group_center_speed_min,
                    self.group_center_speed_max,
                ),
            }
        self._update_group_centers_snapshot()

    def _update_group_centers_snapshot(self):
        num_groups = int(self.episode_group_info.get("num_groups", 0))
        self.episode_group_info["group_centers"] = [
            (
                self.group_motion_state[group_id]["center_x"],
                self.group_motion_state[group_id]["center_y"],
            )
            for group_id in range(num_groups)
            if group_id in self.group_motion_state
        ]

    def _sync_people_group_centers(self):
        if not self.group_motion_state:
            return
        for person in self.people:
            if not person.is_grouped or person.group_id is None:
                continue
            motion = self.group_motion_state.get(person.group_id)
            if motion is None:
                continue
            person.group_center = (motion["center_x"], motion["center_y"])

    def _step_group_centers(self):
        if not self.group_motion_state:
            return

        x_min = self.x_min + self.group_center_motion_margin
        x_max = self.x_max - self.group_center_motion_margin
        y_min = self.y_min + self.group_center_motion_margin
        y_max = self.y_max - self.group_center_motion_margin
        if x_min > x_max:
            x_min = x_max = self.center_x
        if y_min > y_max:
            y_min = y_max = self.center_y

        turn_prob_this_step = self.group_center_turn_prob * self.dt
        for motion in self.group_motion_state.values():
            if self.group_center_turn_std > 0.0 and random.random() < turn_prob_this_step:
                motion["heading"] += random.gauss(0.0, self.group_center_turn_std)
            if self.group_center_speed_max > 0.0:
                motion["speed"] = min(
                    self.group_center_speed_max,
                    max(
                        self.group_center_speed_min,
                        motion["speed"] + random.gauss(0.0, 0.01),
                    ),
                )

            trial_x = motion["center_x"] + motion["speed"] * self.dt * math.cos(motion["heading"])
            trial_y = motion["center_y"] + motion["speed"] * self.dt * math.sin(motion["heading"])

            if trial_x < x_min or trial_x > x_max:
                motion["heading"] = math.pi - motion["heading"]
                trial_x = motion["center_x"] + motion["speed"] * self.dt * math.cos(motion["heading"])
            if trial_y < y_min or trial_y > y_max:
                motion["heading"] = -motion["heading"]
                trial_y = motion["center_y"] + motion["speed"] * self.dt * math.sin(motion["heading"])

            motion["heading"] = (motion["heading"] + math.pi) % (2.0 * math.pi) - math.pi
            motion["center_x"] = max(x_min, min(x_max, trial_x))
            motion["center_y"] = max(y_min, min(y_max, trial_y))

        self._update_group_centers_snapshot()
        self._sync_people_group_centers()

    def _get_drone_states(self):
        return [drone.get_state_dict() for drone in self.drones]

    def _get_people_positions(self):
        return [person.get_position() for person in self.people if person is not None]

    def _build_person_reward_weights(self, num_people: int) -> list[float]:
        assignments = list(self.episode_group_info.get("group_assignments", []))
        if num_people <= 0:
            return []
        if len(assignments) < num_people:
            assignments.extend([None] * (num_people - len(assignments)))
        assignments = assignments[:num_people]

        group_counts: dict[int, int] = {}
        for group_id in assignments:
            if group_id is None:
                continue
            group_counts[int(group_id)] = group_counts.get(int(group_id), 0) + 1

        weights: list[float] = []
        for group_id in assignments:
            if group_id is None:
                weights.append(1.0)
            else:
                weights.append(float(group_counts.get(int(group_id), 1)))
        return weights

    def _phase_context_for_decision_step(self, decision_step: int) -> dict[str, float]:
        if self.search_phase_steps <= 0:
            return {
                "search_phase_progress": 1.0,
                "is_search_phase": 0.0,
                "is_coverage_phase": 1.0,
            }

        clamped_decision_step = max(1, int(decision_step))
        completed_search_steps = min(
            max(clamped_decision_step - 1, 0),
            self.search_phase_steps,
        )
        return {
            "search_phase_progress": completed_search_steps / self.search_phase_steps,
            "is_search_phase": float(clamped_decision_step <= self.search_phase_steps),
            "is_coverage_phase": float(clamped_decision_step > self.search_phase_steps),
        }

    def get_observation_phase_context(self) -> dict[str, float]:
        next_decision_step = self.current_step + 1
        return self._phase_context_for_decision_step(next_decision_step)

    def _update_debug_draw(self, drone_states, visible_ids_per_drone):
        if not self.gui or not p.isConnected():
            return
        self.debug_drawer.clear()
        visible_union = set()
        for visible_ids in visible_ids_per_drone:
            visible_union.update(visible_ids)
        for i in range(len(self.drones)):
            self.debug_drawer.draw_camera_footprint(
                drone_states[i],
                drone_id=i,
                reward_calc=self.reward_calc,
            )
        self.debug_drawer.draw_detections(self.people, visible_union)

    def _close_debug_observation_plot(self):
        if self._debug_obs_plot is None:
            return
        plt = self._debug_obs_plot["plt"]
        fig = self._debug_obs_plot["fig"]
        if plt.fignum_exists(fig.number):
            plt.close(fig)
        self._debug_obs_plot = None

    def _get_or_create_debug_observation_plot(self, actor_channel_names, critic_channel_names):
        import matplotlib.pyplot as plt

        if self._debug_obs_plot is not None:
            fig = self._debug_obs_plot["fig"]
            if plt.fignum_exists(fig.number):
                return self._debug_obs_plot
            self._debug_obs_plot = None

        num_cols = max(len(actor_channel_names), len(critic_channel_names))
        fig, axes = plt.subplots(2, num_cols, figsize=(num_cols * 3.6, 8.0), squeeze=False)
        images = []

        def init_row(row_idx, channel_names, row_label):
            row_images = []
            for col, title in enumerate(channel_names):
                ax = axes[row_idx][col]
                im = ax.imshow(
                    [[0.0]],
                    origin="lower",
                    cmap="hot",
                    vmin=0.0,
                    vmax=1.0,
                    interpolation="nearest",
                )
                fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
                ax.set_title(title, fontsize=8)
                ax.set_xlabel("grid-x")
                ax.set_ylabel("grid-y")
                row_images.append(im)
            for col in range(len(channel_names), num_cols):
                axes[row_idx][col].axis("off")
            axes[row_idx][0].annotate(
                row_label,
                xy=(0, 0.5), xycoords="axes fraction",
                xytext=(-60, 0), textcoords="offset points",
                fontsize=10, fontweight="bold", va="center", ha="right", rotation=90,
            )
            images.append(row_images)

        init_row(0, actor_channel_names, "Actor - Drone 0")
        init_row(1, critic_channel_names, "Critic")

        plt.show(block=False)
        self._debug_obs_plot = {
            "plt": plt,
            "fig": fig,
            "images": images,
        }
        return self._debug_obs_plot

    def _maybe_plot_observation_debug(self, observations, visible_ids_per_drone, people_positions):
        if not self.debug_observation_plots or not observations:
            return
        if self.current_step % self.debug_observation_plot_every != 0:
            return

        import matplotlib.pyplot as plt
        import numpy as np

        from rl.state_utils import build_global_state

        phase_context = self.get_observation_phase_context()
        global_state = build_global_state(
            observations,
            max_agents=self.max_drones,
            obs_builder=self.obs_builder,
            num_people=len(people_positions),
            ever_seen=len(self.ever_seen),
            visible_count=self.last_visible_count,
            current_step=self.current_step,
            episode_steps=self.episode_steps,
            search_phase_progress=phase_context["search_phase_progress"],
            is_search_phase=phase_context["is_search_phase"],
            is_coverage_phase=phase_context["is_coverage_phase"],
        )
        critic_grid = global_state["grid"]
        critic_poses = global_state["poses"]

        actor_obs = observations[0]
        actor_grid = actor_obs["grid"]
        actor_local = actor_obs["local"]
        actor_visible = len(visible_ids_per_drone[0]) if visible_ids_per_drone else 0

        actor_channel_names = [
            f"Actor Ch {idx}\n{name}"
            for idx, name in enumerate(self.obs_builder.actor_channel_names)
        ]
        critic_channel_names = [
            f"Critic Ch {idx}\n{name}"
            for idx, name in enumerate(self.obs_builder.critic_shared_channel_names)
        ]
        critic_shared_channels = len(self.obs_builder.critic_shared_channel_names)
        critic_channel_names += [
            f"Critic Ch {critic_shared_channels + i}\nInstant map - Drone {i}"
            for i in range(self.max_drones)
        ]
        critic_channel_names += [
            f"Critic Ch {critic_shared_channels + self.max_drones + i}\nOwn coverage - Drone {i}"
            for i in range(self.max_drones)
        ]
        critic_channel_names += [
            f"Critic Ch {critic_shared_channels + (2 * self.max_drones) + i}\nEgo map - Drone {i}"
            for i in range(self.max_drones)
        ]

        plot_state = self._get_or_create_debug_observation_plot(
            actor_channel_names=actor_channel_names,
            critic_channel_names=critic_channel_names,
        )
        fig = plot_state["fig"]
        fig.suptitle(
            f"Observation debug | step={self.current_step} | active_drones={len(observations)}"
            f" | actor_visible={actor_visible} | ever_seen={len(self.ever_seen)}\n"
            f"Actor local: {np.array2string(actor_local, precision=3, suppress_small=True)}\n"
            f"Critic poses: {np.array2string(critic_poses, precision=3, suppress_small=True)}",
            fontsize=10,
        )

        for col, mat in enumerate(actor_grid):
            plot_state["images"][0][col].set_data(mat)

        for col, mat in enumerate(critic_grid):
            plot_state["images"][1][col].set_data(mat)

        plt.tight_layout(rect=[0.04, 0, 1, 0.92])
        fig.canvas.draw_idle()
        plt.pause(0.001)

    # ------------------------------------------------------------------ #

    def reset(self):
        if not p.isConnected():
            self.world.connect()

        self.debug_drawer.clear()
        self.world.reset()
        self.world.add_boundary_walls(
            x_min=self.x_min, x_max=self.x_max,
            y_min=self.y_min, y_max=self.y_max,
        )

        self.obs_builder.reset()
        self.current_step = 0
        self.ever_seen    = set()
        self.last_visible_count = 0

        self._spawn_entities()

        drone_states     = self._get_drone_states()
        people_positions = self._get_people_positions()

        observations, visible_ids_per_drone, _ = self.obs_builder.build_observations(
            drone_states,
            people_positions,
            phase_context=self.get_observation_phase_context(),
            current_step=self.current_step,
        )
        self._update_debug_draw(drone_states=drone_states,
                                visible_ids_per_drone=visible_ids_per_drone)
        return observations

    def step(self, actions: List[Tuple[float, float, float, float]]):
        """
        actions : list of (vx, vy, vz, yaw_rate) per drone.
                  Commands are applied with instant velocity response for one
                  control interval dt = 1 / sim_hz. Position and yaw are
                  advanced by velocity * dt and yaw_rate * dt respectively.
        """
        self.current_step += 1

        try:
            # Apply velocity commands
            for drone_idx, (drone, action) in enumerate(zip(self.drones, actions)):
                vx, vy, vz, yaw_rate = action
                drone.apply_velocity(
                    vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate,
                    dt=self.dt,
                    x_min=self.move_x_min, x_max=self.move_x_max,
                    y_min=self.move_y_min, y_max=self.move_y_max,
                    z_min=self.z_min, z_max=self.z_max,
                )
                if self.cmd_history_len > 0:
                    self.obs_builder.update_cmd_history(drone_idx, vx, vy, yaw_rate)

            # Step people and physics
            self._step_group_centers()
            all_people_positions = [
                person.get_position()
                for person in self.people if person is not None
            ]
            for i, person in enumerate(self.people):
                if person is None:
                    continue
                other_positions = [
                    pos for j, pos in enumerate(all_people_positions) if j != i
                ]
                person.step(
                    self.dt,
                    x_min=self.x_min + 0.2, x_max=self.x_max - 0.2,
                    y_min=self.y_min + 0.2, y_max=self.y_max - 0.2,
                    other_people_positions=other_positions,
                )

            self.world.step()

            drone_states     = self._get_drone_states()
            people_positions = self._get_people_positions()
            # Binary per-episode visited map for exploration reward.
            # Old decaying behavior:
            # coverage_map_before_step = self.obs_builder.coverage_map.copy()
            coverage_map_before_step = self.obs_builder.persistent_coverage_map.copy()

            observations, visible_ids_per_drone, detections_per_drone = \
                self.obs_builder.build_observations(
                    drone_states,
                    people_positions,
                    phase_context=self.get_observation_phase_context(),
                    current_step=self.current_step,
                )
            self._maybe_plot_observation_debug(
                observations=observations,
                visible_ids_per_drone=visible_ids_per_drone,
                people_positions=people_positions,
            )
            self._update_debug_draw(drone_states=drone_states,
                                    visible_ids_per_drone=visible_ids_per_drone)

            person_reward_weights = self._build_person_reward_weights(len(people_positions))
            reward, reward_info, new_discoveries = self.reward_calc.compute_reward(
                visible_ids_per_drone=visible_ids_per_drone,
                drone_states=drone_states,
                people_positions=people_positions,
                num_people=len(people_positions),
                ever_seen=self.ever_seen,
                current_step=self.current_step,
                episode_steps=self.episode_steps,
                actions=actions,
                coverage_map=coverage_map_before_step,
                footprint_maps=self.obs_builder.footprint_maps_snapshot,
                person_weights=person_reward_weights,
            )
            self.ever_seen.update(new_discoveries)
            visible_union = set()
            for ids in visible_ids_per_drone:
                visible_union.update(ids)
            self.last_visible_count = len(visible_union)

            done = (self.current_step >= self.episode_steps) or (not p.isConnected())

            info = {
                "visible_ids_per_drone": visible_ids_per_drone,
                "detections_per_drone":  detections_per_drone,
                "reward_info":           reward_info,
                "active_num_drones":     self.active_num_drones,
                "num_groups":            self.episode_group_info["num_groups"],
                "group_centers":         self.episode_group_info["group_centers"],
                "group_info":            self.episode_group_info,
                "ever_seen_count":       len(self.ever_seen),
                "phase_info":            self.get_observation_phase_context(),
            }


            return observations, reward, done, info

        except Exception as e:
            fallback_obs, _, _ = self.obs_builder.build_observations(
                self._get_drone_states(),
                self._get_people_positions(),
                phase_context=self.get_observation_phase_context(),
                current_step=self.current_step,
            )
            info = {
                "simulation_error": str(e),
                "reward_info": {
                    "r_cov": 0.0, "r_fovq": 0.0, "r_disc": 0.0, "new_discovered": 0,
                    "coverage_count": 0, "r_ov": 0.0, "r_idle": 0.0,
                    "r_fov": 0.0, "r_exp": 0.0, "r_bound": 0.0, "r_close": 0.0,
                    "r_fov_overlap": 0.0, "r_coll": 0.0, "r_safe": 0.0, "r_energy": 0.0,
                    "total_reward": -1.0,
                },
                "active_num_drones": self.active_num_drones,
                "num_groups":      self.episode_group_info.get("num_groups", 0),
                "group_centers":   self.episode_group_info.get("group_centers", []),
                "group_info":      self.episode_group_info,
                "ever_seen_count": len(self.ever_seen),
                "phase_info":      self.get_observation_phase_context(),
            }
            return fallback_obs, -1.0, True, info

    def close(self):
        self._close_debug_observation_plot()
        self.debug_drawer.clear()
        if p.isConnected():
            self.world.disconnect()
