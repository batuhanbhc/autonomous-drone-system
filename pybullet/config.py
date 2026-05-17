"""
Shared configuration and CLI helpers for training and evaluation.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Any, Dict

import numpy as np

from env.env import MultiUAVEnv
from rl.action_masking import num_move_actions


@dataclass(frozen=True)
class SharedConfig:
    gui: bool = False
    x_min: float = -15.0
    x_max: float = 15.0
    y_min: float = -15.0
    y_max: float = 15.0
    z_min: float = 0.3
    z_max: float = 8.0
    drone_height: float = 6.0
    num_drones: int = 2
    random_spawn: bool = False
    drone_spawn_radius: float = 5.0
    num_people: int = 20
    min_people: int = 5
    max_people: int = 30
    episode_steps: int = 512
    search_phase_seconds: float = 16.0
    max_groups: int = 4
    num_group_regions: int = 4
    drone_wall_margin: float = 0.0
    person_spawn_margin: float = 0.5
    group_spawn_margin: float = 0.7
    min_person_spawn_dist: float = 0.4
    group_center_speed_min: float = 0.2
    group_center_speed_max: float = 1.5
    group_center_turn_prob: float = 0.25
    group_center_turn_std: float = 0.8
    rl_hz: float = 4.0
    max_range: float = 15.0
    horizontal_fov_deg: float = 55.8  
    vertical_fov_deg: float = 43.3  
    detection_prob: float = 1.0
    position_noise_std: float = 0.1
    detection_forward_decay_start_norm: float = 0.85
    camera_tilt_deg: float = 55.0
    recent_half_life_seconds: float = 10.0
    historic_half_life_seconds: float = 60.0
    coverage_half_life_seconds: float = 20.0
    blob_sigma: float = 1.0
    ego_sigma: float = 1.5
    people_count_normalizer: float = 30.0
    local_people_map_mode: str = "count_density"
    include_local_recent_count_memory_channel: bool = True
    include_shared_count_density_channel: bool = False
    include_instant_fov_channels: bool = True
    hide_person_features_during_search: bool = True
    count_memory_historic_miss_penalty: float = 0.35
    max_horizontal_velocity: float = 1.0
    horizontal_bin_interval: float = 1.0
    max_yaw_rate: float = 0.35
    yaw_bin_interval: float = 0.35
    vel_tau_s: float = 0.5
    yaw_rate_tau_s: float = 0.5
    cmd_history_len: int = 4
    status_history_seconds: int = 10
    hotspot_top_k: int = 2
    hotspot_min_density: float = 1.5
    hotspot_suppression_radius_scale: float = 5.0
    hotspot_suppression_radius_min_cells: int = 2
    reward_top_k_groups: int = 3
    reward_wc: float = 1.0
    reward_coverage_exponent: float = 2.0
    reward_wqual: float = 1.0
    reward_wd: float = 0.0
    reward_wo: float = 0.0
    reward_wx: float = 1.0
    reward_ws: float = 5.0
    reward_wclose: float = 1.0
    reward_wfov_overlap: float = 0.0
    reward_wcoll: float = 0.0
    reward_we: float = 0.0
    reward_wi: float = 0.0
    reward_wfov: float = 0.0
    reward_coverage_edge_quality: float = 0.0
    reward_quality_mode: str = "principal_linear"
    reward_quality_gamma: float = 1.5
    reward_wcompletion: float = 1.0
    reward_completion_power: float = 2.0
    reward_boundary_margin: float = 5.0
    reward_drone_closeness_margin: float = 5.0
    reward_fov_margin: float = 1.0
    debug_observation_plots: bool = False
    debug_observation_plot_every: int = 25
    debug_reward_contours: bool = False
    device: str = "cuda"


@dataclass(frozen=True)
class TrainConfig:
    total_updates: int = 2000
    n_envs: int = 4
    n_steps: int = 512
    num_epochs: int = 2
    batch_size: int = 512
    clip_eps: float = 0.15
    gamma: float = 0.995
    gae_lambda: float = 0.95
    lr_actor: float = 1.0e-4
    lr_critic: float = 1.0e-4
    entropy_coef: float = 0.005
    value_coef: float = 0.5
    anneal_lr: bool = True
    save_dir: str = "checkpoints"
    log_interval: int = 1
    save_interval: int = 25
    tensorboard: bool = False
    live_debug: bool = False
    live_debug_every: int = 100
    sticky_action_prob: float = 0.0
    fixed_active_num_drones: int | None = None
    load: str | None = None


@dataclass(frozen=True)
class EvalConfig:
    load: str = "checkpoints/final.pt"
    episodes: int = 100
    deterministic: bool = True
    realtime: bool = False
    print_actions: bool = False
    show_drone0_inputs: bool = False
    show_drone0_inputs_every: int = 1


@dataclass(frozen=True)
class ModelConfig:
    grid_channels: int = 10
    grid_h: int = 60
    grid_w: int = 60
    cnn_out_dim: int = 128
    hidden_dim: int = 256


@dataclass(frozen=True)
class DiscreteActionSpace:
    vx_bins: np.ndarray
    vy_bins: np.ndarray
    yaw_rate_bins: np.ndarray

    @property
    def num_v_bins(self) -> int:
        return int(self.vx_bins.shape[0])

    @property
    def num_yaw_bins(self) -> int:
        return int(self.yaw_rate_bins.shape[0])

    def decode_actions(self, indices: np.ndarray) -> list[tuple[float, float, float, float]]:
        idx = np.asarray(indices, dtype=np.int64)
        return [
            (
                float(self.vx_bins[action[0]]),
                float(self.vy_bins[action[1]]),
                0.0,
                float(self.yaw_rate_bins[action[2]]),
            )
            for action in idx
        ]


SHARED_DEFAULTS = SharedConfig()
TRAIN_DEFAULTS = TrainConfig()
EVAL_DEFAULTS = EvalConfig()
MODEL_DEFAULTS = ModelConfig()


def infer_checkpoint_actor_grid_channels(ckpt: dict) -> int:
    if "grid_channels" in ckpt:
        return int(ckpt["grid_channels"])
    actor_state = ckpt["actor"]
    return int(actor_state["cnn.net.0.conv.weight"].shape[1])


def infer_checkpoint_include_persistent_coverage_channel(ckpt: dict) -> bool:
    return bool(ckpt.get("include_persistent_coverage_channel", False))


def infer_checkpoint_local_people_map_mode(ckpt: dict) -> str:
    mode = str(ckpt.get("local_people_map_mode", "instant"))
    if mode not in {"instant", "count_density"}:
        raise ValueError(f"Unsupported checkpoint local_people_map_mode={mode!r}")
    return mode


def infer_checkpoint_include_shared_count_density_channel(ckpt: dict) -> bool:
    return bool(ckpt.get("include_shared_count_density_channel", True))


def infer_checkpoint_include_instant_fov_channels(ckpt: dict) -> bool:
    return bool(ckpt.get("include_instant_fov_channels", False))


def infer_checkpoint_hide_person_features_during_search(ckpt: dict) -> bool:
    return bool(ckpt.get("hide_person_features_during_search", False))


def infer_checkpoint_include_local_recent_count_memory_channel(ckpt: dict) -> bool:
    return bool(ckpt.get("include_local_recent_count_memory_channel", False))


def infer_base_actor_grid_channels(
    include_local_recent_count_memory_channel: bool,
    include_persistent_coverage_channel: bool,
    include_shared_count_density_channel: bool,
    include_instant_fov_channels: bool = True,
) -> int:
    return (
        6
        + int(bool(include_local_recent_count_memory_channel))
        + (2 if bool(include_instant_fov_channels) else 0)
        + int(bool(include_persistent_coverage_channel))
        + int(bool(include_shared_count_density_channel))
    )


def infer_shared_people_channels(
    actor_grid_channels: int,
    include_local_recent_count_memory_channel: bool = True,
    include_instant_fov_channels: bool = True,
) -> int:
    fixed_non_shared_channels = (
        7
        + int(bool(include_local_recent_count_memory_channel))
        if bool(include_instant_fov_channels)
        else 5 + int(bool(include_local_recent_count_memory_channel))
    )
    shared_people_channels = int(actor_grid_channels) - fixed_non_shared_channels
    if shared_people_channels not in {1, 2, 3, 4, 5}:
        raise ValueError(
            "Unsupported actor grid layout: expected "
            f"{fixed_non_shared_channels + 1}-{fixed_non_shared_channels + 5} "
            f"actor channels, got {actor_grid_channels}"
        )
    return shared_people_channels


def infer_critic_grid_channels(
    num_drones: int,
    actor_grid_channels: int,
    local_people_map_mode: str = "instant",
    include_local_recent_count_memory_channel: bool = True,
    include_instant_fov_channels: bool = True,
) -> int:
    if local_people_map_mode not in {"instant", "count_density"}:
        raise ValueError(
            "local_people_map_mode must be 'instant' or 'count_density', got "
            f"{local_people_map_mode!r}"
        )
    shared_people_channels = infer_shared_people_channels(
        actor_grid_channels,
        include_local_recent_count_memory_channel=include_local_recent_count_memory_channel,
        include_instant_fov_channels=include_instant_fov_channels,
    )
    shared_local_union_channels = 1 if local_people_map_mode == "instant" else 0
    critic_only_gt_channels = 2
    shared_fov_channels = 3 if bool(include_instant_fov_channels) else 2
    per_drone_channels = (
        4 + int(bool(include_local_recent_count_memory_channel))
        if bool(include_instant_fov_channels)
        else 3 + int(bool(include_local_recent_count_memory_channel))
    )
    return (
        shared_people_channels
        + shared_fov_channels
        + critic_only_gt_channels
        + shared_local_union_channels
        + (per_drone_channels * int(num_drones))
    )


def infer_checkpoint_hotspot_top_k(
    ckpt: dict,
    num_drones: int,
    action_space: DiscreteActionSpace,
    cmd_history_len: int,
    status_history_seconds: int = 0,
) -> int:
    if "hotspot_top_k" in ckpt:
        return int(ckpt["hotspot_top_k"])
    if "local_dim" in ckpt:
        full_local_dim = int(ckpt["local_dim"])
    else:
        full_local_dim = int(ckpt["actor"]["local_mlp.0.weight"].shape[1])
    move_mask_dim = num_move_actions(action_space.vx_bins, action_space.vy_bins)
    base_local_dim = full_local_dim - move_mask_dim
    base_feature_dim = (
        13 if infer_checkpoint_local_visited_fraction_feature(ckpt)
        else (12 if infer_checkpoint_local_visible_delta_feature(ckpt) else 11)
    )
    static_base_dim = (
        base_feature_dim
        + 6 * (int(num_drones) - 1)
        + 5 * int(status_history_seconds)
        + 3 * int(cmd_history_len)
    )
    extra_dim = base_local_dim - static_base_dim
    if extra_dim < 0 or extra_dim % 5 != 0:
        raise ValueError(
            "Could not infer hotspot_top_k from checkpoint local_dim: "
            f"full_local_dim={full_local_dim}, base_local_dim={base_local_dim}, "
            f"static_base_dim={static_base_dim}"
        )
    return extra_dim // 5


def infer_checkpoint_status_history_seconds(ckpt: dict) -> int:
    return int(ckpt.get("status_history_seconds", 0))


def infer_checkpoint_local_visible_delta_feature(ckpt: dict) -> bool:
    return bool(ckpt.get("local_visible_delta_feature", False))


def infer_checkpoint_local_visited_fraction_feature(ckpt: dict) -> bool:
    return bool(ckpt.get("local_visited_fraction_feature", False))


def infer_checkpoint_cmd_history_len(
    ckpt: dict,
    num_drones: int,
    action_space: DiscreteActionSpace,
) -> int:
    if "cmd_history_len" in ckpt:
        return int(ckpt["cmd_history_len"])
    if "status_history_seconds" in ckpt:
        return 0
    if "local_dim" in ckpt:
        full_local_dim = int(ckpt["local_dim"])
    else:
        full_local_dim = int(ckpt["actor"]["local_mlp.0.weight"].shape[1])
    move_mask_dim = num_move_actions(action_space.vx_bins, action_space.vy_bins)
    base_local_dim = full_local_dim - move_mask_dim
    hotspot_top_k = int(ckpt.get("hotspot_top_k", 0))
    static_base_dim = (
        (
            13 if infer_checkpoint_local_visited_fraction_feature(ckpt)
            else (12 if infer_checkpoint_local_visible_delta_feature(ckpt) else 11)
        )
        + 5 * hotspot_top_k
        + 6 * (int(num_drones) - 1)
    )
    extra_dim = base_local_dim - static_base_dim
    if extra_dim < 0 or extra_dim % 3 != 0:
        raise ValueError(
            "Could not infer legacy cmd_history_len from checkpoint local_dim: "
            f"full_local_dim={full_local_dim}, base_local_dim={base_local_dim}, "
            f"static_base_dim={static_base_dim}, hotspot_top_k={hotspot_top_k}"
        )
    return extra_dim // 3


def add_shared_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--gui", action="store_true", default=SHARED_DEFAULTS.gui)
    parser.add_argument("--x_min", type=float, default=SHARED_DEFAULTS.x_min)
    parser.add_argument("--x_max", type=float, default=SHARED_DEFAULTS.x_max)
    parser.add_argument("--y_min", type=float, default=SHARED_DEFAULTS.y_min)
    parser.add_argument("--y_max", type=float, default=SHARED_DEFAULTS.y_max)
    parser.add_argument("--z_min", type=float, default=SHARED_DEFAULTS.z_min)
    parser.add_argument("--z_max", type=float, default=SHARED_DEFAULTS.z_max)
    parser.add_argument("--drone_height", type=float, default=SHARED_DEFAULTS.drone_height)
    parser.add_argument(
        "--num_drones",
        type=int,
        default=SHARED_DEFAULTS.num_drones,
        help="Maximum drones; each episode samples a count uniformly from 1..num_drones.",
    )
    parser.add_argument(
        "--random_spawn",
        action="store_true",
        default=SHARED_DEFAULTS.random_spawn,
        help="Spawn drones uniformly within the reward-boundary rectangle instead of the default center disk region.",
    )
    parser.add_argument(
        "--drone_spawn_radius",
        type=float,
        default=SHARED_DEFAULTS.drone_spawn_radius,
        help="Radius of the default circular drone spawn region centered in the arena.",
    )
    parser.add_argument("--num_people", type=int, default=SHARED_DEFAULTS.num_people)
    parser.add_argument("--min_people", type=int, default=SHARED_DEFAULTS.min_people)
    parser.add_argument("--max_people", type=int, default=SHARED_DEFAULTS.max_people)
    parser.add_argument("--episode_steps", type=int, default=SHARED_DEFAULTS.episode_steps)
    parser.add_argument(
        "--search_phase_seconds",
        type=float,
        default=SHARED_DEFAULTS.search_phase_seconds,
        help="Initial search-only phase duration in seconds. Coverage and FOV-quality rewards are disabled during this phase.",
    )
    parser.add_argument("--max_groups", type=int, default=SHARED_DEFAULTS.max_groups)
    parser.add_argument("--num_group_regions", type=int, default=SHARED_DEFAULTS.num_group_regions)
    parser.add_argument(
        "--drone_wall_margin",
        type=float,
        default=SHARED_DEFAULTS.drone_wall_margin,
        help="Inner safety buffer from the true walls used for drone movement and drone spawn.",
    )
    parser.add_argument("--person_spawn_margin", type=float, default=SHARED_DEFAULTS.person_spawn_margin)
    parser.add_argument("--group_spawn_margin", type=float, default=SHARED_DEFAULTS.group_spawn_margin)
    parser.add_argument("--min_person_spawn_dist", type=float, default=SHARED_DEFAULTS.min_person_spawn_dist)
    parser.add_argument(
        "--group_center_speed_min",
        type=float,
        default=SHARED_DEFAULTS.group_center_speed_min,
        help="Minimum drift speed for moving group centers.",
    )
    parser.add_argument(
        "--group_center_speed_max",
        type=float,
        default=SHARED_DEFAULTS.group_center_speed_max,
        help="Maximum drift speed for moving group centers.",
    )
    parser.add_argument(
        "--group_center_turn_prob",
        type=float,
        default=SHARED_DEFAULTS.group_center_turn_prob,
        help="Per-second probability that a moving group center changes heading.",
    )
    parser.add_argument(
        "--group_center_turn_std",
        type=float,
        default=SHARED_DEFAULTS.group_center_turn_std,
        help="Standard deviation of heading changes for moving group centers, in radians.",
    )
    parser.add_argument(
        "--rl_hz",
        type=float,
        default=SHARED_DEFAULTS.rl_hz,
        help="RL decisions per simulation second.",
    )
    parser.add_argument("--sim_hz", dest="rl_hz", type=float, help=argparse.SUPPRESS)
    parser.add_argument("--max_range", type=float, default=SHARED_DEFAULTS.max_range)
    parser.add_argument(
        "--horizontal_fov_deg",
        type=float,
        default=SHARED_DEFAULTS.horizontal_fov_deg,
        help="Horizontal camera field of view in degrees.",
    )
    parser.add_argument(
        "--vertical_fov_deg",
        type=float,
        default=SHARED_DEFAULTS.vertical_fov_deg,
        help="Vertical camera field of view in degrees.",
    )
    parser.add_argument("--detection_prob", type=float, default=SHARED_DEFAULTS.detection_prob)
    parser.add_argument("--position_noise_std", type=float, default=SHARED_DEFAULTS.position_noise_std)
    parser.add_argument(
        "--detection_forward_decay_start_norm",
        type=float,
        default=SHARED_DEFAULTS.detection_forward_decay_start_norm,
        help=(
            "Forward-norm threshold in [0, 1] where detection probability starts "
            "decaying linearly toward zero at the far edge of the footprint."
        ),
    )
    parser.add_argument(
        "--camera_tilt_deg",
        type=float,
        default=SHARED_DEFAULTS.camera_tilt_deg,
        help="Camera pitch from straight down: 0=nadir, 90=horizon.",
    )
    parser.add_argument(
        "--recent_half_life_seconds",
        type=float,
        default=SHARED_DEFAULTS.recent_half_life_seconds,
    )
    parser.add_argument(
        "--historic_half_life_seconds",
        type=float,
        default=SHARED_DEFAULTS.historic_half_life_seconds,
    )
    parser.add_argument(
        "--coverage_half_life_seconds",
        type=float,
        default=SHARED_DEFAULTS.coverage_half_life_seconds,
    )
    parser.add_argument("--blob_sigma", type=float, default=SHARED_DEFAULTS.blob_sigma)
    parser.add_argument("--ego_sigma", type=float, default=SHARED_DEFAULTS.ego_sigma)
    parser.add_argument(
        "--people_count_normalizer",
        type=float,
        default=SHARED_DEFAULTS.people_count_normalizer,
        help="Normalization divisor for visible-people count features in actor local state and critic context.",
    )
    parser.add_argument(
        "--local_people_map_mode",
        choices=("count_density", "instant"),
        default=SHARED_DEFAULTS.local_people_map_mode,
        help="Actor/critic per-drone people-map type. 'count_density' is the current default; 'instant' is the legacy layout.",
    )
    parser.add_argument(
        "--include_local_recent_count_memory_channel",
        action=argparse.BooleanOptionalAction,
        default=SHARED_DEFAULTS.include_local_recent_count_memory_channel,
        help="Expose per-drone recent count memory as a local actor/critic channel.",
    )
    parser.add_argument(
        "--include_shared_count_density_channel",
        action=argparse.BooleanOptionalAction,
        default=SHARED_DEFAULTS.include_shared_count_density_channel,
        help="Expose the team-shared count-density map in the actor/critic shared stack. Disable for the current local-count layout.",
    )
    parser.add_argument(
        "--include_instant_fov_channels",
        action=argparse.BooleanOptionalAction,
        default=SHARED_DEFAULTS.include_instant_fov_channels,
        help="Expose current-step, non-decaying FOV footprint channels in actor and critic inputs.",
    )
    parser.add_argument(
        "--hide_person_features_during_search",
        action=argparse.BooleanOptionalAction,
        default=SHARED_DEFAULTS.hide_person_features_during_search,
        help="Hide actor-facing person detections, hotspots, and people-memory channels during search while still updating them internally.",
    )
    parser.add_argument(
        "--count_memory_historic_miss_penalty",
        type=float,
        default=SHARED_DEFAULTS.count_memory_historic_miss_penalty,
        help="Extra attenuation applied to raw historic count memory when a visible cell has no strong current density.",
    )
    parser.add_argument(
        "--max_horizontal_velocity",
        type=float,
        default=SHARED_DEFAULTS.max_horizontal_velocity,
    )
    parser.add_argument(
        "--horizontal_bin_interval",
        type=float,
        default=SHARED_DEFAULTS.horizontal_bin_interval,
    )
    parser.add_argument(
        "--max_yaw_rate",
        type=float,
        default=SHARED_DEFAULTS.max_yaw_rate,
    )
    parser.add_argument(
        "--yaw_bin_interval",
        type=float,
        default=SHARED_DEFAULTS.yaw_bin_interval,
    )
    parser.add_argument(
        "--vel_tau_s",
        type=float,
        default=SHARED_DEFAULTS.vel_tau_s,
        help="First-order lag time constant for horizontal velocity (seconds). 0=instant.",
    )
    parser.add_argument(
        "--yaw_rate_tau_s",
        type=float,
        default=SHARED_DEFAULTS.yaw_rate_tau_s,
        help="First-order lag time constant for yaw rate (seconds). 0=instant.",
    )
    parser.add_argument(
        "--cmd_history_len",
        type=int,
        default=SHARED_DEFAULTS.cmd_history_len,
        help="Legacy number of past (vx, vy, yaw_rate) commands appended to the actor local vector. 0=disabled.",
    )
    parser.add_argument(
        "--status_history_seconds",
        type=int,
        default=SHARED_DEFAULTS.status_history_seconds,
        help="Number of 1 Hz status-history entries appended to the actor local vector. 0=disabled.",
    )
    parser.add_argument(
        "--hotspot_top_k",
        type=int,
        default=SHARED_DEFAULTS.hotspot_top_k,
        help="Number of top historic-density hotspots encoded into each drone's local vector.",
    )
    parser.add_argument(
        "--hotspot_min_density",
        type=float,
        default=SHARED_DEFAULTS.hotspot_min_density,
        help="Minimum raw historic count-density value required for a hotspot to be emitted into the local vector.",
    )
    parser.add_argument(
        "--hotspot_suppression_radius_scale",
        type=float,
        default=SHARED_DEFAULTS.hotspot_suppression_radius_scale,
        help="Hotspot non-maximum-suppression radius in blob-sigma units.",
    )
    parser.add_argument(
        "--hotspot_suppression_radius_min_cells",
        type=int,
        default=SHARED_DEFAULTS.hotspot_suppression_radius_min_cells,
        help="Minimum hotspot suppression radius in grid cells.",
    )
    parser.add_argument("--reward_wc", type=float, default=SHARED_DEFAULTS.reward_wc)
    parser.add_argument(
        "--reward_top_k_groups",
        type=int,
        default=SHARED_DEFAULTS.reward_top_k_groups,
        help="Only the top-K densest groups contribute to coverage reward. 0 keeps legacy all-person coverage.",
    )
    parser.add_argument(
        "--reward_coverage_exponent",
        type=float,
        default=SHARED_DEFAULTS.reward_coverage_exponent,
        help="Exponent applied to the coverage ratio before the coverage reward is used.",
    )
    parser.add_argument("--reward_wqual", type=float, default=SHARED_DEFAULTS.reward_wqual)
    parser.add_argument("--reward_wd", type=float, default=SHARED_DEFAULTS.reward_wd)
    parser.add_argument("--reward_wo", type=float, default=SHARED_DEFAULTS.reward_wo)
    parser.add_argument("--reward_wx", type=float, default=SHARED_DEFAULTS.reward_wx)
    parser.add_argument("--reward_ws", type=float, default=SHARED_DEFAULTS.reward_ws)
    parser.add_argument("--reward_wclose", type=float, default=SHARED_DEFAULTS.reward_wclose)
    parser.add_argument(
        "--reward_wfov_overlap",
        type=float,
        default=SHARED_DEFAULTS.reward_wfov_overlap,
        help="Weight for the current-step pairwise FOV IoU overlap penalty.",
    )
    parser.add_argument("--reward_wcoll", type=float, default=SHARED_DEFAULTS.reward_wcoll)
    parser.add_argument("--reward_we", type=float, default=SHARED_DEFAULTS.reward_we)
    parser.add_argument("--reward_wi", type=float, default=SHARED_DEFAULTS.reward_wi)
    parser.add_argument("--reward_wfov", type=float, default=SHARED_DEFAULTS.reward_wfov)
    parser.add_argument(
        "--reward_wcompletion",
        type=float,
        default=SHARED_DEFAULTS.reward_wcompletion,
        help=(
            "Weight for the search-phase completion-progress bonus based on "
            "persistent visited fraction."
        ),
    )
    parser.add_argument(
        "--reward_coverage_edge_quality",
        type=float,
        default=SHARED_DEFAULTS.reward_coverage_edge_quality,
        help="Coverage quality floor for the worst visible framing locations. "
             "Higher values flatten the framing-quality reward.",
    )
    parser.add_argument(
        "--reward_quality_mode",
        type=str,
        choices=("legacy", "principal_linear", "principal_squared", "principal_power"),
        default=SHARED_DEFAULTS.reward_quality_mode,
        help="Framing-quality field used by the FOV quality reward. Principal modes use a circular drop around the principal point.",
    )
    parser.add_argument(
        "--reward_quality_gamma",
        type=float,
        default=SHARED_DEFAULTS.reward_quality_gamma,
        help=(
            "Exponent for principal_power reward quality. gamma=1 matches "
            "principal_linear; higher values penalize off-center framing sooner."
        ),
    )
    parser.add_argument(
        "--reward_completion_power",
        type=float,
        default=SHARED_DEFAULTS.reward_completion_power,
        help=(
            "Exponent for the search-phase completion-progress reward. "
            "power>1 emphasizes late coverage cleanup."
        ),
    )
    parser.add_argument(
        "--reward_boundary_margin",
        type=float,
        default=SHARED_DEFAULTS.reward_boundary_margin,
        help="Inner wall margin used by the boundary reward and the uniform random drone spawn region.",
    )
    parser.add_argument(
        "--reward_drone_closeness_margin",
        type=float,
        default=SHARED_DEFAULTS.reward_drone_closeness_margin,
        help="Distance margin under which drones incur a linear proximity penalty, analogous to the wall-margin penalty.",
    )
    parser.add_argument(
        "--reward_fov_margin",
        type=float,
        default=SHARED_DEFAULTS.reward_fov_margin,
    )
    parser.add_argument(
        "--debug_observation_plots",
        action="store_true",
        default=SHARED_DEFAULTS.debug_observation_plots,
        help="Show matplotlib plots for drone-0 actor inputs and critic inputs during env stepping.",
    )
    parser.add_argument(
        "--debug_observation_plot_every",
        type=int,
        default=SHARED_DEFAULTS.debug_observation_plot_every,
        help="Step interval for observation debug plots when enabled.",
    )
    parser.add_argument(
        "--debug_reward_contours",
        action="store_true",
        default=SHARED_DEFAULTS.debug_reward_contours,
        help="Draw GUI overlays for the current coverage-reward quality contours around each drone's principal point.",
    )
    parser.add_argument("--device", type=str, default=SHARED_DEFAULTS.device)


def add_train_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--total_updates",
        type=int,
        default=TRAIN_DEFAULTS.total_updates,
        help=(
            "Global training horizon in PPO updates. On resume, training "
            "continues only until this lifetime total is reached."
        ),
    )
    parser.add_argument("--n_envs", type=int, default=TRAIN_DEFAULTS.n_envs)
    parser.add_argument("--n_steps", type=int, default=TRAIN_DEFAULTS.n_steps)
    parser.add_argument("--rollout_len", dest="n_steps", type=int, help=argparse.SUPPRESS)
    parser.add_argument("--num_epochs", type=int, default=TRAIN_DEFAULTS.num_epochs)
    parser.add_argument("--batch_size", type=int, default=TRAIN_DEFAULTS.batch_size)
    parser.add_argument("--clip_eps", type=float, default=TRAIN_DEFAULTS.clip_eps)
    parser.add_argument("--gamma", type=float, default=TRAIN_DEFAULTS.gamma)
    parser.add_argument("--gae_lambda", type=float, default=TRAIN_DEFAULTS.gae_lambda)
    parser.add_argument("--lr_actor", type=float, default=TRAIN_DEFAULTS.lr_actor)
    parser.add_argument("--lr_critic", type=float, default=TRAIN_DEFAULTS.lr_critic)
    parser.add_argument("--entropy_coef", type=float, default=TRAIN_DEFAULTS.entropy_coef)
    parser.add_argument("--value_coef", type=float, default=TRAIN_DEFAULTS.value_coef)
    parser.add_argument("--save_dir", type=str, default=TRAIN_DEFAULTS.save_dir)
    parser.add_argument("--log_interval", type=int, default=TRAIN_DEFAULTS.log_interval)
    parser.add_argument(
        "--save_interval",
        type=int,
        default=TRAIN_DEFAULTS.save_interval,
        help="Save an intermediate checkpoint every N PPO updates.",
    )
    parser.add_argument("--tensorboard", action="store_true", default=TRAIN_DEFAULTS.tensorboard)
    parser.add_argument("--live_debug", action="store_true", default=TRAIN_DEFAULTS.live_debug)
    parser.add_argument("--live_debug_every", type=int, default=TRAIN_DEFAULTS.live_debug_every)
    parser.add_argument(
        "--sticky_action_prob",
        type=float,
        default=TRAIN_DEFAULTS.sticky_action_prob,
        help="Probability of repeating each agent's last executed action during training rollouts.",
    )
    parser.add_argument(
        "--anneal_lr",
        action=argparse.BooleanOptionalAction,
        default=TRAIN_DEFAULTS.anneal_lr,
        help="Linearly decay actor and critic LRs to zero over the global total_updates horizon.",
    )
    parser.add_argument(
        "--fixed_active_num_drones",
        type=int,
        default=TRAIN_DEFAULTS.fixed_active_num_drones,
        help="If set, keep the active drone count fixed during training instead of sampling from 1..num_drones.",
    )
    parser.add_argument("--load", type=str, default=TRAIN_DEFAULTS.load)


def add_eval_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--load", type=str, default=EVAL_DEFAULTS.load)
    parser.add_argument("--episodes", type=int, default=EVAL_DEFAULTS.episodes)
    parser.add_argument(
        "--deterministic",
        dest="deterministic",
        action="store_true",
        default=EVAL_DEFAULTS.deterministic,
        help="Run eval with argmax actions.",
    )
    parser.add_argument(
        "--stochastic",
        dest="deterministic",
        action="store_false",
        help="Run eval by sampling from the policy instead of taking argmax actions.",
    )
    parser.add_argument("--realtime", action="store_true", default=EVAL_DEFAULTS.realtime)
    parser.add_argument("--print_actions", action="store_true", default=EVAL_DEFAULTS.print_actions)
    parser.add_argument(
        "--show_drone0_inputs",
        action="store_true",
        default=EVAL_DEFAULTS.show_drone0_inputs,
        help="Open a live matplotlib window showing Drone 0 actor CNN channels and local inputs during eval.",
    )
    parser.add_argument(
        "--show_drone0_inputs_every",
        type=int,
        default=EVAL_DEFAULTS.show_drone0_inputs_every,
        help="Step interval for updating the Drone 0 input debug window during eval.",
    )


def build_env_kwargs(
    args: argparse.Namespace,
    overrides: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    actor_grid_channels = getattr(args, "actor_grid_channels", None)
    if actor_grid_channels is None:
        actor_grid_channels = MODEL_DEFAULTS.grid_channels
    include_persistent_coverage_channel = getattr(
        args,
        "include_persistent_coverage_channel",
        True,
    )
    local_people_map_mode = getattr(
        args,
        "local_people_map_mode",
        SHARED_DEFAULTS.local_people_map_mode,
    )
    include_shared_count_density_channel = getattr(
        args,
        "include_shared_count_density_channel",
        SHARED_DEFAULTS.include_shared_count_density_channel,
    )
    include_local_recent_count_memory_channel = getattr(
        args,
        "include_local_recent_count_memory_channel",
        SHARED_DEFAULTS.include_local_recent_count_memory_channel,
    )
    include_instant_fov_channels = getattr(
        args,
        "include_instant_fov_channels",
        SHARED_DEFAULTS.include_instant_fov_channels,
    )
    kwargs = {
        "gui": args.gui,
        "x_min": args.x_min,
        "x_max": args.x_max,
        "y_min": args.y_min,
        "y_max": args.y_max,
        "z_min": args.z_min,
        "z_max": args.z_max,
        "drone_height": args.drone_height,
        "num_drones": args.num_drones,
        "random_spawn": args.random_spawn,
        "drone_spawn_radius": args.drone_spawn_radius,
        "fixed_active_num_drones": getattr(args, "fixed_active_num_drones", None),
        "num_people": args.num_people,
        "min_people": args.min_people,
        "max_people": args.max_people,
        "episode_steps": args.episode_steps,
        "search_phase_seconds": args.search_phase_seconds,
        "max_groups": args.max_groups,
        "num_group_regions": args.num_group_regions,
        "drone_wall_margin": args.drone_wall_margin,
        "person_spawn_margin": args.person_spawn_margin,
        "group_spawn_margin": args.group_spawn_margin,
        "min_person_spawn_dist": args.min_person_spawn_dist,
        "group_center_speed_min": args.group_center_speed_min,
        "group_center_speed_max": args.group_center_speed_max,
        "group_center_turn_prob": args.group_center_turn_prob,
        "group_center_turn_std": args.group_center_turn_std,
        "sim_hz": args.rl_hz,
        "max_range": args.max_range,
        "horizontal_fov_deg": args.horizontal_fov_deg,
        "vertical_fov_deg": args.vertical_fov_deg,
        "detection_prob": args.detection_prob,
        "position_noise_std": args.position_noise_std,
        "detection_forward_decay_start_norm": args.detection_forward_decay_start_norm,
        "camera_tilt_deg": args.camera_tilt_deg,
        "recent_half_life_seconds": args.recent_half_life_seconds,
        "historic_half_life_seconds": args.historic_half_life_seconds,
        "coverage_half_life_seconds": args.coverage_half_life_seconds,
        "grid_h": MODEL_DEFAULTS.grid_h,
        "grid_w": MODEL_DEFAULTS.grid_w,
        "blob_sigma": args.blob_sigma,
        "ego_sigma": args.ego_sigma,
        "people_count_normalizer": args.people_count_normalizer,
        "local_people_map_mode": local_people_map_mode,
        "include_local_recent_count_memory_channel": include_local_recent_count_memory_channel,
        "include_shared_count_density_channel": include_shared_count_density_channel,
        "include_instant_fov_channels": include_instant_fov_channels,
        "hide_person_features_during_search": args.hide_person_features_during_search,
        "count_memory_historic_miss_penalty": args.count_memory_historic_miss_penalty,
        "reward_wc": args.reward_wc,
        "reward_coverage_exponent": args.reward_coverage_exponent,
        "reward_wqual": args.reward_wqual,
        "reward_wd": args.reward_wd,
        "reward_wo": args.reward_wo,
        "reward_wx": args.reward_wx,
        "reward_ws": args.reward_ws,
        "reward_wclose": args.reward_wclose,
        "reward_wfov_overlap": args.reward_wfov_overlap,
        "reward_wcoll": args.reward_wcoll,
        "reward_we": args.reward_we,
        "reward_wi": args.reward_wi,
        "reward_wfov": args.reward_wfov,
        "reward_wcompletion": args.reward_wcompletion,
        "reward_coverage_edge_quality": args.reward_coverage_edge_quality,
        "reward_quality_mode": args.reward_quality_mode,
        "reward_quality_gamma": args.reward_quality_gamma,
        "reward_completion_power": args.reward_completion_power,
        "reward_boundary_margin": args.reward_boundary_margin,
        "reward_drone_closeness_margin": args.reward_drone_closeness_margin,
        "reward_fov_margin": args.reward_fov_margin,
        "vel_tau_s": args.vel_tau_s,
        "yaw_rate_tau_s": args.yaw_rate_tau_s,
        "cmd_history_len": args.cmd_history_len,
        "status_history_seconds": args.status_history_seconds,
        "hotspot_top_k": args.hotspot_top_k,
        "hotspot_min_density": args.hotspot_min_density,
        "hotspot_suppression_radius_scale": args.hotspot_suppression_radius_scale,
        "hotspot_suppression_radius_min_cells": args.hotspot_suppression_radius_min_cells,
        "reward_top_k_groups": args.reward_top_k_groups,
        "max_horizontal_velocity": args.max_horizontal_velocity,
        "max_yaw_rate": args.max_yaw_rate,
        "debug_observation_plots": args.debug_observation_plots,
        "debug_observation_plot_every": args.debug_observation_plot_every,
        "debug_reward_contours": args.debug_reward_contours,
        "actor_grid_channels": actor_grid_channels,
        "include_persistent_coverage_channel": include_persistent_coverage_channel,
    }
    if overrides:
        kwargs.update(overrides)
    return kwargs


def build_env(
    args: argparse.Namespace,
    overrides: Dict[str, Any] | None = None,
) -> MultiUAVEnv:
    return MultiUAVEnv(**build_env_kwargs(args, overrides=overrides))


def local_dim(
    num_drones: int,
    action_space: DiscreteActionSpace | None = None,
    cmd_history_len: int = 0,
    status_history_seconds: int = 0,
    hotspot_top_k: int = 0,
) -> int:
    # Base local features:
    #   7 ego/search scalars (x, y, sin_yaw, cos_yaw, num_visible,
    #                         delta_visible, visited_fraction)
    #   3 phase scalars (search progress, is_search_phase, is_coverage_phase)
    #   3 explicit detection-centroid vs principal-point alignment scalars
    #   5 scalars per hotspot slot: [valid, rel_dx_world, rel_dy_world, density, age]
    #   6 scalars per teammate slot
    #   5 scalars per 1 Hz status-history entry
    #   3 scalars per legacy command-history entry (vx, vy, yaw_rate)
    base_dim = (
        13
        + 5 * int(hotspot_top_k)
        + 6 * (num_drones - 1)
        + 5 * int(status_history_seconds)
        + 3 * int(cmd_history_len)
    )
    if action_space is None:
        return base_dim
    return base_dim + num_move_actions(action_space.vx_bins, action_space.vy_bins)


def _build_symmetric_bins(max_abs: float, interval: float, name: str) -> np.ndarray:
    if max_abs <= 0.0:
        raise ValueError(f"{name} max must be > 0, got {max_abs}")
    if interval <= 0.0:
        raise ValueError(f"{name} interval must be > 0, got {interval}")

    steps = max_abs / interval
    rounded_steps = round(steps)
    if not math.isclose(steps, rounded_steps, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(
            f"{name} max ({max_abs}) must be divisible by interval ({interval})"
        )

    return np.linspace(
        -max_abs,
        max_abs,
        num=2 * rounded_steps + 1,
        dtype=np.float32,
    )


def build_action_space(args: argparse.Namespace) -> DiscreteActionSpace:
    v_bins = _build_symmetric_bins(
        args.max_horizontal_velocity,
        args.horizontal_bin_interval,
        "horizontal velocity",
    )
    yaw_bins = _build_symmetric_bins(
        args.max_yaw_rate,
        args.yaw_bin_interval,
        "yaw rate",
    )
    return DiscreteActionSpace(
        vx_bins=v_bins,
        vy_bins=v_bins.copy(),
        yaw_rate_bins=yaw_bins,
    )


def actor_kwargs(
    num_drones: int,
    action_space: DiscreteActionSpace,
    cmd_history_len: int = 0,
    status_history_seconds: int = 0,
    hotspot_top_k: int = 0,
    grid_channels: int | None = None,
) -> Dict[str, Any]:
    return {
        "local_dim": local_dim(
            num_drones,
            action_space,
            cmd_history_len,
            status_history_seconds,
            hotspot_top_k,
        ),
        "grid_channels": MODEL_DEFAULTS.grid_channels if grid_channels is None else int(grid_channels),
        "grid_h": MODEL_DEFAULTS.grid_h,
        "grid_w": MODEL_DEFAULTS.grid_w,
        "cnn_out_dim": MODEL_DEFAULTS.cnn_out_dim,
        "hidden_dim": MODEL_DEFAULTS.hidden_dim,
        "num_vx_bins": int(action_space.vx_bins.shape[0]),
        "num_vy_bins": int(action_space.vy_bins.shape[0]),
        "num_yaw_bins": action_space.num_yaw_bins,
    }


def trainer_kwargs(args: argparse.Namespace, action_space: DiscreteActionSpace) -> Dict[str, Any]:
    actor_grid_channels = getattr(args, "actor_grid_channels", None)
    if actor_grid_channels is None:
        actor_grid_channels = MODEL_DEFAULTS.grid_channels
    kwargs = actor_kwargs(
        args.num_drones,
        action_space,
        getattr(args, "cmd_history_len", 0),
        getattr(args, "status_history_seconds", 0),
        getattr(args, "hotspot_top_k", 0),
        grid_channels=actor_grid_channels,
    )
    kwargs.update(
        {
            "n_steps": args.n_steps,
            "num_epochs": args.num_epochs,
            "batch_size": args.batch_size,
            "clip_eps": args.clip_eps,
            "gamma": args.gamma,
            "gae_lambda": args.gae_lambda,
            "lr_actor": args.lr_actor,
            "lr_critic": args.lr_critic,
            "value_coef": args.value_coef,
            "entropy_coef": args.entropy_coef,
            "vx_bins": action_space.vx_bins,
            "vy_bins": action_space.vy_bins,
            "yaw_rate_bins": action_space.yaw_rate_bins,
            "device": args.device,
            "save_dir": args.save_dir,
            "log_interval": args.log_interval,
            "save_interval": args.save_interval,
            "use_tensorboard": args.tensorboard,
            "live_debug": args.live_debug,
            "live_debug_every": args.live_debug_every,
            "sticky_action_prob": args.sticky_action_prob,
            "anneal_lr": args.anneal_lr,
            "gui": args.gui,
        }
    )
    return kwargs
