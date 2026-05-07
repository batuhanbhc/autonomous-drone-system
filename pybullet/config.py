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
    drone_height: float = 6.0
    num_drones: int = 2
    random_spawn: bool = False
    drone_spawn_radius: float = 3.0
    num_people: int = 20
    min_people: int = 10
    max_people: int = 25
    episode_steps: int = 480
    max_groups: int = 3
    num_group_regions: int = 4
    drone_wall_margin: float = 0.0
    person_spawn_margin: float = 0.5
    group_spawn_margin: float = 0.3
    min_person_spawn_dist: float = 0.35
    group_center_speed_min: float = 0.04
    group_center_speed_max: float = 0.12
    group_center_turn_prob: float = 0.2
    group_center_turn_std: float = 0.35
    rl_hz: float = 4.0
    max_range: float = 15.0
    horizontal_fov_deg: float = 55.8
    vertical_fov_deg: float = 43.3
    detection_prob: float = 1.0
    position_noise_std: float = 0.1
    camera_tilt_deg: float = 45.0
    recent_half_life_seconds: float = 10.0
    historic_half_life_seconds: float = 60.0
    coverage_half_life_seconds: float = 60.0
    recent_hit_gain: float = 0.6
    recent_miss_penalty: float = 0.25
    blob_sigma: float = 1.0
    ego_sigma: float = 2.5
    max_horizontal_velocity: float = 1.0
    horizontal_bin_interval: float = 1.0
    max_yaw_rate: float = 0.5
    yaw_bin_interval: float = 0.5
    vel_tau_s: float = 0.4
    yaw_rate_tau_s: float = 0.4
    cmd_history_len: int = 4
    reward_wc: float = 1.0
    reward_wqual: float = 1.5
    reward_wd: float = 0.0
    reward_wo: float = 0.0
    reward_wx: float = 0.0
    reward_ws: float = 1.0
    reward_wclose: float = 1.0
    reward_wfov_overlap: float = 2.0
    reward_wcoll: float = 0.0
    reward_we: float = 0.0
    reward_wi: float = 0.0
    reward_wfov: float = 0.0
    reward_coverage_edge_quality: float = 0.0
    reward_quality_mode: str = "principal_linear"
    reward_boundary_margin: float = 4.0
    reward_drone_closeness_margin: float = 4.0
    reward_fov_margin: float = 1.0
    debug_observation_plots: bool = False
    debug_observation_plot_every: int = 25
    debug_reward_contours: bool = False
    device: str = "cuda"


@dataclass(frozen=True)
class TrainConfig:
    total_updates: int = 10000
    rollout_len: int = 480*4
    num_epochs: int = 2
    batch_size: int = 128
    clip_eps: float = 0.1
    gamma: float = 0.99
    gae_lambda: float = 0.95
    lr_actor: float = 3e-4
    lr_critic: float = 3e-4
    entropy_coef: float = 0.005
    value_coef: float = 0.5
    save_dir: str = "checkpoints"
    log_interval: int = 1
    tensorboard: bool = False
    live_debug: bool = False
    live_debug_every: int = 100
    sticky_action_prob: float = 0.2
    fixed_active_num_drones: int | None = None
    load: str | None = None


@dataclass(frozen=True)
class EvalConfig:
    load: str = "checkpoints/final.pt"
    episodes: int = 10
    deterministic: bool = True
    realtime: bool = False
    print_actions: bool = False


@dataclass(frozen=True)
class ModelConfig:
    grid_channels: int = 8
    grid_h: int = 80
    grid_w: int = 80
    cnn_out_dim: int = 64
    hidden_dim: int = 128


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


def add_shared_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--gui", action="store_true", default=SHARED_DEFAULTS.gui)
    parser.add_argument("--x_min", type=float, default=SHARED_DEFAULTS.x_min)
    parser.add_argument("--x_max", type=float, default=SHARED_DEFAULTS.x_max)
    parser.add_argument("--y_min", type=float, default=SHARED_DEFAULTS.y_min)
    parser.add_argument("--y_max", type=float, default=SHARED_DEFAULTS.y_max)
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
        help="Spawn drones uniformly within the full movable arena instead of the default center disk region.",
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
    parser.add_argument(
        "--recent_hit_gain",
        type=float,
        default=SHARED_DEFAULTS.recent_hit_gain,
        help="Additive gain applied to the shared recent-presence map for each detection blob before clipping to [0, 1].",
    )
    parser.add_argument(
        "--recent_miss_penalty",
        type=float,
        default=SHARED_DEFAULTS.recent_miss_penalty,
        help="Multiplicative penalty applied to the shared recent-presence map in currently visible cells with no team detection support.",
    )
    parser.add_argument("--blob_sigma", type=float, default=SHARED_DEFAULTS.blob_sigma)
    parser.add_argument("--ego_sigma", type=float, default=SHARED_DEFAULTS.ego_sigma)
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
        help="Number of past (vx, vy, yaw_rate) commands appended to the actor local vector. 0=disabled.",
    )
    parser.add_argument("--reward_wc", type=float, default=SHARED_DEFAULTS.reward_wc)
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
        "--reward_coverage_edge_quality",
        type=float,
        default=SHARED_DEFAULTS.reward_coverage_edge_quality,
        help="Coverage quality floor for the worst visible framing locations. "
             "Higher values flatten the framing-quality reward.",
    )
    parser.add_argument(
        "--reward_quality_mode",
        type=str,
        choices=("legacy", "principal_linear", "principal_squared"),
        default=SHARED_DEFAULTS.reward_quality_mode,
        help="Framing-quality field used by the FOV quality reward.",
    )
    parser.add_argument(
        "--reward_boundary_margin",
        type=float,
        default=SHARED_DEFAULTS.reward_boundary_margin,
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
    parser.add_argument("--total_updates", type=int, default=TRAIN_DEFAULTS.total_updates)
    parser.add_argument("--rollout_len", type=int, default=TRAIN_DEFAULTS.rollout_len)
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


def build_env_kwargs(
    args: argparse.Namespace,
    overrides: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    kwargs = {
        "gui": args.gui,
        "x_min": args.x_min,
        "x_max": args.x_max,
        "y_min": args.y_min,
        "y_max": args.y_max,
        "drone_height": args.drone_height,
        "num_drones": args.num_drones,
        "random_spawn": args.random_spawn,
        "drone_spawn_radius": args.drone_spawn_radius,
        "fixed_active_num_drones": getattr(args, "fixed_active_num_drones", None),
        "num_people": args.num_people,
        "min_people": args.min_people,
        "max_people": args.max_people,
        "episode_steps": args.episode_steps,
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
        "camera_tilt_deg": args.camera_tilt_deg,
        "recent_half_life_seconds": args.recent_half_life_seconds,
        "historic_half_life_seconds": args.historic_half_life_seconds,
        "coverage_half_life_seconds": args.coverage_half_life_seconds,
        "recent_hit_gain": args.recent_hit_gain,
        "recent_miss_penalty": args.recent_miss_penalty,
        "grid_h": MODEL_DEFAULTS.grid_h,
        "grid_w": MODEL_DEFAULTS.grid_w,
        "blob_sigma": args.blob_sigma,
        "ego_sigma": args.ego_sigma,
        "reward_wc": args.reward_wc,
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
        "reward_coverage_edge_quality": args.reward_coverage_edge_quality,
        "reward_quality_mode": args.reward_quality_mode,
        "reward_boundary_margin": args.reward_boundary_margin,
        "reward_drone_closeness_margin": args.reward_drone_closeness_margin,
        "reward_fov_margin": args.reward_fov_margin,
        "vel_tau_s": args.vel_tau_s,
        "yaw_rate_tau_s": args.yaw_rate_tau_s,
        "cmd_history_len": args.cmd_history_len,
        "max_horizontal_velocity": args.max_horizontal_velocity,
        "max_yaw_rate": args.max_yaw_rate,
        "debug_observation_plots": args.debug_observation_plots,
        "debug_observation_plot_every": args.debug_observation_plot_every,
        "debug_reward_contours": args.debug_reward_contours,
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
) -> int:
    # Base local features:
    #   5 ego pose/visibility scalars (x, y, sin_yaw, cos_yaw, num_visible)
    #   3 explicit detection-centroid vs principal-point alignment scalars
    #   6 scalars per teammate slot
    #   3 scalars per command history entry (vx, vy, yaw_rate)
    base_dim = 8 + 6 * (num_drones - 1) + 3 * cmd_history_len
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
) -> Dict[str, Any]:
    return {
        "local_dim": local_dim(num_drones, action_space, cmd_history_len),
        "grid_channels": MODEL_DEFAULTS.grid_channels,
        "grid_h": MODEL_DEFAULTS.grid_h,
        "grid_w": MODEL_DEFAULTS.grid_w,
        "cnn_out_dim": MODEL_DEFAULTS.cnn_out_dim,
        "hidden_dim": MODEL_DEFAULTS.hidden_dim,
        "num_vx_bins": int(action_space.vx_bins.shape[0]),
        "num_vy_bins": int(action_space.vy_bins.shape[0]),
        "num_yaw_bins": action_space.num_yaw_bins,
    }


def trainer_kwargs(args: argparse.Namespace, action_space: DiscreteActionSpace) -> Dict[str, Any]:
    kwargs = actor_kwargs(args.num_drones, action_space, getattr(args, "cmd_history_len", 0))
    kwargs.update(
        {
            "rollout_len": args.rollout_len,
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
            "use_tensorboard": args.tensorboard,
            "live_debug": args.live_debug,
            "live_debug_every": args.live_debug_every,
            "sticky_action_prob": args.sticky_action_prob,
            "gui": args.gui,
        }
    )
    return kwargs
