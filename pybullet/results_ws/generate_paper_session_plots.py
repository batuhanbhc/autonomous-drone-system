"""
Generate paper-ready eval-session figures for a single seeded episode.

Outputs four images by default:
  - search_topdown.png
  - search_drone0_cnn.png
  - coverage_topdown.png
  - coverage_drone0_cnn.png
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import textwrap
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import torch

from config import (
    SHARED_DEFAULTS,
    actor_kwargs,
    add_shared_args,
    build_action_space,
    build_env,
    infer_checkpoint_actor_grid_channels,
    infer_checkpoint_cmd_history_len,
    infer_checkpoint_hide_person_features_during_search,
    infer_checkpoint_include_instant_fov_channels,
    infer_checkpoint_include_local_recent_count_memory_channel,
    infer_checkpoint_include_persistent_coverage_channel,
    infer_checkpoint_reward_use_base_person_weight,
    infer_checkpoint_include_shared_count_density_channel,
    infer_checkpoint_include_shared_count_memory_staleness_channel,
    infer_checkpoint_hotspot_top_k,
    infer_checkpoint_local_people_map_mode,
    infer_checkpoint_status_history_seconds,
)
from eval import (
    infer_checkpoint_local_dim,
    infer_trained_num_drones,
    resolve_eval_active_num_drones,
)
from rl.action_masking import append_move_masks_to_local, compute_move_action_masks
from rl.networks import ActorNetwork
from sim.camera_geometry import footprint_corners_world, principal_point_world


DRONE_COLORS = ["#0f766e", "#ea580c", "#4f46e5", "#b91c1c"]


@dataclass
class PhaseSnapshot:
    phase_name: str
    decision_step: int
    sim_time_seconds: float
    grid: np.ndarray
    local_vec: np.ndarray
    drone_states: list[dict[str, Any]]
    people_positions: list[tuple[float, float, float]]
    ever_seen_count: int
    active_drones: int
    episode_steps: int
    search_phase_steps: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    add_shared_args(parser)
    parser.add_argument(
        "--load",
        type=str,
        default=None,
        help="Checkpoint path. Relative paths are resolved from the current working directory.",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=None,
        help="Directory for generated images. Defaults to results_ws/paper_session_plots/<run>.",
    )
    parser.add_argument("--episode_seed", type=int, default=0)
    parser.add_argument(
        "--deterministic",
        dest="deterministic",
        action="store_true",
        default=True,
    )
    parser.add_argument(
        "--stochastic",
        dest="deterministic",
        action="store_false",
    )
    parser.add_argument("--topdown_dpi", type=int, default=220)
    parser.add_argument("--cnn_dpi", type=int, default=220)
    parser.add_argument("--cnn_num_cols", type=int, default=4)
    return parser.parse_args()


def resolve_checkpoint_path(raw_path: str | None) -> Path:
    if raw_path:
        path = Path(raw_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Checkpoint not found: {path}")
        return path

    candidates = sorted(
        (PROJECT_ROOT / "checkpoints").glob("**/final.pt"),
        key=lambda item: item.stat().st_mtime,
    )
    if not candidates:
        raise FileNotFoundError(
            f"No checkpoint specified and no final.pt found under {PROJECT_ROOT / 'checkpoints'}."
        )
    return candidates[-1].resolve()


def default_output_dir(load_path: Path, episode_seed: int) -> Path:
    run_name = load_path.parent.name
    ckpt_name = load_path.stem
    return (
        Path(__file__).resolve().parent
        / "paper_session_plots"
        / f"{run_name}_{ckpt_name}_seed{episode_seed:04d}"
    )


def choose_capture_steps(search_phase_steps: int, episode_steps: int) -> dict[str, int]:
    if search_phase_steps <= 0:
        raise ValueError("This configuration has no search phase to capture.")
    if search_phase_steps >= episode_steps:
        raise ValueError("This configuration has no coverage phase to capture.")

    coverage_steps = episode_steps - search_phase_steps
    return {
        "search": max(1, (search_phase_steps + 1) // 2),
        "coverage": search_phase_steps + max(1, (coverage_steps + 1) // 2),
    }


def select_env_actions(
    env,
    obs,
    actor: ActorNetwork,
    action_space,
    device: torch.device,
    deterministic: bool,
) -> tuple[list[tuple[float, float, float, float]], np.ndarray]:
    move_masks = compute_move_action_masks(
        drone_states=env._get_drone_states(),
        vx_bins=action_space.vx_bins,
        vy_bins=action_space.vy_bins,
        dt=env.dt,
        x_min=env.move_x_min,
        x_max=env.move_x_max,
        y_min=env.move_y_min,
        y_max=env.move_y_max,
    )
    grids = torch.as_tensor(
        np.stack([o["grid"] for o in obs]),
        dtype=torch.float32,
        device=device,
    )
    local_with_mask = append_move_masks_to_local(
        np.stack([o["local"] for o in obs]),
        move_masks,
    )
    locs = torch.as_tensor(local_with_mask, dtype=torch.float32, device=device)
    move_masks_t = torch.as_tensor(move_masks, dtype=torch.float32, device=device)

    with torch.no_grad():
        if deterministic:
            action_indices = actor.get_deterministic_action(
                grids,
                locs,
                move_mask=move_masks_t,
            )
        else:
            action_indices, _, _ = actor.get_action(grids, locs, move_mask=move_masks_t)

    return action_space.decode_actions(action_indices.cpu().numpy()), local_with_mask


def capture_snapshot(env, obs, action_space, phase_name: str, decision_step: int) -> PhaseSnapshot:
    move_masks = compute_move_action_masks(
        drone_states=env._get_drone_states(),
        vx_bins=action_space.vx_bins,
        vy_bins=action_space.vy_bins,
        dt=env.dt,
        x_min=env.move_x_min,
        x_max=env.move_x_max,
        y_min=env.move_y_min,
        y_max=env.move_y_max,
    )
    local_with_mask = append_move_masks_to_local(
        np.stack([o["local"] for o in obs]),
        move_masks,
    )

    return PhaseSnapshot(
        phase_name=phase_name,
        decision_step=int(decision_step),
        sim_time_seconds=max(0.0, float(decision_step - 1) * env.dt),
        grid=np.asarray(obs[0]["grid"], dtype=np.float32).copy(),
        local_vec=np.asarray(local_with_mask[0], dtype=np.float32).copy(),
        drone_states=[dict(state) for state in env._get_drone_states()],
        people_positions=[
            tuple(float(value) for value in position)
            for position in env._get_people_positions()
        ],
        ever_seen_count=int(len(env.ever_seen)),
        active_drones=int(len(env.drones)),
        episode_steps=int(env.episode_steps),
        search_phase_steps=int(env.search_phase_steps),
    )


def run_episode_and_capture(
    env,
    actor: ActorNetwork,
    action_space,
    device: torch.device,
    deterministic: bool,
    capture_steps: dict[str, int],
) -> dict[str, PhaseSnapshot]:
    obs = env.reset()
    current_decision_step = 1
    snapshots: dict[str, PhaseSnapshot] = {}

    ordered_targets = sorted(capture_steps.items(), key=lambda item: item[1])
    for phase_name, target_step in ordered_targets:
        while current_decision_step < target_step:
            env_actions, _ = select_env_actions(
                env=env,
                obs=obs,
                actor=actor,
                action_space=action_space,
                device=device,
                deterministic=deterministic,
            )
            obs, _, done, _ = env.step(env_actions)
            current_decision_step += 1
            if done and current_decision_step <= target_step:
                raise RuntimeError(
                    f"Episode ended before reaching the {phase_name} capture step {target_step}."
                )

        snapshots[phase_name] = capture_snapshot(
            env=env,
            obs=obs,
            action_space=action_space,
            phase_name=phase_name,
            decision_step=current_decision_step,
        )

    return snapshots


def import_matplotlib():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import lines as mlines
    from matplotlib import patches

    return plt, patches, mlines


def wrap_label(label: str, width: int = 22) -> str:
    return "\n".join(textwrap.wrap(label, width=width, break_long_words=False))


def channel_cmap(channel_name: str) -> str:
    return "magma"


def rotated_box_corners(x: float, y: float, yaw: float, half_x: float, half_y: float) -> np.ndarray:
    base = np.array(
        [
            [-half_x, -half_y],
            [half_x, -half_y],
            [half_x, half_y],
            [-half_x, half_y],
        ],
        dtype=np.float32,
    )
    rotation = np.array(
        [
            [math.cos(yaw), -math.sin(yaw)],
            [math.sin(yaw), math.cos(yaw)],
        ],
        dtype=np.float32,
    )
    return (base @ rotation.T) + np.array([x, y], dtype=np.float32)


def add_phase_badge(ax, phase_name: str) -> None:
    ax.text(
        0.02,
        0.98,
        phase_name.upper(),
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=12,
        fontweight="bold",
        color="#111827",
        bbox=dict(
            boxstyle="round,pad=0.28",
            facecolor="white",
            edgecolor="#d1d5db",
            alpha=0.92,
        ),
        zorder=20,
    )


def render_topdown_snapshot(snapshot: PhaseSnapshot, env, output_path: Path, dpi: int) -> None:
    plt, patches, mlines = import_matplotlib()

    plt.rcParams.update(
        {
            "font.family": "DejaVu Serif",
            "axes.facecolor": "#f7f4ed",
            "figure.facecolor": "white",
        }
    )

    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    arena_w = env.x_max - env.x_min
    arena_h = env.y_max - env.y_min
    pad = 0.08 * max(arena_w, arena_h)
    wall_thickness = 0.28

    ax.add_patch(
        patches.Rectangle(
            (env.x_min, env.y_min),
            arena_w,
            arena_h,
            facecolor="#efe8d8",
            edgecolor="#3f3f46",
            linewidth=1.8,
            zorder=0,
        )
    )

    wall_specs = [
        (env.x_min - wall_thickness, env.y_min - wall_thickness, wall_thickness, arena_h + 2 * wall_thickness),
        (env.x_max, env.y_min - wall_thickness, wall_thickness, arena_h + 2 * wall_thickness),
        (env.x_min, env.y_min - wall_thickness, arena_w, wall_thickness),
        (env.x_min, env.y_max, arena_w, wall_thickness),
    ]
    for wx, wy, ww, wh in wall_specs:
        ax.add_patch(
            patches.Rectangle(
                (wx, wy),
                ww,
                wh,
                facecolor="#8b8d96",
                edgecolor="none",
                alpha=0.9,
                zorder=1,
            )
        )

    for person_x, person_y, _ in snapshot.people_positions:
        ax.add_patch(
            patches.Circle(
                (person_x, person_y),
                radius=0.22,
                facecolor="#d94841",
                edgecolor="white",
                linewidth=0.7,
                alpha=0.95,
                zorder=4,
            )
        )

    legend_handles = [
        patches.Patch(facecolor="#d94841", edgecolor="white", label="People"),
    ]

    for drone_idx, drone_state in enumerate(snapshot.drone_states):
        color = DRONE_COLORS[drone_idx % len(DRONE_COLORS)]
        x, y, z = drone_state["position"]
        yaw = float(drone_state["yaw"])
        drone_corners = rotated_box_corners(x, y, yaw, 0.28, 0.28)
        ax.add_patch(
            patches.Polygon(
                drone_corners,
                closed=True,
                facecolor=color,
                edgecolor="white",
                linewidth=1.2,
                alpha=0.96,
                zorder=7,
            )
        )
        ax.annotate(
            "",
            xy=(x + 1.05 * math.cos(yaw), y + 1.05 * math.sin(yaw)),
            xytext=(x, y),
            arrowprops=dict(arrowstyle="-|>", color="white", lw=1.6),
            zorder=8,
        )

        footprint = footprint_corners_world(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            camera_tilt_deg=env.debug_drawer.tilt_deg,
            horizontal_fov_deg=env.debug_drawer.horizontal_fov_deg,
            vertical_fov_deg=env.debug_drawer.vertical_fov_deg,
        )
        principal_x, principal_y = principal_point_world(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            camera_tilt_deg=env.debug_drawer.tilt_deg,
        )
        ax.add_patch(
            patches.Polygon(
                footprint,
                closed=True,
                facecolor=color,
                edgecolor=color,
                linewidth=2.0,
                alpha=0.14,
                zorder=2,
            )
        )
        ax.plot(
            [x, principal_x],
            [y, principal_y],
            color=color,
            linewidth=1.4,
            linestyle="--",
            alpha=0.95,
            zorder=6,
        )
        ax.scatter(
            [principal_x],
            [principal_y],
            s=28,
            color=color,
            edgecolors="white",
            linewidths=0.7,
            zorder=8,
        )
        ax.text(
            x + 0.35,
            y + 0.35,
            f"D{drone_idx}",
            color="#111827",
            fontsize=11,
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.18", facecolor="white", edgecolor="none", alpha=0.85),
            zorder=9,
        )
        legend_handles.append(
            mlines.Line2D([], [], color=color, lw=2.0, label=f"Drone {drone_idx} FOV")
        )

    add_phase_badge(ax, snapshot.phase_name)
    ax.legend(
        handles=legend_handles,
        loc="upper right",
        frameon=True,
        facecolor="white",
        edgecolor="#d1d5db",
        framealpha=0.92,
        fontsize=9,
    )
    ax.set_xlim(env.x_min - pad, env.x_max + pad)
    ax.set_ylim(env.y_min - pad, env.y_max + pad)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)

    fig.subplots_adjust(left=0.02, right=0.985, bottom=0.02, top=0.985)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight", pad_inches=0.03)
    plt.close(fig)


def render_cnn_snapshot(
    snapshot: PhaseSnapshot,
    channel_names: list[str],
    output_path: Path,
    dpi: int,
    num_cols: int,
) -> None:
    plt, _, _ = import_matplotlib()

    plt.rcParams.update(
        {
            "font.family": "DejaVu Serif",
            "axes.facecolor": "#fcfcfb",
            "figure.facecolor": "white",
        }
    )

    grid = np.asarray(snapshot.grid, dtype=np.float32)
    num_channels = int(grid.shape[0])
    num_cols = max(1, int(num_cols))
    num_rows = int(math.ceil(num_channels / num_cols))

    fig, axes = plt.subplots(
        num_rows,
        num_cols,
        figsize=(num_cols * 3.0, num_rows * 3.0 + 0.8),
        squeeze=False,
    )

    for channel_idx in range(num_rows * num_cols):
        ax = axes[channel_idx // num_cols][channel_idx % num_cols]
        if channel_idx >= num_channels:
            ax.axis("off")
            continue

        channel = grid[channel_idx]
        channel_name = (
            channel_names[channel_idx]
            if channel_idx < len(channel_names)
            else f"Channel {channel_idx}"
        )
        ax.imshow(
            channel,
            origin="lower",
            cmap=channel_cmap(channel_name),
            vmin=0.0,
            vmax=1.0,
            interpolation="nearest",
        )
        ax.text(
            0.03,
            0.97,
            wrap_label(channel_name, width=18),
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=9.5,
            color="#111827",
            bbox=dict(
                boxstyle="round,pad=0.22",
                facecolor="white",
                edgecolor="#e5e7eb",
                alpha=0.9,
            ),
        )
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_color("#d1d5db")
            spine.set_linewidth(0.8)

    fig.text(
        0.02,
        0.925,
        snapshot.phase_name.upper(),
        ha="left",
        va="top",
        fontsize=12,
        fontweight="bold",
        color="#111827",
        bbox=dict(
            boxstyle="round,pad=0.28",
            facecolor="white",
            edgecolor="#d1d5db",
            alpha=0.92,
        ),
    )
    fig.subplots_adjust(left=0.02, right=0.985, bottom=0.02, top=0.875, wspace=0.08, hspace=0.12)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    load_path = resolve_checkpoint_path(args.load)
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else default_output_dir(load_path, args.episode_seed).resolve()
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.device == SHARED_DEFAULTS.device and not torch.cuda.is_available():
        args.device = "cpu"
    device = torch.device(args.device)

    ckpt = torch.load(load_path, map_location=device)
    trained_num_drones = infer_trained_num_drones(ckpt)
    if trained_num_drones < 2:
        raise ValueError(
            f"Checkpoint supports only {trained_num_drones} drone(s); a 2-drone session is required."
        )

    trained_grid_channels = infer_checkpoint_actor_grid_channels(ckpt)
    trained_include_persistent_coverage = (
        infer_checkpoint_include_persistent_coverage_channel(ckpt)
    )
    trained_include_instant_fov_channels = (
        infer_checkpoint_include_instant_fov_channels(ckpt)
    )
    trained_hide_person_features_during_search = (
        infer_checkpoint_hide_person_features_during_search(ckpt)
    )
    trained_include_local_recent_count_memory_channel = (
        infer_checkpoint_include_local_recent_count_memory_channel(ckpt)
    )
    trained_local_people_map_mode = infer_checkpoint_local_people_map_mode(ckpt)
    trained_include_shared_count_density = (
        infer_checkpoint_include_shared_count_density_channel(ckpt)
    )
    trained_include_shared_count_memory_staleness = (
        infer_checkpoint_include_shared_count_memory_staleness_channel(ckpt)
    )
    trained_reward_use_base_person_weight = (
        infer_checkpoint_reward_use_base_person_weight(ckpt)
    )

    requested_active_num_drones = resolve_eval_active_num_drones(
        requested_num_drones=2,
        trained_num_drones=trained_num_drones,
    )
    action_space = build_action_space(args)
    args.cmd_history_len = infer_checkpoint_cmd_history_len(
        ckpt,
        num_drones=trained_num_drones,
        action_space=action_space,
    )
    args.status_history_seconds = infer_checkpoint_status_history_seconds(ckpt)
    trained_hotspot_top_k = infer_checkpoint_hotspot_top_k(
        ckpt,
        num_drones=trained_num_drones,
        action_space=action_space,
        cmd_history_len=args.cmd_history_len,
        status_history_seconds=args.status_history_seconds,
    )

    env = build_env(
        args,
        overrides={
            "gui": False,
            "num_drones": trained_num_drones,
            "fixed_active_num_drones": requested_active_num_drones,
            "actor_grid_channels": trained_grid_channels,
            "include_persistent_coverage_channel": trained_include_persistent_coverage,
            "include_instant_fov_channels": trained_include_instant_fov_channels,
            "hide_person_features_during_search": trained_hide_person_features_during_search,
            "include_local_recent_count_memory_channel": (
                trained_include_local_recent_count_memory_channel
            ),
            "local_people_map_mode": trained_local_people_map_mode,
            "include_shared_count_density_channel": trained_include_shared_count_density,
            "include_shared_count_memory_staleness_channel": (
                trained_include_shared_count_memory_staleness
            ),
            "reward_use_base_person_weight": trained_reward_use_base_person_weight,
            "hotspot_top_k": trained_hotspot_top_k,
        },
    )

    actor_config = actor_kwargs(
        trained_num_drones,
        action_space,
        cmd_history_len=args.cmd_history_len,
        status_history_seconds=args.status_history_seconds,
        hotspot_top_k=trained_hotspot_top_k,
        grid_channels=trained_grid_channels,
        include_local_recent_count_memory_channel=(
            trained_include_local_recent_count_memory_channel
        ),
        include_instant_fov_channels=trained_include_instant_fov_channels,
        include_persistent_coverage_channel=trained_include_persistent_coverage,
    )
    actor_config["local_dim"] = infer_checkpoint_local_dim(ckpt)
    actor = ActorNetwork(**actor_config).to(device)
    actor.load_state_dict(ckpt["actor"])
    actor.eval()

    env.set_episode_seed(args.episode_seed)
    capture_steps = choose_capture_steps(
        search_phase_steps=env.search_phase_steps,
        episode_steps=env.episode_steps,
    )

    try:
        snapshots = run_episode_and_capture(
            env=env,
            actor=actor,
            action_space=action_space,
            device=device,
            deterministic=args.deterministic,
            capture_steps=capture_steps,
        )

        channel_names = list(env.obs_builder.actor_channel_names)
        generated_files: dict[str, str] = {}
        for phase_name in ("search", "coverage"):
            snapshot = snapshots[phase_name]
            topdown_path = output_dir / f"{phase_name}_topdown.png"
            cnn_path = output_dir / f"{phase_name}_drone0_cnn.png"
            render_topdown_snapshot(snapshot, env, topdown_path, args.topdown_dpi)
            render_cnn_snapshot(
                snapshot,
                channel_names,
                cnn_path,
                args.cnn_dpi,
                args.cnn_num_cols,
            )
            generated_files[f"{phase_name}_topdown"] = str(topdown_path)
            generated_files[f"{phase_name}_drone0_cnn"] = str(cnn_path)

        metadata = {
            "checkpoint": str(load_path),
            "output_dir": str(output_dir),
            "episode_seed": int(args.episode_seed),
            "deterministic": bool(args.deterministic),
            "trained_num_drones": int(trained_num_drones),
            "active_num_drones": int(requested_active_num_drones),
            "search_phase_steps": int(env.search_phase_steps),
            "episode_steps": int(env.episode_steps),
            "capture_steps": capture_steps,
            "files": generated_files,
        }
        metadata_path = output_dir / "session_metadata.json"
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

        print(f"[PaperPlots] checkpoint={load_path}")
        print(f"[PaperPlots] output_dir={output_dir}")
        for label, file_path in generated_files.items():
            print(f"[PaperPlots] {label}={file_path}")
        print(f"[PaperPlots] metadata={metadata_path}")
    finally:
        env.close()


if __name__ == "__main__":
    main()
