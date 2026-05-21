"""
eval_metrics.py - offline metrics evaluation for dense crowd tracking.

Runs no-GUI evaluation for active drone counts 1 and 2 separately (when
supported by the checkpoint), aggregates episode metrics, and writes a
human-readable report to an output file.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import torch

from config import (
    add_eval_args,
    add_shared_args,
    actor_kwargs,
    build_action_space,
    build_env,
    infer_checkpoint_actor_grid_channels,
    infer_checkpoint_cmd_history_len,
    infer_checkpoint_hide_person_features_during_search,
    infer_checkpoint_hotspot_top_k,
    infer_checkpoint_include_instant_fov_channels,
    infer_checkpoint_include_local_recent_count_memory_channel,
    infer_checkpoint_include_persistent_coverage_channel,
    infer_checkpoint_include_shared_count_density_channel,
    infer_checkpoint_local_people_map_mode,
    infer_checkpoint_status_history_seconds,
)
from eval import (
    infer_checkpoint_local_dim,
    infer_trained_num_drones,
)
from rl.action_masking import append_move_masks_to_local, compute_move_action_masks
from rl.networks import ActorNetwork


DEFAULT_DRONE_COUNTS = (1, 2)
TRACK_LOSS_THRESHOLD_STEPS = 10
DENSEST_GROUP_HOLD_THRESHOLD = 0.5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    add_shared_args(parser)
    add_eval_args(parser)
    parser.add_argument(
        "--output",
        type=str,
        default="eval_metrics_report.txt",
        help="Path to the human-readable metrics report.",
    )
    parser.add_argument(
        "--drone_counts",
        type=str,
        default="1,2",
        help="Comma-separated active drone counts to evaluate, e.g. '1,2'.",
    )
    parser.add_argument(
        "--base_seed",
        type=int,
        default=0,
        help="Base episode seed. Episode i uses seed base_seed + i for all drone-count runs.",
    )
    return parser.parse_args()


def parse_drone_counts(raw_value: str) -> list[int]:
    counts: list[int] = []
    for token in raw_value.split(","):
        token = token.strip()
        if not token:
            continue
        value = int(token)
        if value <= 0:
            raise ValueError(f"drone count must be > 0, got {value}")
        counts.append(value)
    if not counts:
        return list(DEFAULT_DRONE_COUNTS)
    return sorted(set(counts))


def safe_mean(values: list[float]) -> float:
    if not values:
        return float("nan")
    return float(np.mean(np.asarray(values, dtype=np.float64)))


def safe_std(values: list[float]) -> float:
    if not values:
        return float("nan")
    return float(np.std(np.asarray(values, dtype=np.float64)))


def safe_percentile(values: list[float], percentile: float) -> float:
    if not values:
        return float("nan")
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile))


def safe_fraction(numerator: float, denominator: float) -> float:
    if denominator <= 0:
        return float("nan")
    return float(numerator / denominator)


def step_actions(obs, env, actor, action_space, device, deterministic: bool):
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
    grids = torch.FloatTensor(np.stack([o["grid"] for o in obs])).to(device)
    locs = torch.FloatTensor(
        append_move_masks_to_local(
            np.stack([o["local"] for o in obs]),
            move_masks,
        )
    ).to(device)
    move_masks_t = torch.FloatTensor(move_masks).to(device)

    with torch.no_grad():
        if deterministic:
            action_indices = actor.get_deterministic_action(
                grids,
                locs,
                move_mask=move_masks_t,
            )
        else:
            action_indices, _, _ = actor.get_action(
                grids,
                locs,
                move_mask=move_masks_t,
            )
    return action_space.decode_actions(action_indices.cpu().numpy())


def build_actor_and_env(
    args: argparse.Namespace,
    ckpt: dict,
    trained_num_drones: int,
    active_num_drones: int,
    device: torch.device,
):
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
            "fixed_active_num_drones": active_num_drones,
            "actor_grid_channels": trained_grid_channels,
            "include_persistent_coverage_channel": trained_include_persistent_coverage,
            "include_instant_fov_channels": trained_include_instant_fov_channels,
            "hide_person_features_during_search": trained_hide_person_features_during_search,
            "include_local_recent_count_memory_channel": (
                trained_include_local_recent_count_memory_channel
            ),
            "local_people_map_mode": trained_local_people_map_mode,
            "include_shared_count_density_channel": trained_include_shared_count_density,
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

    return env, actor, action_space


def densest_group_member_ids(group_info: dict, num_people: int) -> set[int]:
    group_sizes = {
        int(group_id): int(size)
        for group_id, size in group_info.get("group_sizes", {}).items()
    }
    if not group_sizes:
        return set()
    densest_group_id = min(
        group_sizes,
        key=lambda group_id: (-group_sizes[group_id], group_id),
    )
    assignments = list(group_info.get("group_assignments", []))[:num_people]
    return {
        person_id
        for person_id, group_id in enumerate(assignments)
        if group_id is not None and int(group_id) == densest_group_id
    }


def longest_streak(values: list[float], threshold: float) -> int:
    best = 0
    current = 0
    for value in values:
        if value >= threshold:
            current += 1
            if current > best:
                best = current
        else:
            current = 0
    return best


def compute_episode_metrics(
    env,
    actor,
    action_space,
    device: torch.device,
    deterministic: bool,
) -> dict[str, float]:
    obs = env.reset()
    num_people = len(env.people)
    group_info = dict(env.episode_group_info)
    densest_members = densest_group_member_ids(group_info, num_people)

    done = False
    step = 0
    ep_reward = 0.0
    ever_seen: set[int] = set()

    search_steps = 0
    search_new_discoveries = 0
    search_end_seen_ratio = 0.0
    search_time_to_50: int | None = None
    search_time_to_80: int | None = None
    search_densest_seen_ratio = float("nan")

    coverage_steps = 0
    coverage_ratios: list[float] = []
    coverage_qualities: list[float] = []
    coverage_overlap_ratios: list[float] = []
    coverage_densest_ratios: list[float] = []
    coverage_dual_idle_count = 0
    coverage_single_idle_count = 0

    person_visible_counts = [0 for _ in range(num_people)]
    person_switch_counts = [0 for _ in range(num_people)]
    person_prev_visible = [False for _ in range(num_people)]
    person_seen_once = [False for _ in range(num_people)]
    person_gap_start: list[int | None] = [None for _ in range(num_people)]
    reacquisition_latencies: list[int] = []
    total_loss_events = 0
    long_loss_events = 0

    while not done:
        step += 1
        env_actions = step_actions(
            obs=obs,
            env=env,
            actor=actor,
            action_space=action_space,
            device=device,
            deterministic=deterministic,
        )
        obs, reward, done, info = env.step(env_actions)
        ep_reward += float(reward)

        reward_info = info.get("reward_info", {})
        visible_ids_per_drone = info.get("visible_ids_per_drone", [])
        visible_union = set()
        idle_drones = 0
        for ids in visible_ids_per_drone:
            visible_union.update(ids)
            if len(ids) == 0:
                idle_drones += 1

        new_discoveries = visible_union - ever_seen
        ever_seen.update(visible_union)
        coverage_ratio = safe_fraction(len(visible_union), num_people)

        if reward_info.get("is_search_phase", 0.0) > 0.5:
            search_steps += 1
            search_new_discoveries += len(new_discoveries)
            search_end_seen_ratio = safe_fraction(len(ever_seen), num_people)
            if search_time_to_50 is None and search_end_seen_ratio >= 0.5:
                search_time_to_50 = search_steps
            if search_time_to_80 is None and search_end_seen_ratio >= 0.8:
                search_time_to_80 = search_steps
            if densest_members:
                search_densest_seen_ratio = safe_fraction(
                    len(ever_seen & densest_members),
                    len(densest_members),
                )

        if reward_info.get("is_coverage_phase", 0.0) > 0.5:
            coverage_steps += 1
            coverage_ratios.append(coverage_ratio)
            coverage_qualities.append(
                float(reward_info.get("mean_visible_quality", 0.0))
            )
            coverage_overlap_ratios.append(float(reward_info.get("r_ov", 0.0)))

            if len(visible_ids_per_drone) == 2:
                if idle_drones == 2:
                    coverage_dual_idle_count += 1
                elif idle_drones == 1:
                    coverage_single_idle_count += 1

            if densest_members:
                densest_ratio = safe_fraction(
                    len(visible_union & densest_members),
                    len(densest_members),
                )
                coverage_densest_ratios.append(densest_ratio)

            for person_id in range(num_people):
                visible = person_id in visible_union
                if visible:
                    person_visible_counts[person_id] += 1
                    person_seen_once[person_id] = True
                    if person_gap_start[person_id] is not None:
                        gap_len = coverage_steps - person_gap_start[person_id]
                        reacquisition_latencies.append(gap_len)
                        if gap_len > TRACK_LOSS_THRESHOLD_STEPS:
                            long_loss_events += 1
                        person_gap_start[person_id] = None
                else:
                    if (
                        person_seen_once[person_id]
                        and person_prev_visible[person_id]
                        and person_gap_start[person_id] is None
                    ):
                        person_gap_start[person_id] = coverage_steps
                        total_loss_events += 1

                if visible != person_prev_visible[person_id]:
                    person_switch_counts[person_id] += 1
                person_prev_visible[person_id] = visible

    if coverage_steps > 0:
        for gap_start in person_gap_start:
            if gap_start is None:
                continue
            gap_len = coverage_steps - gap_start + 1
            if gap_len > TRACK_LOSS_THRESHOLD_STEPS:
                long_loss_events += 1

    person_visibility_ratios = [
        safe_fraction(count, coverage_steps)
        for count in person_visible_counts
    ] if coverage_steps > 0 else []

    person_switch_rates = [
        safe_fraction(max(switch_count - 1, 0), max(coverage_steps - 1, 1))
        for switch_count in person_switch_counts
    ] if coverage_steps > 0 else []

    metrics = {
        "episode_reward": ep_reward,
        "episode_steps": float(step),
        "num_people": float(num_people),
        "search_steps": float(search_steps),
        "coverage_steps": float(coverage_steps),
        "search_end_seen_ratio": search_end_seen_ratio,
        "search_mean_new_seen_ratio_per_step": safe_fraction(
            search_new_discoveries,
            max(num_people * search_steps, 1),
        ),
        "search_reach_50_rate": 1.0 if search_time_to_50 is not None else 0.0,
        "search_reach_80_rate": 1.0 if search_time_to_80 is not None else 0.0,
        "search_time_to_50_steps": (
            float(search_time_to_50) if search_time_to_50 is not None else float("nan")
        ),
        "search_time_to_80_steps": (
            float(search_time_to_80) if search_time_to_80 is not None else float("nan")
        ),
        "densest_group_found_by_search_end_rate": (
            1.0 if densest_members and search_densest_seen_ratio > 0.0 else 0.0
        ),
        "search_densest_group_seen_ratio": search_densest_seen_ratio,
        "coverage_mean_ratio": safe_mean(coverage_ratios),
        "coverage_peak_ratio": max(coverage_ratios) if coverage_ratios else float("nan"),
        "coverage_p95_ratio": safe_percentile(coverage_ratios, 95.0),
        "coverage_time_above_70": safe_fraction(
            sum(value >= 0.70 for value in coverage_ratios),
            len(coverage_ratios),
        ),
        "coverage_time_above_90": safe_fraction(
            sum(value >= 0.90 for value in coverage_ratios),
            len(coverage_ratios),
        ),
        "coverage_mean_visible_quality": safe_mean(coverage_qualities),
        "coverage_stability_std": safe_std(coverage_ratios),
        "person_visibility_ratio_mean": safe_mean(person_visibility_ratios),
        "person_visibility_ratio_p10": safe_percentile(person_visibility_ratios, 10.0),
        "track_fragmentation_mean": safe_mean(person_switch_rates),
        "reacquisition_latency_steps_mean": safe_mean(reacquisition_latencies),
        "lost_track_rate_10": safe_fraction(long_loss_events, total_loss_events),
        "densest_group_mean_coverage": safe_mean(coverage_densest_ratios),
        "densest_group_peak_coverage": (
            max(coverage_densest_ratios)
            if coverage_densest_ratios else float("nan")
        ),
        "densest_group_tracking_time_above_50": safe_fraction(
            sum(value >= DENSEST_GROUP_HOLD_THRESHOLD for value in coverage_densest_ratios),
            len(coverage_densest_ratios),
        ),
        "densest_group_longest_hold_above_50_steps": float(
            longest_streak(coverage_densest_ratios, DENSEST_GROUP_HOLD_THRESHOLD)
        ) if coverage_densest_ratios else float("nan"),
        "coverage_overlap_ratio_mean": safe_mean(coverage_overlap_ratios),
        "coverage_dual_idle_rate": safe_fraction(
            coverage_dual_idle_count,
            coverage_steps,
        ),
        "coverage_single_idle_rate": safe_fraction(
            coverage_single_idle_count,
            coverage_steps,
        ),
    }
    return metrics


def aggregate_metrics(per_episode_metrics: list[dict[str, float]]) -> dict[str, float]:
    if not per_episode_metrics:
        return {}

    aggregated: dict[str, float] = {}
    metric_names = sorted(per_episode_metrics[0].keys())
    for metric_name in metric_names:
        values = [
            float(metrics[metric_name])
            for metrics in per_episode_metrics
            if metric_name in metrics and not math.isnan(float(metrics[metric_name]))
        ]
        aggregated[metric_name] = safe_mean(values)
    aggregated["episodes_run"] = float(len(per_episode_metrics))
    return aggregated


def evaluate_drone_count(
    args: argparse.Namespace,
    ckpt: dict,
    trained_num_drones: int,
    active_num_drones: int,
    device: torch.device,
) -> list[dict[str, float]]:
    env, actor, action_space = build_actor_and_env(
        args=args,
        ckpt=ckpt,
        trained_num_drones=trained_num_drones,
        active_num_drones=active_num_drones,
        device=device,
    )
    try:
        per_episode_metrics: list[dict[str, float]] = []
        for episode_idx in range(args.episodes):
            env.set_episode_seed(args.base_seed + episode_idx)
            metrics = compute_episode_metrics(
                env=env,
                actor=actor,
                action_space=action_space,
                device=device,
                deterministic=args.deterministic,
            )
            metrics["episode_seed"] = float(args.base_seed + episode_idx)
            per_episode_metrics.append(metrics)
    finally:
        env.close()
    return per_episode_metrics


def compute_paired_deltas(
    per_drone_episode_metrics: dict[int, list[dict[str, float]]],
) -> dict[str, float]:
    if 1 not in per_drone_episode_metrics or 2 not in per_drone_episode_metrics:
        return {}

    metrics_1 = per_drone_episode_metrics[1]
    metrics_2 = per_drone_episode_metrics[2]
    if len(metrics_1) != len(metrics_2):
        return {}

    paired_metric_names = (
        "search_end_seen_ratio",
        "coverage_mean_ratio",
        "coverage_peak_ratio",
        "densest_group_mean_coverage",
        "person_visibility_ratio_mean",
    )
    paired: dict[str, float] = {}
    for metric_name in paired_metric_names:
        deltas = []
        wins = 0
        losses = 0
        for episode_metrics_1, episode_metrics_2 in zip(metrics_1, metrics_2):
            value_1 = float(episode_metrics_1.get(metric_name, float("nan")))
            value_2 = float(episode_metrics_2.get(metric_name, float("nan")))
            if math.isnan(value_1) or math.isnan(value_2):
                continue
            delta = value_2 - value_1
            deltas.append(delta)
            if delta > 0.0:
                wins += 1
            elif delta < 0.0:
                losses += 1
        paired[f"delta_{metric_name}"] = safe_mean(deltas)
        paired[f"win_rate_{metric_name}"] = safe_fraction(wins, wins + losses)
    return paired


def format_metric_value(value: float) -> str:
    if math.isnan(value):
        return "nan"
    if abs(value) >= 1000.0:
        return f"{value:.1f}"
    return f"{value:.4f}"


def render_report(
    checkpoint_path: str,
    episodes: int,
    base_seed: int,
    per_drone_results: dict[int, dict[str, float]],
    paired_deltas: dict[str, float],
) -> str:
    lines = [
        "Dense Crowd Tracking Evaluation Report",
        f"checkpoint: {checkpoint_path}",
        f"episodes_per_drone_count: {episodes}",
        f"paired_episode_seeds: {base_seed}..{base_seed + episodes - 1}",
        "",
        "Metric notes:",
        "- 1-drone and 2-drone runs use the same per-episode crowd/layout/motion seeds",
        f"- search_mean_new_seen_ratio_per_step = newly discovered people / (num_people * search_steps)",
        f"- lost_track_rate_10 uses {TRACK_LOSS_THRESHOLD_STEPS} coverage steps as the loss threshold",
        f"- densest_group_* metrics use the largest ground-truth group in each episode",
        f"- densest_group_tracking_time_above_50 uses threshold {DENSEST_GROUP_HOLD_THRESHOLD:.2f}",
        "",
    ]

    ordered_metrics = [
        "episodes_run",
        "episode_reward",
        "episode_steps",
        "num_people",
        "search_steps",
        "coverage_steps",
        "search_end_seen_ratio",
        "search_mean_new_seen_ratio_per_step",
        "search_reach_50_rate",
        "search_time_to_50_steps",
        "search_reach_80_rate",
        "search_time_to_80_steps",
        "densest_group_found_by_search_end_rate",
        "search_densest_group_seen_ratio",
        "coverage_mean_ratio",
        "coverage_peak_ratio",
        "coverage_p95_ratio",
        "coverage_time_above_70",
        "coverage_time_above_90",
        "coverage_mean_visible_quality",
        "coverage_stability_std",
        "person_visibility_ratio_mean",
        "person_visibility_ratio_p10",
        "track_fragmentation_mean",
        "reacquisition_latency_steps_mean",
        "lost_track_rate_10",
        "densest_group_mean_coverage",
        "densest_group_peak_coverage",
        "densest_group_tracking_time_above_50",
        "densest_group_longest_hold_above_50_steps",
        "coverage_overlap_ratio_mean",
        "coverage_dual_idle_rate",
        "coverage_single_idle_rate",
    ]

    for active_num_drones in sorted(per_drone_results):
        metrics = per_drone_results[active_num_drones]
        lines.append(f"{active_num_drones} Active Drone(s)")
        for metric_name in ordered_metrics:
            if metric_name not in metrics:
                continue
            if active_num_drones == 1 and metric_name in {
                "coverage_overlap_ratio_mean",
                "coverage_dual_idle_rate",
                "coverage_single_idle_rate",
            }:
                continue
            lines.append(f"- {metric_name}: {format_metric_value(metrics[metric_name])}")
        lines.append("")

    if paired_deltas:
        lines.append("2 Drone Gain Over 1 Drone")
        for metric_name in (
            "delta_search_end_seen_ratio",
            "win_rate_search_end_seen_ratio",
            "delta_coverage_mean_ratio",
            "win_rate_coverage_mean_ratio",
            "delta_coverage_peak_ratio",
            "win_rate_coverage_peak_ratio",
            "delta_densest_group_mean_coverage",
            "win_rate_densest_group_mean_coverage",
            "delta_person_visibility_ratio_mean",
            "win_rate_person_visibility_ratio_mean",
        ):
            if metric_name not in paired_deltas:
                continue
            lines.append(f"- {metric_name}: {format_metric_value(paired_deltas[metric_name])}")
        lines.append("")

    return "\n".join(lines).rstrip() + "\n"


def main() -> None:
    args = parse_args()
    args.gui = False
    args.realtime = False
    args.show_drone0_inputs = False

    device = torch.device(args.device)
    ckpt = torch.load(args.load, map_location=device)
    trained_num_drones = infer_trained_num_drones(ckpt)

    requested_counts = parse_drone_counts(args.drone_counts)
    active_drone_counts = [
        count for count in requested_counts
        if count <= trained_num_drones
    ]
    if not active_drone_counts:
        raise ValueError(
            "No requested drone counts are supported by this checkpoint: "
            f"requested={requested_counts}, trained_num_drones={trained_num_drones}"
        )

    per_drone_episode_metrics: dict[int, list[dict[str, float]]] = {}
    per_drone_results: dict[int, dict[str, float]] = {}
    for active_num_drones in active_drone_counts:
        per_drone_episode_metrics[active_num_drones] = evaluate_drone_count(
            args=args,
            ckpt=ckpt,
            trained_num_drones=trained_num_drones,
            active_num_drones=active_num_drones,
            device=device,
        )
        per_drone_results[active_num_drones] = aggregate_metrics(
            per_drone_episode_metrics[active_num_drones]
        )

    paired_deltas = compute_paired_deltas(per_drone_episode_metrics)

    report = render_report(
        checkpoint_path=args.load,
        episodes=args.episodes,
        base_seed=args.base_seed,
        per_drone_results=per_drone_results,
        paired_deltas=paired_deltas,
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(report, encoding="utf-8")
    print(f"[EvalMetrics] wrote report to {output_path}")


if __name__ == "__main__":
    main()
