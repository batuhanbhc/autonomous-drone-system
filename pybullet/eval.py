"""
eval.py — run a trained actor in the environment (no training).

Usage:
    ./run_eval.sh --load checkpoints/final.pt --gui
    ./run_eval.sh --load checkpoints/final.pt --gui --deterministic
    ./run_eval.sh --load checkpoints/final.pt --num_drones 1 --gui
"""

import argparse
import time
import numpy as np
import pybullet as p
import torch

from config import (
    SHARED_DEFAULTS,
    add_eval_args,
    add_shared_args,
    actor_kwargs,
    build_action_space,
    build_env,
    infer_checkpoint_actor_grid_channels,
    infer_checkpoint_cmd_history_len,
    infer_checkpoint_enable_agent_ids,
    infer_checkpoint_hide_person_features_during_search,
    infer_checkpoint_include_local_recent_count_memory_channel,
    infer_checkpoint_include_instant_fov_channels,
    infer_checkpoint_include_shared_count_memory_staleness_channel,
    infer_checkpoint_include_shared_count_density_channel,
    infer_checkpoint_include_persistent_coverage_channel,
    infer_checkpoint_reward_person_weight_mode,
    infer_checkpoint_hotspot_top_k,
    infer_checkpoint_local_people_map_mode,
    infer_checkpoint_status_history_seconds,
)
from rl.action_masking import append_move_masks_to_local, compute_move_action_masks
from rl.live_debug import LiveDebugConfig, LiveDebugWindow
from rl.networks import ActorNetwork


def parse_args():
    parser = argparse.ArgumentParser()
    add_shared_args(parser)
    add_eval_args(parser)
    return parser.parse_args()


def infer_trained_num_drones(ckpt: dict) -> int:
    enable_agent_ids = infer_checkpoint_enable_agent_ids(ckpt)

    def infer_from_base_dim(total_local_dim: int) -> int | None:
        for base_dim in (13, 12, 11, 9, 8, 6):
            adjusted_base_dim = base_dim + int(enable_agent_ids)
            if (
                total_local_dim >= adjusted_base_dim
                and (total_local_dim - adjusted_base_dim) % 6 == 0
            ):
                return ((total_local_dim - adjusted_base_dim) // 6) + 1
        return None

    if "num_agents" in ckpt:
        return int(ckpt["num_agents"])

    actor_state = ckpt["actor"]
    local_weight = actor_state["local_mlp.0.weight"]
    local_dim = int(local_weight.shape[1])
    move_head_weight = actor_state.get("move_head.weight")
    if move_head_weight is not None:
        move_mask_dim = int(move_head_weight.shape[0])
        base_local_dim = local_dim - move_mask_dim
        inferred = infer_from_base_dim(base_local_dim)
        if inferred is None:
            raise ValueError(
                "Could not infer trained num_drones from checkpoint: "
                f"local_dim={local_dim}, move_mask_dim={move_mask_dim}"
            )
        return inferred

    inferred = infer_from_base_dim(local_dim)
    if inferred is None:
        raise ValueError(
            "Could not infer trained num_drones from checkpoint: "
            f"unexpected local_dim={local_dim}"
        )
    return inferred


def resolve_eval_active_num_drones(
    requested_num_drones: int,
    trained_num_drones: int,
) -> int:
    if requested_num_drones == trained_num_drones:
        return trained_num_drones

    # If eval kept the shared default but the checkpoint was trained with a
    # different max drone count, prefer the checkpoint architecture.
    if (
        requested_num_drones == SHARED_DEFAULTS.num_drones
        and trained_num_drones != SHARED_DEFAULTS.num_drones
    ):
        return trained_num_drones

    if 1 <= requested_num_drones <= trained_num_drones:
        return requested_num_drones

    raise ValueError(
        f"Requested eval num_drones={requested_num_drones}, but checkpoint was "
        f"trained with num_drones={trained_num_drones}. "
        "For eval, pass a value within [1, trained_num_drones]."
    )


def infer_checkpoint_local_dim(ckpt: dict) -> int:
    if "local_dim" in ckpt:
        return int(ckpt["local_dim"])
    actor_state = ckpt["actor"]
    local_weight = actor_state["local_mlp.0.weight"]
    return int(local_weight.shape[1])


def current_eval_phase_name(env) -> str:
    phase_context = env.get_observation_phase_context()
    return "SEARCH" if phase_context.get("is_search_phase", 0.0) > 0.5 else "COVERAGE"


def resolve_eval_rtf(args) -> float | None:
    rtf = args.rtf
    if rtf is not None:
        if rtf <= 0:
            raise ValueError(f"--rtf must be > 0, got {rtf}")
        return float(rtf)
    if args.realtime:
        return 1.0
    return None


def sync_phase_gui_param(param_id: int | None, phase_name: str) -> int | None:
    if param_id is not None:
        try:
            p.removeUserDebugItem(param_id)
        except Exception:
            pass

    try:
        return p.addUserDebugParameter(f"Phase: {phase_name}", 1, 0, 1)
    except Exception:
        return None


def run_episode(env, actor, action_space, device, args, live_debug=None):
    obs       = env.reset()
    ep_reward = 0.0
    done      = False
    step      = 0
    info      = {}
    max_rtf = resolve_eval_rtf(args)
    ep_wall_start = time.perf_counter()

    # Add "New Episode" button after reset (resetSimulation clears debug params).
    new_ep_btn = None
    btn_val = None
    phase_param = None
    phase_name = None
    if args.gui:
        try:
            phase_name = current_eval_phase_name(env)
            phase_param = sync_phase_gui_param(None, phase_name)
            new_ep_btn = p.addUserDebugParameter("New Episode", 1, 0, 1)
            btn_val = p.readUserDebugParameter(new_ep_btn)
        except Exception:
            new_ep_btn = None
            phase_param = None

    while not done:
        step += 1

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

        if live_debug is not None and step % live_debug.config.every_steps == 0:
            live_debug.update(
                step=step,
                update=0,
                active_drones=len(obs),
                grid=np.asarray(obs[0]["grid"], dtype=np.float32),
                local_vec=np.asarray(locs[0].detach().cpu().numpy(), dtype=np.float32),
            )

        with torch.no_grad():
            if args.deterministic:
                action_indices = actor.get_deterministic_action(grids, locs, move_mask=move_masks_t)
            else:
                action_indices, _, _ = actor.get_action(grids, locs, move_mask=move_masks_t)

        env_actions = action_space.decode_actions(action_indices.cpu().numpy())
        if args.print_actions:
            print(f"Step {step}: actions={env_actions}")

        obs, reward, done, info = env.step(env_actions)
        ep_reward += reward

        if args.gui:
            updated_phase_name = current_eval_phase_name(env)
            if updated_phase_name != phase_name:
                phase_param = sync_phase_gui_param(phase_param, updated_phase_name)
                phase_name = updated_phase_name

        if new_ep_btn is not None and btn_val is not None:
            try:
                new_val = p.readUserDebugParameter(new_ep_btn)
                if new_val != btn_val:
                    done = True
            except Exception:
                pass

        if max_rtf is not None:
            target_elapsed = (step * env.dt) / max_rtf
            sleep_for = target_elapsed - (time.perf_counter() - ep_wall_start)
            if sleep_for > 0:
                time.sleep(sleep_for)

    return ep_reward, step, info


def main():
    args   = parse_args()
    device = torch.device(args.device)
    ckpt = torch.load(args.load, map_location=device)
    trained_num_drones = infer_trained_num_drones(ckpt)
    trained_grid_channels = infer_checkpoint_actor_grid_channels(ckpt)
    trained_include_persistent_coverage = (
        infer_checkpoint_include_persistent_coverage_channel(ckpt)
    )
    trained_enable_agent_ids = infer_checkpoint_enable_agent_ids(ckpt)
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
    args.enable_agent_ids = infer_checkpoint_enable_agent_ids(ckpt)
    trained_include_shared_count_density = (
        infer_checkpoint_include_shared_count_density_channel(ckpt)
    )
    trained_include_shared_count_memory_staleness = (
        infer_checkpoint_include_shared_count_memory_staleness_channel(ckpt)
    )
    trained_reward_person_weight_mode = infer_checkpoint_reward_person_weight_mode(ckpt)
    active_num_drones = resolve_eval_active_num_drones(
        requested_num_drones=args.num_drones,
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
            "include_shared_count_memory_staleness_channel": (
                trained_include_shared_count_memory_staleness
            ),
            "reward_person_weight_mode": trained_reward_person_weight_mode,
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
        grid_h=args.grid_h,
        grid_w=args.grid_w,
        include_local_recent_count_memory_channel=(
            trained_include_local_recent_count_memory_channel
        ),
        include_instant_fov_channels=trained_include_instant_fov_channels,
        include_persistent_coverage_channel=trained_include_persistent_coverage,
        enable_agent_ids=trained_enable_agent_ids,
    )
    actor_config["local_dim"] = infer_checkpoint_local_dim(ckpt)
    actor = ActorNetwork(**actor_config).to(device)

    actor.load_state_dict(ckpt["actor"])
    actor.eval()

    print(
        f"[Eval] checkpoint_num_drones={trained_num_drones}  "
        f"active_num_drones={active_num_drones}"
    )

    live_debug = None
    if args.show_drone0_inputs:
        live_debug = LiveDebugWindow(
            LiveDebugConfig(
                enabled=True,
                every_steps=max(1, int(args.show_drone0_inputs_every)),
                num_drones=trained_num_drones,
                cmd_history_len=args.cmd_history_len,
                status_history_seconds=args.status_history_seconds,
                hotspot_top_k=trained_hotspot_top_k,
                move_mask_dim=len(action_space.vx_bins) * len(action_space.vy_bins),
                actor_channel_names=env.obs_builder.actor_channel_names,
            )
        )

    try:
        for ep in range(args.episodes):
            ep_start = time.perf_counter()
            ep_reward, steps, info = run_episode(
                env,
                actor,
                action_space,
                device,
                args,
                live_debug=live_debug,
            )
            elapsed = time.perf_counter() - ep_start
            sim_seconds = steps * env.dt
            rtf = sim_seconds / elapsed if elapsed > 0 else float("inf")

            reward_info    = info.get("reward_info", {})
            active_num_drones = info.get("active_num_drones", len(env.drones))
            coverage_count = reward_info.get("coverage_count", 0)
            r_disc         = reward_info.get("r_disc", 0.0)
            new_disc       = reward_info.get("new_discovered", 0)

            print(
                f"Episode {ep + 1:2d}: "
                f"active_drones={active_num_drones}  "
                f"total_reward={ep_reward:8.3f}  "
                f"steps={steps}  "
                f"sim_time={sim_seconds:6.2f}s  "
                f"RTF={rtf:6.2f}x  "
                f"last_coverage={coverage_count}  "
                f"last_r_disc={r_disc:.3f}  "
                f"last_new_disc={new_disc}"
            )
    finally:
        if live_debug is not None:
            live_debug.close()
        env.close()


if __name__ == "__main__":
    main()
