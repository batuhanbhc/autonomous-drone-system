"""
hotspot_debug_eval.py — deterministic hotspot ablation eval.

Run the exact same seeded environment in two modes:
  --episode_num 1 : normal actor inputs
  --episode_num 2 : hotspot slice in the local vector is zeroed before acting

Usage:
    python hotspot_debug_eval.py --load checkpoints/final.pt --episode_num 1
    python hotspot_debug_eval.py --load checkpoints/final.pt --episode_num 2
"""

import argparse
import hashlib
import json
import random
import time
from typing import Any

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
    infer_checkpoint_include_instant_fov_channels,
    infer_checkpoint_include_local_recent_count_memory_channel,
    infer_checkpoint_include_shared_count_density_channel,
    infer_checkpoint_include_persistent_coverage_channel,
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
from rl.live_debug import LiveDebugConfig, LiveDebugWindow
from rl.networks import ActorNetwork


def parse_args():
    parser = argparse.ArgumentParser()
    add_shared_args(parser)
    add_eval_args(parser)
    parser.add_argument(
        "--episode_num",
        type=int,
        choices=(1, 2),
        required=True,
        help="1 = normal inputs, 2 = zero hotspot inputs before the actor forward pass.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=12345,
        help="Global seed used to recreate the exact same environment instance across runs.",
    )
    return parser.parse_args()


def set_global_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    if hasattr(torch.backends, "cudnn"):
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False
    try:
        torch.use_deterministic_algorithms(True)
    except Exception:
        pass


def zero_hotspot_slice(local_obs: np.ndarray, hotspot_top_k: int) -> np.ndarray:
    if hotspot_top_k <= 0:
        return local_obs
    local_obs = np.array(local_obs, copy=True)
    hotspot_start = 11
    hotspot_end = hotspot_start + 5 * int(hotspot_top_k)
    local_obs[:, hotspot_start:hotspot_end] = 0.0
    return local_obs


def build_scenario_signature(env) -> tuple[str, dict[str, Any]]:
    drone_states = env._get_drone_states()
    people_positions = env._get_people_positions()
    payload = {
        "active_num_drones": int(env.active_num_drones),
        "drone_states": [
            {
                "position": [round(float(v), 6) for v in state["position"]],
                "yaw": round(float(state["yaw"]), 6),
            }
            for state in drone_states
        ],
        "people_positions": [
            [round(float(v), 6) for v in pos]
            for pos in people_positions
        ],
        "group_info": env.episode_group_info,
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    digest = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]
    return digest, payload


def run_episode(env, actor, action_space, device, args, hotspot_top_k, live_debug=None):
    obs = env.reset()
    scenario_hash, scenario_payload = build_scenario_signature(env)
    ep_reward = 0.0
    done = False
    step = 0
    info = {}

    print(
        f"[HotspotDebug] mode={'normal' if args.episode_num == 1 else 'zero_hotspots'}  "
        f"seed={args.seed}  scenario_hash={scenario_hash}"
    )
    print(
        f"[HotspotDebug] active_drones={scenario_payload['active_num_drones']}  "
        f"num_groups={scenario_payload['group_info'].get('num_groups', 0)}"
    )

    new_ep_btn = None
    btn_val = None
    if args.gui:
        try:
            new_ep_btn = p.addUserDebugParameter("New Episode", 1, 0, 1)
            btn_val = p.readUserDebugParameter(new_ep_btn)
        except Exception:
            new_ep_btn = None

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
        local_obs = np.stack([o["local"] for o in obs])
        if args.episode_num == 2:
            local_obs = zero_hotspot_slice(local_obs, hotspot_top_k)
        grids = torch.FloatTensor(np.stack([o["grid"] for o in obs])).to(device)
        locs = torch.FloatTensor(
            append_move_masks_to_local(
                local_obs,
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

        if new_ep_btn is not None and btn_val is not None:
            try:
                new_val = p.readUserDebugParameter(new_ep_btn)
                if new_val != btn_val:
                    done = True
            except Exception:
                pass

        if args.realtime:
            time.sleep(env.dt)

    return ep_reward, step, info


def main():
    args = parse_args()
    args.episodes = 1
    args.deterministic = True
    set_global_seed(args.seed)

    device = torch.device(args.device)
    ckpt = torch.load(args.load, map_location=device)
    trained_num_drones = infer_trained_num_drones(ckpt)
    trained_grid_channels = infer_checkpoint_actor_grid_channels(ckpt)
    trained_include_persistent_coverage = (
        infer_checkpoint_include_persistent_coverage_channel(ckpt)
    )
    trained_include_instant_fov_channels = (
        infer_checkpoint_include_instant_fov_channels(ckpt)
    )
    trained_include_local_recent_count_memory_channel = (
        infer_checkpoint_include_local_recent_count_memory_channel(ckpt)
    )
    trained_local_people_map_mode = infer_checkpoint_local_people_map_mode(ckpt)
    trained_include_shared_count_density = (
        infer_checkpoint_include_shared_count_density_channel(ckpt)
    )
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
    )
    actor_config["local_dim"] = infer_checkpoint_local_dim(ckpt)
    actor = ActorNetwork(**actor_config).to(device)
    actor.load_state_dict(ckpt["actor"])
    actor.eval()

    print(
        f"[HotspotDebug] checkpoint_num_drones={trained_num_drones}  "
        f"active_num_drones={active_num_drones}  "
        f"trained_hotspot_top_k={trained_hotspot_top_k}  "
        f"episode_num={args.episode_num}"
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
        ep_start = time.perf_counter()
        ep_reward, steps, info = run_episode(
            env=env,
            actor=actor,
            action_space=action_space,
            device=device,
            args=args,
            hotspot_top_k=trained_hotspot_top_k,
            live_debug=live_debug,
        )
        elapsed = time.perf_counter() - ep_start
        sim_seconds = steps * env.dt
        rtf = sim_seconds / elapsed if elapsed > 0 else float("inf")

        reward_info = info.get("reward_info", {})
        active_num_drones = info.get("active_num_drones", len(env.drones))
        coverage_count = reward_info.get("coverage_count", 0)
        r_disc = reward_info.get("r_disc", 0.0)
        new_disc = reward_info.get("new_discovered", 0)

        print(
            f"Episode {args.episode_num:2d}: "
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
        env.close()


if __name__ == "__main__":
    main()
