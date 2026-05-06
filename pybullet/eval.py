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
)
from rl.action_masking import append_move_masks_to_local, compute_move_action_masks
from rl.networks import ActorNetwork


def parse_args():
    parser = argparse.ArgumentParser()
    add_shared_args(parser)
    add_eval_args(parser)
    return parser.parse_args()


def infer_trained_num_drones(ckpt: dict) -> int:
    def infer_from_base_dim(total_local_dim: int) -> int | None:
        for base_dim in (9, 8, 6):
            if total_local_dim >= base_dim and (total_local_dim - base_dim) % 6 == 0:
                return ((total_local_dim - base_dim) // 6) + 1
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


def run_episode(env, actor, action_space, device, args):
    obs       = env.reset()
    ep_reward = 0.0
    done      = False
    step      = 0
    info      = {}

    # Add "New Episode" button after reset (resetSimulation clears debug params).
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
        grids = torch.FloatTensor(np.stack([o["grid"] for o in obs])).to(device)
        locs = torch.FloatTensor(
            append_move_masks_to_local(
                np.stack([o["local"] for o in obs]),
                move_masks,
            )
        ).to(device)
        move_masks_t = torch.FloatTensor(move_masks).to(device)

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
    args   = parse_args()
    device = torch.device(args.device)
    ckpt = torch.load(args.load, map_location=device)
    trained_num_drones = infer_trained_num_drones(ckpt)
    active_num_drones = resolve_eval_active_num_drones(
        requested_num_drones=args.num_drones,
        trained_num_drones=trained_num_drones,
    )

    env = build_env(
        args,
        overrides={
            "num_drones": trained_num_drones,
            "fixed_active_num_drones": active_num_drones,
        },
    )
    action_space = build_action_space(args)
    actor_config = actor_kwargs(trained_num_drones, action_space)
    actor_config["local_dim"] = infer_checkpoint_local_dim(ckpt)
    actor = ActorNetwork(**actor_config).to(device)

    actor.load_state_dict(ckpt["actor"])
    actor.eval()

    print(
        f"[Eval] checkpoint_num_drones={trained_num_drones}  "
        f"active_num_drones={active_num_drones}"
    )

    for ep in range(args.episodes):
        ep_start = time.perf_counter()
        ep_reward, steps, info = run_episode(env, actor, action_space, device, args)
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

    env.close()


if __name__ == "__main__":
    main()
