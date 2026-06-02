"""
train.py — entry point for MAPPO training.

Usage:
    ./run_train.sh --num_drones 2 --total_updates 3000 --device cuda
    ./run_train.sh --load checkpoints/final.pt --total_updates 3000  # resume to global update 3000
    ./run_train.sh --run_name two_drone_assignment_test
"""

import argparse
import os
from datetime import datetime
import torch

from config import (
    add_shared_args,
    add_train_args,
    build_action_space,
    build_env,
    infer_checkpoint_actor_grid_channels,
    infer_checkpoint_cmd_history_len,
    infer_checkpoint_hide_person_features_during_search,
    infer_checkpoint_enable_agent_ids,
    infer_checkpoint_include_local_recent_count_memory_channel,
    infer_checkpoint_include_instant_fov_channels,
    infer_checkpoint_include_shared_count_memory_staleness_channel,
    infer_checkpoint_include_shared_count_density_channel,
    infer_checkpoint_include_persistent_coverage_channel,
    infer_checkpoint_hotspot_top_k,
    infer_checkpoint_local_people_map_mode,
    infer_checkpoint_status_history_seconds,
    trainer_kwargs,
)
from rl.trainer import MAPPOTrainer


def parse_args():
    p = argparse.ArgumentParser()
    add_shared_args(p)
    add_train_args(p)
    return p.parse_args()


def build_run_save_dir(
    base_save_dir: str,
    load_path: str | None,
    run_name: str | None,
) -> str:
    if load_path:
        # Resume in-place: keep appending checkpoints and CSV rows to the
        # directory that already contains the provided checkpoint.
        return os.path.dirname(os.path.abspath(load_path)) or os.getcwd()

    if run_name:
        candidate = os.path.join(base_save_dir, run_name)
        if os.path.exists(candidate):
            raise FileExistsError(
                "Requested run_name already exists for a fresh run: "
                f"{os.path.abspath(candidate)}"
            )
        return candidate

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    candidate = os.path.join(base_save_dir, f"run_{timestamp}")
    suffix = 1
    while os.path.exists(candidate):
        candidate = os.path.join(base_save_dir, f"run_{timestamp}_{suffix:02d}")
        suffix += 1
    return candidate


def main():
    args = parse_args()
    if args.n_envs <= 0:
        raise ValueError(f"n_envs must be > 0, got {args.n_envs}")
    if args.gui and args.n_envs > 1:
        raise ValueError("GUI training supports only n_envs=1.")
    action_space = build_action_space(args)
    if args.load:
        ckpt = torch.load(args.load, map_location="cpu")
        args.actor_grid_channels = infer_checkpoint_actor_grid_channels(ckpt)
        args.include_persistent_coverage_channel = (
            infer_checkpoint_include_persistent_coverage_channel(ckpt)
        )
        args.include_instant_fov_channels = (
            infer_checkpoint_include_instant_fov_channels(ckpt)
        )
        args.hide_person_features_during_search = (
            infer_checkpoint_hide_person_features_during_search(ckpt)
        )
        args.include_local_recent_count_memory_channel = (
            infer_checkpoint_include_local_recent_count_memory_channel(ckpt)
        )
        args.local_people_map_mode = infer_checkpoint_local_people_map_mode(ckpt)
        args.enable_agent_ids = infer_checkpoint_enable_agent_ids(ckpt)
        args.include_shared_count_density_channel = (
            infer_checkpoint_include_shared_count_density_channel(ckpt)
        )
        args.include_shared_count_memory_staleness_channel = (
            infer_checkpoint_include_shared_count_memory_staleness_channel(ckpt)
        )
        trained_num_drones = int(ckpt.get("num_agents", args.num_drones))
        args.cmd_history_len = infer_checkpoint_cmd_history_len(
            ckpt,
            num_drones=trained_num_drones,
            action_space=action_space,
        )
        args.status_history_seconds = infer_checkpoint_status_history_seconds(ckpt)
        args.hotspot_top_k = infer_checkpoint_hotspot_top_k(
            ckpt,
            num_drones=trained_num_drones,
            action_space=action_space,
            cmd_history_len=args.cmd_history_len,
            status_history_seconds=args.status_history_seconds,
        )
    args.save_dir = build_run_save_dir(args.save_dir, args.load, args.run_name)
    envs = [build_env(args) for _ in range(args.n_envs)]
    trainer = MAPPOTrainer(envs=envs, **trainer_kwargs(args, action_space))
    print(f"[MAPPO] Saving this training run under {args.save_dir}")

    if args.load:
        trainer.load(args.load)

    try:
        trainer.train(total_updates=args.total_updates)
    finally:
        for env in envs:
            env.close()


if __name__ == "__main__":
    main()
