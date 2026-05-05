"""
train.py — entry point for MAPPO training.

Usage:
    ./run_train.sh --num_drones 2 --total_updates 3000 --device cuda
    ./run_train.sh --load checkpoints/final.pt  # resume
"""

import argparse
import os
from datetime import datetime

from config import add_shared_args, add_train_args, build_action_space, build_env, trainer_kwargs
from rl.trainer import MAPPOTrainer


def parse_args():
    p = argparse.ArgumentParser()
    add_shared_args(p)
    add_train_args(p)
    return p.parse_args()


def build_run_save_dir(base_save_dir: str, load_path: str | None) -> str:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_prefix = "run"
    if load_path:
        source_name = os.path.splitext(os.path.basename(load_path))[0]
        run_prefix = f"resume_{source_name}"

    candidate = os.path.join(base_save_dir, f"{run_prefix}_{timestamp}")
    suffix = 1
    while os.path.exists(candidate):
        candidate = os.path.join(base_save_dir, f"{run_prefix}_{timestamp}_{suffix:02d}")
        suffix += 1
    return candidate
def main():
    args = parse_args()
    args.save_dir = build_run_save_dir(args.save_dir, args.load)
    env = build_env(args)
    action_space = build_action_space(args)
    trainer = MAPPOTrainer(env=env, **trainer_kwargs(args, action_space))
    print(f"[MAPPO] Saving this training run under {args.save_dir}")

    if args.load:
        trainer.load(args.load)

    try:
        trainer.train(total_updates=args.total_updates)
    finally:
        env.close()


if __name__ == "__main__":
    main()
