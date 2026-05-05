#!/usr/bin/env python3
"""
Export a trained actor checkpoint to a deterministic ONNX policy.

The exported ONNX graph takes:
  - grid:       [B, C, H, W]
  - local_base: [B, base_local_dim]
  - move_mask:  [B, num_move_bins]

and returns:
  - action:     [B, 3] = (vx, vy, yaw_rate)

`local_base` excludes the appended move-mask features. The wrapper
concatenates `local_base + move_mask` internally before running the actor.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
import torch.nn as nn

from config import SHARED_DEFAULTS, actor_kwargs, build_action_space
from rl.networks import ActorNetwork


class DeterministicActorONNX(nn.Module):
    def __init__(
        self,
        actor: nn.Module,
        vx_bins: torch.Tensor,
        vy_bins: torch.Tensor,
        yaw_rate_bins: torch.Tensor,
        base_local_dim: int,
    ):
        super().__init__()
        self.actor = actor
        self.base_local_dim = int(base_local_dim)
        self.num_vy_bins = int(vy_bins.numel())
        self.register_buffer("vx_bins", vx_bins.to(dtype=torch.float32))
        self.register_buffer("vy_bins", vy_bins.to(dtype=torch.float32))
        self.register_buffer("yaw_rate_bins", yaw_rate_bins.to(dtype=torch.float32))

    def forward(
        self,
        grid: torch.Tensor,
        local_base: torch.Tensor,
        move_mask: torch.Tensor,
    ) -> torch.Tensor:
        local_full = torch.cat([local_base, move_mask], dim=-1)
        move_logits, yaw_logits = self.actor(grid, local_full)

        move_mask = move_mask.to(device=move_logits.device, dtype=move_logits.dtype)
        invalid_fill = torch.full_like(move_logits, torch.finfo(move_logits.dtype).min)
        masked_move_logits = torch.where(move_mask > 0.0, move_logits, invalid_fill)

        move_action = torch.argmax(masked_move_logits, dim=-1)
        vx_idx = torch.floor_divide(move_action, self.num_vy_bins)
        vy_idx = torch.remainder(move_action, self.num_vy_bins)
        yaw_idx = torch.argmax(yaw_logits, dim=-1)

        vx = self.vx_bins.index_select(0, vx_idx)
        vy = self.vy_bins.index_select(0, vy_idx)
        yaw_rate = self.yaw_rate_bins.index_select(0, yaw_idx)

        return torch.stack([vx, vy, yaw_rate], dim=-1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True, help="Path to .pt checkpoint")
    parser.add_argument(
        "--output",
        default=None,
        help="Output ONNX path. Defaults to <checkpoint-stem>_deterministic.onnx next to the checkpoint.",
    )
    parser.add_argument("--device", default="cpu", help="Device used while exporting")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version")
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
    return parser.parse_args()


def infer_trained_num_drones(ckpt: dict) -> int:
    if "num_agents" in ckpt:
        return int(ckpt["num_agents"])

    actor_state = ckpt["actor"]
    local_dim = int(actor_state["local_mlp.0.weight"].shape[1])
    move_mask_dim = int(actor_state["move_head.weight"].shape[0])
    base_local_dim = local_dim - move_mask_dim

    for base_dim in (8, 6):
        if base_local_dim >= base_dim and (base_local_dim - base_dim) % 6 == 0:
            return ((base_local_dim - base_dim) // 6) + 1

    raise ValueError(
        "Could not infer trained num_drones from checkpoint: "
        f"local_dim={local_dim}, move_mask_dim={move_mask_dim}"
    )


def build_export_model(args: argparse.Namespace) -> tuple[nn.Module, int, int, int]:
    device = torch.device(args.device)
    ckpt = torch.load(args.checkpoint, map_location=device)
    trained_num_drones = infer_trained_num_drones(ckpt)

    action_space = build_action_space(args)
    actor = ActorNetwork(**actor_kwargs(trained_num_drones, action_space)).to(device)

    expected_move_bins = int(action_space.vx_bins.shape[0] * action_space.vy_bins.shape[0])
    expected_yaw_bins = int(action_space.yaw_rate_bins.shape[0])
    ckpt_move_bins = int(ckpt["actor"]["move_head.weight"].shape[0])
    ckpt_yaw_bins = int(ckpt["actor"]["yaw_head.weight"].shape[0])
    if ckpt_move_bins != expected_move_bins or ckpt_yaw_bins != expected_yaw_bins:
        raise ValueError(
            "Checkpoint action-head sizes do not match the configured action bins: "
            f"checkpoint move/yaw=({ckpt_move_bins}, {ckpt_yaw_bins}) vs "
            f"configured move/yaw=({expected_move_bins}, {expected_yaw_bins})."
        )

    actor.load_state_dict(ckpt["actor"])
    actor.eval()

    full_local_dim = int(actor.local_mlp[0].in_features)
    base_local_dim = full_local_dim - expected_move_bins
    if base_local_dim <= 0:
        raise ValueError(
            f"Invalid local dimensions: full_local_dim={full_local_dim}, move_mask_dim={expected_move_bins}"
        )

    wrapper = DeterministicActorONNX(
        actor=actor,
        vx_bins=torch.as_tensor(action_space.vx_bins),
        vy_bins=torch.as_tensor(action_space.vy_bins),
        yaw_rate_bins=torch.as_tensor(action_space.yaw_rate_bins),
        base_local_dim=base_local_dim,
    ).to(device)
    wrapper.eval()

    grid_channels = int(ckpt["actor"]["cnn.net.0.weight"].shape[1])
    return wrapper, grid_channels, base_local_dim, expected_move_bins


def resolve_output_path(args: argparse.Namespace) -> Path:
    if args.output:
        return Path(args.output)
    ckpt_path = Path(args.checkpoint)
    return ckpt_path.with_name(f"{ckpt_path.stem}_deterministic.onnx")


def main() -> None:
    args = parse_args()
    output_path = resolve_output_path(args)
    model, grid_channels, base_local_dim, move_mask_dim = build_export_model(args)

    device = torch.device(args.device)
    dummy_grid = torch.zeros((1, grid_channels, 50, 50), dtype=torch.float32, device=device)
    dummy_local_base = torch.zeros((1, base_local_dim), dtype=torch.float32, device=device)
    dummy_move_mask = torch.ones((1, move_mask_dim), dtype=torch.float32, device=device)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with torch.no_grad():
        torch.onnx.export(
            model,
            (dummy_grid, dummy_local_base, dummy_move_mask),
            output_path.as_posix(),
            input_names=["grid", "local_base", "move_mask"],
            output_names=["action"],
            dynamic_axes={
                "grid": {0: "batch"},
                "local_base": {0: "batch"},
                "move_mask": {0: "batch"},
                "action": {0: "batch"},
            },
            opset_version=args.opset,
        )

    print(f"Exported deterministic ONNX model -> {output_path}")


if __name__ == "__main__":
    main()
