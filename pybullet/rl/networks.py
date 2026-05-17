"""
Actor-Critic networks for MAPPO.

Actor:  CNN(grid) + MLP(local) → fused → masked joint move head over (vx, vy)
        plus a yaw-rate head
Critic: CNN(global grid) + MLP(all_poses) → fused → scalar value

Both networks process the spatial grid through the same CNN architecture.
"""

import torch
import torch.nn as nn
from torch.distributions import Categorical
from typing import Tuple


# ------------------------------------------------------------------ #
#  Weight initialisation helpers
# ------------------------------------------------------------------ #

def _orthogonal_init(module: nn.Module, gain: float = 1.0):
    """Apply orthogonal init to Linear and Conv2d layers."""
    for m in module.modules():
        if isinstance(m, (nn.Linear, nn.Conv2d)):
            nn.init.orthogonal_(m.weight, gain=gain)
            if m.bias is not None:
                nn.init.zeros_(m.bias)
    return module


# ------------------------------------------------------------------ #
#  CNN encoder — shared architecture for both actor and critic
# ------------------------------------------------------------------ #


class ResidualConvStage(nn.Module):
    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        stride: int = 1,
        kernel_size: int = 3,
        padding: int = 1,
        skip_kernel_size: int = 1,
        skip_padding: int = 0,
    ):
        super().__init__()
        self.conv = nn.Conv2d(
            in_channels,
            out_channels,
            kernel_size=kernel_size,
            stride=stride,
            padding=padding,
        )
        self.skip = None
        if stride != 1 or in_channels != out_channels:
            self.skip = nn.Conv2d(
                in_channels,
                out_channels,
                kernel_size=skip_kernel_size,
                stride=stride,
                padding=skip_padding,
            )
        self.act = nn.SiLU()
        _orthogonal_init(self)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        residual = x if self.skip is None else self.skip(x)
        return self.act(self.conv(x) + residual)

class CNNEncoder(nn.Module):
    def __init__(self, in_channels: int = 2, grid_h: int = 32, grid_w: int = 32, out_dim: int = 128):
        super().__init__()
        self.net = nn.Sequential(
            ResidualConvStage(in_channels, 16, stride=1),
            ResidualConvStage(16, 32, stride=2),
            ResidualConvStage(32, 64, stride=2),
            ResidualConvStage(
                64,
                128,
                stride=2,
                kernel_size=2,
                padding=0,
                skip_kernel_size=2,
                skip_padding=0,
            ),
        )
        conv_c, conv_h, conv_w = self._infer_conv_output_shape(grid_h, grid_w)
        self.proj = nn.Sequential(
            nn.Linear(conv_c * conv_h * conv_w, out_dim),
            nn.SiLU(),
        )
        _orthogonal_init(self)

    def _infer_conv_output_shape(self, grid_h: int, grid_w: int) -> tuple[int, int, int]:
        with torch.no_grad():
            dummy = torch.zeros(1, self.net[0].conv.in_channels, grid_h, grid_w)
            out = self.net(dummy)
        return int(out.shape[-3]), int(out.shape[-2]), int(out.shape[-1])

    def forward(self, grid: torch.Tensor) -> torch.Tensor:
        feat = self.net(grid)                          # (B, 64, H', W')
        flattened = feat.flatten(start_dim=1)         # (B, 64 * H' * W')
        return self.proj(flattened)                   # (B, out_dim)


# ------------------------------------------------------------------ #
#  Actor network
# ------------------------------------------------------------------ #

class ActorNetwork(nn.Module):
    def __init__(
        self,
        local_dim: int = 8,
        grid_channels: int = 2,
        grid_h: int = 32,
        grid_w: int = 32,
        cnn_out_dim: int = 128,
        hidden_dim: int = 256,
        num_vx_bins: int = 9,
        num_vy_bins: int = 9,
        num_yaw_bins: int = 9,
    ):
        super().__init__()
        self.num_vx_bins = int(num_vx_bins)
        self.num_vy_bins = int(num_vy_bins)
        self.num_move_bins = self.num_vx_bins * self.num_vy_bins
        self.cnn = CNNEncoder(grid_channels, grid_h, grid_w, cnn_out_dim)

        self.local_mlp = nn.Sequential(
            nn.Linear(local_dim, 64),
            nn.SiLU(),
        )

        fused_dim = cnn_out_dim + 64
        self.shared = nn.Sequential(
            nn.Linear(fused_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
        )
        self.move_head = nn.Linear(hidden_dim, self.num_move_bins)
        self.yaw_head = nn.Linear(hidden_dim, num_yaw_bins)

        _orthogonal_init(self.local_mlp, gain=1.0)
        _orthogonal_init(self.shared, gain=1.0)
        _orthogonal_init(self.move_head, gain=0.01)
        _orthogonal_init(self.yaw_head, gain=0.01)

    def forward(self, grid: torch.Tensor, local_vec: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        cnn_feat = self.cnn(grid)
        loc_feat = self.local_mlp(local_vec)
        fused = torch.cat([cnn_feat, loc_feat], dim=-1)
        shared = self.shared(fused)
        return self.move_head(shared), self.yaw_head(shared)

    @staticmethod
    def _apply_move_mask(move_logits: torch.Tensor, move_mask: torch.Tensor | None) -> torch.Tensor:
        if move_mask is None:
            return move_logits
        if move_mask.shape != move_logits.shape:
            raise ValueError(
                "move_mask shape must match move_logits shape, "
                f"got {tuple(move_mask.shape)} and {tuple(move_logits.shape)}"
            )
        if not torch.all(move_mask.sum(dim=-1) > 0):
            raise ValueError("Each movement mask row must contain at least one valid action.")
        invalid_fill = torch.finfo(move_logits.dtype).min
        return move_logits.masked_fill(move_mask <= 0.0, invalid_fill)

    def _decode_move_action(self, move_action: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        vx_action = torch.div(move_action, self.num_vy_bins, rounding_mode="floor")
        vy_action = move_action.remainder(self.num_vy_bins)
        return vx_action, vy_action

    def _encode_move_action(self, vx_action: torch.Tensor, vy_action: torch.Tensor) -> torch.Tensor:
        return vx_action.long() * self.num_vy_bins + vy_action.long()

    def get_distributions(self, grid, local_vec, move_mask=None) -> Tuple[Categorical, Categorical]:
        move_logits, yaw_logits = self.forward(grid, local_vec)
        if move_mask is not None:
            move_mask = move_mask.to(device=move_logits.device, dtype=move_logits.dtype)
        move_logits = self._apply_move_mask(move_logits, move_mask)
        return (
            Categorical(logits=move_logits),
            Categorical(logits=yaw_logits),
        )

    def get_deterministic_action(self, grid, local_vec, move_mask=None) -> torch.Tensor:
        move_logits, yaw_logits = self.forward(grid, local_vec)
        if move_mask is not None:
            move_mask = move_mask.to(device=move_logits.device, dtype=move_logits.dtype)
        move_logits = self._apply_move_mask(move_logits, move_mask)
        move_action = move_logits.argmax(dim=-1)
        vx_action, vy_action = self._decode_move_action(move_action)
        return torch.stack(
            [
                vx_action,
                vy_action,
                yaw_logits.argmax(dim=-1),
            ],
            dim=-1,
        )

    def get_action(self, grid, local_vec, move_mask=None):
        move_dist, yaw_dist = self.get_distributions(grid, local_vec, move_mask=move_mask)
        move_action = move_dist.sample()
        yaw_action = yaw_dist.sample()
        vx_action, vy_action = self._decode_move_action(move_action)
        action = torch.stack([vx_action, vy_action, yaw_action], dim=-1)
        log_prob = move_dist.log_prob(move_action) + yaw_dist.log_prob(yaw_action)
        entropy = move_dist.entropy() + yaw_dist.entropy()
        return action, log_prob, entropy

    def evaluate_actions(self, grid, local_vec, actions, move_mask=None):
        actions = actions.long()
        move_action = self._encode_move_action(actions[:, 0], actions[:, 1])
        move_dist, yaw_dist = self.get_distributions(grid, local_vec, move_mask=move_mask)
        log_prob = move_dist.log_prob(move_action) + yaw_dist.log_prob(actions[:, 2])
        entropy = move_dist.entropy() + yaw_dist.entropy()
        return log_prob, entropy


# ------------------------------------------------------------------ #
#  Critic network  — CNN for grid, MLP for all-agent poses, then fuse
# ------------------------------------------------------------------ #

class CriticNetwork(nn.Module):
    """
    Processes the shared grid spatially (CNN) and all drone poses as a
    flat vector (small MLP), then fuses both into a scalar value estimate.

    The grid and poses are kept separate until after the CNN and pose-MLP,
    then concatenated and passed through the shared MLP head.
    """
    def __init__(
        self,
        grid_channels: int = 2,
        grid_h: int = 32,
        grid_w: int = 32,
        cnn_out_dim: int = 128,
        poses_dim: int = 16, 
        hidden_dim: int = 256,
    ):
        super().__init__()

        # Spatial branch — same CNN architecture as the actor
        self.cnn = CNNEncoder(grid_channels, grid_h, grid_w, cnn_out_dim)

        # Pose branch — LayerNorm then small MLP
        self.pose_mlp = nn.Sequential(
            nn.LayerNorm(poses_dim),
            nn.Linear(poses_dim, 64),
            nn.SiLU(),
        )

        # Shared head
        fused_dim = cnn_out_dim + 64
        self.shared = nn.Sequential(
            nn.Linear(fused_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, 1),
        )

        _orthogonal_init(self.cnn,      gain=1.0)
        _orthogonal_init(self.pose_mlp, gain=1.0)
        _orthogonal_init(self.shared,   gain=1.0)
        # Small init on output so critic starts near zero
        nn.init.orthogonal_(self.shared[-1].weight, gain=0.01)
        nn.init.zeros_(self.shared[-1].bias)

    def forward(self, grid: torch.Tensor, poses: torch.Tensor) -> torch.Tensor:
        cnn_feat  = self.cnn(grid)
        pose_feat = self.pose_mlp(poses)
        fused     = torch.cat([cnn_feat, pose_feat], dim=-1)
        return self.shared(fused)
