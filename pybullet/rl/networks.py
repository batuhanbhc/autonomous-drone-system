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


class ConvStem(nn.Module):
    def __init__(
        self,
        in_channels: int,
        out_channels: int,
    ):
        super().__init__()
        self.conv = nn.Conv2d(
            in_channels,
            out_channels,
            kernel_size=3,
            stride=1,
            padding=1,
        )
        self.act = nn.SiLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.act(self.conv(x))


class IdentityResidualBlock(nn.Module):
    def __init__(self, channels: int):
        super().__init__()
        self.conv1 = nn.Conv2d(channels, channels, kernel_size=3, stride=1, padding=1)
        self.conv2 = nn.Conv2d(channels, channels, kernel_size=3, stride=1, padding=1)
        self.act = nn.SiLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        residual = x
        x = self.act(self.conv1(x))
        x = self.conv2(x)
        return self.act(x + residual)


class DownsampleResidualBlock(nn.Module):
    def __init__(self, in_channels: int, out_channels: int):
        super().__init__()
        self.conv1 = nn.Conv2d(
            in_channels,
            out_channels,
            kernel_size=3,
            stride=2,
            padding=1,
        )
        self.conv2 = nn.Conv2d(
            out_channels,
            out_channels,
            kernel_size=3,
            stride=1,
            padding=1,
        )
        self.skip = nn.Conv2d(
            in_channels,
            out_channels,
            kernel_size=1,
            stride=2,
            padding=0,
        )
        self.act = nn.SiLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        residual = self.skip(x)
        x = self.act(self.conv1(x))
        x = self.conv2(x)
        return self.act(x + residual)

class CNNEncoder(nn.Module):
    def __init__(self, in_channels: int = 2, grid_h: int = 32, grid_w: int = 32, out_dim: int = 128):
        super().__init__()
        self.net = nn.Sequential(
            ConvStem(in_channels, 16),
            IdentityResidualBlock(16),
            DownsampleResidualBlock(16, 32),
            IdentityResidualBlock(32),
            DownsampleResidualBlock(32, 64),
            IdentityResidualBlock(64),
            DownsampleResidualBlock(64, 128),
            IdentityResidualBlock(128),
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
        feat = self.net(grid)                          # (B, C', H', W')
        flattened = feat.flatten(start_dim=1)         # (B, C' * H' * W')
        return self.proj(flattened)                   # (B, out_dim)


def _infer_actor_branch_indices(
    grid_channels: int,
    include_local_recent_count_memory_channel: bool,
    include_instant_fov_channels: bool,
    include_persistent_coverage_channel: bool,
) -> tuple[list[int], list[int]]:
    fixed_non_shared_channels = (
        7 + int(bool(include_local_recent_count_memory_channel))
        if bool(include_instant_fov_channels)
        else 5 + int(bool(include_local_recent_count_memory_channel))
    )
    shared_people_channels = int(grid_channels) - fixed_non_shared_channels
    if shared_people_channels not in {1, 2, 3, 4, 5}:
        raise ValueError(
            "Unsupported actor grid layout: expected shared people channels in [1, 5], "
            f"got grid_channels={grid_channels}, shared_people_channels={shared_people_channels}"
        )

    local_recent_offset = 1 + int(bool(include_local_recent_count_memory_channel))
    shared_start = local_recent_offset
    shared_end = shared_start + shared_people_channels

    people_branch_indices = [0]
    if include_local_recent_count_memory_channel:
        people_branch_indices.append(1)

    if include_persistent_coverage_channel:
        people_branch_indices.extend(range(shared_start, shared_end - 1))
        context_branch_indices = [shared_end - 1]
    else:
        people_branch_indices.extend(range(shared_start, shared_end))
        context_branch_indices = []

    context_branch_indices.extend(range(shared_end, int(grid_channels)))
    return people_branch_indices, context_branch_indices


class BranchedActorEncoder(nn.Module):
    def __init__(
        self,
        grid_channels: int,
        grid_h: int,
        grid_w: int,
        out_dim: int,
        include_local_recent_count_memory_channel: bool = True,
        include_instant_fov_channels: bool = True,
        include_persistent_coverage_channel: bool = False,
    ):
        super().__init__()
        people_branch_indices, context_branch_indices = _infer_actor_branch_indices(
            grid_channels=grid_channels,
            include_local_recent_count_memory_channel=include_local_recent_count_memory_channel,
            include_instant_fov_channels=include_instant_fov_channels,
            include_persistent_coverage_channel=include_persistent_coverage_channel,
        )
        self.grid_channels = int(grid_channels)
        self.register_buffer(
            "people_branch_indices",
            torch.tensor(people_branch_indices, dtype=torch.long),
            persistent=False,
        )
        self.register_buffer(
            "context_branch_indices",
            torch.tensor(context_branch_indices, dtype=torch.long),
            persistent=False,
        )
        self.people_branch = CNNEncoder(
            len(people_branch_indices),
            grid_h,
            grid_w,
            out_dim,
        )
        self.context_branch = CNNEncoder(
            len(context_branch_indices),
            grid_h,
            grid_w,
            out_dim,
        )
        self.proj = nn.Sequential(
            nn.Linear(out_dim * 2, out_dim),
            nn.SiLU(),
        )
        _orthogonal_init(self.proj, gain=1.0)

    def forward(self, grid: torch.Tensor) -> torch.Tensor:
        people_feat = self.people_branch(grid.index_select(1, self.people_branch_indices))
        context_feat = self.context_branch(grid.index_select(1, self.context_branch_indices))
        return self.proj(torch.cat([people_feat, context_feat], dim=-1))


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
        include_local_recent_count_memory_channel: bool = True,
        include_instant_fov_channels: bool = True,
        include_persistent_coverage_channel: bool = False,
    ):
        super().__init__()
        self.grid_channels = int(grid_channels)
        self.num_vx_bins = int(num_vx_bins)
        self.num_vy_bins = int(num_vy_bins)
        self.num_move_bins = self.num_vx_bins * self.num_vy_bins
        self.cnn = BranchedActorEncoder(
            grid_channels=grid_channels,
            grid_h=grid_h,
            grid_w=grid_w,
            out_dim=cnn_out_dim,
            include_local_recent_count_memory_channel=include_local_recent_count_memory_channel,
            include_instant_fov_channels=include_instant_fov_channels,
            include_persistent_coverage_channel=include_persistent_coverage_channel,
        )

        self.local_mlp = nn.Sequential(
            nn.Linear(local_dim, 128),
            nn.SiLU(),
        )

        fused_dim = cnn_out_dim + 128
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
        self.grid_channels = int(grid_channels)

        # Spatial branch — same CNN architecture as the actor
        self.cnn = CNNEncoder(grid_channels, grid_h, grid_w, cnn_out_dim)

        # Pose branch — LayerNorm then small MLP
        self.pose_mlp = nn.Sequential(
            nn.LayerNorm(poses_dim),
            nn.Linear(poses_dim, 128),
            nn.SiLU(),
        )

        # Shared head
        fused_dim = cnn_out_dim + 128
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
