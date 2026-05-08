"""
Rollout buffer for MAPPO.

Actor and critic grids have different channel counts:
  - Actor grid  : (actor_grid_channels, H, W)  — 8 or 10 channels
  - Critic grid : (critic_grid_channels, H, W) — actor_grid_channels - 2 + 3 * num_agents channels
                  (shared channels + per-drone instant/coverage/ego maps)

Stored separately so neither shape pollutes the other.

Key design decision: returns are NOT normalised here.
Only advantages are normalised (zero mean, unit variance per rollout).
"""

import numpy as np
import torch


class RolloutBuffer:
    def __init__(
        self,
        rollout_len: int,
        num_agents: int,
        grid_shape: tuple,          # actor grid shape  (C_actor, H, W)
        critic_grid_shape: tuple,   # critic grid shape (C_critic, H, W)
        local_dim: int,
        action_dim: int,
        move_mask_dim: int,
        poses_dim: int,
        gamma: float = 0.99,
        gae_lambda: float = 0.95,
        device: str = "cpu",
    ):
        self.rollout_len       = rollout_len
        self.num_agents        = num_agents
        self.grid_shape        = grid_shape
        self.critic_grid_shape = critic_grid_shape
        self.local_dim         = local_dim
        self.action_dim        = action_dim
        self.move_mask_dim     = move_mask_dim
        self.poses_dim         = poses_dim
        self.gamma             = gamma
        self.gae_lambda        = gae_lambda
        self.device            = device
        self._init_buffers()

    def _init_buffers(self):
        T, N       = self.rollout_len, self.num_agents
        C, H, W    = self.grid_shape
        Cc, _, _   = self.critic_grid_shape

        # Actor inputs (per agent per step)
        self.grids     = np.zeros((T, N, C,  H, W), dtype=np.float32)
        self.locals    = np.zeros((T, N, self.local_dim), dtype=np.float32)
        self.actions   = np.zeros((T, N, self.action_dim), dtype=np.int64)
        self.log_probs = np.zeros((T, N), dtype=np.float32)
        # Per-step movement masks must be replayed during PPO updates so the
        # policy is evaluated under the same valid-action set as rollout time.
        self.move_masks = np.ones((T, N, self.move_mask_dim), dtype=np.float32)
        self.actor_masks = np.zeros((T, N), dtype=np.float32)
        self.critic_weights = np.zeros((T, N), dtype=np.float32)

        self.rewards = np.zeros((T,), dtype=np.float32)
        self.dones   = np.zeros((T,), dtype=np.float32)

        # Critic inputs (global, one per step) — separate grid shape from actor
        self.global_grids = np.zeros((T, Cc, H, W), dtype=np.float32)
        self.global_poses = np.zeros((T, self.poses_dim), dtype=np.float32)

        self.values     = np.zeros((T,), dtype=np.float32)
        self.returns    = np.zeros((T,), dtype=np.float32)
        self.advantages = np.zeros((T,), dtype=np.float32)
        self.ptr  = 0
        self.full = False

    def add(self, grids, locals_, actions, log_probs, move_masks, actor_masks, critic_weights,
            reward, done, global_grid, global_poses, value):
        t = self.ptr
        self.grids[t]         = grids
        self.locals[t]        = locals_
        self.actions[t]       = actions
        self.log_probs[t]     = log_probs
        self.move_masks[t]    = move_masks
        self.actor_masks[t]   = actor_masks
        self.critic_weights[t] = critic_weights
        self.rewards[t]       = reward
        self.dones[t]         = float(done)
        self.global_grids[t]  = global_grid
        self.global_poses[t]  = global_poses
        self.values[t]        = value
        self.ptr += 1
        if self.ptr >= self.rollout_len:
            self.full = True
            self.ptr  = 0

    def compute_returns(self, last_value: float):
        T   = self.rollout_len
        gae = 0.0
        for t in reversed(range(T)):
            next_value        = last_value if t == T - 1 else self.values[t + 1]
            next_non_terminal = 1.0 - self.dones[t]
            delta = (
                self.rewards[t]
                + self.gamma * next_value * next_non_terminal
                - self.values[t]
            )
            gae = delta + self.gamma * self.gae_lambda * next_non_terminal * gae
            self.advantages[t] = gae
            self.returns[t]    = gae + self.values[t]

        # Normalise ONLY advantages (zero mean, unit variance).
        # Returns are kept in their original scale so the critic learns
        # the true value function.
        adv = self.advantages
        self.advantages = (adv - adv.mean()) / (adv.std() + 1e-8)

    def get_batches(self, batch_size: int):
        T, N    = self.rollout_len, self.num_agents
        C, H, W = self.grid_shape
        Cc      = self.critic_grid_shape[0]

        grids      = self.grids.reshape(T * N, C, H, W)
        locals_    = self.locals.reshape(T * N, self.local_dim)
        actions    = self.actions.reshape(T * N, self.action_dim)
        log_probs  = self.log_probs.reshape(T * N)
        move_masks = self.move_masks.reshape(T * N, self.move_mask_dim)
        actor_masks = self.actor_masks.reshape(T * N)
        critic_weights = self.critic_weights.reshape(T * N)
        advantages = np.repeat(self.advantages, N)
        returns    = np.repeat(self.returns, N)

        # Repeat critic grid and poses for each agent
        g_grids = np.repeat(self.global_grids, N, axis=0)  # (T*N, Cc, H, W)
        g_poses = np.repeat(self.global_poses, N, axis=0)  # (T*N, poses_dim)

        indices = np.arange(T * N)
        np.random.shuffle(indices)

        for start in range(0, T * N, batch_size):
            idx = indices[start: start + batch_size]
            yield (
                torch.FloatTensor(grids[idx]).to(self.device),
                torch.FloatTensor(locals_[idx]).to(self.device),
                torch.LongTensor(actions[idx]).to(self.device),
                torch.FloatTensor(log_probs[idx]).to(self.device),
                torch.FloatTensor(move_masks[idx]).to(self.device),
                torch.FloatTensor(actor_masks[idx]).to(self.device),
                torch.FloatTensor(critic_weights[idx]).to(self.device),
                torch.FloatTensor(advantages[idx]).to(self.device),
                torch.FloatTensor(returns[idx]).to(self.device),
                torch.FloatTensor(g_grids[idx]).to(self.device),
                torch.FloatTensor(g_poses[idx]).to(self.device),
            )
