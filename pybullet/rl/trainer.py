"""
MAPPO Trainer for a masked joint-move actor over ((vx, vy), yaw_rate).

Actor grid: (grid_channels, H, W)               — 8 channels
Critic grid: (6 + 3 * num_agents, H, W)         — shared + per-drone instant/coverage/ego maps
"""

import csv
import os
import time
import math
import re
import numpy as np
import torch
import torch.nn as nn
from typing import Dict, List

from rl.action_masking import append_move_masks_to_local, compute_move_action_masks
from rl.live_debug import LiveDebugConfig, LiveDebugWindow
from rl.networks import ActorNetwork, CriticNetwork
from rl.buffer import RolloutBuffer
from rl.state_utils import build_global_state


class RunningMeanStd:
    """Numerically stable running mean/std for value-target normalization."""

    def __init__(self, epsilon: float = 1e-4):
        self.mean = 0.0
        self.var = 1.0
        self.count = epsilon

    def update(self, values: np.ndarray):
        values = np.asarray(values, dtype=np.float64)
        if values.size == 0:
            return
        batch_mean = float(values.mean())
        batch_var = float(values.var())
        batch_count = float(values.size)
        self._update_from_moments(batch_mean, batch_var, batch_count)

    def _update_from_moments(self, batch_mean: float, batch_var: float, batch_count: float):
        delta = batch_mean - self.mean
        total_count = self.count + batch_count

        new_mean = self.mean + delta * batch_count / total_count
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m2 = m_a + m_b + (delta * delta) * self.count * batch_count / total_count

        self.mean = new_mean
        self.var = max(m2 / total_count, 1e-8)
        self.count = total_count

    @property
    def std(self) -> float:
        return math.sqrt(max(self.var, 1e-8))

    def normalize_tensor(self, tensor: torch.Tensor) -> torch.Tensor:
        mean_t = torch.as_tensor(self.mean, dtype=tensor.dtype, device=tensor.device)
        std_t = torch.as_tensor(self.std, dtype=tensor.dtype, device=tensor.device)
        return (tensor - mean_t) / std_t

    def denormalize_tensor(self, tensor: torch.Tensor) -> torch.Tensor:
        mean_t = torch.as_tensor(self.mean, dtype=tensor.dtype, device=tensor.device)
        std_t = torch.as_tensor(self.std, dtype=tensor.dtype, device=tensor.device)
        return tensor * std_t + mean_t

    def state_dict(self) -> Dict[str, float]:
        return {
            "mean": self.mean,
            "var": self.var,
            "count": self.count,
        }

    def load_state_dict(self, state: Dict[str, float]):
        self.mean = float(state.get("mean", 0.0))
        self.var = max(float(state.get("var", 1.0)), 1e-8)
        self.count = float(state.get("count", 1e-4))


class MAPPOTrainer:
    def __init__(
        self,
        env,
        local_dim: int = 11,
        grid_channels: int = 8,      # actor: local instant + shared maps + own/teammate coverage + own ego
        grid_h: int = 32,
        grid_w: int = 32,
        cnn_out_dim: int = 128,
        hidden_dim: int = 256,
        num_vx_bins: int = 9,
        num_vy_bins: int = 9,
        num_yaw_bins: int = 9,
        rollout_len: int = 300,
        num_epochs: int = 4,
        batch_size: int = 64,
        clip_eps: float = 0.1,
        gamma: float = 0.95,
        gae_lambda: float = 0.95,
        lr_actor: float = 3e-4,
        lr_critic: float = 3e-4,
        value_coef: float = 0.5,
        entropy_coef: float = 0.0,
        max_grad_norm: float = 0.5,
        vx_bins = None,
        vy_bins = None,
        yaw_rate_bins = None,
        device: str = "cpu",
        save_dir: str = "checkpoints",
        log_interval: int = 10,
        use_tensorboard: bool = False,
        live_debug: bool = False,
        live_debug_every: int = 100,
        sticky_action_prob: float = 0.0,
        gui: bool = False,
    ):
        self.env        = env
        self.num_agents = env.max_drones
        self.device     = torch.device(device)
        self.save_dir   = save_dir
        self.log_interval = log_interval

        self.rollout_len   = rollout_len
        self.num_epochs    = num_epochs
        self.batch_size    = batch_size
        self.clip_eps      = clip_eps
        self.value_coef    = value_coef
        self.entropy_coef  = entropy_coef
        self.max_grad_norm = max_grad_norm
        self.gui = gui
        self._pause_btn = None
        self._pause_btn_last = None
        self._paused = False
        self.sticky_action_prob = float(sticky_action_prob)
        if not 0.0 <= self.sticky_action_prob <= 1.0:
            raise ValueError(
                f"sticky_action_prob must be within [0, 1], got {self.sticky_action_prob}"
            )
        self.action_dim = 3
        if vx_bins is None or vy_bins is None or yaw_rate_bins is None:
            raise ValueError("Discrete action bins must be provided to MAPPOTrainer")
        self.vx_bins = np.asarray(vx_bins, dtype=np.float32)
        self.vy_bins = np.asarray(vy_bins, dtype=np.float32)
        self.yaw_rate_bins = np.asarray(yaw_rate_bins, dtype=np.float32)
        self.move_mask_dim = int(self.vx_bins.shape[0] * self.vy_bins.shape[0])

        self.grid_channels        = grid_channels
        self.critic_grid_channels = 6 + (3 * self.num_agents)
        self.grid_h               = grid_h
        self.grid_w               = grid_w

        # poses: [mask + full_local_vector] per agent
        #      + [num_people/30, ever_seen_ratio, active_agent_ratio,
        #         step_progress, remaining_progress]
        self.poses_dim = self.num_agents * (1 + local_dim) + 6

        self.actor = ActorNetwork(
            local_dim=local_dim,
            grid_channels=self.grid_channels,
            grid_h=grid_h,
            grid_w=grid_w,
            cnn_out_dim=cnn_out_dim,
            hidden_dim=hidden_dim,
            num_vx_bins=num_vx_bins,
            num_vy_bins=num_vy_bins,
            num_yaw_bins=num_yaw_bins,
        ).to(self.device)

        self.critic = CriticNetwork(
            grid_channels=self.critic_grid_channels,
            grid_h=grid_h,
            grid_w=grid_w,
            cnn_out_dim=cnn_out_dim,
            poses_dim=self.poses_dim,
            hidden_dim=hidden_dim,
        ).to(self.device)

        self.actor_opt  = torch.optim.Adam(self.actor.parameters(),  lr=lr_actor)
        self.critic_opt = torch.optim.Adam(self.critic.parameters(), lr=lr_critic)
        self.value_rms = RunningMeanStd()

        self.buffer = RolloutBuffer(
            rollout_len=rollout_len,
            num_agents=self.num_agents,
            grid_shape=(self.grid_channels, grid_h, grid_w),
            critic_grid_shape=(self.critic_grid_channels, grid_h, grid_w),
            local_dim=local_dim,
            action_dim=self.action_dim,
            move_mask_dim=self.move_mask_dim,
            poses_dim=self.poses_dim,
            gamma=gamma,
            gae_lambda=gae_lambda,
            device=device,
        )

        self.total_steps = 0
        self.current_update = 0
        self._sticky_env_actions = None
        self._last_rollout_sticky_rate = 0.0

        if use_tensorboard:
            try:
                from torch.utils.tensorboard import SummaryWriter
                os.makedirs(save_dir, exist_ok=True)
                self.writer = SummaryWriter(log_dir=os.path.join(save_dir, "tb"))
            except ImportError:
                print("[MAPPO] TensorBoard not available, skipping.")
                self.writer = None
        else:
            self.writer = None

        os.makedirs(save_dir, exist_ok=True)
        self.metrics_log_path = os.path.join(save_dir, "train_metrics.csv")
        self.critic_diag_log_path = os.path.join(save_dir, "critic_diagnostics.csv")
        self._ensure_metrics_log()
        self._ensure_critic_diag_log()
        self.live_debug = LiveDebugWindow(
            LiveDebugConfig(
                enabled=live_debug,
                every_steps=max(1, int(live_debug_every)),
                num_drones=self.num_agents,
            )
        )

    def _ensure_metrics_log(self):
        if os.path.exists(self.metrics_log_path) and os.path.getsize(self.metrics_log_path) > 0:
            return
        with open(self.metrics_log_path, "w", newline="", encoding="ascii") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "update",
                    "total_updates",
                    "steps",
                    "sim_time_seconds",
                    "rtf",
                    "decisions_per_real_second",
                    "avg_ep_reward",
                    "actor_loss",
                    "critic_loss",
                    "entropy",
                    "elapsed_seconds",
                ]
            )

    def _ensure_critic_diag_log(self):
        if os.path.exists(self.critic_diag_log_path) and os.path.getsize(self.critic_diag_log_path) > 0:
            return
        with open(self.critic_diag_log_path, "w", newline="", encoding="ascii") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "update",
                    "steps",
                    "value_mean",
                    "value_std",
                    "return_mean",
                    "return_std",
                    "value_return_corr",
                    "explained_variance",
                    "value_return_mae",
                    "value_return_rmse",
                    "zero_baseline_mae",
                    "zero_baseline_rmse",
                    "value_bias_mean",
                    "frac_abs_value_lt_1",
                    "frac_abs_return_lt_1",
                ]
            )

    def _append_metrics_row(
        self,
        update: int,
        total_updates: int,
        sim_seconds: float,
        rtf: float,
        decisions_per_real: float,
        avg_ep_reward: float,
        actor_loss: float,
        critic_loss: float,
        entropy: float,
        elapsed: float,
    ):
        with open(self.metrics_log_path, "a", newline="", encoding="ascii") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    update,
                    total_updates,
                    self.total_steps,
                    sim_seconds,
                    rtf,
                    decisions_per_real,
                    avg_ep_reward,
                    actor_loss,
                    critic_loss,
                    entropy,
                    elapsed,
                ]
            )

    def _compute_critic_diagnostics(self) -> Dict[str, float]:
        values = self.buffer.values.astype(np.float64)
        returns = self.buffer.returns.astype(np.float64)
        errors = values - returns

        value_std = float(values.std())
        return_std = float(returns.std())
        error_var = float(np.var(errors))
        return_var = float(np.var(returns))

        if value_std > 1e-8 and return_std > 1e-8:
            corr = float(np.corrcoef(values, returns)[0, 1])
        else:
            corr = float("nan")

        if return_var > 1e-8:
            explained_variance = 1.0 - (error_var / return_var)
        else:
            explained_variance = float("nan")

        return {
            "value_mean": float(values.mean()),
            "value_std": value_std,
            "return_mean": float(returns.mean()),
            "return_std": return_std,
            "value_return_corr": corr,
            "explained_variance": float(explained_variance),
            "value_return_mae": float(np.abs(errors).mean()),
            "value_return_rmse": float(math.sqrt(np.square(errors).mean())),
            "zero_baseline_mae": float(np.abs(returns).mean()),
            "zero_baseline_rmse": float(math.sqrt(np.square(returns).mean())),
            "value_bias_mean": float(errors.mean()),
            "frac_abs_value_lt_1": float(np.mean(np.abs(values) < 1.0)),
            "frac_abs_return_lt_1": float(np.mean(np.abs(returns) < 1.0)),
        }

    def _append_critic_diag_row(self, update: int, diagnostics: Dict[str, float]):
        with open(self.critic_diag_log_path, "a", newline="", encoding="ascii") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    update,
                    self.total_steps,
                    diagnostics["value_mean"],
                    diagnostics["value_std"],
                    diagnostics["return_mean"],
                    diagnostics["return_std"],
                    diagnostics["value_return_corr"],
                    diagnostics["explained_variance"],
                    diagnostics["value_return_mae"],
                    diagnostics["value_return_rmse"],
                    diagnostics["zero_baseline_mae"],
                    diagnostics["zero_baseline_rmse"],
                    diagnostics["value_bias_mean"],
                    diagnostics["frac_abs_value_lt_1"],
                    diagnostics["frac_abs_return_lt_1"],
                ]
            )

    def _obs_to_tensors(self, obs: List[Dict]):
        grids = np.stack([o["grid"] for o in obs], axis=0)
        locs  = np.stack([o["local"] for o in obs], axis=0)
        return (
            torch.FloatTensor(grids).to(self.device),
            torch.FloatTensor(locs).to(self.device),
        )

    def _pack_actor_inputs(
        self,
        grids_t: torch.Tensor,
        locs_t: torch.Tensor,
        move_masks_t: torch.Tensor,
    ):
        active_agents = grids_t.shape[0]
        packed_grids = np.zeros(
            (self.num_agents, self.grid_channels, self.grid_h, self.grid_w),
            dtype=np.float32,
        )
        packed_locs = np.zeros((self.num_agents, locs_t.shape[1]), dtype=np.float32)
        packed_actions = np.zeros((self.num_agents, self.action_dim), dtype=np.int64)
        packed_log_probs = np.zeros((self.num_agents,), dtype=np.float32)
        packed_move_masks = np.ones((self.num_agents, self.move_mask_dim), dtype=np.float32)
        actor_masks = np.zeros((self.num_agents,), dtype=np.float32)
        critic_weights = np.zeros((self.num_agents,), dtype=np.float32)
        packed_grids[:active_agents] = grids_t.cpu().numpy()
        packed_locs[:active_agents] = locs_t.cpu().numpy()
        packed_move_masks[:active_agents] = move_masks_t.cpu().numpy()
        actor_masks[:active_agents] = 1.0
        if active_agents > 0:
            critic_weights[:active_agents] = 1.0 / active_agents
        return (
            packed_grids,
            packed_locs,
            packed_actions,
            packed_log_probs,
            packed_move_masks,
            actor_masks,
            critic_weights,
        )

    def _critic_forward(self, global_state: Dict) -> torch.Tensor:
        """Run critic forward pass and return denormalized value prediction."""
        grid_t  = torch.FloatTensor(global_state["grid"]).unsqueeze(0).to(self.device)
        poses_t = torch.FloatTensor(global_state["poses"]).unsqueeze(0).to(self.device)
        norm_value = self.critic(grid_t, poses_t)
        return self.value_rms.denormalize_tensor(norm_value)

    def _compute_move_masks(self) -> np.ndarray:
        return compute_move_action_masks(
            drone_states=self.env._get_drone_states(),
            vx_bins=self.vx_bins,
            vy_bins=self.vy_bins,
            dt=self.env.dt,
            x_min=self.env.move_x_min,
            x_max=self.env.move_x_max,
            y_min=self.env.move_y_min,
            y_max=self.env.move_y_max,
        )

    def _augment_obs_with_move_masks(self, obs: List[Dict], move_masks: np.ndarray) -> List[Dict]:
        base_locals = np.stack([o["local"] for o in obs], axis=0)
        aug_locals = append_move_masks_to_local(base_locals, move_masks)
        augmented_obs = []
        for idx, item in enumerate(obs):
            augmented_item = dict(item)
            augmented_item["local"] = aug_locals[idx]
            augmented_item["move_mask"] = move_masks[idx]
            augmented_obs.append(augmented_item)
        return augmented_obs

    def _actions_to_env(self, action_indices: torch.Tensor) -> List:
        decoded = action_indices.detach().cpu().numpy().astype(np.int64)
        return [
            (
                float(self.vx_bins[action[0]]),
                float(self.vy_bins[action[1]]),
                0.0,
                float(self.yaw_rate_bins[action[2]]),
            )
            for action in decoded
        ]

    def _reset_sticky_actions(self):
        self._sticky_env_actions = None

    def _apply_sticky_actions(self, env_actions: List):
        """
        Apply standard sticky-action environment noise.

        The policy still samples a fresh action every decision step and PPO
        stores that sampled action/log-prob. Sticky actions only change which
        command the environment executes, matching the ALE/Gymnasium semantics
        where the environment may reuse the previously executed action.
        """
        if not env_actions:
            self._sticky_env_actions = None
            return env_actions, 0, 0

        executed_actions = list(env_actions)
        prev_actions = self._sticky_env_actions
        sticky_overrides = 0
        sticky_candidates = 0

        if (
            self.sticky_action_prob > 0.0
            and prev_actions is not None
            and len(prev_actions) == len(env_actions)
        ):
            sticky_mask = np.random.random(len(env_actions)) < self.sticky_action_prob
            sticky_overrides = int(sticky_mask.sum())
            sticky_candidates = len(env_actions)
            executed_actions = [
                prev_actions[i] if sticky_mask[i] else env_actions[i]
                for i in range(len(env_actions))
            ]

        self._sticky_env_actions = list(executed_actions)
        return executed_actions, sticky_overrides, sticky_candidates

    def _build_global_state(self, obs: List[Dict]) -> Dict:
        """Build critic global state using current env context."""
        return build_global_state(
            obs,
            self.num_agents,
            obs_builder=self.env.obs_builder,
            num_people=len(self.env.people),
            ever_seen=len(self.env.ever_seen),
            visible_count=self.env.last_visible_count,
            current_step=self.env.current_step,
            episode_steps=self.env.episode_steps,
        )

    def _maybe_update_live_debug(self, obs: List[Dict], update: int):
        if not self.live_debug.config.enabled:
            return
        if not obs:
            return
        if self.total_steps % self.live_debug.config.every_steps != 0:
            return
        self.live_debug.update(
            step=self.total_steps,
            update=update,
            active_drones=len(obs),
            grid=obs[0]["grid"],
            local_vec=obs[0]["local"],
        )

    def _check_pause(self):
        if self._pause_btn is None:
            return
        try:
            import pybullet as p
            val = p.readUserDebugParameter(self._pause_btn)
            if val != self._pause_btn_last:
                self._pause_btn_last = val
                self._paused = not self._paused
                print("[train] " + ("Paused." if self._paused else "Resumed."))
            while self._paused:
                time.sleep(0.05)
                val = p.readUserDebugParameter(self._pause_btn)
                if val != self._pause_btn_last:
                    self._pause_btn_last = val
                    self._paused = False
                    print("[train] Resumed.")
        except Exception:
            self._pause_btn = None

    @torch.no_grad()
    def _collect_rollout(self, obs: List[Dict], update: int) -> List[Dict]:
        self.actor.eval()
        self.critic.eval()
        sticky_overrides = 0
        sticky_candidates = 0

        for step_i in range(self.rollout_len):
            self._check_pause()
            move_masks = self._compute_move_masks()
            obs_with_masks = self._augment_obs_with_move_masks(obs, move_masks)
            self._maybe_update_live_debug(obs_with_masks, update)
            grids_t, locs_t = self._obs_to_tensors(obs_with_masks)
            move_masks_t = torch.FloatTensor(move_masks).to(self.device)
            action_indices, log_probs, _ = self.actor.get_action(
                grids_t,
                locs_t,
                move_mask=move_masks_t,
            )
            active_agents = action_indices.shape[0]
            (
                packed_grids,
                packed_locs,
                packed_actions,
                packed_log_probs,
                packed_move_masks,
                actor_masks,
                critic_weights,
            ) = self._pack_actor_inputs(grids_t, locs_t, move_masks_t)
            packed_actions[:active_agents] = action_indices.cpu().numpy()
            packed_log_probs[:active_agents] = log_probs.cpu().numpy()

            # Value estimate BEFORE stepping
            global_state = self._build_global_state(obs_with_masks)
            value        = self._critic_forward(global_state).item()

            proposed_actions = self._actions_to_env(action_indices)
            executed_actions, step_overrides, step_candidates = self._apply_sticky_actions(
                proposed_actions
            )
            sticky_overrides += step_overrides
            sticky_candidates += step_candidates
            next_obs, reward, done, info = self.env.step(executed_actions)

            self.buffer.add(
                grids=packed_grids,
                locals_=packed_locs,
                actions=packed_actions,
                log_probs=packed_log_probs,
                move_masks=packed_move_masks,
                actor_masks=actor_masks,
                critic_weights=critic_weights,
                reward=float(reward),
                done=done,
                global_grid=global_state["grid"],
                global_poses=global_state["poses"],
                value=value,
            )

            self.total_steps += 1
            obs = next_obs
            if done:
                obs = self.env.reset()
                self._reset_sticky_actions()

        last_move_masks = self._compute_move_masks()
        last_obs_with_masks = self._augment_obs_with_move_masks(obs, last_move_masks)
        last_global_state = self._build_global_state(last_obs_with_masks)
        last_value        = self._critic_forward(last_global_state).item()
        self.buffer.compute_returns(last_value)
        self._last_rollout_sticky_rate = (
            sticky_overrides / sticky_candidates if sticky_candidates > 0 else 0.0
        )

        return obs

    def _update(self):
        self.actor.train()
        self.critic.train()
        self.value_rms.update(self.buffer.returns)

        actor_losses, critic_losses, entropies = [], [], []
        ratios_all = []

        for _ in range(self.num_epochs):
            for batch in self.buffer.get_batches(self.batch_size):
                (
                    grids_b, locs_b, actions_b,
                    old_log_probs_b, move_masks_b, actor_masks_b, critic_weights_b, advantages_b,
                    returns_b, g_grids_b, g_poses_b,
                ) = batch

                # Actor update
                new_log_probs, entropy = self.actor.evaluate_actions(
                    grids_b,
                    locs_b,
                    actions_b,
                    move_mask=move_masks_b,
                )
                ratio  = (new_log_probs - old_log_probs_b).exp()
                surr1  = ratio * advantages_b * actor_masks_b
                surr2  = ratio.clamp(1.0 - self.clip_eps, 1.0 + self.clip_eps) * advantages_b * actor_masks_b
                valid_count = actor_masks_b.sum().clamp_min(1.0)
                actor_loss = -torch.min(surr1, surr2).sum() / valid_count
                if self.entropy_coef > 0:
                    actor_loss = actor_loss - self.entropy_coef * (entropy * actor_masks_b).sum() / valid_count

                self.actor_opt.zero_grad()
                actor_loss.backward()
                nn.utils.clip_grad_norm_(self.actor.parameters(), self.max_grad_norm)
                self.actor_opt.step()

                # Critic update
                values_pred = self.critic(g_grids_b, g_poses_b).squeeze(-1)
                returns_target = self.value_rms.normalize_tensor(returns_b)
                critic_loss = self.value_coef * (
                    nn.functional.smooth_l1_loss(values_pred, returns_target, reduction="none") * critic_weights_b
                ).sum() / critic_weights_b.sum().clamp_min(1.0)

                self.critic_opt.zero_grad()
                critic_loss.backward()
                nn.utils.clip_grad_norm_(self.critic.parameters(), 0.5)
                self.critic_opt.step()

                actor_losses.append(actor_loss.item())
                critic_losses.append(critic_loss.item())
                entropies.append(((entropy * actor_masks_b).sum() / valid_count).item())
                ratios_all.append(ratio.detach().cpu())

        return np.mean(actor_losses), np.mean(critic_losses), np.mean(entropies)

    def _infer_loaded_update(self, path: str, ckpt: Dict[str, object]) -> int:
        if "update" in ckpt:
            return int(ckpt["update"])

        match = re.search(r"ckpt_update_(\d+)\.pt$", os.path.basename(path))
        if match:
            return int(match.group(1))

        total_steps = ckpt.get("total_steps")
        rollout_len = ckpt.get("rollout_len", self.rollout_len)
        if total_steps is not None and rollout_len:
            return int(total_steps) // int(rollout_len)
        return 0

    def train(self, total_updates: int):
        obs       = self.env.reset()
        self._reset_sticky_actions()
        ep_reward = 0.0
        ep_count  = 0
        start      = time.time()
        steps_at_start = self.total_steps
        target_update = self.current_update + total_updates

        if self.gui:
            try:
                import pybullet as p
                self._pause_btn = p.addUserDebugParameter("Pause / Resume", 1, 0, 1)
                self._pause_btn_last = p.readUserDebugParameter(self._pause_btn)
            except Exception:
                self._pause_btn = None

        for _ in range(total_updates):
            update = self.current_update + 1
            obs = self._collect_rollout(obs, update)
            ep_reward += self.buffer.rewards.sum()
            done_count = int(self.buffer.dones.sum())
            if done_count > 0:
                ep_count += done_count
            critic_diag = self._compute_critic_diagnostics()

            a_loss, c_loss, ent = self._update()
            self.current_update = update

            if update % self.log_interval == 0:
                elapsed       = time.time() - start
                avg_ep_reward = ep_reward / max(ep_count, 1)
                sim_seconds        = self.total_steps * self.env.dt
                steps_this_run     = self.total_steps - steps_at_start
                rtf                = (steps_this_run * self.env.dt) / elapsed if elapsed > 0 else float("inf")
                decisions_per_real = steps_this_run / elapsed if elapsed > 0 else float("inf")
                print(
                    f"[Update {update:4d}/{target_update}]  "
                    f"steps={self.total_steps:7d}  "
                    f"sim_time={sim_seconds:8.1f}s  "
                    f"RTF={rtf:6.2f}x  "
                    f"decisions_per_real={decisions_per_real:7.2f}/s  "
                    f"sticky_rate={self._last_rollout_sticky_rate:5.2%}  "
                    f"avg_ep_rew={avg_ep_reward:.3f}  "
                    f"actor_loss={a_loss:.4f}  "
                    f"critic_loss={c_loss:.4f}  "
                    f"v_mean={critic_diag['value_mean']:.2f}  "
                    f"v_std={critic_diag['value_std']:.2f}  "
                    f"ret_mean={critic_diag['return_mean']:.2f}  "
                    f"ret_std={critic_diag['return_std']:.2f}  "
                    f"v_mae={critic_diag['value_return_mae']:.2f}  "
                    f"ev={critic_diag['explained_variance']:.3f}  "
                    f"corr={critic_diag['value_return_corr']:.3f}  "
                    f"entropy={ent:.3f}  "
                    f"elapsed={elapsed:.1f}s"
                )
                if self.writer:
                    self.writer.add_scalar("train/avg_ep_reward", avg_ep_reward, update)
                    self.writer.add_scalar("train/actor_loss",    a_loss,        update)
                    self.writer.add_scalar("train/critic_loss",   c_loss,        update)
                    self.writer.add_scalar("train/entropy",       ent,           update)
                    self.writer.add_scalar("critic/value_mean", critic_diag["value_mean"], update)
                    self.writer.add_scalar("critic/value_std", critic_diag["value_std"], update)
                    self.writer.add_scalar("critic/return_mean", critic_diag["return_mean"], update)
                    self.writer.add_scalar("critic/return_std", critic_diag["return_std"], update)
                    self.writer.add_scalar("critic/value_return_corr", critic_diag["value_return_corr"], update)
                    self.writer.add_scalar("critic/explained_variance", critic_diag["explained_variance"], update)
                    self.writer.add_scalar("critic/value_return_mae", critic_diag["value_return_mae"], update)
                    self.writer.add_scalar("critic/value_return_rmse", critic_diag["value_return_rmse"], update)
                    self.writer.add_scalar("critic/zero_baseline_mae", critic_diag["zero_baseline_mae"], update)
                    self.writer.add_scalar("critic/zero_baseline_rmse", critic_diag["zero_baseline_rmse"], update)
                    self.writer.add_scalar("critic/value_bias_mean", critic_diag["value_bias_mean"], update)
                    self.writer.add_scalar(
                        "train/sticky_rate",
                        self._last_rollout_sticky_rate,
                        update,
                    )
                self._append_metrics_row(
                    update=update,
                    total_updates=target_update,
                    sim_seconds=sim_seconds,
                    rtf=rtf,
                    decisions_per_real=decisions_per_real,
                    avg_ep_reward=avg_ep_reward,
                    actor_loss=a_loss,
                    critic_loss=c_loss,
                    entropy=ent,
                    elapsed=elapsed,
                )
                self._append_critic_diag_row(update=update, diagnostics=critic_diag)
                ep_reward = 0.0
                ep_count  = 0

            if update % (self.log_interval * 10) == 0:
                self.save(f"ckpt_update_{update}.pt")

        self.save("final.pt")
        print("[MAPPO] Training complete.")

    def save(self, filename: str):
        path = os.path.join(self.save_dir, filename)
        torch.save(
            {
                "actor":       self.actor.state_dict(),
                "critic":      self.critic.state_dict(),
                "actor_opt":   self.actor_opt.state_dict(),
                "critic_opt":  self.critic_opt.state_dict(),
                "num_agents":  self.num_agents,
                "local_dim": self.buffer.local_dim,
                "move_mask_dim": self.move_mask_dim,
                "total_steps": self.total_steps,
                "update": self.current_update,
                "rollout_len": self.rollout_len,
                "value_rms": self.value_rms.state_dict(),
            },
            path,
        )
        print(f"[MAPPO] Saved checkpoint -> {path}")

    def load(self, path: str):
        ckpt = torch.load(path, map_location=self.device)
        ckpt_local_dim = int(
            ckpt.get(
                "local_dim",
                ckpt["actor"]["local_mlp.0.weight"].shape[1],
            )
        )
        current_local_dim = int(self.actor.local_mlp[0].in_features)
        if ckpt_local_dim != current_local_dim:
            raise ValueError(
                "Checkpoint local_dim does not match the current observation layout: "
                f"checkpoint={ckpt_local_dim}, current={current_local_dim}. "
                "This checkpoint was trained with a different local feature vector."
            )
        self.actor.load_state_dict(ckpt["actor"])
        self.critic.load_state_dict(ckpt["critic"])
        self.actor_opt.load_state_dict(ckpt["actor_opt"])
        self.critic_opt.load_state_dict(ckpt["critic_opt"])
        self.total_steps = ckpt.get("total_steps", 0)
        self.current_update = self._infer_loaded_update(path, ckpt)
        if "value_rms" in ckpt:
            self.value_rms.load_state_dict(ckpt["value_rms"])
        print(f"[MAPPO] Loaded checkpoint <- {path} (update={self.current_update})")
