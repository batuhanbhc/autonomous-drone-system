"""
Utilities for building the centralised global state used by the critic.

The critic grid has shape (6 + 2 * max_agents, H, W):
  channel  0              — shared instantaneous detections union
  channel  1              — shared recent presence
  channel  2              — shared historic presence
  channel  3              — shared current-step count density
  channel  4              — shared FOV coverage
  channel  5              — shared drone map
  channels 6..6+N-1       — per-drone instantaneous maps, padded to max_agents
  channels 6+N..6+2N-1    — per-drone ego maps, padded to max_agents

With 2 drones the critic grid is (10, H, W).

The poses vector is (max_agents * (1 + local_dim) + 5,):
  per-agent slice: [mask, *full_local_vector]
  context:         [
                      num_people / 30.0,
                      ever_seen / num_people,
                      active_agents / max_agents,
                      current_step / episode_steps,
                      remaining_steps / episode_steps,
                   ]
"""

import numpy as np


def build_global_state(
    obs,
    max_agents,
    obs_builder,
    num_people: int = 0,
    ever_seen: int = 0,
    current_step: int = 0,
    episode_steps: int = 1,
):
    """
    Parameters
    ----------
    obs         : list of per-drone observation dicts (grid + local)
    max_agents  : maximum number of drones
    obs_builder : ObservationBuilder instance — provides instant_maps_snapshot
                  (per-slot, shape (max_agents, H, W))
    num_people  : current number of people in the env
    ever_seen   : cumulative count of people ever seen this episode
    current_step: current step index inside the episode
    episode_steps: maximum steps in the episode
    """
    active_agents = len(obs)
    local_dim = obs[0]["local"].shape[0]
    pose_slices = []
    for agent_idx in range(active_agents):
        pose_slices.append(np.concatenate([np.array([1.0], dtype=np.float32), obs[agent_idx]["local"]], axis=0))
    for _ in range(max_agents - active_agents):
        pose_slices.append(np.zeros((1 + local_dim,), dtype=np.float32))
    poses = np.concatenate(pose_slices, axis=0).astype(np.float32)

    step_denom = max(episode_steps, 1)
    current_step_clamped = min(max(current_step, 0), step_denom)
    step_progress = current_step_clamped / step_denom
    remaining_progress = max(step_denom - current_step_clamped, 0) / step_denom

    context = np.array(
        [
            num_people / 30.0,
            ever_seen / max(num_people, 1),
            active_agents / max(max_agents, 1),
            step_progress,
            remaining_progress,
        ],
        dtype=np.float32,
    )

    # Ch0 for critic: union (max) of all per-drone instantaneous detections.
    # instant_maps_snapshot is padded to max_agents slots by ObservationBuilder.
    shared_instant = obs_builder.instant_maps_snapshot.max(axis=0)

    # Shared actor channels 1-5 (recent/historic presence, count density,
    # coverage, drone map) are identical across all active drones.
    shared_recent = obs[0]["grid"][1]     # (H, W)
    shared_historic = obs[0]["grid"][2]   # (H, W)
    shared_count_density = obs[0]["grid"][3]   # (H, W)
    shared_coverage = obs[0]["grid"][4]   # (H, W)
    shared_drone_map = obs[0]["grid"][5]  # (H, W)

    shared = np.stack(
        [
            shared_instant,
            shared_recent,
            shared_historic,
            shared_count_density,
            shared_coverage,
            shared_drone_map,
        ],
        axis=0,
    )

    # Per-drone instantaneous maps preserve who sees what on this step.
    # ObservationBuilder stores one slot per max agent, so inactive slots stay zero.
    instant_maps = obs_builder.instant_maps_snapshot[:max_agents].astype(np.float32)

    # Critic keeps one ego map per active drone, separate from the actor's
    # shared drone-position channel.
    active_ego_maps = [o.get("own_ego_map", o["grid"][-1]) for o in obs]
    while len(active_ego_maps) < max_agents:
        active_ego_maps.append(np.zeros_like(obs[0]["grid"][-1]))
    ego_maps = np.stack(active_ego_maps[:max_agents], axis=0)

    # Critic grid: shared actor-style channels + per-drone instantaneous + per-drone ego.
    # Inactive drone slots remain zero-filled.
    grid = np.concatenate([shared, instant_maps, ego_maps], axis=0).astype(np.float32)

    return {
        "grid":  grid,
        "poses": np.concatenate([poses, context]),
    }
