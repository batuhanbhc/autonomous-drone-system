"""
Utilities for building the centralised global state used by the critic.

The critic grid has shape ((shared_people_channels + 3) + 3 * max_agents, H, W):
  channel  0              — shared instantaneous spatial support union
  channel  1              — shared recent spatial support
  channel  2              — shared historic spatial support
  channel  3              — shared current-step count density
  channel  4              — shared recent count memory (optional)
  channel  5              — shared historic count memory (optional)
  channel ...             — shared FOV coverage union
  channel ...             — shared drone map
  channels ...            — per-drone instantaneous maps, padded to max_agents
  channels ...            — per-drone own coverage maps, padded to max_agents
  channels ...            — per-drone ego maps, padded to max_agents

The poses vector is (max_agents * (1 + local_dim) + 9,):
  per-agent slice: [mask, *full_local_vector]
  context:         [
                      num_people / 30.0,
                      ever_seen / num_people,
                      visible_count / num_people,
                      active_agents / max_agents,
                      current_step / episode_steps,
                      remaining_steps / episode_steps,
                      search_phase_progress,
                      is_search_phase,
                      is_coverage_phase,
                   ]
"""

import numpy as np


def build_global_state(
    obs,
    max_agents,
    obs_builder,
    num_people: int = 0,
    ever_seen: int = 0,
    visible_count: int = 0,
    current_step: int = 0,
    episode_steps: int = 1,
    search_phase_progress: float = 1.0,
    is_search_phase: float = 0.0,
    is_coverage_phase: float = 1.0,
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
            visible_count / max(num_people, 1),
            active_agents / max(max_agents, 1),
            step_progress,
            remaining_progress,
            min(max(float(search_phase_progress), 0.0), 1.0),
            float(is_search_phase),
            float(is_coverage_phase),
        ],
        dtype=np.float32,
    )

    # Ch0 for critic: union (max) of all per-drone instantaneous spatial-support maps.
    # instant_maps_snapshot is padded to max_agents slots by ObservationBuilder.
    shared_instant = obs_builder.instant_maps_snapshot.max(axis=0)

    shared_people_end = 1 + obs_builder.shared_people_channels
    own_coverage_channel = obs_builder.actor_own_coverage_channel
    shared_drone_channel = obs_builder.actor_shared_drone_channel

    # Shared actor channels 1..shared_people_end-1 are identical across drones.
    shared_people = obs[0]["grid"][1:shared_people_end]
    shared_coverage = obs_builder.coverage_map.astype(np.float32)
    shared_drone_map = obs[0]["grid"][shared_drone_channel]

    shared = np.concatenate(
        [
            shared_instant[np.newaxis, :, :],
            shared_people,
            shared_coverage[np.newaxis, :, :],
            shared_drone_map[np.newaxis, :, :],
        ],
        axis=0,
    )

    # Per-drone instantaneous maps preserve who sees what on this step.
    # ObservationBuilder stores one slot per max agent, so inactive slots stay zero.
    instant_maps = obs_builder.instant_maps_snapshot[:max_agents].astype(np.float32)

    own_coverage_maps = [o["grid"][own_coverage_channel] for o in obs]
    while len(own_coverage_maps) < max_agents:
        own_coverage_maps.append(np.zeros_like(obs[0]["grid"][own_coverage_channel]))
    own_coverage_maps = np.stack(own_coverage_maps[:max_agents], axis=0)

    # Critic keeps one ego map per active drone, separate from the actor's shared
    # drone-position channel.
    active_ego_maps = [o.get("own_ego_map", o["grid"][-1]) for o in obs]
    while len(active_ego_maps) < max_agents:
        active_ego_maps.append(np.zeros_like(obs[0]["grid"][-1]))
    ego_maps = np.stack(active_ego_maps[:max_agents], axis=0)

    # Critic grid: shared channels + per-drone instantaneous + per-drone own
    # coverage + per-drone ego.
    # Inactive drone slots remain zero-filled.
    grid = np.concatenate(
        [shared, instant_maps, own_coverage_maps, ego_maps],
        axis=0,
    ).astype(np.float32)

    return {
        "grid":  grid,
        "poses": np.concatenate([poses, context]),
    }
