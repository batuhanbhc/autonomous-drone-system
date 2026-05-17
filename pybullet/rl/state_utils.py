"""
Utilities for building the centralised global state used by the critic.

Current local-count layout:
  critic grid shape = ((shared_people_channels + 5) + 5 * max_agents, H, W)
    shared channels  — actor shared people channels, shared instant FOV
                       footprint, shared FOV coverage, shared drone map,
                       GT people occupancy, GT people density
    per-drone blocks — local count density, local recent count memory,
                       own instant footprint, own coverage, ego

Legacy instant-map layout:
  critic grid shape = ((shared_people_channels + 6) + (4 or 5) * max_agents, H, W)
    leading shared channel — shared instantaneous spatial-support union
    per-drone blocks       — local instant map, optional local recent count
                             memory, own instant footprint, own coverage, ego

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
    local_key = "critic_local" if "critic_local" in obs[0] else "local"
    local_dim = obs[0][local_key].shape[0]
    pose_slices = []
    for agent_idx in range(active_agents):
        pose_slices.append(
            np.concatenate(
                [np.array([1.0], dtype=np.float32), obs[agent_idx][local_key]],
                axis=0,
            )
        )
    for _ in range(max_agents - active_agents):
        pose_slices.append(np.zeros((1 + local_dim,), dtype=np.float32))
    poses = np.concatenate(pose_slices, axis=0).astype(np.float32)

    step_denom = max(episode_steps, 1)
    current_step_clamped = min(max(current_step, 0), step_denom)
    step_progress = current_step_clamped / step_denom
    remaining_progress = max(step_denom - current_step_clamped, 0) / step_denom

    people_count_normalizer = float(
        getattr(obs_builder, "people_count_normalizer", 30.0)
    )
    context = np.array(
        [
            num_people / people_count_normalizer,
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

    own_instant_coverage_channel = getattr(
        obs_builder,
        "actor_own_instant_coverage_channel",
        None,
    )
    own_coverage_channel = obs_builder.actor_own_coverage_channel
    shared_drone_channel = obs_builder.actor_shared_drone_channel

    shared_people_parts = []
    if getattr(obs_builder, "exposes_spatial_memory_channels", False):
        shared_people_parts.extend(
            [
                np.asarray(obs_builder.people_belief_recent, dtype=np.float32)[np.newaxis, :, :],
                np.asarray(obs_builder.people_belief_historic, dtype=np.float32)[np.newaxis, :, :],
            ]
        )
    if getattr(obs_builder, "include_shared_count_density_channel", False):
        shared_people_parts.append(
            np.asarray(obs_builder.people_count_density, dtype=np.float32)[np.newaxis, :, :]
        )
    shared_people_parts.append(
        np.asarray(obs_builder.people_count_memory_historic, dtype=np.float32)[np.newaxis, :, :]
    )
    if getattr(obs_builder, "include_persistent_coverage_channel", False):
        shared_people_parts.append(
            np.asarray(obs_builder.persistent_coverage_map, dtype=np.float32)[np.newaxis, :, :]
        )
    shared_people = np.concatenate(shared_people_parts, axis=0).astype(np.float32)
    shared_instant_coverage = obs_builder.footprint_maps_snapshot.max(axis=0).astype(
        np.float32
    )
    shared_coverage = obs_builder.coverage_map.astype(np.float32)
    shared_drone_map = obs[0]["grid"][shared_drone_channel]
    gt_people_binary = getattr(
        obs_builder,
        "gt_people_binary_snapshot",
        np.zeros_like(shared_coverage, dtype=np.float32),
    ).astype(np.float32)
    gt_people_density = getattr(
        obs_builder,
        "gt_people_density_snapshot",
        np.zeros_like(shared_coverage, dtype=np.float32),
    ).astype(np.float32)

    shared_parts = [
        shared_people,
    ]
    if own_instant_coverage_channel is not None:
        shared_parts.append(shared_instant_coverage[np.newaxis, :, :])
    shared_parts.extend([
        shared_coverage[np.newaxis, :, :],
        shared_drone_map[np.newaxis, :, :],
        gt_people_binary[np.newaxis, :, :],
        gt_people_density[np.newaxis, :, :],
    ])
    if getattr(obs_builder, "critic_has_shared_local_people_union", False):
        # Legacy layout keeps a shared union of the per-drone instantaneous maps.
        shared_local_people_union = obs_builder.local_people_maps_snapshot.max(axis=0)
        shared_parts.insert(0, shared_local_people_union[np.newaxis, :, :])
    shared = np.concatenate(shared_parts, axis=0)

    # Per-drone local people maps preserve who observed the local density/support
    # this step. ObservationBuilder stores one slot per max agent, so inactive
    # slots stay zero.
    local_people_maps = obs_builder.local_people_maps_snapshot[:max_agents].astype(np.float32)
    local_recent_count_memory_maps = obs_builder.local_recent_count_memory_maps_snapshot[
        :max_agents
    ].astype(np.float32)

    own_instant_coverage_maps = []
    if own_instant_coverage_channel is not None:
        own_instant_coverage_maps = [
            o["grid"][own_instant_coverage_channel] for o in obs
        ]
        while len(own_instant_coverage_maps) < max_agents:
            own_instant_coverage_maps.append(
                np.zeros_like(obs[0]["grid"][own_instant_coverage_channel])
            )
        own_instant_coverage_maps = np.stack(
            own_instant_coverage_maps[:max_agents],
            axis=0,
        )

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

    # Critic grid: shared channels + per-drone local people maps + optional
    # per-drone local recent count memory + per-drone instant footprint
    # + per-drone own coverage + per-drone ego.
    # Inactive drone slots remain zero-filled.
    grid_parts = [shared, local_people_maps]
    if getattr(obs_builder, "critic_has_local_recent_count_memory_channel", False):
        grid_parts.append(local_recent_count_memory_maps)
    if own_instant_coverage_channel is not None:
        grid_parts.append(own_instant_coverage_maps)
    grid_parts.extend([own_coverage_maps, ego_maps])
    grid = np.concatenate(grid_parts, axis=0).astype(np.float32)

    return {
        "grid":  grid,
        "poses": np.concatenate([poses, context]),
    }
