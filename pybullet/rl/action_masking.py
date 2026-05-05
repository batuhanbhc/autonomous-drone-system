"""
Helpers for direction-aware movement action masking.

The movement policy chooses a joint (vx, vy) action, so masks are computed
over full planar directions rather than over vx/vy independently.
"""

from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np


def num_move_actions(vx_bins: Sequence[float], vy_bins: Sequence[float]) -> int:
    return len(vx_bins) * len(vy_bins)


def compute_move_action_masks(
    drone_states: Iterable[dict],
    vx_bins: Sequence[float],
    vy_bins: Sequence[float],
    dt: float,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> np.ndarray:
    """
    Return a float32 array of shape (num_active_drones, num_move_actions).

    The move index ordering is:
      move_idx = vx_idx * len(vy_bins) + vy_idx
    """
    vx_bins = np.asarray(vx_bins, dtype=np.float32)
    vy_bins = np.asarray(vy_bins, dtype=np.float32)
    masks = []
    for drone_state in drone_states:
        x, y, _ = drone_state["position"]
        mask = np.zeros((num_move_actions(vx_bins, vy_bins),), dtype=np.float32)
        out_idx = 0
        for vx in vx_bins:
            for vy in vy_bins:
                new_x = x + float(vx) * dt
                new_y = y + float(vy) * dt
                mask[out_idx] = float(
                    x_min <= new_x <= x_max
                    and y_min <= new_y <= y_max
                )
                out_idx += 1
        if not mask.any():
            raise ValueError("Expected at least one valid movement action for each drone.")
        masks.append(mask)

    if not masks:
        return np.zeros((0, num_move_actions(vx_bins, vy_bins)), dtype=np.float32)
    return np.stack(masks, axis=0)


def append_move_masks_to_local(local_vectors: np.ndarray, move_masks: np.ndarray) -> np.ndarray:
    if local_vectors.shape[0] != move_masks.shape[0]:
        raise ValueError(
            "local_vectors and move_masks must share the batch dimension, "
            f"got {local_vectors.shape[0]} and {move_masks.shape[0]}"
        )
    return np.concatenate([local_vectors, move_masks], axis=1).astype(np.float32)
