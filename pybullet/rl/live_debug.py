from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass
class LiveDebugConfig:
    enabled: bool = False
    every_steps: int = 100
    num_drones: int = 2


class LiveDebugWindow:
    def __init__(self, config: LiveDebugConfig):
        self.config = config
        self._plt = None
        self._fig = None
        self._image_axes = []
        self._images = []
        self._local_text_ax = None
        self._meta_text_ax = None
        self._local_text = None
        self._meta_text = None
        self._labels = self._build_local_labels(config.num_drones)
        self._available = False

        if not config.enabled:
            return

        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("[LiveDebug] matplotlib not available; disabling live debug window.")
            return

        self._plt = plt
        plt.ion()
        fig, axes = plt.subplots(2, 4, figsize=(15, 8))
        self._fig = fig

        channel_titles = [
            "Recent Presence",
            "Historic Presence",
            "Recent Density",
            "Historic Density",
            "Coverage",
            "Shared Drone Map",
        ]
        image_axes = [axes[0, 0], axes[0, 1], axes[0, 2], axes[0, 3], axes[1, 0], axes[1, 1]]
        for ax, title in zip(image_axes, channel_titles):
            im = ax.imshow(
                np.zeros((1, 1), dtype=np.float32),
                origin="lower",
                cmap="viridis",
                vmin=0.0,
                vmax=1.0,
                interpolation="nearest",
            )
            ax.set_title(title)
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            self._images.append(im)
        self._image_axes = image_axes

        self._local_text_ax = axes[1, 2]
        self._meta_text_ax = axes[1, 3]
        self._local_text_ax.axis("off")
        self._meta_text_ax.axis("off")
        self._local_text = self._local_text_ax.text(
            0.0,
            1.0,
            "",
            va="top",
            ha="left",
            family="monospace",
            fontsize=9,
        )
        self._meta_text = self._meta_text_ax.text(
            0.0,
            1.0,
            "",
            va="top",
            ha="left",
            family="monospace",
            fontsize=10,
        )

        fig.suptitle("Live Debug: Drone 0 Actor Inputs")
        fig.tight_layout()
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        self._available = True

    @staticmethod
    def _build_local_labels(num_drones: int) -> list[str]:
        labels = [
            "own_x",
            "own_y",
            "own_z",
            "own_sin_yaw",
            "own_cos_yaw",
            "own_num_visible",
        ]
        for teammate_idx in range(max(0, num_drones - 1)):
            labels.extend(
                [
                    f"tm{teammate_idx}_mask",
                    f"tm{teammate_idx}_rel_x",
                    f"tm{teammate_idx}_rel_y",
                    f"tm{teammate_idx}_rel_z",
                    f"tm{teammate_idx}_sin_yaw",
                    f"tm{teammate_idx}_cos_yaw",
                ]
            )
        return labels

    def update(
        self,
        *,
        step: int,
        update: int,
        active_drones: int,
        grid: np.ndarray,
        local_vec: np.ndarray,
    ) -> None:
        if not self._available:
            return
        if self._fig is None or not self._plt.fignum_exists(self._fig.number):
            self._available = False
            return

        for channel_idx, im in enumerate(self._images):
            channel = np.asarray(grid[channel_idx], dtype=np.float32)
            im.set_data(channel)
            im.set_clim(0.0, max(float(channel.max()), 1.0))

        local_lines = []
        for idx, value in enumerate(np.asarray(local_vec, dtype=np.float32)):
            label = self._labels[idx] if idx < len(self._labels) else f"local_{idx}"
            local_lines.append(f"{idx:02d} {label:<16} {value: .4f}")
        self._local_text.set_text("\n".join(local_lines))

        self._meta_text.set_text(
            "\n".join(
                [
                    f"train_step: {step}",
                    f"update:     {update}",
                    f"active:     {active_drones}",
                    f"grid_shape: {tuple(grid.shape)}",
                    f"local_dim:  {local_vec.shape[0]}",
                ]
            )
        )

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()
        self._plt.pause(0.001)
