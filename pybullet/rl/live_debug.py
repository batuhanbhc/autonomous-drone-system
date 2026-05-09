from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass
class LiveDebugConfig:
    enabled: bool = False
    every_steps: int = 100
    num_drones: int = 2
    cmd_history_len: int = 0
    hotspot_top_k: int = 0
    move_mask_dim: int = 0
    actor_channel_names: Sequence[str] | None = None


class LiveDebugWindow:
    def __init__(self, config: LiveDebugConfig):
        self.config = config
        self._plt = None
        self._fig = None
        self._image_axes = []
        self._images = []
        self._colorbars = []
        self._channel_titles = []
        self._local_text_ax = None
        self._meta_text_ax = None
        self._local_text = None
        self._meta_text = None
        self._labels = self._build_local_labels(
            config.num_drones,
            config.cmd_history_len,
            config.hotspot_top_k,
            config.move_mask_dim,
        )
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
        channel_titles = list(config.actor_channel_names or [])
        if not channel_titles:
            channel_titles = [
                "Instant Spatial Support",
                "Count Density",
                "Historic Count Memory",
                "Persistent Coverage",
                "Own Coverage",
                "Teammate Coverage",
                "Shared Drone Map",
                "Own Ego Map",
            ]
        num_channels = len(channel_titles)
        num_cols = 5
        num_rows = max(2, (num_channels + num_cols - 1) // num_cols)
        fig = plt.figure(figsize=(16, (3.2 * num_rows) + 4.6))
        self._fig = fig
        grid_spec = fig.add_gridspec(
            nrows=num_rows + 2,
            ncols=num_cols,
            height_ratios=[1.0] * num_rows + [0.85, 0.35],
            hspace=0.55,
            wspace=0.35,
        )
        image_axes = [
            fig.add_subplot(grid_spec[idx // num_cols, idx % num_cols])
            for idx in range(num_rows * num_cols)
        ]
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
            self._channel_titles.append(title)
            self._colorbars.append(
                fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
            )
        for ax in image_axes[len(channel_titles):]:
            ax.axis("off")
        self._image_axes = image_axes

        self._local_text_ax = fig.add_subplot(grid_spec[num_rows, :])
        self._meta_text_ax = fig.add_subplot(grid_spec[num_rows + 1, :])
        self._local_text_ax.axis("off")
        self._meta_text_ax.axis("off")
        self._local_text = self._local_text_ax.text(
            0.0,
            1.0,
            "",
            va="top",
            ha="left",
            family="monospace",
            fontsize=8.5,
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
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        self._available = True

    @staticmethod
    def _build_local_labels(
        num_drones: int,
        cmd_history_len: int,
        hotspot_top_k: int,
        move_mask_dim: int,
    ) -> list[str]:
        labels = [
            "own_x",
            "own_y",
            "own_sin_yaw",
            "own_cos_yaw",
            "own_num_visible",
            "search_prog",
            "is_search",
            "is_coverage",
            "centroid_valid",
            "centroid_fwd",
            "centroid_lat",
        ]
        for hotspot_idx in range(max(0, hotspot_top_k)):
            labels.extend(
                [
                    f"hs{hotspot_idx}_valid",
                    f"hs{hotspot_idx}_dx",
                    f"hs{hotspot_idx}_dy",
                    f"hs{hotspot_idx}_dens",
                    f"hs{hotspot_idx}_age",
                ]
            )
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
        for hist_idx in range(max(0, cmd_history_len)):
            labels.extend(
                [
                    f"cmd{hist_idx}_vx",
                    f"cmd{hist_idx}_vy",
                    f"cmd{hist_idx}_yaw",
                ]
            )
        for move_idx in range(max(0, move_mask_dim)):
            labels.append(f"move_mask_{move_idx}")
        return labels

    @staticmethod
    def _format_local_text(lines: list[str], num_cols: int = 3) -> str:
        if not lines:
            return ""
        num_cols = max(1, int(num_cols))
        rows = int(math.ceil(len(lines) / num_cols))
        columns = [
            lines[col_idx * rows:(col_idx + 1) * rows]
            for col_idx in range(num_cols)
        ]
        widths = [max((len(line) for line in column), default=0) for column in columns]
        formatted = []
        for row_idx in range(rows):
            parts = []
            for col_idx, column in enumerate(columns):
                if row_idx < len(column):
                    parts.append(column[row_idx].ljust(widths[col_idx] + 4))
            formatted.append("".join(parts).rstrip())
        return "\n".join(formatted)

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
            vmax = max(float(channel.max()), 1.0)
            im.set_data(channel)
            im.set_clim(0.0, vmax)
            self._image_axes[channel_idx].set_title(
                f"{self._channel_titles[channel_idx]}\nscale [0.00, {vmax:.2f}]"
            )
            self._colorbars[channel_idx].update_normal(im)

        local_lines = []
        for idx, value in enumerate(np.asarray(local_vec, dtype=np.float32)):
            label = self._labels[idx] if idx < len(self._labels) else f"local_{idx}"
            local_lines.append(f"{idx:02d} {label:<16} {value: .4f}")
        self._local_text.set_text(self._format_local_text(local_lines))

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
