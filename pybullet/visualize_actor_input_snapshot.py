#!/usr/bin/env python3
"""
Visualize actor-input snapshots alongside their nearest recorded video frame.

Usage:
    python visualize_actor_input_snapshot.py /path/to/recording_dir
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class FrameIndexEntry:
    frame_index: int
    timestamp_ns: int


@dataclass(frozen=True)
class FrameMatch:
    entry: FrameIndexEntry
    delta_ns: int
    clamped: bool


@dataclass(frozen=True)
class RecordingSession:
    root_dir: Path
    snapshot_files: list[Path]
    video_path: Path
    frame_index_path: Path
    frame_entries: list[FrameIndexEntry]
    reverse_mounted: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Open a recording directory containing actor_input_snapshots/, videos/, and data/, "
            "then show a synchronized snapshot/video viewer."
        )
    )
    parser.add_argument(
        "path",
        nargs="?",
        type=Path,
        help="Recording directory path.",
    )
    parser.add_argument(
        "--snapshot-index",
        type=int,
        default=0,
        help="Starting snapshot index. Defaults to 0.",
    )
    parser.add_argument(
        "--list-only",
        action="store_true",
        help="Print resolved files and timestamp match info, then exit.",
    )
    return parser.parse_args()


def prompt_path() -> Path:
    while True:
        raw = input("Recording directory path: ").strip()
        if raw:
            return Path(raw).expanduser()
        print("Enter a valid recording directory path.")


def timestamp_to_ns(sec: int, nanosec: int) -> int:
    return int(sec) * 1_000_000_000 + int(nanosec)


def resolve_recording_session(path: Path) -> RecordingSession:
    root_dir = path.expanduser().resolve()
    if not root_dir.exists():
        raise FileNotFoundError(f"Path does not exist: {root_dir}")
    if not root_dir.is_dir():
        raise ValueError(f"Expected a recording directory: {root_dir}")

    snapshot_dir = root_dir / "actor_input_snapshots"
    videos_dir = root_dir / "videos"
    data_dir = root_dir / "data"
    if not snapshot_dir.is_dir():
        raise FileNotFoundError(f"Missing actor_input_snapshots directory under {root_dir}")
    if not videos_dir.is_dir():
        raise FileNotFoundError(f"Missing videos directory under {root_dir}")
    if not data_dir.is_dir():
        raise FileNotFoundError(f"Missing data directory under {root_dir}")

    snapshot_files = sorted(snapshot_dir.glob("snapshot_*.json"))
    if not snapshot_files:
        raise FileNotFoundError(f"No snapshot_*.json files found under {snapshot_dir}")

    video_files = sorted(videos_dir.glob("*.avi"))
    if not video_files:
        raise FileNotFoundError(f"No .avi files found under {videos_dir}")
    video_path = video_files[0]

    preferred_frame_index = data_dir / f"{video_path.stem}_frame_index.csv"
    if preferred_frame_index.is_file():
        frame_index_path = preferred_frame_index
    else:
        frame_index_files = sorted(data_dir.glob("*_frame_index.csv"))
        if not frame_index_files:
            raise FileNotFoundError(f"No *_frame_index.csv files found under {data_dir}")
        frame_index_path = frame_index_files[0]

    frame_entries = load_frame_index(frame_index_path)
    reverse_mounted = load_reverse_mounted_flag(root_dir / "metadata.txt")
    return RecordingSession(
        root_dir=root_dir,
        snapshot_files=snapshot_files,
        video_path=video_path,
        frame_index_path=frame_index_path,
        frame_entries=frame_entries,
        reverse_mounted=reverse_mounted,
    )


def load_reverse_mounted_flag(path: Path) -> bool:
    if not path.is_file():
        return False
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        if key.strip() == "reverse_mounted":
            return value.strip().lower() == "true"
    return False


def load_frame_index(path: Path) -> list[FrameIndexEntry]:
    entries: list[FrameIndexEntry] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            entries.append(
                FrameIndexEntry(
                    frame_index=int(row["frame_index"]),
                    timestamp_ns=timestamp_to_ns(row["stamp_sec"], row["stamp_nanosec"]),
                )
            )
    if not entries:
        raise ValueError(f"Frame index CSV is empty: {path}")
    return entries


def load_snapshot(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def choose_snapshot_index(files: list[Path], requested_index: int) -> int:
    if requested_index < 0 or requested_index >= len(files):
        raise IndexError(f"snapshot index {requested_index} is out of range for {len(files)} files")
    return requested_index


def get_channel_entries(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    root_names = snapshot.get("channel_names", [])
    raw_entries = snapshot["actor_input"]["grid"]
    entries: list[dict[str, Any]] = []
    for idx, entry in enumerate(raw_entries):
        if isinstance(entry, dict):
            entries.append(
                {
                    "index": int(entry.get("channel_index", idx)),
                    "name": str(entry.get("name", root_names[idx] if idx < len(root_names) else f"channel_{idx}")),
                    "values": entry["values"],
                }
            )
        else:
            entries.append(
                {
                    "index": idx,
                    "name": root_names[idx] if idx < len(root_names) else f"channel_{idx}",
                    "values": entry,
                }
            )
    return entries


def get_local_entries(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    named = snapshot["actor_input"].get("local_base_named")
    if named:
        return [
            {
                "index": int(entry["index"]),
                "name": str(entry["name"]),
                "value": float(entry["value"]),
            }
            for entry in named
        ]

    names = snapshot.get("local_feature_names", [])
    values = snapshot["actor_input"]["local_base"]
    return [
        {
            "index": idx,
            "name": names[idx] if idx < len(names) else f"local_{idx}",
            "value": float(value),
        }
        for idx, value in enumerate(values)
    ]


def infer_move_mask_shape(mask: list[float]) -> tuple[int, int]:
    if not mask:
        return 1, 1
    side = int(round(math.sqrt(len(mask))))
    if side * side == len(mask):
        return side, side
    return 1, len(mask)


def format_timestamp_ns(timestamp_ns: int) -> str:
    sec = timestamp_ns // 1_000_000_000
    nanosec = timestamp_ns % 1_000_000_000
    return f"{sec}.{nanosec:09d}"


def snapshot_timestamp_ns(snapshot: dict[str, Any]) -> int:
    stamp = snapshot.get("timestamp", {})
    return timestamp_to_ns(stamp.get("sec", 0), stamp.get("nanosec", 0))


def match_frame(frame_entries: list[FrameIndexEntry], target_timestamp_ns: int) -> FrameMatch:
    timestamps = [entry.timestamp_ns for entry in frame_entries]
    insert_at = bisect.bisect_left(timestamps, target_timestamp_ns)
    clamped = target_timestamp_ns < timestamps[0] or target_timestamp_ns > timestamps[-1]

    if insert_at <= 0:
        chosen = frame_entries[0]
    elif insert_at >= len(frame_entries):
        chosen = frame_entries[-1]
    else:
        before = frame_entries[insert_at - 1]
        after = frame_entries[insert_at]
        before_delta = abs(before.timestamp_ns - target_timestamp_ns)
        after_delta = abs(after.timestamp_ns - target_timestamp_ns)
        chosen = before if before_delta <= after_delta else after

    return FrameMatch(
        entry=chosen,
        delta_ns=chosen.timestamp_ns - target_timestamp_ns,
        clamped=clamped,
    )


def print_recording_summary(session: RecordingSession, start_index: int) -> None:
    first_frame_ts = session.frame_entries[0].timestamp_ns
    last_frame_ts = session.frame_entries[-1].timestamp_ns
    print("Recording summary")
    print(f"  root_dir:         {session.root_dir}")
    print(f"  snapshots:        {len(session.snapshot_files)} files")
    print(f"  start_index:      {start_index}")
    print(f"  video_path:       {session.video_path}")
    print(f"  frame_index_path: {session.frame_index_path}")
    print(f"  reverse_mounted:  {session.reverse_mounted}")
    print(f"  frame_range:      {session.frame_entries[0].frame_index}..{session.frame_entries[-1].frame_index}")
    print(f"  frame_times:      {format_timestamp_ns(first_frame_ts)} .. {format_timestamp_ns(last_frame_ts)}")

    snapshot = load_snapshot(session.snapshot_files[start_index])
    snap_ts = snapshot_timestamp_ns(snapshot)
    match = match_frame(session.frame_entries, snap_ts)
    print("\nInitial snapshot match")
    print(f"  snapshot_file:    {session.snapshot_files[start_index].name}")
    print(f"  snapshot_step:    {snapshot.get('step', '?')}")
    print(f"  snapshot_time:    {format_timestamp_ns(snap_ts)}")
    print(f"  matched_frame:    {match.entry.frame_index}")
    print(f"  matched_time:     {format_timestamp_ns(match.entry.timestamp_ns)}")
    print(f"  delta_ms:         {match.delta_ns / 1_000_000.0:+.3f}")
    print(f"  clamped:          {match.clamped}")


def import_viewer_dependencies():
    try:
        import cv2
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.widgets import Button
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "This viewer requires opencv-python, matplotlib, and numpy. "
            "Install them in your Python environment and rerun the script."
        ) from exc
    return cv2, plt, np, Button


class SnapshotVideoViewer:
    def __init__(self, session: RecordingSession, start_index: int):
        cv2, plt, np, button_cls = import_viewer_dependencies()
        self.cv2 = cv2
        self.plt = plt
        self.np = np
        self.button_cls = button_cls
        self.session = session
        self.current_index = start_index
        self.frame_cache: OrderedDict[int, Any] = OrderedDict()
        self.capture = cv2.VideoCapture(str(session.video_path))
        if not self.capture.isOpened():
            raise RuntimeError(f"Failed to open video file: {session.video_path}")

        self.figure = plt.figure(figsize=(24, 13))
        self.figure.subplots_adjust(left=0.025, right=0.99, top=0.95, bottom=0.08, wspace=0.12)
        outer = self.figure.add_gridspec(1, 2, width_ratios=[1.65, 1.0])
        left = outer[0, 0].subgridspec(2, 1, height_ratios=[3.7, 1.65], hspace=0.18)
        channel_grid = left[0].subgridspec(4, 3, wspace=0.16, hspace=0.28)
        bottom_grid = left[1].subgridspec(1, 3, width_ratios=[0.85, 1.85, 1.0], wspace=0.32)

        self.channel_axes = [
            self.figure.add_subplot(channel_grid[row, col])
            for row in range(4)
            for col in range(3)
        ]
        self.move_mask_ax = self.figure.add_subplot(bottom_grid[0, 0])
        self.local_ax = self.figure.add_subplot(bottom_grid[0, 1])
        self.info_ax = self.figure.add_subplot(bottom_grid[0, 2])
        self.frame_ax = self.figure.add_subplot(outer[0, 1])

        self.prev_button_ax = self.figure.add_axes([0.455, 0.015, 0.04, 0.035])
        self.next_button_ax = self.figure.add_axes([0.505, 0.015, 0.04, 0.035])
        self.prev_button = self.button_cls(self.prev_button_ax, "<")
        self.next_button = self.button_cls(self.next_button_ax, ">")
        self.prev_button.on_clicked(lambda _event: self.step(-1))
        self.next_button.on_clicked(lambda _event: self.step(1))

        self.figure.canvas.mpl_connect("key_press_event", self.on_key_press)
        self.figure.canvas.mpl_connect("close_event", self.on_close)

    def on_close(self, _event: Any) -> None:
        self.capture.release()

    def on_key_press(self, event: Any) -> None:
        if event.key in {"left", "a"}:
            self.step(-1)
        elif event.key in {"right", "d"}:
            self.step(1)

    def step(self, delta: int) -> None:
        self.current_index = (self.current_index + delta) % len(self.session.snapshot_files)
        self.render()

    def read_video_frame(self, frame_index: int):
        cached = self.frame_cache.get(frame_index)
        if cached is not None:
            self.frame_cache.move_to_end(frame_index)
            return cached

        self.capture.set(self.cv2.CAP_PROP_POS_FRAMES, frame_index)
        ok, frame_bgr = self.capture.read()
        if not ok or frame_bgr is None:
            raise RuntimeError(
                f"Failed to decode frame {frame_index} from {self.session.video_path}"
            )

        if self.session.reverse_mounted:
            frame_bgr = self.cv2.rotate(frame_bgr, self.cv2.ROTATE_180)
        frame_rgb = self.cv2.cvtColor(frame_bgr, self.cv2.COLOR_BGR2RGB)
        self.frame_cache[frame_index] = frame_rgb
        while len(self.frame_cache) > 32:
            self.frame_cache.popitem(last=False)
        return frame_rgb

    def draw_channels(self, snapshot: dict[str, Any]) -> None:
        entries = get_channel_entries(snapshot)
        tracks = snapshot.get("scene", {}).get("tracks", [])

        for axis, entry in zip(self.channel_axes, entries):
            axis.clear()
            mat = self.np.asarray(entry["values"], dtype=self.np.float32)
            vmin = float(mat.min())
            vmax = float(mat.max())
            if math.isclose(vmin, vmax):
                vmax = vmin + 1e-6
            axis.imshow(mat, origin="lower", cmap="viridis", vmin=vmin, vmax=vmax, interpolation="nearest")
            for track in tracks:
                if "grid_u" in track and "grid_v" in track:
                    axis.scatter(
                        float(track["grid_u"]),
                        float(track["grid_v"]),
                        c="tomato",
                        s=22,
                        edgecolors="black",
                        linewidths=0.5,
                    )
            axis.set_title(
                f"[{entry['index']}] {entry['name']}\n"
                f"min={float(mat.min()):.3f} max={float(mat.max()):.3f}",
                fontsize=9,
            )
            axis.set_xticks([])
            axis.set_yticks([])

        for axis in self.channel_axes[len(entries):]:
            axis.clear()
            axis.axis("off")

    def draw_move_mask(self, snapshot: dict[str, Any]) -> None:
        self.move_mask_ax.clear()
        mask = snapshot["actor_input"].get("move_mask", [])
        rows, cols = infer_move_mask_shape(mask)
        mat = self.np.asarray(mask, dtype=self.np.float32).reshape(rows, cols)
        self.move_mask_ax.imshow(
            mat,
            origin="lower",
            cmap="gray_r",
            vmin=0.0,
            vmax=1.0,
            interpolation="nearest",
        )
        for row in range(rows):
            for col in range(cols):
                self.move_mask_ax.text(
                    col,
                    row,
                    f"{mat[row, col]:.0f}",
                    ha="center",
                    va="center",
                    color="tab:blue",
                    fontsize=10,
                )
        self.move_mask_ax.set_title(f"Move mask ({rows}x{cols})", fontsize=10)
        self.move_mask_ax.set_xticks([])
        self.move_mask_ax.set_yticks([])

    def draw_local_entries(self, snapshot: dict[str, Any]) -> None:
        self.local_ax.clear()
        entries = get_local_entries(snapshot)
        labels = [f"[{entry['index']:02d}] {entry['name']}" for entry in entries]
        values = [entry["value"] for entry in entries]
        positions = list(range(len(entries)))
        colors = ["tab:blue" if value >= 0.0 else "tab:red" for value in values]
        self.local_ax.barh(positions, values, color=colors, alpha=0.88)
        self.local_ax.set_yticks(positions, labels, fontsize=6)
        self.local_ax.invert_yaxis()
        self.local_ax.axvline(0.0, color="black", linewidth=0.9)
        self.local_ax.grid(axis="x", alpha=0.2)
        self.local_ax.set_xlabel("value", fontsize=9)
        self.local_ax.set_title("Local actor vector", fontsize=10)

    def draw_info(self, snapshot_path: Path, snapshot: dict[str, Any], match: FrameMatch) -> None:
        self.info_ax.clear()
        self.info_ax.axis("off")

        scene = snapshot.get("scene", {})
        actor_input = snapshot.get("actor_input", {})
        action = snapshot.get("predicted_action", {})
        lines = [
            f"snapshot: {snapshot_path.name}",
            f"index: {self.current_index}/{len(self.session.snapshot_files) - 1}",
            f"step: {snapshot.get('step', '?')}",
            f"time: {format_timestamp_ns(snapshot_timestamp_ns(snapshot))}",
            "",
            f"video: {self.session.video_path.name}",
            f"frame: {match.entry.frame_index}",
            f"frame_time: {format_timestamp_ns(match.entry.timestamp_ns)}",
            f"delta_ms: {match.delta_ns / 1_000_000.0:+.3f}",
            f"clamped: {match.clamped}",
            "",
            f"visible_count: {actor_input.get('visible_count', '?')}",
            f"tracks: {len(scene.get('tracks', []))}",
            f"hotspots: {len(scene.get('hotspots', []))}",
            f"odom_valid: {scene.get('odom_valid', '?')}",
            "",
            "drone pose:",
            f"  x={scene.get('drone_x', 0.0):.3f}",
            f"  y={scene.get('drone_y', 0.0):.3f}",
            f"  z={scene.get('drone_z', 0.0):.3f}",
            f"  yaw={scene.get('drone_yaw', 0.0):.3f}",
            "",
            "action:",
            f"  vx={action.get('vx', 0.0):.3f}",
            f"  vy={action.get('vy', 0.0):.3f}",
            f"  yaw={action.get('yaw_rate', 0.0):.3f}",
        ]
        self.info_ax.text(
            0.0,
            1.0,
            "\n".join(lines),
            va="top",
            ha="left",
            family="monospace",
            fontsize=9,
        )

    def draw_frame(self, frame_rgb, match: FrameMatch) -> None:
        self.frame_ax.clear()
        self.frame_ax.imshow(frame_rgb)
        self.frame_ax.set_title(
            f"Recorded frame {match.entry.frame_index} | {self.session.video_path.name}",
            fontsize=12,
        )
        self.frame_ax.axis("off")

    def render(self) -> None:
        snapshot_path = self.session.snapshot_files[self.current_index]
        snapshot = load_snapshot(snapshot_path)
        match = match_frame(self.session.frame_entries, snapshot_timestamp_ns(snapshot))
        frame_rgb = self.read_video_frame(match.entry.frame_index)

        self.draw_channels(snapshot)
        self.draw_move_mask(snapshot)
        self.draw_local_entries(snapshot)
        self.draw_info(snapshot_path, snapshot, match)
        self.draw_frame(frame_rgb, match)

        self.figure.suptitle(
            "Actor input snapshot viewer  |  left/right arrows or < > buttons to navigate",
            fontsize=16,
        )
        self.figure.canvas.draw_idle()

    def show(self) -> None:
        self.render()
        self.plt.show()


def main() -> None:
    args = parse_args()
    path = args.path if args.path is not None else prompt_path()
    session = resolve_recording_session(path)
    start_index = choose_snapshot_index(session.snapshot_files, args.snapshot_index)

    if args.list_only:
        print_recording_summary(session, start_index)
        return

    viewer = SnapshotVideoViewer(session, start_index)
    viewer.show()


if __name__ == "__main__":
    main()
