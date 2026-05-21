#!/usr/bin/env python3
"""
Interactively inspect ROS2 actor-input snapshot JSON files.

Examples:
    python visualize_actor_input_snapshot.py /path/to/actor_input_snapshots
    python visualize_actor_input_snapshot.py /path/to/snapshot_000010_step_00000040.json
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Load actor-input snapshot JSON files, print the saved channel/local index layout, "
            "and interactively plot selected tensors."
        )
    )
    parser.add_argument(
        "path",
        nargs="?",
        type=Path,
        help=(
            "Snapshot JSON file, actor_input_snapshots directory, or a session directory "
            "containing actor_input_snapshots."
        ),
    )
    parser.add_argument(
        "--snapshot-index",
        type=int,
        help="Snapshot index inside the resolved directory listing. Defaults to prompting for it.",
    )
    parser.add_argument(
        "--list-only",
        action="store_true",
        help="Print the saved index layout and exit without opening plots.",
    )
    return parser.parse_args()


def prompt_path() -> Path:
    while True:
        raw = input("Snapshot path (.json, actor_input_snapshots dir, or session dir): ").strip()
        if raw:
            return Path(raw).expanduser()
        print("Enter a valid path.")


def resolve_snapshot_files(path: Path) -> list[Path]:
    resolved = path.expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"Path does not exist: {resolved}")
    if resolved.is_file():
        if resolved.suffix.lower() != ".json":
            raise ValueError(f"Expected a JSON snapshot file: {resolved}")
        return [resolved]

    snapshot_dir = resolved
    nested_snapshot_dir = resolved / "actor_input_snapshots"
    if nested_snapshot_dir.is_dir():
        snapshot_dir = nested_snapshot_dir

    files = sorted(snapshot_dir.glob("snapshot_*.json"))
    if not files:
        raise FileNotFoundError(f"No snapshot_*.json files found under {snapshot_dir}")
    return files


def load_snapshot(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def choose_snapshot_index(files: list[Path], requested_index: int | None) -> int:
    if len(files) == 1:
        return 0
    if requested_index is not None:
        if requested_index < 0 or requested_index >= len(files):
            raise IndexError(
                f"snapshot index {requested_index} is out of range for {len(files)} files"
            )
        return requested_index

    print("\nAvailable snapshots:")
    for idx, path in enumerate(files):
        try:
            snapshot = load_snapshot(path)
            step = snapshot.get("step", "?")
            timestamp = snapshot.get("timestamp", {})
            sec = timestamp.get("sec", "?")
            nanosec = timestamp.get("nanosec", "?")
            print(f"  [{idx:03d}] step={step:<8} time={sec}.{nanosec:09d}  {path.name}")
        except Exception:
            print(f"  [{idx:03d}] {path.name}")

    default_index = len(files) - 1
    while True:
        raw = input(f"Choose snapshot index [{default_index}]: ").strip()
        if not raw:
            return default_index
        try:
            idx = int(raw)
        except ValueError:
            print("Enter an integer snapshot index.")
            continue
        if 0 <= idx < len(files):
            return idx
        print(f"Index must be between 0 and {len(files) - 1}.")


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


def parse_index_selection(raw: str, max_len: int) -> list[int]:
    text = raw.strip().lower()
    if text in {"all", "*"}:
        return list(range(max_len))
    indices: set[int] = set()
    for part in raw.split(","):
        token = part.strip()
        if not token:
            continue
        if "-" in token:
            start_text, end_text = token.split("-", 1)
            start = int(start_text)
            end = int(end_text)
            if start > end:
                start, end = end, start
            indices.update(range(start, end + 1))
        else:
            indices.add(int(token))
    ordered = sorted(indices)
    for idx in ordered:
        if idx < 0 or idx >= max_len:
            raise IndexError(f"Index {idx} is out of range [0, {max_len - 1}]")
    return ordered


def print_snapshot_summary(path: Path, snapshot: dict[str, Any]) -> None:
    scene = snapshot.get("scene", {})
    actor_input = snapshot.get("actor_input", {})
    timestamp = snapshot.get("timestamp", {})
    sec = timestamp.get("sec", "?")
    nanosec = timestamp.get("nanosec", "?")
    print("\nSnapshot summary")
    print(f"  file:        {path}")
    print(f"  step:        {snapshot.get('step', '?')}")
    print(f"  timestamp:   {sec}.{int(nanosec):09d}" if isinstance(nanosec, int) else f"  timestamp:   {sec}.{nanosec}")
    print(f"  grid_shape:  {snapshot.get('grid_shape', '?')}")
    print(f"  local_dim:   {snapshot.get('local_base_dim', len(actor_input.get('local_base', [])))}")
    print(f"  move_mask:   {snapshot.get('move_mask_dim', len(actor_input.get('move_mask', [])))}")
    print(f"  tracks:      {len(scene.get('tracks', []))}")
    print(f"  hotspots:    {len(scene.get('hotspots', []))}")
    if "predicted_action" in snapshot:
        action = snapshot["predicted_action"]
        print(
            "  action:      "
            f"vx={action.get('vx', 0.0): .3f} "
            f"vy={action.get('vy', 0.0): .3f} "
            f"yaw={action.get('yaw_rate', 0.0): .3f}"
        )


def print_index_layout(snapshot: dict[str, Any]) -> None:
    channel_entries = get_channel_entries(snapshot)
    local_entries = get_local_entries(snapshot)
    print("\nCNN channel indices")
    for entry in channel_entries:
        print(f"  [{entry['index']:02d}] {entry['name']}")
    print("\nLocal vector indices")
    for entry in local_entries:
        print(f"  [{entry['index']:02d}] {entry['name']}")


def import_plotting():
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "matplotlib and numpy are required for snapshot visualization. "
            "Install them in your Python environment and rerun this script."
        ) from exc
    return plt, np


def plot_channels(snapshot: dict[str, Any], indices: list[int]) -> None:
    plt, np = import_plotting()
    entries = get_channel_entries(snapshot)
    selected = [entries[idx] for idx in indices]
    num_cols = min(4, max(1, len(selected)))
    num_rows = math.ceil(len(selected) / num_cols)
    fig, axes = plt.subplots(
        num_rows,
        num_cols,
        figsize=(4.4 * num_cols, 4.0 * num_rows),
        squeeze=False,
    )
    tracks = snapshot.get("scene", {}).get("tracks", [])

    for ax, entry in zip(axes.flat, selected):
        mat = np.asarray(entry["values"], dtype=np.float32)
        im = ax.imshow(mat, origin="lower", cmap="viridis", interpolation="nearest")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        for track in tracks:
            if "grid_u" in track and "grid_v" in track:
                ax.scatter(
                    float(track["grid_u"]),
                    float(track["grid_v"]),
                    c="tomato",
                    s=28,
                    edgecolors="black",
                    linewidths=0.6,
                )
        ax.set_title(
            f"[{entry['index']}] {entry['name']}\n"
            f"min={float(mat.min()):.3f} max={float(mat.max()):.3f} mean={float(mat.mean()):.3f}"
        )
        ax.set_xlabel("grid u")
        ax.set_ylabel("grid v")

    for ax in axes.flat[len(selected):]:
        ax.axis("off")

    fig.suptitle(f"Actor CNN Channels | step={snapshot.get('step', '?')}")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    plt.show()


def plot_local_entries(snapshot: dict[str, Any], indices: list[int]) -> None:
    plt, _ = import_plotting()
    entries = get_local_entries(snapshot)
    selected = [entries[idx] for idx in indices]
    labels = [f"[{entry['index']:02d}] {entry['name']}" for entry in selected]
    values = [entry["value"] for entry in selected]
    height = max(5.0, 0.38 * len(selected) + 1.6)
    fig, ax = plt.subplots(figsize=(12.0, height))
    positions = list(range(len(selected)))
    colors = ["tab:blue" if value >= 0.0 else "tab:red" for value in values]
    ax.barh(positions, values, color=colors, alpha=0.88)
    ax.set_yticks(positions, labels)
    ax.invert_yaxis()
    ax.axvline(0.0, color="black", linewidth=1.0)
    ax.set_xlabel("value")
    ax.set_title(f"Actor local_base entries | step={snapshot.get('step', '?')}")
    ax.grid(axis="x", alpha=0.25)
    fig.tight_layout()
    plt.show()


def infer_move_mask_shape(mask: list[float]) -> tuple[int, int]:
    if not mask:
        return 1, 1
    side = int(round(math.sqrt(len(mask))))
    if side * side == len(mask):
        return side, side
    return 1, len(mask)


def plot_move_mask(snapshot: dict[str, Any]) -> None:
    plt, np = import_plotting()
    mask = snapshot["actor_input"]["move_mask"]
    rows, cols = infer_move_mask_shape(mask)
    mat = np.asarray(mask, dtype=np.float32).reshape(rows, cols)
    fig, ax = plt.subplots(figsize=(4.5 + cols * 0.45, 3.5 + rows * 0.45))
    im = ax.imshow(mat, origin="lower", cmap="gray_r", vmin=0.0, vmax=1.0, interpolation="nearest")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    for row in range(rows):
        for col in range(cols):
            ax.text(col, row, f"{mat[row, col]:.0f}", ha="center", va="center", color="tab:blue")
    ax.set_title(f"Move mask | inferred shape={rows}x{cols} | step={snapshot.get('step', '?')}")
    ax.set_xlabel("col")
    ax.set_ylabel("row")
    fig.tight_layout()
    plt.show()


def prompt_indices(kind: str, max_len: int) -> list[int]:
    while True:
        raw = input(
            f"Enter {kind} indices (example: 0,3-5 or all): "
        ).strip()
        if not raw:
            print("Enter at least one index or 'all'.")
            continue
        try:
            return parse_index_selection(raw, max_len)
        except Exception as exc:
            print(f"Invalid selection: {exc}")


def interactive_loop(files: list[Path], start_index: int, list_only: bool) -> None:
    current_index = start_index
    while True:
        current_path = files[current_index]
        snapshot = load_snapshot(current_path)
        print_snapshot_summary(current_path, snapshot)
        print_index_layout(snapshot)
        if list_only:
            return

        channel_count = len(get_channel_entries(snapshot))
        local_count = len(get_local_entries(snapshot))

        while True:
            print(
                "\nActions: [a] all channels  [c] selected channels  [l] local entries  "
                "[m] move mask  [p] print layout  [n] next snapshot  [s] choose snapshot  [q] quit"
            )
            action = input("Choose action: ").strip().lower()
            if action == "a":
                plot_channels(snapshot, list(range(channel_count)))
            elif action == "c":
                plot_channels(snapshot, prompt_indices("channel", channel_count))
            elif action == "l":
                plot_local_entries(snapshot, prompt_indices("local", local_count))
            elif action == "m":
                plot_move_mask(snapshot)
            elif action == "p":
                print_snapshot_summary(current_path, snapshot)
                print_index_layout(snapshot)
            elif action == "n":
                current_index = (current_index + 1) % len(files)
                break
            elif action == "s":
                current_index = choose_snapshot_index(files, requested_index=None)
                break
            elif action == "q":
                return
            else:
                print("Unknown action.")


def main() -> None:
    args = parse_args()
    path = args.path if args.path is not None else prompt_path()
    files = resolve_snapshot_files(path)
    start_index = choose_snapshot_index(files, args.snapshot_index)
    interactive_loop(files, start_index, args.list_only)


if __name__ == "__main__":
    main()
