#!/usr/bin/env python3
"""
Generate a paper-oriented training-results figure from MAPPO CSV logs.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "training_plots"

PANEL_ORDER = [
    "avg_ep_reward",
    "explained_variance",
    "entropy",
    "clip_frac",
]

PANEL_LABELS = {
    "avg_ep_reward": "Average Episode Reward",
    "explained_variance": "Explained Variance",
    "entropy": "Policy Entropy",
    "clip_frac": "PPO Clip Fraction",
    "update": "Update",
}

PANEL_COLORS = {
    "avg_ep_reward": "#b91c1c",
    "explained_variance": "#1d4ed8",
    "entropy": "#0f766e",
    "clip_frac": "#7c3aed",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=None,
        help="Run directory containing train_metrics.csv and critic_diagnostics.csv.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PNG path. Defaults to results_ws/training_plots/<run>_training_results.png.",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=25,
        help="Trailing moving-average window.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=220,
    )
    parser.add_argument(
        "--no-raw",
        action="store_true",
        help="Hide the faint raw traces.",
    )
    return parser.parse_args()


def resolve_run_dir(raw_run_dir: Path | None) -> Path:
    if raw_run_dir is not None:
        run_dir = raw_run_dir.expanduser().resolve()
        if not run_dir.is_dir():
            raise FileNotFoundError(f"Run directory not found: {run_dir}")
        return run_dir

    candidates = sorted(
        (PROJECT_ROOT / "checkpoints").rglob("train_metrics.csv"),
        key=lambda path: path.stat().st_mtime,
    )
    if not candidates:
        raise FileNotFoundError(
            f"No train_metrics.csv files found under {PROJECT_ROOT / 'checkpoints'}."
        )
    return candidates[-1].parent.resolve()


def default_output_path(run_dir: Path) -> Path:
    return DEFAULT_OUTPUT_DIR / f"{run_dir.name}_training_results.png"


def load_csv_rows(path: Path) -> list[dict[str, float]]:
    with path.open("r", newline="", encoding="ascii") as handle:
        reader = csv.DictReader(handle)
        rows: list[dict[str, float]] = []
        for row in reader:
            parsed: dict[str, float] = {}
            for key, value in row.items():
                parsed[key] = float(value) if value not in {None, ""} else float("nan")
            rows.append(parsed)
    if not rows:
        raise ValueError(f"CSV file is empty: {path}")
    return rows


def merge_logs(run_dir: Path) -> dict[str, list[float]]:
    metrics_path = run_dir / "train_metrics.csv"
    critic_path = run_dir / "critic_diagnostics.csv"
    if not metrics_path.exists():
        raise FileNotFoundError(f"Missing metrics CSV: {metrics_path}")

    merged_rows = {int(row["update"]): dict(row) for row in load_csv_rows(metrics_path)}
    if critic_path.exists():
        for row in load_csv_rows(critic_path):
            merged_rows.setdefault(int(row["update"]), {}).update(row)

    ordered_updates = sorted(merged_rows)
    keys: set[str] = set()
    for update in ordered_updates:
        keys.update(merged_rows[update].keys())

    data: dict[str, list[float]] = {}
    for key in sorted(keys):
        data[key] = [merged_rows[update].get(key, float("nan")) for update in ordered_updates]
    return data


def moving_average(values: list[float], window: int) -> list[float]:
    if window <= 1 or len(values) < 2:
        return list(values)

    window = min(window, len(values))
    sanitized = [value if math.isfinite(value) else 0.0 for value in values]
    counts_src = [1.0 if math.isfinite(value) else 0.0 for value in values]

    cumulative: list[float] = []
    counts: list[float] = []
    running_total = 0.0
    running_count = 0.0
    for value, count in zip(sanitized, counts_src):
        running_total += value
        running_count += count
        cumulative.append(running_total)
        counts.append(running_count)

    smoothed: list[float] = [float("nan")] * len(values)
    for idx in range(len(values)):
        start = max(0, idx - window + 1)
        total = cumulative[idx] - (cumulative[start - 1] if start > 0 else 0.0)
        count = counts[idx] - (counts[start - 1] if start > 0 else 0.0)
        smoothed[idx] = total / count if count > 0 else float("nan")
    return smoothed


def last_finite(values: list[float]) -> tuple[int, float] | None:
    for idx in range(len(values) - 1, -1, -1):
        value = values[idx]
        if math.isfinite(value):
            return idx, value
    return None


def format_value(metric: str, value: float) -> str:
    if metric == "avg_ep_reward":
        return f"{value:.0f}"
    if metric == "explained_variance":
        return f"{value:.2f}"
    if metric == "entropy":
        return f"{value:.2f}"
    if metric == "clip_frac":
        return f"{value:.3f}"
    return f"{value:.2f}"


def plot_results(
    data: dict[str, list[float]],
    run_dir: Path,
    output_path: Path,
    window: int,
    show_raw: bool,
    dpi: int,
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "matplotlib is required to plot training results. "
            "Install it in your Python environment and rerun this script."
        ) from exc

    x_values = data.get("update")
    if x_values is None:
        raise KeyError("Missing 'update' column in merged logs.")

    plt.rcParams.update(
        {
            "font.family": "DejaVu Serif",
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.facecolor": "#fcfcfb",
            "figure.facecolor": "white",
        }
    )

    fig, axes = plt.subplots(2, 2, figsize=(11.8, 7.6), squeeze=False)
    axes_flat = list(axes.flat)

    for axis, metric in zip(axes_flat, PANEL_ORDER):
        if metric not in data:
            raise KeyError(f"Missing metric in logs: {metric}")

        color = PANEL_COLORS[metric]
        raw_values = data[metric]
        smooth_values = moving_average(raw_values, window)
        finite_smooth = [v for v in smooth_values if math.isfinite(v)]
        baseline_value = (
            0.0 if metric in {"explained_variance", "clip_frac"} else min(finite_smooth)
        )

        if show_raw:
            axis.plot(
                x_values,
                raw_values,
                color="#cbd5e1",
                linewidth=1.0,
                alpha=0.55,
                zorder=1,
            )

        axis.plot(
            x_values,
            smooth_values,
            color=color,
            linewidth=2.6,
            zorder=3,
        )
        axis.fill_between(
            x_values,
            smooth_values,
            [baseline_value] * len(smooth_values),
            color=color,
            alpha=0.08,
            zorder=2,
        )

        axis.set_title(PANEL_LABELS[metric], fontsize=13, pad=10, fontweight="bold")
        axis.set_xlabel(PANEL_LABELS["update"], fontsize=10)
        axis.grid(True, alpha=0.18, linewidth=0.8)
        axis.tick_params(labelsize=9)

        if metric == "explained_variance":
            axis.set_ylim(-0.4, 1.02)
            axis.axhline(0.0, color="#9ca3af", linewidth=1.0, alpha=0.7, zorder=0)
        if metric == "clip_frac":
            top = max(0.12, max(v for v in raw_values if math.isfinite(v)) * 1.12)
            axis.set_ylim(0.0, top)
        if metric == "entropy":
            vals = [v for v in raw_values if math.isfinite(v)]
            axis.set_ylim(max(0.0, min(vals) - 0.12), max(vals) + 0.08)

        final = last_finite(smooth_values)
        if final is not None:
            idx, value = final
            axis.scatter([x_values[idx]], [value], s=26, color=color, zorder=4)
            axis.annotate(
                format_value(metric, value),
                xy=(x_values[idx], value),
                xytext=(8, 0),
                textcoords="offset points",
                ha="left",
                va="center",
                fontsize=9.5,
                color=color,
                fontweight="bold",
            )

    fig.subplots_adjust(left=0.07, right=0.985, bottom=0.085, top=0.96, wspace=0.22, hspace=0.28)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=dpi)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    run_dir = resolve_run_dir(args.run_dir)
    output_path = (
        args.output.expanduser().resolve()
        if args.output is not None
        else default_output_path(run_dir).resolve()
    )
    data = merge_logs(run_dir)
    plot_results(
        data=data,
        run_dir=run_dir,
        output_path=output_path,
        window=max(1, int(args.window)),
        show_raw=not args.no_raw,
        dpi=int(args.dpi),
    )
    print(f"[plot_training_results] run_dir={run_dir}")
    print(f"[plot_training_results] output={output_path}")


if __name__ == "__main__":
    main()
