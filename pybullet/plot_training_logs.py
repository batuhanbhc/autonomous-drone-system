#!/usr/bin/env python3
"""
Plot smoothed training curves from MAPPO CSV logs.

Examples:
    python plot_training_logs.py
    python plot_training_logs.py --run-dir checkpoints/run_20260506_223802
    python plot_training_logs.py --window 50 --x-axis steps
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


DEFAULT_METRICS = [
    "avg_ep_reward",
    "actor_loss",
    "critic_loss",
    "entropy",
    "explained_variance",
    "value_return_mae",
    "rtf",
    "decisions_per_real_second",
]

METRIC_LABELS = {
    "avg_ep_reward": "Average Episode Reward",
    "actor_loss": "Actor Loss",
    "critic_loss": "Critic Loss",
    "entropy": "Entropy",
    "explained_variance": "Explained Variance",
    "value_return_mae": "Value-Return MAE",
    "value_return_corr": "Value-Return Correlation",
    "value_return_rmse": "Value-Return RMSE",
    "value_mean": "Value Mean",
    "value_std": "Value Std",
    "return_mean": "Return Mean",
    "return_std": "Return Std",
    "rtf": "Real-Time Factor",
    "decisions_per_real_second": "Decisions / Real Second",
    "elapsed_seconds": "Elapsed Seconds",
    "sim_time_seconds": "Sim Time (s)",
    "steps": "Environment Steps",
    "update": "Update",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot smoothed training metrics from a run directory.")
    parser.add_argument(
        "--run-dir",
        type=Path,
        help="Path to a run directory containing train_metrics.csv.",
    )
    parser.add_argument(
        "--base-dir",
        type=Path,
        default=Path("checkpoints"),
        help="Base directory used to auto-select the latest run when --run-dir is omitted.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output PNG path. Defaults to <run-dir>/training_metrics_smoothed.png.",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=25,
        help="Trailing moving-average window used for smoothing.",
    )
    parser.add_argument(
        "--x-axis",
        choices=["update", "steps", "sim_time_seconds", "elapsed_seconds"],
        default="update",
        help="Column used for the x-axis.",
    )
    parser.add_argument(
        "--metrics",
        nargs="+",
        default=DEFAULT_METRICS,
        help="Metrics to plot. Missing metrics are skipped.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=150,
        help="Output image DPI.",
    )
    parser.add_argument(
        "--no-raw",
        action="store_true",
        help="Hide the unsmoothed series and plot only the smoothed line.",
    )
    return parser.parse_args()


def find_latest_run_dir(base_dir: Path) -> Path:
    candidates = sorted(base_dir.rglob("train_metrics.csv"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"No train_metrics.csv files found under {base_dir}")
    return candidates[-1].parent


def load_csv_rows(path: Path) -> list[dict[str, float]]:
    with path.open("r", newline="", encoding="ascii") as handle:
        reader = csv.DictReader(handle)
        rows: list[dict[str, float]] = []
        for row in reader:
            parsed: dict[str, float] = {}
            for key, value in row.items():
                if value is None or value == "":
                    parsed[key] = float("nan")
                else:
                    parsed[key] = float(value)
            rows.append(parsed)
    if not rows:
        raise ValueError(f"CSV file is empty: {path}")
    return rows


def merge_logs(metrics_path: Path, critic_path: Path | None) -> dict[str, list[float]]:
    metrics_rows = load_csv_rows(metrics_path)
    merged_rows = {int(row["update"]): dict(row) for row in metrics_rows}

    if critic_path is not None and critic_path.exists():
        for row in load_csv_rows(critic_path):
            update = int(row["update"])
            merged_rows.setdefault(update, {}).update(row)

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


def plot_metrics(
    data: dict[str, list[float]],
    run_dir: Path,
    output_path: Path,
    metrics: list[str],
    x_axis: str,
    window: int,
    show_raw: bool,
    dpi: int,
) -> list[str]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "matplotlib is required to plot training logs. "
            "Install it in your Python environment and rerun this script."
        ) from exc

    if x_axis not in data:
        raise KeyError(f"Requested x-axis column '{x_axis}' is not available in the logs.")

    available_metrics = [metric for metric in metrics if metric in data]
    if not available_metrics:
        raise KeyError("None of the requested metrics are present in the logs.")

    x_values = data[x_axis]
    columns = 2
    rows = math.ceil(len(available_metrics) / columns)
    fig, axes = plt.subplots(rows, columns, figsize=(14, 3.8 * rows), squeeze=False)

    for axis, metric in zip(axes.flat, available_metrics):
        y_values = data[metric]
        y_smoothed = moving_average(y_values, window)

        if show_raw:
            axis.plot(x_values, y_values, color="tab:blue", alpha=0.22, linewidth=1.0, label="raw")
        axis.plot(x_values, y_smoothed, color="tab:blue", linewidth=2.2, label=f"moving avg ({window})")
        axis.set_title(METRIC_LABELS.get(metric, metric.replace("_", " ").title()))
        axis.set_xlabel(METRIC_LABELS.get(x_axis, x_axis))
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")

    for axis in axes.flat[len(available_metrics):]:
        axis.axis("off")

    fig.suptitle(f"{run_dir.name} training metrics", fontsize=14)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=dpi)
    plt.close(fig)
    return available_metrics


def main() -> None:
    args = parse_args()

    run_dir = args.run_dir or find_latest_run_dir(args.base_dir)
    metrics_path = run_dir / "train_metrics.csv"
    critic_path = run_dir / "critic_diagnostics.csv"
    output_path = args.output or (run_dir / "training_metrics_smoothed.png")

    if not metrics_path.exists():
        raise FileNotFoundError(f"Missing metrics CSV: {metrics_path}")

    data = merge_logs(metrics_path, critic_path if critic_path.exists() else None)
    used_metrics = plot_metrics(
        data=data,
        run_dir=run_dir,
        output_path=output_path,
        metrics=args.metrics,
        x_axis=args.x_axis,
        window=max(1, args.window),
        show_raw=not args.no_raw,
        dpi=args.dpi,
    )

    print(f"Run directory: {run_dir}")
    print(f"Saved plot to: {output_path}")
    print(f"Metrics plotted: {', '.join(used_metrics)}")


if __name__ == "__main__":
    main()
