#!/usr/bin/env python3
"""Generate pseudo-ink trajectory visualization from controller internal CSV data.

Maps measured force magnitude to line width and color intensity, producing
a paper-like XY plot that approximates ink-on-paper appearance.
"""

import argparse
import glob
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.collections import LineCollection


def plot_pseudo_ink(
    internal_path: str,
    output_path: str,
    skip_seconds: float = 2.0,
    colormap: str = "Blues",
    dpi: int = 200,
    title_suffix: str = "",
):
    """Generate a pseudo-ink trajectory plot from internal.csv.

    Args:
        internal_path: Path to internal.csv with columns
            time, x_des, y_des, x_meas, y_meas, ..., fz_meas
        output_path: Where to save the PNG.
        skip_seconds: Skip initial transient (soft-start ramp).
        colormap: Matplotlib colormap for force intensity.
        dpi: Output resolution.
        title_suffix: Extra text appended to plot title.

    Returns:
        output_path on success, None on failure.
    """
    if not os.path.exists(internal_path):
        print(f"[WARN] File not found: {internal_path}")
        return None

    df = pd.read_csv(internal_path)
    required = {"time", "x_meas", "y_meas", "fz_meas", "x_des", "y_des"}
    if not required.issubset(set(df.columns)) or len(df) < 2:
        print(f"[WARN] Missing columns or too few rows in {internal_path}")
        return None

    # Skip soft-start transient.
    df = df[df["time"] >= skip_seconds].reset_index(drop=True)
    if len(df) < 2:
        print(f"[WARN] Not enough data after skipping {skip_seconds}s")
        return None

    x = df["x_meas"].to_numpy()
    y = df["y_meas"].to_numpy()
    fz = np.abs(df["fz_meas"].to_numpy())

    x_des = df["x_des"].to_numpy()
    y_des = df["y_des"].to_numpy()

    # Normalize force for styling.
    fz_min, fz_max = fz.min(), fz.max()
    fz_range = fz_max - fz_min
    if fz_range < 1e-9:
        fz_norm = np.ones_like(fz) * 0.5
    else:
        fz_norm = (fz - fz_min) / fz_range

    # Map to line widths and colors.
    widths = 0.5 + 3.0 * fz_norm[:-1]  # per-segment
    cmap = plt.get_cmap(colormap)
    # Shift range to [0.3, 1.0] so lightest ink is still visible.
    colors = cmap(0.3 + 0.7 * fz_norm[:-1])

    # Build line segments for LineCollection.
    points = np.column_stack([x, y]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    fig, ax = plt.subplots(figsize=(7, 7), facecolor="white")
    ax.set_facecolor("white")

    # Reference circle (desired trajectory).
    ax.plot(x_des, y_des, color="gray", linewidth=0.8, linestyle="--",
            alpha=0.5, label="Desired", zorder=1)

    # Pseudo-ink trajectory.
    lc = LineCollection(segments, linewidths=widths, colors=colors, zorder=2)
    ax.add_collection(lc)

    ax.set_aspect("equal")
    margin = 0.003
    ax.set_xlim(x.min() - margin, x.max() + margin)
    ax.set_ylim(y.min() - margin, y.max() + margin)
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")

    title = "Pseudo-Ink Trajectory (Force-Weighted)"
    if title_suffix:
        title += f" — {title_suffix}"
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)

    # Colorbar for force magnitude.
    sm = plt.cm.ScalarMappable(
        cmap=cmap,
        norm=plt.Normalize(vmin=fz_min, vmax=fz_max),
    )
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("|Fz| (N)")

    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi)
    plt.close(fig)
    print(f"[OK] Pseudo-ink plot: {output_path}")
    return output_path


def batch_process(results_dir: str, **kwargs):
    """Process all run_*/internal.csv in a results directory."""
    run_dirs = sorted(glob.glob(os.path.join(results_dir, "run_*")))
    if not run_dirs:
        print(f"[WARN] No run_* directories found in {results_dir}")
        return

    for run_dir in run_dirs:
        internal_path = os.path.join(run_dir, "internal.csv")
        output_path = os.path.join(run_dir, "pseudo_ink.png")
        run_name = os.path.basename(run_dir)
        plot_pseudo_ink(internal_path, output_path, title_suffix=run_name, **kwargs)


def main():
    parser = argparse.ArgumentParser(description="Generate pseudo-ink trajectory plots")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--internal-csv", help="Path to a single internal.csv")
    group.add_argument("--results-dir", help="Results directory with run_* folders")

    parser.add_argument("--output", default=None, help="Output PNG path (single file mode)")
    parser.add_argument("--skip-seconds", type=float, default=2.0, help="Skip initial transient")
    parser.add_argument("--colormap", default="Blues", help="Matplotlib colormap")
    parser.add_argument("--dpi", type=int, default=200, help="Output DPI")
    args = parser.parse_args()

    kwargs = dict(skip_seconds=args.skip_seconds, colormap=args.colormap, dpi=args.dpi)

    if args.internal_csv:
        out = args.output or args.internal_csv.replace("internal.csv", "pseudo_ink.png")
        plot_pseudo_ink(args.internal_csv, out, **kwargs)
    else:
        batch_process(args.results_dir, **kwargs)


if __name__ == "__main__":
    main()
