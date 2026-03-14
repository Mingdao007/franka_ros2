#!/usr/bin/env python3
"""Generate a complete report for hybrid circle-force experiments."""

import argparse
import glob
import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

from generate_pseudo_ink import plot_pseudo_ink



# Default YAML config path (relative to this script).
DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(__file__),
    "franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml",
)


def read_controller_config(config_path=None):
    """Read actual controller parameters from YAML config."""
    path = config_path or DEFAULT_CONFIG_PATH
    defaults = {
        "circle_radius": 0.05,
        "circle_frequency": 0.2,
        "force_desired": 5.0,
    }
    if not os.path.exists(path):
        return defaults
    try:
        with open(path, "r") as f:
            cfg = yaml.safe_load(f)
        params = cfg.get("hybrid_circle_force_controller", {}).get("ros__parameters", {})
        for key in defaults:
            if key in params:
                defaults[key] = float(params[key])
    except Exception:
        pass
    return defaults


def compute_run_metrics(wrench_path, internal_path, force_desired=5.0, circle_radius=0.03):
    try:
        wrench_df = pd.read_csv(wrench_path)
    except Exception as exc:
        print(f"[WARN] Failed to parse {wrench_path}: {exc}")
        return None, None
    if wrench_df.empty or "time" not in wrench_df.columns:
        print(f"[WARN] Empty or malformed wrench CSV: {wrench_path}")
        return None, None
    metrics = {
        "samples": len(wrench_df),
        "duration": float(wrench_df["time"].max()) if len(wrench_df) > 0 else 0.0,
    }

    fz_err = wrench_df["cmd_fz"] - wrench_df["est_fz"]
    metrics["fz_mean_abs_err"] = float(abs(np.mean(fz_err)))
    metrics["fz_mae"] = float(np.mean(np.abs(fz_err)))
    metrics["fz_rmse"] = float(np.sqrt(np.mean(np.square(fz_err))))
    metrics["fz_std_err"] = float(np.std(fz_err))

    # Normalized metrics.
    metrics["fz_nrmse_target"] = metrics["fz_rmse"] / abs(force_desired) if force_desired != 0 else float("nan")
    fz_range = float(wrench_df["est_fz"].max() - wrench_df["est_fz"].min())
    metrics["fz_nrmse_range"] = metrics["fz_rmse"] / fz_range if fz_range > 1e-9 else float("nan")

    if internal_path and os.path.exists(internal_path):
        internal_df = pd.read_csv(internal_path)
        if {"ex", "ey"}.issubset(set(internal_df.columns)) and len(internal_df) > 0:
            ex = internal_df["ex"].to_numpy()
            ey = internal_df["ey"].to_numpy()
            xy_norm = np.sqrt(np.square(ex) + np.square(ey))
            metrics["xy_rms"] = float(np.sqrt(np.mean(np.square(xy_norm))))
            metrics["xy_max"] = float(np.max(np.abs(xy_norm)))
            metrics["xy_nrmse_radius"] = metrics["xy_rms"] / circle_radius if circle_radius > 1e-9 else float("nan")
        else:
            metrics["xy_rms"] = float("nan")
            metrics["xy_max"] = float("nan")
            metrics["xy_nrmse_radius"] = float("nan")
    else:
        metrics["xy_rms"] = float("nan")
        metrics["xy_max"] = float("nan")
        metrics["xy_nrmse_radius"] = float("nan")

    return metrics, wrench_df


def plot_run(wrench_df, run_name, output_path):
    t = wrench_df["time"].to_numpy()
    cmd = wrench_df["cmd_fz"].to_numpy()
    est = wrench_df["est_fz"].to_numpy()
    err = cmd - est

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(t, cmd, label="Cmd Fz", linewidth=1.8)
    axes[0].plot(t, est, label="Est Fz", linewidth=1.5)
    axes[0].set_ylabel("Force (N)")
    axes[0].set_title(f"{run_name}: Fz Tracking")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(t, err, color="tab:green", linewidth=1.2)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Error (N)")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)


def plot_summary(metrics_list, output_path):
    names = [m["run_name"] for m in metrics_list]
    fz_rmse = [m["fz_rmse"] for m in metrics_list]
    fz_mean_abs = [m["fz_mean_abs_err"] for m in metrics_list]
    xy_rms_raw = [m["xy_rms"] for m in metrics_list]
    has_xy = [not np.isnan(v) for v in xy_rms_raw]
    xy_rms = [v if ok else 0.0 for v, ok in zip(xy_rms_raw, has_xy)]

    x = np.arange(len(names))
    width = 0.25

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(x - width, fz_rmse, width, label="Fz RMSE")
    ax.bar(x, fz_mean_abs, width, label="|Fz Mean Error|")
    ax.bar(x + width, xy_rms, width, label="XY RMS")

    # Annotate N/A for runs missing XY data.
    for i, ok in enumerate(has_xy):
        if not ok:
            ax.text(x[i] + width, 0.001, "N/A", ha="center", va="bottom", fontsize=8, color="gray")


    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Metric Value")
    ax.set_title("Hybrid Experiment Summary Metrics")
    ax.grid(True, alpha=0.2)
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)




def read_run_meta(run_dir):
    """Read run_meta.txt as key=value dict."""
    meta_path = os.path.join(run_dir, "run_meta.txt")
    meta = {}
    if os.path.exists(meta_path):
        with open(meta_path, "r") as f:
            for line in f:
                line = line.strip()
                if "=" in line:
                    k, v = line.split("=", 1)
                    meta[k.strip()] = v.strip()
    return meta


def write_markdown_report(report_path, results_dir, metrics_list, summary_plot, run_meta=None, ctrl_config=None):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    total = len(metrics_list)

    meta = run_meta or {}
    cfg = ctrl_config or {}
    duration_s = meta.get("duration", "15")
    controller = meta.get("controller", "hybrid_circle_force_controller")
    radius = cfg.get("circle_radius", 0.05)
    freq_hz = cfg.get("circle_frequency", 0.2)
    force_target = cfg.get("force_desired", 5.0)

    # Allow experiment-level frequency override from run_meta.
    meta_freq = meta.get("circle_frequency", "")
    if meta_freq and meta_freq != "default":
        freq_hz = float(meta_freq)

    lines = []
    lines.append("# Hybrid Circle + Constant Force Experiment Report")
    lines.append("")
    lines.append(f"Generated: {now}")
    lines.append("")

    lines.append("## 1. Experiment Setup")
    lines.append(f"- Trajectory: XY circle, radius={radius:.3f} m, frequency={freq_hz} Hz")
    lines.append(f"- Force target: constant normal force Fz={force_target} N")
    lines.append(f"- Controller: `{controller}`")
    lines.append(f"- Collection duration per run: {duration_s}s")
    lines.append(f"- Total runs: {total}")
    lines.append("- Auto shutdown: timeout + cleanup double protection")
    lines.append("")
    lines.append("## 2. Run-by-Run Results")
    lines.append(f"Conditions: radius={radius:.3f} m, frequency={freq_hz} Hz, Fz_target={force_target} N")
    lines.append("")
    lines.append("| Run | |mean(Fz err)|(N) | Fz RMSE(N) | Fz NRMSE_target | Fz NRMSE_range | XY RMS(m) | XY NRMSE_radius |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|")
    for m in metrics_list:
        xy_rms = "N/A" if np.isnan(m["xy_rms"]) else f"{m['xy_rms']:.6f}"
        nrmse_t = "N/A" if np.isnan(m["fz_nrmse_target"]) else f"{m['fz_nrmse_target']:.1%}"
        nrmse_r = "N/A" if np.isnan(m["fz_nrmse_range"]) else f"{m['fz_nrmse_range']:.1%}"
        xy_nrmse = "N/A" if np.isnan(m["xy_nrmse_radius"]) else f"{m['xy_nrmse_radius']:.1%}"
        lines.append(
            f"| {m['run_name']} | {m['fz_mean_abs_err']:.4f} | "
            f"{m['fz_rmse']:.4f} | {nrmse_t} | {nrmse_r} | {xy_rms} | {xy_nrmse} |"
        )
    lines.append("")
    lines.append("## 3. Figures")
    lines.append(f"- Summary plot: `{os.path.relpath(summary_plot, results_dir)}`")
    for m in metrics_list:
        rel = os.path.relpath(m["run_plot"], results_dir)
        lines.append(f"- {m['run_name']} tracking plot: `{rel}`")
        if "ink_plot" in m:
            ink_rel = os.path.relpath(m["ink_plot"], results_dir)
            lines.append(f"- {m['run_name']} pseudo-ink trajectory: `{ink_rel}`")

    lines.append("")
    lines.append("## 4. Notes")
    # Compute averages for summary.
    avg_fz_rmse = np.mean([m["fz_rmse"] for m in metrics_list])
    avg_fz_nrmse = np.mean([m["fz_nrmse_target"] for m in metrics_list if not np.isnan(m["fz_nrmse_target"])])
    xy_vals = [m["xy_nrmse_radius"] for m in metrics_list if not np.isnan(m["xy_nrmse_radius"])]
    avg_xy_nrmse = np.mean(xy_vals) if xy_vals else float("nan")
    lines.append(f"- Average Fz RMSE: {avg_fz_rmse:.4f} N (NRMSE_target: {avg_fz_nrmse:.1%})")
    if not np.isnan(avg_xy_nrmse):
        lines.append(f"- Average XY NRMSE_radius: {avg_xy_nrmse:.1%}")

    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Generate report for hybrid experiments")
    parser.add_argument("--results-dir", required=True, help="Directory containing run_* folders")
    parser.add_argument("--report-name", default="report.md", help="Markdown report filename")
    parser.add_argument("--summary-plot", default="summary_metrics.png", help="Summary plot filename")
    args = parser.parse_args()

    ctrl_config = read_controller_config()

    run_dirs = sorted(glob.glob(os.path.join(args.results_dir, "run_*")))
    if not run_dirs:
        raise RuntimeError(f"No run_* folders found in {args.results_dir}")

    metrics_list = []
    for run_dir in run_dirs:
        run_name = os.path.basename(run_dir)
        wrench_path = os.path.join(run_dir, "wrench.csv")
        internal_path = os.path.join(run_dir, "internal.csv")
        if not os.path.exists(wrench_path):
            continue

        metrics, wrench_df = compute_run_metrics(
            wrench_path, internal_path,
            force_desired=ctrl_config["force_desired"],
            circle_radius=ctrl_config["circle_radius"],
        )
        if metrics is None:
            print(f"[WARN] Skipping {run_name}: could not compute metrics")
            continue
        metrics["run_name"] = run_name

        run_plot = os.path.join(run_dir, "fz_tracking.png")
        plot_run(wrench_df, run_name, run_plot)
        metrics["run_plot"] = run_plot

        # Generate pseudo-ink trajectory plot if internal.csv exists.
        ink_plot_path = os.path.join(run_dir, "pseudo_ink.png")
        if os.path.exists(internal_path):
            result = plot_pseudo_ink(internal_path, ink_plot_path, title_suffix=run_name)
            if result:
                metrics["ink_plot"] = ink_plot_path

        metrics_list.append(metrics)

    if not metrics_list:
        raise RuntimeError("No valid run metrics computed. Check wrench.csv files.")

    # Read meta from first available run for experiment-level info.
    first_meta = read_run_meta(run_dirs[0])

    summary_plot = os.path.join(args.results_dir, args.summary_plot)
    plot_summary(metrics_list, summary_plot)

    report_path = os.path.join(args.results_dir, args.report_name)
    write_markdown_report(report_path, args.results_dir, metrics_list, summary_plot, first_meta, ctrl_config)

    print(f"[DONE] Report: {report_path}")
    print(f"[DONE] Summary plot: {summary_plot}")


if __name__ == "__main__":
    main()
