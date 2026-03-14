#!/usr/bin/env python3
"""Generate cross-frequency comparison report for frequency sweep experiments."""

import argparse
import glob
import os
import re
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Must match generate_hybrid_experiment_report.py gates.
FORCE_MEAN_ABS_LIMIT = 0.3
FORCE_RMSE_LIMIT = 0.6
XY_RMS_LIMIT = 0.005


def compute_run_metrics(wrench_path, internal_path):
    """Compute metrics for a single run (same logic as report generator)."""
    try:
        wrench_df = pd.read_csv(wrench_path)
    except Exception as exc:
        print(f"[WARN] Failed to parse {wrench_path}: {exc}")
        return None

    if wrench_df.empty or "time" not in wrench_df.columns:
        return None

    fz_err = wrench_df["cmd_fz"] - wrench_df["est_fz"]
    metrics = {
        "fz_mean_abs_err": float(abs(np.mean(fz_err))),
        "fz_rmse": float(np.sqrt(np.mean(np.square(fz_err)))),
        "fz_std_err": float(np.std(fz_err)),
    }

    if internal_path and os.path.exists(internal_path):
        internal_df = pd.read_csv(internal_path)
        if {"ex", "ey"}.issubset(set(internal_df.columns)) and len(internal_df) > 0:
            xy_norm = np.sqrt(
                np.square(internal_df["ex"].to_numpy())
                + np.square(internal_df["ey"].to_numpy())
            )
            metrics["xy_rms"] = float(np.sqrt(np.mean(np.square(xy_norm))))
        else:
            metrics["xy_rms"] = float("nan")
    else:
        metrics["xy_rms"] = float("nan")

    gate_force_mean = metrics["fz_mean_abs_err"] < FORCE_MEAN_ABS_LIMIT
    gate_force_rmse = metrics["fz_rmse"] < FORCE_RMSE_LIMIT
    gate_xy = np.isnan(metrics["xy_rms"]) or metrics["xy_rms"] < XY_RMS_LIMIT
    metrics["pass_gate"] = bool(gate_force_mean and gate_force_rmse and gate_xy)
    return metrics


def collect_frequency_data(sweep_dir):
    """Collect metrics across all freq_* subdirectories."""
    freq_dirs = sorted(glob.glob(os.path.join(sweep_dir, "freq_*")))
    if not freq_dirs:
        raise RuntimeError(f"No freq_* directories found in {sweep_dir}")

    freq_data = []
    for freq_dir in freq_dirs:
        # Extract frequency from directory name.
        match = re.search(r"freq_([\d.]+)", os.path.basename(freq_dir))
        if not match:
            continue
        freq_hz = float(match.group(1))

        run_dirs = sorted(glob.glob(os.path.join(freq_dir, "run_*")))
        run_metrics_list = []
        for run_dir in run_dirs:
            wrench_path = os.path.join(run_dir, "wrench.csv")
            internal_path = os.path.join(run_dir, "internal.csv")
            if not os.path.exists(wrench_path):
                continue
            m = compute_run_metrics(wrench_path, internal_path)
            if m is not None:
                run_metrics_list.append(m)

        if not run_metrics_list:
            continue

        # Aggregate metrics across runs.
        agg = {
            "freq_hz": freq_hz,
            "n_runs": len(run_metrics_list),
            "pass_rate": sum(1 for m in run_metrics_list if m["pass_gate"]) / len(run_metrics_list),
        }
        for key in ["fz_mean_abs_err", "fz_rmse", "fz_std_err", "xy_rms"]:
            vals = [m[key] for m in run_metrics_list if not np.isnan(m[key])]
            if vals:
                agg[f"{key}_mean"] = float(np.mean(vals))
                agg[f"{key}_std"] = float(np.std(vals))
            else:
                agg[f"{key}_mean"] = float("nan")
                agg[f"{key}_std"] = float("nan")

        agg["freq_dir"] = freq_dir
        freq_data.append(agg)

    freq_data.sort(key=lambda d: d["freq_hz"])
    return freq_data


def plot_sweep_comparison(freq_data, output_path):
    """Generate comparison plot: metrics vs. frequency with error bars."""
    freqs = [d["freq_hz"] for d in freq_data]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # Subplot 1: Fz RMSE.
    means = [d["fz_rmse_mean"] for d in freq_data]
    stds = [d["fz_rmse_std"] for d in freq_data]
    axes[0].errorbar(freqs, means, yerr=stds, fmt="o-", capsize=5, color="tab:blue")
    axes[0].axhline(FORCE_RMSE_LIMIT, color="tab:red", linestyle="--", label=f"Gate ({FORCE_RMSE_LIMIT} N)")
    axes[0].set_xlabel("Circle Frequency (Hz)")
    axes[0].set_ylabel("Fz RMSE (N)")
    axes[0].set_title("Fz RMSE vs. Frequency")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # Subplot 2: |mean(Fz error)|.
    means = [d["fz_mean_abs_err_mean"] for d in freq_data]
    stds = [d["fz_mean_abs_err_std"] for d in freq_data]
    axes[1].errorbar(freqs, means, yerr=stds, fmt="o-", capsize=5, color="tab:orange")
    axes[1].axhline(FORCE_MEAN_ABS_LIMIT, color="tab:red", linestyle="--", label=f"Gate ({FORCE_MEAN_ABS_LIMIT} N)")
    axes[1].set_xlabel("Circle Frequency (Hz)")
    axes[1].set_ylabel("|mean(Fz error)| (N)")
    axes[1].set_title("|Mean Fz Error| vs. Frequency")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # Subplot 3: XY RMS.
    means = [d["xy_rms_mean"] for d in freq_data]
    stds = [d["xy_rms_std"] for d in freq_data]
    valid = [not np.isnan(m) for m in means]
    if any(valid):
        v_freqs = [f for f, v in zip(freqs, valid) if v]
        v_means = [m for m, v in zip(means, valid) if v]
        v_stds = [s for s, v in zip(stds, valid) if v]
        axes[2].errorbar(v_freqs, v_means, yerr=v_stds, fmt="o-", capsize=5, color="tab:green")
    axes[2].axhline(XY_RMS_LIMIT, color="tab:red", linestyle="--", label=f"Gate ({XY_RMS_LIMIT} m)")
    axes[2].set_xlabel("Circle Frequency (Hz)")
    axes[2].set_ylabel("XY RMS (m)")
    axes[2].set_title("XY RMS vs. Frequency")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"[OK] Sweep comparison plot: {output_path}")


def write_sweep_report(sweep_dir, freq_data, comparison_plot):
    """Generate sweep_report.md."""
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    report_path = os.path.join(sweep_dir, "sweep_report.md")

    lines = []
    lines.append("# Frequency Sweep Comparison Report")
    lines.append("")
    lines.append(f"Generated: {now}")
    lines.append("")
    lines.append("## 1. Acceptance Gates")
    lines.append(f"- |mean(Fz error)| < {FORCE_MEAN_ABS_LIMIT:.3f} N")
    lines.append(f"- Fz RMSE < {FORCE_RMSE_LIMIT:.3f} N")
    lines.append(f"- XY RMS < {XY_RMS_LIMIT:.3f} m")
    lines.append("")
    lines.append("## 2. Cross-Frequency Summary")
    lines.append(
        "| Freq (Hz) | Runs | Fz RMSE (N) | |Fz Mean Err| (N) | XY RMS (m) | Pass Rate |"
    )
    lines.append("|---:|---:|---:|---:|---:|---:|")

    for d in freq_data:
        def fmt(key):
            m = d.get(f"{key}_mean", float("nan"))
            s = d.get(f"{key}_std", float("nan"))
            if np.isnan(m):
                return "N/A"
            return f"{m:.4f} +/- {s:.4f}"

        lines.append(
            f"| {d['freq_hz']:.1f} | {d['n_runs']} | {fmt('fz_rmse')} | "
            f"{fmt('fz_mean_abs_err')} | {fmt('xy_rms')} | {d['pass_rate']:.0%} |"
        )

    lines.append("")
    lines.append("## 3. Comparison Plot")
    rel_plot = os.path.relpath(comparison_plot, sweep_dir)
    lines.append(f"![Sweep Comparison]({rel_plot})")
    lines.append("")

    # Per-frequency report links.
    lines.append("## 4. Per-Frequency Reports")
    for d in freq_data:
        freq_report = os.path.join(d["freq_dir"], "report.md")
        if os.path.exists(freq_report):
            rel = os.path.relpath(freq_report, sweep_dir)
            lines.append(f"- {d['freq_hz']:.1f} Hz: `{rel}`")

    lines.append("")
    lines.append("## 5. Trend Analysis")

    # Check if performance degrades at higher frequencies.
    if len(freq_data) >= 2:
        rmse_vals = [d["fz_rmse_mean"] for d in freq_data if not np.isnan(d["fz_rmse_mean"])]
        if len(rmse_vals) >= 2:
            if rmse_vals[-1] > rmse_vals[0] * 1.2:
                lines.append("- Fz RMSE increases with frequency — force tracking degrades at higher speeds.")
            else:
                lines.append("- Fz RMSE is stable across tested frequencies.")

        xy_vals = [d["xy_rms_mean"] for d in freq_data if not np.isnan(d["xy_rms_mean"])]
        if len(xy_vals) >= 2:
            if xy_vals[-1] > xy_vals[0] * 1.2:
                lines.append("- XY RMS increases with frequency — trajectory tracking degrades at higher speeds.")
            else:
                lines.append("- XY RMS is stable across tested frequencies.")

    # Overall verdict.
    all_pass = all(d["pass_rate"] == 1.0 for d in freq_data)
    passing_freqs = [f"{d['freq_hz']:.1f}" for d in freq_data if d["pass_rate"] == 1.0]
    failing_freqs = [f"{d['freq_hz']:.1f}" for d in freq_data if d["pass_rate"] < 1.0]

    lines.append("")
    lines.append("## 6. Verdict")
    if all_pass:
        lines.append("All tested frequencies pass acceptance gates.")
    else:
        lines.append(f"- Passing: {', '.join(passing_freqs) if passing_freqs else 'none'} Hz")
        lines.append(f"- Failing: {', '.join(failing_freqs)} Hz")

    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[OK] Sweep report: {report_path}")


def main():
    parser = argparse.ArgumentParser(description="Generate cross-frequency sweep comparison")
    parser.add_argument("--sweep-dir", required=True, help="Sweep results directory with freq_* folders")
    args = parser.parse_args()

    freq_data = collect_frequency_data(args.sweep_dir)

    comparison_plot = os.path.join(args.sweep_dir, "sweep_comparison.png")
    plot_sweep_comparison(freq_data, comparison_plot)
    write_sweep_report(args.sweep_dir, freq_data, comparison_plot)


if __name__ == "__main__":
    main()
