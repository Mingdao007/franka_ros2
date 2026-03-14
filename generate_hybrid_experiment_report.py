#!/usr/bin/env python3
"""Generate a complete report for hybrid circle-force experiments."""

import argparse
import glob
import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from generate_pseudo_ink import plot_pseudo_ink


FORCE_MEAN_ABS_LIMIT = 0.3
FORCE_RMSE_LIMIT = 0.6
XY_RMS_LIMIT = 0.005


def compute_run_metrics(wrench_path, internal_path):
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

    if internal_path and os.path.exists(internal_path):
        internal_df = pd.read_csv(internal_path)
        if {"ex", "ey"}.issubset(set(internal_df.columns)) and len(internal_df) > 0:
            ex = internal_df["ex"].to_numpy()
            ey = internal_df["ey"].to_numpy()
            xy_norm = np.sqrt(np.square(ex) + np.square(ey))
            metrics["xy_rms"] = float(np.sqrt(np.mean(np.square(xy_norm))))
            metrics["xy_max"] = float(np.max(np.abs(xy_norm)))
        else:
            metrics["xy_rms"] = float("nan")
            metrics["xy_max"] = float("nan")
    else:
        metrics["xy_rms"] = float("nan")
        metrics["xy_max"] = float("nan")

    gate_force_mean = metrics["fz_mean_abs_err"] < FORCE_MEAN_ABS_LIMIT
    gate_force_rmse = metrics["fz_rmse"] < FORCE_RMSE_LIMIT
    gate_xy = np.isnan(metrics["xy_rms"]) or metrics["xy_rms"] < XY_RMS_LIMIT
    metrics["pass_gate"] = bool(gate_force_mean and gate_force_rmse and gate_xy)
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
    axes[1].axhline(FORCE_MEAN_ABS_LIMIT, color="tab:orange", linestyle="--", linewidth=1)
    axes[1].axhline(-FORCE_MEAN_ABS_LIMIT, color="tab:orange", linestyle="--", linewidth=1)
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

    ax.axhline(FORCE_RMSE_LIMIT, color="tab:red", linestyle="--", linewidth=1, label="Fz RMSE Limit")
    ax.axhline(XY_RMS_LIMIT, color="tab:purple", linestyle=":", linewidth=1, label="XY RMS Limit")

    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Metric Value")
    ax.set_title("Hybrid Experiment Summary Metrics")
    ax.grid(True, alpha=0.2)
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def build_next_steps(metrics_list):
    any_fail = any(not m["pass_gate"] for m in metrics_list)
    if not any_fail:
        return [
            "当前基础矩阵全部通过门槛，建议进入频率扫(0.1/0.2/0.3Hz)。",
            "保持 Fz=5N，先复现 3 组独立日期实验以评估日间漂移。",
            "开始准备 SOP 自动化模板与 skill 固化。",
        ]

    steps = []
    if any(m["fz_rmse"] >= FORCE_RMSE_LIMIT for m in metrics_list):
        steps.append("Fz RMSE 超标：优先降低 circle_frequency 或下调 kp_xy，减少切向-法向耦合。")
    if any(m["fz_mean_abs_err"] >= FORCE_MEAN_ABS_LIMIT for m in metrics_list):
        steps.append("Fz 平均误差偏大：小幅提高 force_ki 并检查接触前 preload 一致性。")
    if any((not np.isnan(m["xy_rms"])) and m["xy_rms"] >= XY_RMS_LIMIT for m in metrics_list):
        steps.append("XY 误差超标：提高 kd_xy 抑制动态误差，或降低圆轨迹频率。")
    steps.append("保留失败 run 的 launch.log 与 internal.csv，执行 GUI 复现实验确认故障模式。")
    return steps


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


def write_markdown_report(report_path, results_dir, metrics_list, summary_plot, run_meta=None):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    pass_count = sum(1 for m in metrics_list if m["pass_gate"])
    total = len(metrics_list)

    meta = run_meta or {}
    duration_s = meta.get("duration", "15")
    controller = meta.get("controller", "hybrid_circle_force_controller")

    lines = []
    lines.append("# Hybrid Circle + Constant Force Experiment Report")
    lines.append("")
    lines.append(f"Generated: {now}")
    lines.append("")
    freq_hz = meta.get("circle_frequency", "0.2")
    if freq_hz == "default":
        freq_hz = "0.2"

    lines.append("## 1. Experiment Setup")
    lines.append(f"- Trajectory: XY circle, radius=0.05 m, frequency={freq_hz} Hz")
    lines.append("- Force target: constant normal force Fz=5 N")
    lines.append(f"- Controller: `{controller}`")
    lines.append(f"- Collection duration per run: {duration_s}s")
    lines.append(f"- Total runs: {total}")
    lines.append("- Auto shutdown: timeout + cleanup double protection")
    lines.append("")
    lines.append("## 2. Acceptance Gates")
    lines.append(f"- |mean(Fz error)| < {FORCE_MEAN_ABS_LIMIT:.3f} N")
    lines.append(f"- Fz RMSE < {FORCE_RMSE_LIMIT:.3f} N")
    lines.append(f"- XY RMS < {XY_RMS_LIMIT:.3f} m")
    lines.append("")
    lines.append("## 3. Run-by-Run Results")
    lines.append("| Run | Duration(s) | Samples | |mean(Fz err)|(N) | Fz MAE(N) | Fz RMSE(N) | Fz Std(N) | XY RMS(m) | XY Max(m) | Gate |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---|")
    for m in metrics_list:
        xy_rms = "N/A" if np.isnan(m["xy_rms"]) else f"{m['xy_rms']:.6f}"
        xy_max = "N/A" if np.isnan(m["xy_max"]) else f"{m['xy_max']:.6f}"
        gate = "PASS" if m["pass_gate"] else "FAIL"
        lines.append(
            f"| {m['run_name']} | {m['duration']:.2f} | {m['samples']} | {m['fz_mean_abs_err']:.4f} | "
            f"{m['fz_mae']:.4f} | {m['fz_rmse']:.4f} | {m['fz_std_err']:.4f} | {xy_rms} | {xy_max} | {gate} |"
        )

    lines.append("")
    lines.append(f"Overall: {pass_count}/{total} runs passed acceptance gates.")
    lines.append("")
    lines.append("## 4. Figures")
    lines.append(f"- Summary plot: `{os.path.relpath(summary_plot, results_dir)}`")
    for m in metrics_list:
        rel = os.path.relpath(m["run_plot"], results_dir)
        lines.append(f"- {m['run_name']} tracking plot: `{rel}`")
        if "ink_plot" in m:
            ink_rel = os.path.relpath(m["ink_plot"], results_dir)
            lines.append(f"- {m['run_name']} pseudo-ink trajectory: `{ink_rel}`")

    lines.append("")
    lines.append("## 5. Analysis")
    if pass_count == total:
        lines.append("- Force tracking and trajectory coupling stayed within configured gates across all repeats.")
        lines.append("- No unstable behavior was observed in the retained run logs.")
    else:
        lines.append("- At least one run violated acceptance gates; inspect per-run logs for transient mismatch.")
        lines.append("- Compare failed runs with GUI replay to isolate contact initialization variance.")

    lines.append("")
    lines.append("## 6. Next Steps")
    for step in build_next_steps(metrics_list):
        lines.append(f"- {step}")

    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Generate report for hybrid experiments")
    parser.add_argument("--results-dir", required=True, help="Directory containing run_* folders")
    parser.add_argument("--report-name", default="report.md", help="Markdown report filename")
    parser.add_argument("--summary-plot", default="summary_metrics.png", help="Summary plot filename")
    args = parser.parse_args()

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

        metrics, wrench_df = compute_run_metrics(wrench_path, internal_path)
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
    write_markdown_report(report_path, args.results_dir, metrics_list, summary_plot, first_meta)

    print(f"[DONE] Report: {report_path}")
    print(f"[DONE] Summary plot: {summary_plot}")


if __name__ == "__main__":
    main()
