"""
compare_controllers.py — Compare pid vs impedance-pid force control.

Reads experiment CSVs, computes metrics, generates comparison plots.

Usage:
    python compare_controllers.py
"""

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 8,
    "axes.labelsize": 8,
    "legend.fontsize": 7,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
})

RESULT_DIR = Path("/home/andy/franka_ros2_ws/src/results/hybrid_circle_force")
FIG_DIR = Path("/home/andy/franka_ros2_ws/src/results/figures")


def classify_run(csv_path: Path) -> str:
    """Classify a run as 'pid' or 'impedance-pid' by CSV column count."""
    header = open(csv_path).readline().strip()
    ncols = len(header.split(","))
    if ncols >= 15:
        return "pid"
    elif ncols >= 10:
        return "impedance-pid"
    return "unknown"


def load_run(csv_path: Path) -> pd.DataFrame:
    """Load a single run CSV."""
    df = pd.read_csv(csv_path)
    return df


def compute_metrics(df: pd.DataFrame) -> dict:
    """Compute force and XY tracking metrics for one run."""
    # Detect contact phase
    if "phase" in df.columns:
        contact = df[df["phase"] == 1].copy()
    else:
        # Fallback: detect by fz threshold
        contact = df[df["fz_meas"].abs() > 1.0].copy()

    if len(contact) < 100:
        return None

    t = contact["time"].values
    t0 = t[0]

    # Settle time: first 0.5s after contact = transition (M1)
    m1 = contact[(t - t0) < 0.5]
    m2 = contact[(t - t0) >= 0.5]

    fz_des = contact["fz_des"].iloc[0]
    fz = contact["fz_meas"].values
    fz_err = fz_des - fz

    # XY errors
    ex = contact["ex"].values if "ex" in contact.columns else (contact["x_des"] - contact["x_meas"]).values
    ey = contact["ey"].values if "ey" in contact.columns else (contact["y_des"] - contact["y_meas"]).values
    exy = np.sqrt(ex**2 + ey**2)

    metrics = {}

    # Force metrics (full contact)
    metrics["fz_rmse"] = np.sqrt(np.mean(fz_err**2))
    metrics["fz_mae"] = np.mean(np.abs(fz_err))

    # Force metrics (M2 steady state only)
    if len(m2) > 50:
        fz_m2 = m2["fz_meas"].values
        fz_err_m2 = fz_des - fz_m2
        metrics["fz_rmse_m2"] = np.sqrt(np.mean(fz_err_m2**2))
        metrics["fz_std_m2"] = np.std(fz_m2)
    else:
        metrics["fz_rmse_m2"] = np.nan
        metrics["fz_std_m2"] = np.nan

    # Overshoot (M1)
    if len(m1) > 10:
        fz_m1 = m1["fz_meas"].values
        peak = np.max(np.abs(fz_m1))
        metrics["fz_overshoot"] = (peak - abs(fz_des)) / abs(fz_des) * 100  # percent
    else:
        metrics["fz_overshoot"] = np.nan

    # XY metrics
    metrics["xy_rmse"] = np.sqrt(np.mean(exy**2)) * 1000  # mm
    metrics["xy_max"] = np.max(exy) * 1000  # mm

    # XY M2 only
    if len(m2) > 50:
        ex_m2 = m2["ex"].values if "ex" in m2.columns else (m2["x_des"] - m2["x_meas"]).values
        ey_m2 = m2["ey"].values if "ey" in m2.columns else (m2["y_des"] - m2["y_meas"]).values
        exy_m2 = np.sqrt(ex_m2**2 + ey_m2**2)
        metrics["xy_rmse_m2"] = np.sqrt(np.mean(exy_m2**2)) * 1000
    else:
        metrics["xy_rmse_m2"] = np.nan

    return metrics


def collect_all_runs():
    """Collect metrics from all runs, grouped by controller type."""
    results = {"pid": [], "impedance-pid": []}

    for run_dir in sorted(RESULT_DIR.iterdir()):
        csv = run_dir / "run_1" / "internal.csv"
        if not csv.exists():
            continue
        ctrl = classify_run(csv)
        if ctrl not in results:
            continue
        df = load_run(csv)
        m = compute_metrics(df)
        if m is not None:
            m["run"] = run_dir.name
            results[ctrl].append(m)

    return results


def print_comparison(results: dict):
    """Print comparison table."""
    print("\n" + "=" * 70)
    print("CONTROLLER COMPARISON: pid vs impedance-pid")
    print("=" * 70)

    metrics_keys = [
        ("fz_rmse", "Fz RMSE (N)", ".4f"),
        ("fz_rmse_m2", "Fz RMSE M2 (N)", ".4f"),
        ("fz_std_m2", "Fz std M2 (N)", ".4f"),
        ("fz_overshoot", "Fz overshoot (%)", ".1f"),
        ("xy_rmse", "XY RMSE (mm)", ".3f"),
        ("xy_rmse_m2", "XY RMSE M2 (mm)", ".3f"),
        ("xy_max", "XY max err (mm)", ".3f"),
    ]

    print(f"\n{'Metric':<22} {'pid':>15} {'impedance-pid':>15}")
    print("-" * 55)
    for key, label, fmt in metrics_keys:
        vals = {}
        for ctrl in ["pid", "impedance-pid"]:
            v = [r[key] for r in results[ctrl] if not np.isnan(r[key])]
            if v:
                vals[ctrl] = f"{np.mean(v):{fmt}} +/- {np.std(v):{fmt}}"
            else:
                vals[ctrl] = "N/A"
        print(f"{label:<22} {vals.get('pid','N/A'):>15} {vals.get('impedance-pid','N/A'):>15}")

    print(f"\n{'Runs':<22} {len(results['pid']):>15} {len(results['impedance-pid']):>15}")


def plot_fz_comparison(results: dict, save_dir: Path):
    """Overlay Fz time series from one representative run of each controller."""
    fig, ax = plt.subplots(figsize=(6, 3))

    for ctrl, color, ls in [("pid", "k", "-"), ("impedance-pid", "0.4", "--")]:
        if not results[ctrl]:
            continue
        # Pick the first run
        run_name = results[ctrl][0]["run"]
        csv = RESULT_DIR / run_name / "run_1" / "internal.csv"
        df = pd.read_csv(csv)
        if "phase" in df.columns:
            contact = df[df["phase"] == 1]
        else:
            contact = df[df["fz_meas"].abs() > 1.0]
        t = contact["time"].values
        t0 = t[0]
        ax.plot((t - t0) * 1000, contact["fz_meas"].values,
                color=color, linestyle=ls, linewidth=0.8, label=ctrl)

    ax.axhline(y=5.0, color="r", linestyle=":", linewidth=0.5, label="desired")
    ax.set_xlabel("Time after contact (ms)")
    ax.set_ylabel("$f_z$ (N)")
    ax.set_title("Contact force comparison")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 3000)
    plt.tight_layout()
    fig.savefig(save_dir / "fz_comparison.pdf")
    fig.savefig(save_dir / "fz_comparison.png")
    print(f"Saved: {save_dir}/fz_comparison.pdf")
    plt.close(fig)


def plot_xy_comparison(results: dict, save_dir: Path):
    """Overlay XY trajectories from one representative run of each controller."""
    fig, ax = plt.subplots(figsize=(4, 4))

    for ctrl, color, ls in [("pid", "k", "-"), ("impedance-pid", "0.4", "--")]:
        if not results[ctrl]:
            continue
        run_name = results[ctrl][0]["run"]
        csv = RESULT_DIR / run_name / "run_1" / "internal.csv"
        df = pd.read_csv(csv)
        if "phase" in df.columns:
            contact = df[df["phase"] == 1]
        else:
            contact = df[df["fz_meas"].abs() > 1.0]

        ax.plot(contact["x_meas"].values * 1000, contact["y_meas"].values * 1000,
                color=color, linestyle=ls, linewidth=0.6, label=f"{ctrl} actual")

    # Plot desired from pid run
    if results["pid"]:
        run_name = results["pid"][0]["run"]
        csv = RESULT_DIR / run_name / "run_1" / "internal.csv"
        df = pd.read_csv(csv)
        contact = df[df["phase"] == 1]
        ax.plot(contact["x_des"].values * 1000, contact["y_des"].values * 1000,
                "b:", linewidth=0.5, label="desired")

    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("XY trajectory comparison")
    ax.set_aspect("equal")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(save_dir / "xy_comparison.pdf")
    fig.savefig(save_dir / "xy_comparison.png")
    print(f"Saved: {save_dir}/xy_comparison.pdf")
    plt.close(fig)


def plot_fz_error_comparison(results: dict, save_dir: Path):
    """Bar chart of Fz RMSE and XY RMSE for both controllers."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(6, 3))

    ctrls = ["pid", "impedance-pid"]
    colors = ["0.3", "0.6"]

    # Fz RMSE
    for i, ctrl in enumerate(ctrls):
        vals = [r["fz_rmse_m2"] for r in results[ctrl] if not np.isnan(r["fz_rmse_m2"])]
        if vals:
            mean, std = np.mean(vals), np.std(vals)
            ax1.bar(i, mean, yerr=std, color=colors[i], capsize=4, label=ctrl)
    ax1.set_xticks(range(len(ctrls)))
    ax1.set_xticklabels(ctrls)
    ax1.set_ylabel("$f_z$ RMSE (N)")
    ax1.set_title("Steady-state force error (M2)")

    # XY RMSE
    for i, ctrl in enumerate(ctrls):
        vals = [r["xy_rmse_m2"] for r in results[ctrl] if not np.isnan(r["xy_rmse_m2"])]
        if vals:
            mean, std = np.mean(vals), np.std(vals)
            ax2.bar(i, mean, yerr=std, color=colors[i], capsize=4, label=ctrl)
    ax2.set_xticks(range(len(ctrls)))
    ax2.set_xticklabels(ctrls)
    ax2.set_ylabel("XY RMSE (mm)")
    ax2.set_title("Steady-state tracking error (M2)")

    plt.tight_layout()
    fig.savefig(save_dir / "metrics_bar.pdf")
    fig.savefig(save_dir / "metrics_bar.png")
    print(f"Saved: {save_dir}/metrics_bar.pdf")
    plt.close(fig)


def main():
    FIG_DIR.mkdir(parents=True, exist_ok=True)

    results = collect_all_runs()
    print_comparison(results)

    plot_fz_comparison(results, FIG_DIR)
    plot_xy_comparison(results, FIG_DIR)
    plot_fz_error_comparison(results, FIG_DIR)

    # Save markdown table
    md_path = FIG_DIR / "comparison.md"
    with open(md_path, "w") as f:
        f.write("# Controller Comparison\n\n")
        f.write("| Metric | pid | impedance-pid |\n")
        f.write("|---|---|---|\n")
        for key, label, fmt in [
            ("fz_rmse", "Fz RMSE (N)", ".4f"),
            ("fz_rmse_m2", "Fz RMSE M2 (N)", ".4f"),
            ("fz_overshoot", "Fz overshoot (%)", ".1f"),
            ("xy_rmse", "XY RMSE (mm)", ".3f"),
            ("xy_rmse_m2", "XY RMSE M2 (mm)", ".3f"),
        ]:
            vals = {}
            for ctrl in ["pid", "impedance-pid"]:
                v = [r[key] for r in results[ctrl] if not np.isnan(r[key])]
                vals[ctrl] = f"{np.mean(v):{fmt}}" if v else "N/A"
            f.write(f"| {label} | {vals['pid']} | {vals['impedance-pid']} |\n")
        f.write(f"\n| Runs | {len(results['pid'])} | {len(results['impedance-pid'])} |\n")
    print(f"Saved: {md_path}")


if __name__ == "__main__":
    main()
