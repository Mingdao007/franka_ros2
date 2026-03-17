"""
plot_results.py — Generate all figures for the Koopman v1 report.

Figures:
1. M1 rollout bar chart: fz and exy NRMSE @5/@20/@50 for 5 methods
2. M1 rollout trajectories: predicted vs actual fz for EDMD-d and Linear at horizon 20
3. Mode partition: raw fz/z time series with M0/M1/M2 shading
4. Ablation contribution chart

Usage:
    python plot_results.py
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from preprocess import STATE_COLS, INPUT_COLS
from baselines import load_table, split_by_run, get_arrays, nrmse, ModeScaler, fit_arx, arx_predict
from edmd import fit_edmd, edmd_predict, lift
from evaluate import (
    FZ_IDX, Z_IDX, EX_IDX, EY_IDX, _exy,
    _compute_data_bounds, _is_diverged, _shift_delayed_state,
    make_dmd_fn, make_linear_fn, make_edmd_nodelay_fn, make_edmd_fn,
)

plt.rcParams.update({
    "font.size": 9,
    "figure.dpi": 200,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
})


def fig1_rollout_bar(results: dict, save_dir: Path):
    """M1 rollout bar chart: fz and exy at @5, @20, @50."""
    methods = [("DMD", "dmd"), ("Linear", "linear"), ("Linear-d", "lineard"),
               ("EDMD", "edmd"), ("EDMD-d", "edmdd")]
    colors = ["#aaaaaa", "#5DA5DA", "#60BD68", "#F17CB0", "#B276B2"]
    horizons = [5, 20, 50]

    fig, axes = plt.subplots(1, 2, figsize=(7, 3))

    for col, (metric, label) in enumerate([("fz", "fz NRMSE"), ("exy", "exy NRMSE")]):
        ax = axes[col]
        x = np.arange(len(horizons))
        width = 0.15

        for i, ((method, kp), color) in enumerate(zip(methods, colors)):
            vals = []
            for h in horizons:
                key = f"{kp}_roll{h}"
                rv = results.get(key, np.full(9, np.nan))
                if np.all(np.isnan(rv)):
                    vals.append(np.nan)
                elif metric == "fz":
                    vals.append(rv[FZ_IDX])
                else:
                    vals.append(_exy(rv))
            ax.bar(x + i * width, vals, width, label=method, color=color)

        ax.set_xticks(x + width * 2)
        ax.set_xticklabels([f"@{h}" for h in horizons])
        ax.set_ylabel(label)
        ax.set_yscale("log")
        if col == 0:
            ax.legend(fontsize=7, ncol=2)

    fig.suptitle("M1 Transition — Rollout NRMSE", fontsize=11)
    plt.tight_layout()
    fig.savefig(save_dir / "m1_rollout_bar.pdf")
    fig.savefig(save_dir / "m1_rollout_bar.png")
    print(f"  Saved: m1_rollout_bar.pdf/png")
    plt.close(fig)


def fig2_rollout_trajectory(table, scaler, test, save_dir: Path):
    """M1 fz rollout trajectory: actual vs predicted for Linear, Linear-d, EDMD-d."""
    test_m1 = test[test["mode"] == "M1"]
    first_run = test_m1["run_id"].iloc[0]
    run_data = test_m1[test_m1["run_id"] == first_run]
    s_run, u_run, sn_run = get_arrays(run_data)
    bounds_lo, bounds_hi = _compute_data_bounds(table)

    # Fit models
    train = table  # Use full table for fitting (same as evaluate.py)
    train_full, _, _, _ = split_by_run(table)
    train_m1 = train_full[train_full["mode"] == "M1"]
    s_tr, u_tr, sn_tr = get_arrays(train_m1)
    s_tr_z = scaler.transform_s(s_tr, "M1")
    u_tr_z = scaler.transform_u(u_tr, "M1")
    sn_tr_z = scaler.transform_target(sn_tr, "M1")

    from baselines import fit_arx
    A_lin, B_lin = fit_arx(s_tr_z[:, :9], u_tr_z, sn_tr_z)
    A_lind, B_lind = fit_arx(s_tr_z, u_tr_z, sn_tr_z)
    K_edmd_d = fit_edmd(s_tr_z, u_tr_z, sn_tr_z, alpha=0.0)

    def make_lineard_fn(A, B, scaler, mode):
        def fn(s, u):
            s_z = scaler.transform_s(s, mode)
            u_z = scaler.transform_u(u, mode)
            pred_z = arx_predict(s_z, u_z, A, B)
            return scaler.inverse_target(pred_z, mode)
        return fn

    fns = {
        "Linear": make_linear_fn(A_lin, B_lin, scaler, "M1"),
        "Linear-d": make_lineard_fn(A_lind, B_lind, scaler, "M1"),
        "EDMD-d": make_edmd_fn(K_edmd_d, scaler, "M1"),
    }

    horizon = min(50, len(s_run))
    dt = 0.001  # 1kHz
    t = np.arange(horizon + 1) * dt

    # Actual fz trajectory
    fz_actual = np.concatenate([s_run[:horizon, FZ_IDX], sn_run[horizon-1:horizon, FZ_IDX]])

    fig, ax = plt.subplots(figsize=(6, 3))
    ax.plot(t * 1000, fz_actual, "k-", linewidth=1.5, label="Actual")

    line_styles = {"Linear": "--", "Linear-d": "-.", "EDMD-d": "-"}
    line_colors = {"Linear": "#5DA5DA", "Linear-d": "#60BD68", "EDMD-d": "#B276B2"}

    for name, fn in fns.items():
        s_cur = s_run[0].copy()
        fz_pred = [s_cur[FZ_IDX]]
        diverged = False
        for step in range(horizon):
            s_next = fn(s_cur.reshape(1, -1), u_run[step].reshape(1, -1)).ravel()
            s_cur = _shift_delayed_state(s_cur, s_next)
            if _is_diverged(s_cur, bounds_lo, bounds_hi):
                diverged = True
                break
            fz_pred.append(s_cur[FZ_IDX])
        t_pred = np.arange(len(fz_pred)) * dt
        ls = line_styles[name]
        ax.plot(t_pred * 1000, fz_pred, ls, color=line_colors[name], linewidth=1.2,
                label=f"{name}" + (" (div)" if diverged else ""))

    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("fz (N)")
    ax.set_title("M1 Transition — fz Rollout (50 steps)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(save_dir / "m1_fz_rollout.pdf")
    fig.savefig(save_dir / "m1_fz_rollout.png")
    print(f"  Saved: m1_fz_rollout.pdf/png")
    plt.close(fig)


def fig3_mode_partition(table, save_dir: Path):
    """Time series with M0/M1/M2 shading from a single run."""
    # Pick first test run
    _, test, _, test_ids = split_by_run(table)
    first_run_id = test_ids[0]
    run = test[test["run_id"] == first_run_id]
    s, u, sn = get_arrays(run)

    dt = 0.001
    T = len(s)
    t = np.arange(T) * dt

    fz = s[:, FZ_IDX]
    z = s[:, STATE_COLS.index("z_meas")]
    modes = run["mode"].values

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 4), sharex=True)

    # Shade modes
    mode_colors = {"M0": "#FFE0B2", "M1": "#FFCCBC", "M2": "#C8E6C9"}
    mode_labels_done = set()
    for i in range(T - 1):
        m = modes[i]
        c = mode_colors.get(m, "white")
        label = m if m not in mode_labels_done else None
        ax1.axvspan(t[i], t[i+1], color=c, alpha=0.5, label=label)
        ax2.axvspan(t[i], t[i+1], color=c, alpha=0.5)
        mode_labels_done.add(m)

    ax1.plot(t, fz, "b-", linewidth=0.8)
    ax1.set_ylabel("fz (N)")
    ax1.legend(fontsize=8, loc="upper right")

    ax2.plot(t, z * 1000, "r-", linewidth=0.8)  # z in mm
    ax2.set_ylabel("z (mm)")
    ax2.set_xlabel("Time (s)")

    fig.suptitle(f"Run {first_run_id} — Mode Partition", fontsize=11)
    plt.tight_layout()
    fig.savefig(save_dir / "mode_partition.pdf")
    fig.savefig(save_dir / "mode_partition.png")
    print(f"  Saved: mode_partition.pdf/png")
    plt.close(fig)


def fig4_ablation(save_dir: Path):
    """Ablation contribution bar chart at M1 @50."""
    # Data from experiment_log / comparison table
    steps = ["DMD→Linear\n(+input)", "Linear→Linear-d\n(+delay)", "Linear-d→EDMD-d\n(+lifting)"]
    fz_improvement = [1.0, 2.3, 1.0]  # approximate ratios
    exy_improvement = [6.0, 6.8, 3.4]

    fig, ax = plt.subplots(figsize=(5, 3))
    x = np.arange(len(steps))
    width = 0.35
    ax.bar(x - width/2, fz_improvement, width, label="fz improvement", color="#B276B2")
    ax.bar(x + width/2, exy_improvement, width, label="exy improvement", color="#F17CB0")
    ax.set_xticks(x)
    ax.set_xticklabels(steps, fontsize=8)
    ax.set_ylabel("Improvement factor (×)")
    ax.set_title("Ablation: Each Component's Contribution (M1 @50)")
    ax.legend()
    ax.axhline(y=1.0, color="gray", linestyle="--", alpha=0.5)
    plt.tight_layout()
    fig.savefig(save_dir / "ablation.pdf")
    fig.savefig(save_dir / "ablation.png")
    print(f"  Saved: ablation.pdf/png")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Generate report figures")
    parser.add_argument("--data-dir", type=Path,
                        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"))
    parser.add_argument("--t-settle", type=float, default=0.5)
    args = parser.parse_args()

    save_dir = Path("/home/andy/franka_ros2_ws/src/koopman/figures")
    save_dir.mkdir(parents=True, exist_ok=True)

    table = load_table(args.data_dir, args.t_settle)
    train, test, _, _ = split_by_run(table)
    scaler = ModeScaler()
    scaler.fit(train)

    print("Generating figures...")

    # Fig 1: Rollout bar chart — use results from run_full_evaluation
    from evaluate import run_full_evaluation
    results = run_full_evaluation(table, args.t_settle)
    if "M1" in results:
        fig1_rollout_bar(results["M1"], save_dir)

    # Fig 2: Rollout trajectory
    fig2_rollout_trajectory(table, scaler, test, save_dir)

    # Fig 3: Mode partition
    fig3_mode_partition(table, save_dir)

    # Fig 4: Ablation
    fig4_ablation(save_dir)

    print(f"\nAll figures saved to: {save_dir}")


if __name__ == "__main__":
    main()
