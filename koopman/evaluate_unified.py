"""
Unified evaluation: all 7 models, all 6 test runs, consistent M1 start point.

Produces:
  - results/eval_step0.json   (all models from M1 step 0, delayed use M0 history)
  - results/eval_step10.json  (all models from M1 step 10, all history within M1)
  - fig_rollout_fz.pdf        (fz trajectory, test run 0, from step 0)
  - fig_rollout_combined.pdf  (fz + x + y, test run 0, from step 0)
  - fig_nrmse_horizon.pdf     (NRMSE vs horizon, multi-run mean)
  - Terminal: side-by-side comparison table
"""
import sys
sys.path.insert(0, "/home/andy/franka_ros2_ws/src/koopman")

import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime

from preprocess import STATE_COLS, INPUT_COLS

# ── Constants ────────────────────────────────────────────────────────────
FZ_IDX = STATE_COLS.index("fz_meas")  # 8
EX_IDX = STATE_COLS.index("ex")       # 6
EY_IDX = STATE_COLS.index("ey")       # 7
X_IDX  = STATE_COLS.index("x_meas")   # 0
Y_IDX  = STATE_COLS.index("y_meas")   # 1

DELAY_VARS = ["z_meas", "vz", "fz_meas", "ex", "ey"]
DELAY_VAR_IDX = [STATE_COLS.index(c) for c in DELAY_VARS]
HORIZONS = [1, 5, 10, 20, 50, 100]
DT = 0.001

MODEL_CONFIGS = {
    "DMD":              ("dmd",    None,       "base"),
    "Linear":           ("linear", None,       "base"),
    "EDMD":             ("edmd",   None,       "base"),
    "Linear-d [1,2]":   ("lineard", [1, 2],    "[1,2]"),
    "Linear-d [1,5,10]":("lineard", [1, 5, 10],"[1,5,10]"),
    "EDMD-d [1,2]":     ("edmdd",  [1, 2],     "[1,2]"),
    "EDMD-d [1,5,10]":  ("edmdd",  [1, 5, 10], "[1,5,10]"),
}

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 11,
    "axes.labelsize": 12,
    "legend.fontsize": 9,
    "figure.dpi": 200,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
})


# ── Data loading (from gen_rollout_exy.py) ───────────────────────────────
def load_runs_from_csv(csv_path: Path):
    """Load runs from preprocessed CSV. Returns list of (s, u, mode) per run."""
    df = pd.read_csv(csv_path)
    s_cols = [f"s_{c}" for c in STATE_COLS]
    u_cols = [f"u_{c}" for c in INPUT_COLS]
    sn_cols = [f"sn_{c}" for c in STATE_COLS]

    runs = []
    for rid in sorted(df["run_id"].unique()):
        rdf = df[df["run_id"] == rid]
        s_t = rdf[s_cols].values.astype(np.float64)
        u_t = rdf[u_cols].values.astype(np.float64)
        sn_last = rdf[sn_cols].iloc[-1].values.astype(np.float64)
        modes = rdf["mode"].values
        s_full = np.vstack([s_t, sn_last.reshape(1, -1)])
        u_full = np.vstack([u_t, u_t[-1:]])
        m_full = np.append(modes, modes[-1])
        runs.append((s_full, u_full, m_full))
    return runs


def build_delay_table(s, u, m, delay_indices=None):
    """Build one-step table, optionally delay-augmented."""
    if delay_indices is None:
        n = len(s) - 1
        return s[:n].copy(), u[:n].copy(), s[1:n+1].copy(), m[:n]

    max_lag = max(delay_indices)
    T = len(s)
    n = T - 1 - max_lag
    if n <= 0:
        return None, None, None, None

    s_base = s[max_lag:max_lag + n]
    u_t = u[max_lag:max_lag + n]
    sn = s[max_lag + 1:max_lag + n + 1]
    mode = m[max_lag:max_lag + n]

    delay_cols = []
    for lag in delay_indices:
        offset = max_lag - lag
        delay_cols.append(s[offset:offset + n][:, DELAY_VAR_IDX])
    delays = np.hstack(delay_cols)
    s_aug = np.hstack([s_base, delays])
    return s_aug, u_t, sn, mode


def split_runs(runs, seed=42, test_frac=0.25):
    rng = np.random.RandomState(seed)
    n = len(runs)
    n_test = max(1, int(n * test_frac))
    idx = list(range(n))
    rng.shuffle(idx)
    test_idx = set(idx[:n_test])
    train_idx = set(idx[n_test:])
    return sorted(train_idx), sorted(test_idx)


# ── Standardization (from gen_rollout_exy.py) ────────────────────────────
class Scaler:
    def __init__(self):
        self.s_mean = self.s_std = None
        self.u_mean = self.u_std = None
        self.t_mean = self.t_std = None

    def fit(self, s_list, u_list, sn_list):
        all_s = np.vstack(s_list)
        all_u = np.vstack(u_list)
        all_sn = np.vstack(sn_list)
        pool_9 = np.vstack([all_s[:, :9], all_sn])
        self.t_mean = pool_9.mean(axis=0)
        self.t_std = pool_9.std(axis=0)
        self.t_std[self.t_std < 1e-12] = 1.0
        self.s_mean = all_s.mean(axis=0)
        self.s_std = all_s.std(axis=0)
        self.s_std[self.s_std < 1e-12] = 1.0
        self.s_mean[:9] = self.t_mean
        self.s_std[:9] = self.t_std
        self.u_mean = all_u.mean(axis=0)
        self.u_std = all_u.std(axis=0)
        self.u_std[self.u_std < 1e-12] = 1.0

    def zs(self, s): return (s - self.s_mean) / self.s_std
    def zu(self, u): return (u - self.u_mean) / self.u_std
    def zt(self, sn): return (sn - self.t_mean) / self.t_std
    def inv_t(self, sn_z): return sn_z * self.t_std + self.t_mean


# ── Lifting (from gen_rollout_exy.py) ────────────────────────────────────
def lift(s_z, u_z):
    linear = np.column_stack([s_z, u_z])
    squared = np.column_stack([
        s_z[:, 8] ** 2, s_z[:, 6] ** 2, s_z[:, 7] ** 2,
    ])
    cross = np.column_stack([
        s_z[:, 6] * s_z[:, 3], s_z[:, 7] * s_z[:, 4],
        s_z[:, 2] * s_z[:, 8], s_z[:, 0] * u_z[:, 0],
        s_z[:, 1] * u_z[:, 1],
    ])
    return np.column_stack([linear, squared, cross])


# ── Model fitting (from gen_rollout_exy.py) ──────────────────────────────
def fit_ols(X, Y, alpha=0.0):
    if alpha > 0:
        W = np.linalg.solve(X.T @ X + alpha * np.eye(X.shape[1]), X.T @ Y)
    else:
        W, _, _, _ = np.linalg.lstsq(X, Y, rcond=None)
    return W.T


def fit_all_models(runs, train_idx, delay_configs):
    """Fit all 7 models on M1 train data."""
    models = {}
    for dc_name, delay_indices in delay_configs.items():
        s_list, u_list, sn_list = [], [], []
        for i in train_idx:
            s, u, m = runs[i]
            s_aug, u_t, sn, mode = build_delay_table(s, u, m, delay_indices)
            if s_aug is None:
                continue
            mask = mode == "M1"
            if mask.sum() > 0:
                s_list.append(s_aug[mask])
                u_list.append(u_t[mask])
                sn_list.append(sn[mask])
        if not s_list:
            continue

        scaler = Scaler()
        scaler.fit(s_list, u_list, sn_list)
        models[f"scaler_{dc_name}"] = scaler

        S = np.vstack(s_list)
        U = np.vstack(u_list)
        SN = np.vstack(sn_list)
        Sz, Uz, SNz = scaler.zs(S), scaler.zu(U), scaler.zt(SN)

        if dc_name == "base":
            models["DMD"] = (fit_ols(Sz, SNz), scaler)
            models["Linear"] = (fit_ols(np.column_stack([Sz, Uz]), SNz), scaler)
            models["EDMD"] = (fit_ols(lift(Sz, Uz), SNz), scaler)
        else:
            models[f"Linear-d {dc_name}"] = (fit_ols(np.column_stack([Sz, Uz]), SNz), scaler)
            models[f"EDMD-d {dc_name}"] = (fit_ols(lift(Sz, Uz), SNz), scaler)
    return models


# ── Rollout (from gen_rollout_exy.py) ────────────────────────────────────
def rollout(s0_aug, u_seq, model_info, model_type, delay_indices, horizon,
            s_history_init=None):
    W, scaler = model_info
    preds = [s0_aug[:9].copy()]

    if delay_indices is not None:
        history = list(s_history_init) if s_history_init is not None else [s0_aug[:9].copy()]
    else:
        history = [s0_aug[:9].copy()]

    for t in range(min(horizon, len(u_seq))):
        cur_9 = history[-1]
        if delay_indices is not None:
            delays = []
            for lag in delay_indices:
                idx = len(history) - 1 - lag
                if idx < 0:
                    delays.append(history[0][DELAY_VAR_IDX])
                else:
                    delays.append(history[idx][DELAY_VAR_IDX])
            s_aug = np.concatenate([cur_9, *delays])
        else:
            s_aug = cur_9.copy()

        sz = scaler.zs(s_aug.reshape(1, -1))
        uz = scaler.zu(u_seq[t].reshape(1, -1))

        if model_type == "dmd":
            inp = sz
        elif model_type in ("linear", "lineard"):
            inp = np.column_stack([sz, uz])
        elif model_type in ("edmd", "edmdd"):
            inp = lift(sz, uz)
        else:
            raise ValueError(model_type)

        pred_z = inp @ W.T
        s_next_9 = scaler.inv_t(pred_z).ravel()

        if np.any(np.abs(s_next_9) > 100):
            break

        history.append(s_next_9.copy())
        preds.append(s_next_9.copy())

    return np.array(preds)


# ── NRMSE ────────────────────────────────────────────────────────────────
def nrmse(pred, actual):
    """Per-variable NRMSE, normalized by variable range."""
    ranges = actual.max(axis=0) - actual.min(axis=0)
    ranges = np.where(ranges < 1e-12, 1.0, ranges)
    rmse = np.sqrt(np.mean((pred - actual) ** 2, axis=0))
    return rmse / ranges


# ── Unified evaluation ───────────────────────────────────────────────────
def build_rollout_init(s_full, m1_start, delay_idx):
    """Build initial augmented state and history buffer for a rollout."""
    if delay_idx is not None:
        max_lag = max(delay_idx)
        if m1_start < max_lag:
            return None, None
        s_history = [s_full[m1_start - max_lag + i].copy()
                     for i in range(max_lag + 1)]
        delays = [s_full[m1_start - lag][DELAY_VAR_IDX] for lag in delay_idx]
        s0_aug = np.concatenate([s_full[m1_start], *delays])
    else:
        s0_aug = s_full[m1_start].copy()
        s_history = None
    return s0_aug, s_history


def onestep_predict(s_aug, u, model_info, model_type):
    """Single one-step prediction from augmented state."""
    W, scaler = model_info
    sz = scaler.zs(s_aug.reshape(1, -1))
    uz = scaler.zu(u.reshape(1, -1))
    if model_type == "dmd":
        inp = sz
    elif model_type in ("linear", "lineard"):
        inp = np.column_stack([sz, uz])
    elif model_type in ("edmd", "edmdd"):
        inp = lift(sz, uz)
    else:
        raise ValueError(model_type)
    return scaler.inv_t(inp @ W.T).ravel()


def evaluate_onestep(runs, test_idx, models):
    """One-step prediction NRMSE over all M1 steps, all test runs.
    Delayed models use full-run history (M0 data for early M1 steps).
    All models evaluated on the same M1 steps (0 to end).
    """
    per_run = {}

    for ti in test_idx:
        s_full, u_full, m_full = runs[ti]
        m1_mask = m_full == "M1"
        m1_indices = np.where(m1_mask)[0]
        m1_abs_start = m1_indices[0]

        for name, (mtype, delay_idx, dc_key) in MODEL_CONFIGS.items():
            if name not in models:
                continue

            # Build delay table from FULL run, then filter to M1
            s_aug_full, u_full_dt, sn_full_dt, mode_full = build_delay_table(
                s_full, u_full, m_full, delay_idx
            )
            if s_aug_full is None:
                continue

            # Filter to M1 rows
            m1_dt_mask = mode_full == "M1"
            s_aug_m1 = s_aug_full[m1_dt_mask]
            u_m1 = u_full_dt[m1_dt_mask]
            sn_m1 = sn_full_dt[m1_dt_mask]  # ground truth next state (9-dim)

            # Predict one step for each M1 row
            preds = np.array([
                onestep_predict(s_aug_m1[i], u_m1[i], models[name], mtype)
                for i in range(len(s_aug_m1))
            ])

            err = nrmse(preds, sn_m1)
            per_run.setdefault(name, {})[str(ti)] = {
                "fz": float(err[FZ_IDX]),
                "exy": float(0.5 * (err[EX_IDX] + err[EY_IDX])),
            }

    # Aggregate
    summary = {}
    for name in per_run:
        fz_vals = [per_run[name][r]["fz"] for r in per_run[name]]
        exy_vals = [per_run[name][r]["exy"] for r in per_run[name]]
        summary[name] = {
            "fz_mean": float(np.mean(fz_vals)),
            "fz_std": float(np.std(fz_vals)),
            "exy_mean": float(np.mean(exy_vals)),
            "exy_std": float(np.std(exy_vals)),
        }
    return per_run, summary


def evaluate_all(runs, test_idx, models, m1_offset):
    """Evaluate all models on all test runs from M1 step m1_offset.
    Returns per_run dict and summary dict.
    """
    per_run = {}
    rollout_trajs = {}  # for plotting: {model_name: preds} from first test run

    for ti_idx, ti in enumerate(test_idx):
        s_full, u_full, m_full = runs[ti]
        m1_mask = m_full == "M1"
        m1_abs_start = np.where(m1_mask)[0][0]
        m1_start = m1_abs_start + m1_offset

        for name, (mtype, delay_idx, dc_key) in MODEL_CONFIGS.items():
            model_key = name
            if model_key not in models:
                continue

            s0_aug, s_history = build_rollout_init(s_full, m1_start, delay_idx)
            if s0_aug is None:
                continue

            max_horizon = max(HORIZONS)
            u_seq = u_full[m1_start:m1_start + max_horizon]
            preds = rollout(s0_aug, u_seq, models[model_key], mtype,
                            delay_idx, max_horizon, s_history_init=s_history)

            # Save trajectory from first test run for plotting
            if ti_idx == 0:
                rollout_trajs[name] = preds

            # Compute NRMSE at each horizon
            fz_vals = []
            exy_vals = []
            for h in HORIZONS:
                gt = s_full[m1_start:m1_start + h + 1]
                n = min(len(preds), len(gt))
                if n < 2:
                    fz_vals.append(float("nan"))
                    exy_vals.append(float("nan"))
                    continue
                err = nrmse(preds[:n], gt[:n])
                fz_vals.append(float(err[FZ_IDX]))
                exy_vals.append(float(0.5 * (err[EX_IDX] + err[EY_IDX])))

            per_run.setdefault(name, {})[str(ti)] = {
                "fz": fz_vals, "exy": exy_vals
            }

    # Aggregate mean ± std across test runs
    summary = {}
    for name in per_run:
        fz_all = np.array([per_run[name][r]["fz"] for r in per_run[name]])
        exy_all = np.array([per_run[name][r]["exy"] for r in per_run[name]])
        summary[name] = {
            "fz_mean": np.nanmean(fz_all, axis=0).tolist(),
            "fz_std": np.nanstd(fz_all, axis=0).tolist(),
            "exy_mean": np.nanmean(exy_all, axis=0).tolist(),
            "exy_std": np.nanstd(exy_all, axis=0).tolist(),
        }

    # Ground truth info from first test run (for plotting)
    ti0 = test_idx[0]
    s0, u0, m0 = runs[ti0]
    m1_abs = np.where(m0 == "M1")[0][0]
    gt0 = s0[m1_abs + m1_offset : m1_abs + m1_offset + max(HORIZONS) + 1]

    return per_run, summary, rollout_trajs, gt0


# ── Printing ─────────────────────────────────────────────────────────────
def print_table(rollout_summary, onestep_summary, label):
    """Print NRMSE table with 1-step + rollout columns."""
    model_order = ["DMD", "Linear", "EDMD",
                   "Linear-d [1,2]", "Linear-d [1,5,10]",
                   "EDMD-d [1,2]", "EDMD-d [1,5,10]"]
    hdr = "  {:25s} {:>12s}".format("Model", "1-step")
    for h in HORIZONS:
        hdr += " {:>8s}".format(f"@{h}")
    print(f"\n{'='*105}")
    print(f"  {label}")
    print(f"{'='*105}")

    # fz
    print(f"\n  fz NRMSE:")
    print(hdr)
    print("  " + "-" * 87)
    for name in model_order:
        if name not in rollout_summary:
            continue
        os_val = onestep_summary[name]["fz_mean"] if name in onestep_summary else float("nan")
        rollout_vals = rollout_summary[name]["fz_mean"]
        row = "  {:25s} {:>12.6e}".format(name, os_val)
        for v in rollout_vals:
            if np.isnan(v):
                row += " {:>8s}".format("div")
            else:
                row += " {:>8.3f}".format(v)
        print(row)

    # exy
    print(f"\n  exy NRMSE:")
    print(hdr)
    print("  " + "-" * 87)
    for name in model_order:
        if name not in rollout_summary:
            continue
        os_val = onestep_summary[name]["exy_mean"] if name in onestep_summary else float("nan")
        rollout_vals = rollout_summary[name]["exy_mean"]
        row = "  {:25s} {:>12.6e}".format(name, os_val)
        for v in rollout_vals:
            if np.isnan(v):
                row += " {:>8s}".format("div")
            else:
                row += " {:>8.4f}".format(v) if v < 1 else " {:>8.2f}".format(v)
        print(row)


# ── Plotting ─────────────────────────────────────────────────────────────
LINE_STYLES = {
    "DMD":              {"color": "#888888", "ls": ":",  "lw": 1.8},
    "Linear":           {"color": "#888888", "ls": "--", "lw": 1.8},
    "EDMD":             {"color": "#D6A6B8", "ls": "-",  "lw": 1.0, "alpha": 0.6, "zorder": 1},
    "Linear-d [1,2]":   {"color": "#5DA5DA", "ls": "-.", "lw": 1.8},
    "Linear-d [1,5,10]":{"color": "#FF8C00", "ls": "-",  "lw": 2.2},
    "EDMD-d [1,2]":     {"color": "#60BD68", "ls": "-.", "lw": 1.8},
    "EDMD-d [1,5,10]":  {"color": "#d62728", "ls": "-",  "lw": 2.5},
}

HORIZON_STYLES = {
    "DMD":              {"color": "#888888", "marker": "o", "ls": "-",  "lw": 1.8, "ms": 8},
    "Linear":           {"color": "#5DA5DA", "marker": "s", "ls": "-",  "lw": 1.8, "ms": 8},
    "EDMD":             {"color": "#F17CB0", "marker": "^", "ls": "-",  "lw": 1.8, "ms": 8},
    "Linear-d [1,2]":   {"color": "#60BD68", "marker": "s", "ls": "--", "lw": 2.0, "ms": 9},
    "Linear-d [1,5,10]":{"color": "#60BD68", "marker": "D", "ls": "-",  "lw": 2.2, "ms": 9},
    "EDMD-d [1,2]":     {"color": "#B276B2", "marker": "^", "ls": "--", "lw": 2.0, "ms": 9},
    "EDMD-d [1,5,10]":  {"color": "#B276B2", "marker": "*", "ls": "-",  "lw": 2.5, "ms": 12},
}


def plot_rollout_fz(gt, rollout_trajs, out_dir, suffix=""):
    """Standalone fz rollout figure."""
    horizon = len(gt) - 1
    t_gt = np.arange(len(gt)) * DT

    fig, ax = plt.subplots(figsize=(7, 3))
    ax.plot(t_gt, gt[:, FZ_IDX], "k-", lw=2.5, label="Ground truth")
    for name, preds in rollout_trajs.items():
        s = LINE_STYLES[name]
        t_p = np.arange(len(preds)) * DT
        ax.plot(t_p, preds[:, FZ_IDX], ls=s["ls"], color=s["color"],
                lw=s["lw"], alpha=s.get("alpha", 1.0), zorder=s.get("zorder", 2),
                label=name)
    ax.set_ylim(5.0, 6.2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(r"$f_z$ (N)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", ncol=2, fontsize=8, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_dir / f"fig_rollout_fz{suffix}.pdf")
    fig.savefig(out_dir / f"fig_rollout_fz{suffix}.png")
    print(f"  Saved: fig_rollout_fz{suffix}.pdf")
    plt.close(fig)


def plot_rollout_combined(gt, rollout_trajs, out_dir, suffix=""):
    """Combined fz + x + y figure."""
    t_gt = np.arange(len(gt)) * DT

    fig = plt.figure(figsize=(10, 7))
    ax_fz = fig.add_subplot(2, 1, 1)
    ax_x = fig.add_subplot(2, 2, 3)
    ax_y = fig.add_subplot(2, 2, 4)

    ax_fz.plot(t_gt, gt[:, FZ_IDX], "k-", lw=2.5, label="Ground truth")
    for name, preds in rollout_trajs.items():
        s = LINE_STYLES[name]
        t_p = np.arange(len(preds)) * DT
        ax_fz.plot(t_p, preds[:, FZ_IDX], ls=s["ls"], color=s["color"],
                   lw=s["lw"], label=name)
    ax_fz.set_ylim(5.0, 6.2)
    ax_fz.set_xlabel("Time (s)")
    ax_fz.set_ylabel(r"$f_z$ (N)")
    ax_fz.grid(True, alpha=0.3)

    # xy — exclude DMD (no input, xy meaningless) and EDMD (diverges)
    for ax, idx, label in [(ax_x, X_IDX, r"$x$ (mm)"),
                           (ax_y, Y_IDX, r"$y$ (mm)")]:
        ax.plot(t_gt, gt[:, idx] * 1000, "k-", lw=2.5)
        for name, preds in rollout_trajs.items():
            if name in ("DMD", "EDMD"):
                continue
            s = LINE_STYLES[name]
            t_p = np.arange(len(preds)) * DT
            ax.plot(t_p, preds[:, idx] * 1000, ls=s["ls"],
                    color=s["color"], lw=s["lw"])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)

    handles, labels = ax_fz.get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, fontsize=9,
               framealpha=0.9, bbox_to_anchor=(0.5, 1.02))
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(out_dir / f"fig_rollout_combined{suffix}.pdf")
    fig.savefig(out_dir / f"fig_rollout_combined{suffix}.png")
    print(f"  Saved: fig_rollout_combined{suffix}.pdf")
    plt.close(fig)


def plot_nrmse_horizon(summary, out_dir, suffix=""):
    """NRMSE vs horizon figure (log-log, two panels: fz and exy)."""
    model_order = ["DMD", "Linear", "EDMD",
                   "Linear-d [1,2]", "Linear-d [1,5,10]",
                   "EDMD-d [1,2]", "EDMD-d [1,5,10]"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))

    for name in model_order:
        if name not in summary:
            continue
        s = HORIZON_STYLES[name]
        fz_vals = summary[name]["fz_mean"]
        exy_vals = summary[name]["exy_mean"]
        ax1.plot(HORIZONS, fz_vals, marker=s["marker"], color=s["color"],
                 ls=s["ls"], lw=s["lw"], markersize=s["ms"], label=name)
        ax2.plot(HORIZONS, exy_vals, marker=s["marker"], color=s["color"],
                 ls=s["ls"], lw=s["lw"], markersize=s["ms"], label=name)

    for ax, ylabel in [(ax1, r"$f_z$ NRMSE"), (ax2, r"$e_{xy}$ NRMSE")]:
        ax.set_xlabel("Rollout horizon")
        ax.set_ylabel(ylabel)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xticks(HORIZONS)
        ax.set_xticklabels([str(h) for h in HORIZONS])
        ax.grid(True, alpha=0.3, which="both")

    ax1.legend(loc="upper left", ncol=2, fontsize=8.5, framealpha=0.9)
    fig.suptitle(r"M1 contact transition: NRMSE vs. rollout horizon", fontsize=13)
    plt.tight_layout()
    fig.savefig(out_dir / f"fig_nrmse_horizon{suffix}.pdf")
    fig.savefig(out_dir / f"fig_nrmse_horizon{suffix}.png")
    print(f"  Saved: fig_nrmse_horizon{suffix}.pdf")
    plt.close(fig)


# ── Main ─────────────────────────────────────────────────────────────────
def main():
    csv_path = Path("/home/andy/franka_ros2_ws/src/koopman/data/train_table_settle0.5.csv")
    out_dir = Path("/home/andy/franka_ros2_ws/src/koopman")
    results_dir = out_dir / "results"
    results_dir.mkdir(exist_ok=True)

    print("Loading runs from CSV...")
    runs = load_runs_from_csv(csv_path)
    print(f"  {len(runs)} runs loaded")

    train_idx, test_idx = split_runs(runs)
    print(f"  Train: {len(train_idx)}, Test: {len(test_idx)}")
    print(f"  Test run indices: {test_idx}")

    delay_configs = {"base": None, "[1,2]": [1, 2], "[1,5,10]": [1, 5, 10]}

    print("Fitting models...")
    models = fit_all_models(runs, train_idx, delay_configs)
    model_names = [k for k in models if not k.startswith("scaler")]
    print(f"  Models: {model_names}")

    # ── One-step evaluation (computed once, shared across offsets) ────────
    print("\nComputing one-step prediction NRMSE (all M1 steps, M0 history)...")
    onestep_per_run, onestep_summary = evaluate_onestep(runs, test_idx, models)
    print("  Done.")

    # ── Evaluate ──────────────────────────────────────────────────────────
    for m1_offset in [10]:
        label = f"step {m1_offset}"
        suffix = f"_step{m1_offset}"
        print(f"\n{'#'*90}")
        print(f"  Evaluating from M1 {label}  (m1_offset={m1_offset})")
        print(f"{'#'*90}")

        per_run, summary, rollout_trajs, gt = evaluate_all(
            runs, test_idx, models, m1_offset
        )

        # Save JSON
        result = {
            "metadata": {
                "m1_offset": m1_offset,
                "seed": 42,
                "n_train": len(train_idx),
                "n_test": len(test_idx),
                "test_run_indices": test_idx,
                "horizons": HORIZONS,
                "timestamp": datetime.now().isoformat(),
            },
            "onestep_per_run": onestep_per_run,
            "onestep_summary": onestep_summary,
            "per_run": per_run,
            "summary": summary,
        }
        json_path = results_dir / f"eval{suffix}.json"
        with open(json_path, "w") as f:
            json.dump(result, f, indent=2)
        print(f"  Saved: {json_path}")

        # Print table
        print_table(summary, onestep_summary,
                    f"M1 from {label} — {len(test_idx)} test runs")

        # Generate figures
        print(f"\n  Generating figures ({label})...")
        plot_rollout_fz(gt, rollout_trajs, out_dir, suffix)
        plot_rollout_combined(gt, rollout_trajs, out_dir, suffix)
        plot_nrmse_horizon(summary, out_dir, suffix)

    print("\nDone.")


if __name__ == "__main__":
    main()
