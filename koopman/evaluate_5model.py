"""
evaluate_5model.py — 5-model Koopman comparison with configurable delay lags.

Models: DMD, Linear, Linear-d, EDMD, EDMD-d
Delay embedding: configurable lag indices (default [1,2], try [1,5,10])

Usage:
    python evaluate_5model.py --lags 1,2        # default (consecutive)
    python evaluate_5model.py --lags 1,5,10     # sparse lags
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from preprocess import STATE_COLS, INPUT_COLS
from baselines import load_table, split_by_run, get_arrays, nrmse, ModeScaler

# Indices into STATE_COLS for delay variables
# z_meas(2), vz(5), fz_meas(8), ex(6), ey(7)
DELAY_VAR_INDICES = [2, 5, 8, 6, 7]
DELAY_VAR_NAMES = ["z_meas", "vz", "fz_meas", "ex", "ey"]

FZ_IDX = STATE_COLS.index("fz_meas")
Z_IDX = STATE_COLS.index("z_meas")
EX_IDX = STATE_COLS.index("ex")
EY_IDX = STATE_COLS.index("ey")

ROLLOUT_HORIZONS = [1, 5, 10, 20, 50, 100]


# ---------------------------------------------------------------------------
# Delay embedding
# ---------------------------------------------------------------------------

def build_delayed_arrays(s, u, sn, run_ids, lag_indices):
    """
    Build delay-augmented state arrays from per-row data.

    For each row at time t, appends s[t-lag][DELAY_VAR_INDICES] for each lag.
    Rows without sufficient history (per-run) are dropped.

    Returns: s_out, u_out, sn_out, s_delayed, valid_run_ids
        s_delayed: (N_valid, n_delay_vars * n_lags)
    """
    max_lag = max(lag_indices)
    n_delay = len(DELAY_VAR_INDICES)

    unique_runs = np.unique(run_ids)
    keep_mask = np.zeros(len(s), dtype=bool)
    s_delayed = np.zeros((len(s), n_delay * len(lag_indices)))

    for rid in unique_runs:
        run_mask = run_ids == rid
        run_indices = np.where(run_mask)[0]

        if len(run_indices) <= max_lag:
            continue

        # For each valid row in this run (those with enough history)
        for i, idx in enumerate(run_indices):
            if i < max_lag:
                continue
            keep_mask[idx] = True

            # Build delay features
            delay_feats = []
            for lag in lag_indices:
                src_idx = run_indices[i - lag]
                delay_feats.append(s[src_idx, DELAY_VAR_INDICES])
            s_delayed[idx] = np.concatenate(delay_feats)

    valid = keep_mask
    return s[valid], u[valid], sn[valid], s_delayed[valid], run_ids[valid]


def augment_state(s, s_delayed):
    """Concatenate current state with delay features: [s; delay]."""
    return np.column_stack([s, s_delayed])


# ---------------------------------------------------------------------------
# Lifting (nonlinear features on top of state+input)
# ---------------------------------------------------------------------------

def lift_no_delay(s, u):
    """Lift [s, u] -> Psi (20-dim). Same as edmd.py."""
    linear = np.column_stack([s, u])  # (N, 12)
    squared = np.column_stack([
        s[:, 8] ** 2,   # fz^2
        s[:, 6] ** 2,   # ex^2
        s[:, 7] ** 2,   # ey^2
    ])
    cross = np.column_stack([
        s[:, 6] * s[:, 3],  # ex * vx
        s[:, 7] * s[:, 4],  # ey * vy
        s[:, 2] * s[:, 8],  # z * fz
        s[:, 0] * u[:, 0],  # x * x_des
        s[:, 1] * u[:, 1],  # y * y_des
    ])
    return np.column_stack([linear, squared, cross])


def lift_with_delay(s_aug, u):
    """
    Lift [s_aug, u] -> Psi.
    s_aug = [s(9); delay_features(n_delay)].
    Nonlinear features use only the current state (first 9 dims).
    """
    s = s_aug[:, :9]  # current state
    linear = np.column_stack([s_aug, u])  # (N, 9+n_delay+3)
    squared = np.column_stack([
        s[:, 8] ** 2,
        s[:, 6] ** 2,
        s[:, 7] ** 2,
    ])
    cross = np.column_stack([
        s[:, 6] * s[:, 3],
        s[:, 7] * s[:, 4],
        s[:, 2] * s[:, 8],
        s[:, 0] * u[:, 0],
        s[:, 1] * u[:, 1],
    ])
    return np.column_stack([linear, squared, cross])


# ---------------------------------------------------------------------------
# Model fitting (all OLS in standardized space)
# ---------------------------------------------------------------------------

def fit_ols(X, Y):
    """Fit Y = X @ W via OLS, return W.T so prediction is X @ W.T'."""
    W, _, _, _ = np.linalg.lstsq(X, Y, rcond=None)
    return W.T  # (n_out, n_in)


# ---------------------------------------------------------------------------
# Delay-aware ModeScaler
# ---------------------------------------------------------------------------

class DelayModeScaler(ModeScaler):
    """Extends ModeScaler with delay feature standardization."""

    def __init__(self):
        super().__init__()
        self.delay_stats = {}  # {mode: (d_mean, d_std)}

    def fit_with_delay(self, train_df, s_delayed, run_ids):
        """Fit state/input scaler from df, delay scaler from arrays."""
        super().fit(train_df)

        for mode in ["M0", "M1", "M2"]:
            mode_mask = train_df["mode"].values == mode
            if not mode_mask.any():
                continue
            d = s_delayed[mode_mask]
            d_mean = d.mean(axis=0)
            d_std = d.std(axis=0)
            d_std = np.where(d_std < 1e-12, 1.0, d_std)
            self.delay_stats[mode] = (d_mean, d_std)

    def transform_delay(self, d, mode):
        m, sd = self.delay_stats[mode]
        return (d - m) / sd

    def transform_s_aug(self, s_aug, mode):
        """Transform [s; delay] -> [s_z; delay_z]."""
        s = s_aug[:, :9]
        d = s_aug[:, 9:]
        s_z = self.transform_s(s, mode)
        d_z = self.transform_delay(d, mode)
        return np.column_stack([s_z, d_z])


# ---------------------------------------------------------------------------
# Prediction functions (raw space in, raw space out)
# ---------------------------------------------------------------------------

def make_dmd_fn(W, scaler, mode):
    """DMD: s_next = W @ s"""
    def fn(s, u):
        s_z = scaler.transform_s(s, mode)
        pred_z = s_z @ W.T
        return scaler.inverse_s(pred_z, mode)
    return fn


def make_linear_fn(W, scaler, mode):
    """Linear: s_next = W @ [s; u]"""
    def fn(s, u):
        s_z = scaler.transform_s(s, mode)
        u_z = scaler.transform_u(u, mode)
        x = np.column_stack([s_z, u_z])
        pred_z = x @ W.T
        return scaler.inverse_s(pred_z, mode)
    return fn


def make_linear_d_fn(W, scaler, mode):
    """Linear-d: s_next = W @ [s_aug; u]"""
    def fn(s_aug, u):
        s_aug_z = scaler.transform_s_aug(s_aug, mode)
        u_z = scaler.transform_u(u, mode)
        x = np.column_stack([s_aug_z, u_z])
        pred_z = x @ W.T
        return scaler.inverse_s(pred_z, mode)
    return fn


def make_edmd_fn(W, scaler, mode):
    """EDMD: s_next = W @ lift([s; u])"""
    def fn(s, u):
        s_z = scaler.transform_s(s, mode)
        u_z = scaler.transform_u(u, mode)
        psi = lift_no_delay(s_z, u_z)
        pred_z = psi @ W.T
        return scaler.inverse_s(pred_z, mode)
    return fn


def make_edmd_d_fn(W, scaler, mode):
    """EDMD-d: s_next = W @ lift([s_aug; u])"""
    def fn(s_aug, u):
        s_aug_z = scaler.transform_s_aug(s_aug, mode)
        u_z = scaler.transform_u(u, mode)
        psi = lift_with_delay(s_aug_z, u_z)
        pred_z = psi @ W.T
        return scaler.inverse_s(pred_z, mode)
    return fn


# ---------------------------------------------------------------------------
# Rollout
# ---------------------------------------------------------------------------

def compute_data_bounds(s_all):
    """Compute 10x bounds for divergence detection."""
    s_min = s_all.min(axis=0)
    s_max = s_all.max(axis=0)
    s_range = s_max - s_min
    return s_min - 10 * s_range, s_max + 10 * s_range


def is_diverged(s, lo, hi):
    return np.any(s < lo) or np.any(s > hi) or np.any(np.isnan(s))


def rollout_nodelay(s0, u_seq, predict_fn, horizon, bounds_lo, bounds_hi):
    """Rollout for non-delayed models (DMD, Linear, EDMD)."""
    n = min(horizon, len(u_seq))
    preds = np.zeros((n, len(s0)))
    s_cur = s0.copy()
    for t in range(n):
        if is_diverged(s_cur, bounds_lo, bounds_hi):
            preds[t:] = np.nan
            return preds, True
        preds[t] = s_cur
        if t < n - 1:
            s_cur = predict_fn(
                s_cur.reshape(1, -1), u_seq[t].reshape(1, -1)
            ).ravel()
    return preds, False


def rollout_delayed(s0, delay_buf_0, u_seq, predict_fn, horizon,
                    bounds_lo, bounds_hi, lag_indices):
    """
    Rollout for delayed models (Linear-d, EDMD-d).

    delay_buf_0: dict {lag: s_values} — initial delay buffer from ground truth
    At each step, shift the buffer forward using predicted states.
    """
    n = min(horizon, len(u_seq))
    preds = np.zeros((n, 9))
    s_cur = s0.copy()

    # History ring buffer: stores past states for delay lookup
    max_lag = max(lag_indices)
    # Initialize with ground truth history
    history = list(delay_buf_0)  # list of state vectors, oldest first
    # history[-1] = s0, history[-2] = s at t-1, etc.

    for t in range(n):
        if is_diverged(s_cur, bounds_lo, bounds_hi):
            preds[t:] = np.nan
            return preds, True
        preds[t] = s_cur

        if t < n - 1:
            # Build delay features from history
            delay_feats = []
            for lag in lag_indices:
                idx = len(history) - 1 - lag
                if idx < 0:
                    delay_feats.append(history[0][DELAY_VAR_INDICES])
                else:
                    delay_feats.append(history[idx][DELAY_VAR_INDICES])
            d = np.concatenate(delay_feats)
            s_aug = np.concatenate([s_cur, d]).reshape(1, -1)
            s_cur = predict_fn(s_aug, u_seq[t].reshape(1, -1)).ravel()
            history.append(s_cur.copy())

    return preds, False


def evaluate_rollouts(s_runs, u_runs, predict_fn, horizons, bounds,
                      is_delayed, lag_indices=None,
                      s_delayed_runs=None, s_full_runs=None):
    """
    Evaluate rollout NRMSE across multiple test runs at given horizons.
    Returns dict: {horizon: {'fz': mean_nrmse, 'exy': mean_nrmse, 'diverged': count}}
    """
    bounds_lo, bounds_hi = bounds
    results = {h: {'fz': [], 'exy': [], 'diverged': 0, 'total': 0} for h in horizons}

    for run_idx in range(len(s_runs)):
        s_run = s_runs[run_idx]
        u_run = u_runs[run_idx]

        for h in horizons:
            if len(s_run) < h:
                continue
            results[h]['total'] += 1

            s0 = s_run[0]
            s_actual = s_run[:h]

            if is_delayed:
                # Build initial history buffer from ground truth
                s_full = s_full_runs[run_idx]  # full state sequence for this run
                max_lag = max(lag_indices)
                # We need states going back max_lag steps before rollout start
                # s_full includes pre-delay-trim states
                history = list(s_full[:max_lag + 1])  # states from t-max_lag to t0

                preds, div = rollout_delayed(
                    s0, history, u_run, predict_fn, h,
                    bounds_lo, bounds_hi, lag_indices
                )
            else:
                preds, div = rollout_nodelay(
                    s0, u_run, predict_fn, h, bounds_lo, bounds_hi
                )

            if div:
                results[h]['diverged'] += 1
            else:
                nrmse_vals = nrmse(preds, s_actual)
                results[h]['fz'].append(nrmse_vals[FZ_IDX])
                exy = 0.5 * (nrmse_vals[EX_IDX] + nrmse_vals[EY_IDX])
                results[h]['exy'].append(exy)

    # Aggregate
    out = {}
    for h in horizons:
        r = results[h]
        out[h] = {
            'fz': np.mean(r['fz']) if r['fz'] else np.nan,
            'exy': np.mean(r['exy']) if r['exy'] else np.nan,
            'diverged': r['diverged'],
            'total': r['total'],
        }
    return out


# ---------------------------------------------------------------------------
# Main evaluation
# ---------------------------------------------------------------------------

def get_per_run_arrays(df):
    """Split df into per-run arrays. Returns lists of (s, u, sn) per run."""
    s_runs, u_runs, sn_runs = [], [], []
    for rid in sorted(df["run_id"].unique()):
        mask = df["run_id"] == rid
        s, u, sn = get_arrays(df[mask])
        s_runs.append(s)
        u_runs.append(u)
        sn_runs.append(sn)
    return s_runs, u_runs, sn_runs


def run_evaluation(data_dir, t_settle, lag_indices):
    table = load_table(data_dir, t_settle)
    train, test, train_ids, test_ids = split_by_run(table)

    print(f"Lag indices: {lag_indices}")
    print(f"Train runs: {train_ids} ({len(train)} samples)")
    print(f"Test runs:  {test_ids} ({len(test)} samples)")

    # Build delay features for full table
    s_all, u_all, sn_all = get_arrays(table)
    run_ids_all = table["run_id"].values.astype(int)

    s_valid, u_valid, sn_valid, s_delayed_valid, rids_valid = build_delayed_arrays(
        s_all, u_all, sn_all, run_ids_all, lag_indices
    )

    # Build delayed train/test subsets
    train_rids = set(train_ids)
    test_rids = set(test_ids)
    train_delay_mask = np.isin(rids_valid, list(train_rids))
    test_delay_mask = np.isin(rids_valid, list(test_rids))

    # Compute bounds from all training data
    s_tr_all, _, _ = get_arrays(train)
    bounds = compute_data_bounds(s_tr_all)

    # Scaler
    scaler = DelayModeScaler()
    # Create a temporary df for delay-valid training rows
    valid_train_df = table.iloc[np.where(np.isin(run_ids_all, list(train_rids)))[0]]
    # For delay scaler, use delay-valid training rows
    scaler.fit(train)
    # Fit delay stats from delay-valid training data
    for mode in ["M0", "M1", "M2"]:
        mode_vals = table["mode"].values
        # Get mode mask for delay-valid training rows
        full_mask = np.zeros(len(table), dtype=bool)
        # Map back: s_valid corresponds to rows where build_delayed_arrays kept them
        # We need the mode labels for the valid rows
        pass

    # Simpler approach: rebuild per-mode
    # Get mode labels for valid rows
    mode_labels_all = table["mode"].values
    # build_delayed_arrays drops rows — we need to track which rows survived
    # Let me redo this more cleanly

    # Rebuild: get per-row keep mask
    max_lag = max(lag_indices)
    keep_indices = []
    for rid in np.unique(run_ids_all):
        run_mask = run_ids_all == rid
        run_idx = np.where(run_mask)[0]
        if len(run_idx) <= max_lag:
            continue
        keep_indices.extend(run_idx[max_lag:].tolist())

    keep_indices = np.array(keep_indices)
    modes_valid = mode_labels_all[keep_indices]
    rids_valid2 = run_ids_all[keep_indices]

    # Verify alignment
    assert len(keep_indices) == len(s_valid), \
        f"Mismatch: {len(keep_indices)} vs {len(s_valid)}"

    # Fit delay scaler per mode on training subset
    train_valid_mask = np.isin(rids_valid2, list(train_rids))
    for mode in ["M0", "M1", "M2"]:
        mode_mask = (modes_valid == mode) & train_valid_mask
        if not mode_mask.any():
            continue
        d = s_delayed_valid[mode_mask]
        d_mean = d.mean(axis=0)
        d_std = d.std(axis=0)
        d_std = np.where(d_std < 1e-12, 1.0, d_std)
        scaler.delay_stats[mode] = (d_mean, d_std)

    print()
    print("=" * 100)
    print(f"{'':>12} | {'One-step NRMSE':^24} | ", end="")
    for h in ROLLOUT_HORIZONS:
        print(f"{'Rollout @' + str(h):^24} | ", end="")
    print()
    print(f"{'Method':>12} | {'fz':>10} {'exy':>10} | ", end="")
    for h in ROLLOUT_HORIZONS:
        print(f"{'fz':>10} {'exy':>10} | ", end="")
    print()
    print("-" * 100)

    for mode in ["M0", "M1", "M2"]:
        # Non-delayed data
        train_m = train[train["mode"] == mode]
        test_m = test[test["mode"] == mode]
        if len(test_m) == 0 or len(train_m) == 0:
            continue
        if mode not in scaler.stats:
            continue

        s_tr, u_tr, sn_tr = get_arrays(train_m)
        s_te, u_te, sn_te = get_arrays(test_m)

        # Delayed data for this mode
        train_mode_delay = (modes_valid == mode) & train_valid_mask
        test_mode_delay = (modes_valid == mode) & ~train_valid_mask

        s_tr_d = s_valid[train_mode_delay]
        u_tr_d = u_valid[train_mode_delay]
        sn_tr_d = sn_valid[train_mode_delay]
        sd_tr = s_delayed_valid[train_mode_delay]

        s_te_d = s_valid[test_mode_delay]
        u_te_d = u_valid[test_mode_delay]
        sn_te_d = sn_valid[test_mode_delay]
        sd_te = s_delayed_valid[test_mode_delay]

        # Per-run test arrays for rollout
        test_run_s, test_run_u, test_run_sn = [], [], []
        test_run_s_full = []  # for delayed rollout initialization
        for rid in sorted(test_m["run_id"].unique()):
            # Non-delayed per-run
            rm = test_m["run_id"] == rid
            s_r, u_r, sn_r = get_arrays(test_m[rm])
            test_run_s.append(s_r)
            test_run_u.append(u_r)
            test_run_sn.append(sn_r)

        # For delayed rollout: need full state history per run
        # (including the max_lag rows before the valid window)
        test_run_s_delayed, test_run_u_delayed = [], []
        test_run_s_full_delayed = []
        for rid in sorted(test_m["run_id"].unique()):
            # Get all rows for this run in this mode (from full table)
            full_run_mode = table[(table["run_id"] == rid) & (table["mode"] == mode)]
            if len(full_run_mode) <= max_lag:
                continue
            s_fr, u_fr, sn_fr = get_arrays(full_run_mode)
            # Valid rows start at max_lag
            test_run_s_delayed.append(s_fr[max_lag:])
            test_run_u_delayed.append(u_fr[max_lag:])
            # Full state for history initialization (all rows including pre-lag)
            test_run_s_full_delayed.append(s_fr)

        # Standardize training data
        s_tr_z = scaler.transform_s(s_tr, mode)
        u_tr_z = scaler.transform_u(u_tr, mode)
        sn_tr_z = scaler.transform_s(sn_tr, mode)

        s_tr_d_z = scaler.transform_s(s_tr_d, mode)
        u_tr_d_z = scaler.transform_u(u_tr_d, mode)
        sn_tr_d_z = scaler.transform_s(sn_tr_d, mode)
        sd_tr_z = scaler.transform_delay(sd_tr, mode)
        s_aug_tr_z = np.column_stack([s_tr_d_z, sd_tr_z])

        print(f"\n  Mode {mode} (train: {len(train_m)}, test: {len(test_m)}, "
              f"train-delayed: {len(s_tr_d)}, test-delayed: {len(s_te_d)})")
        print("-" * 100)

        methods = {}

        # --- 1. DMD: s_next = W @ s ---
        W_dmd = fit_ols(s_tr_z, sn_tr_z)
        dmd_fn = make_dmd_fn(W_dmd, scaler, mode)
        dmd_pred = np.vstack([dmd_fn(s_te[i:i+1], u_te[i:i+1]) for i in range(len(s_te))])
        dmd_onestep = nrmse(dmd_pred, sn_te)
        dmd_rollout = evaluate_rollouts(
            test_run_s, test_run_u, dmd_fn, ROLLOUT_HORIZONS, bounds,
            is_delayed=False
        )
        methods["DMD"] = (dmd_onestep, dmd_rollout)

        # --- 2. Linear: s_next = W @ [s; u] ---
        X_lin = np.column_stack([s_tr_z, u_tr_z])
        W_lin = fit_ols(X_lin, sn_tr_z)
        lin_fn = make_linear_fn(W_lin, scaler, mode)
        lin_pred = np.vstack([lin_fn(s_te[i:i+1], u_te[i:i+1]) for i in range(len(s_te))])
        lin_onestep = nrmse(lin_pred, sn_te)
        lin_rollout = evaluate_rollouts(
            test_run_s, test_run_u, lin_fn, ROLLOUT_HORIZONS, bounds,
            is_delayed=False
        )
        methods["Linear"] = (lin_onestep, lin_rollout)

        # --- 3. Linear-d: s_next = W @ [s_aug; u] ---
        if len(s_tr_d) > 0 and mode in scaler.delay_stats:
            X_lind = np.column_stack([s_aug_tr_z, u_tr_d_z])
            W_lind = fit_ols(X_lind, sn_tr_d_z)
            lind_fn = make_linear_d_fn(W_lind, scaler, mode)

            # One-step on delayed test
            s_te_d_z = scaler.transform_s(s_te_d, mode)
            sd_te_z = scaler.transform_delay(sd_te, mode)
            s_aug_te = np.column_stack([s_te_d, sd_te])
            lind_pred = np.vstack([
                lind_fn(s_aug_te[i:i+1], u_te_d[i:i+1])
                for i in range(len(s_te_d))
            ])
            lind_onestep = nrmse(lind_pred, sn_te_d)

            lind_rollout = evaluate_rollouts(
                test_run_s_delayed, test_run_u_delayed, lind_fn,
                ROLLOUT_HORIZONS, bounds, is_delayed=True,
                lag_indices=lag_indices, s_full_runs=test_run_s_full_delayed
            )
            methods["Linear-d"] = (lind_onestep, lind_rollout)
        else:
            methods["Linear-d"] = (np.full(9, np.nan), {h: {'fz': np.nan, 'exy': np.nan, 'diverged': 0, 'total': 0} for h in ROLLOUT_HORIZONS})

        # --- 4. EDMD: s_next = W @ lift([s; u]) ---
        psi_tr = lift_no_delay(s_tr_z, u_tr_z)
        W_edmd = fit_ols(psi_tr, sn_tr_z)
        edmd_fn = make_edmd_fn(W_edmd, scaler, mode)
        edmd_pred = np.vstack([edmd_fn(s_te[i:i+1], u_te[i:i+1]) for i in range(len(s_te))])
        edmd_onestep = nrmse(edmd_pred, sn_te)
        edmd_rollout = evaluate_rollouts(
            test_run_s, test_run_u, edmd_fn, ROLLOUT_HORIZONS, bounds,
            is_delayed=False
        )
        methods["EDMD"] = (edmd_onestep, edmd_rollout)

        # --- 5. EDMD-d: s_next = W @ lift([s_aug; u]) ---
        if len(s_tr_d) > 0 and mode in scaler.delay_stats:
            psi_tr_d = lift_with_delay(s_aug_tr_z, u_tr_d_z)
            W_edmdd = fit_ols(psi_tr_d, sn_tr_d_z)
            edmdd_fn = make_edmd_d_fn(W_edmdd, scaler, mode)

            s_aug_te_raw = np.column_stack([s_te_d, sd_te])
            edmdd_pred = np.vstack([
                edmdd_fn(s_aug_te_raw[i:i+1], u_te_d[i:i+1])
                for i in range(len(s_te_d))
            ])
            edmdd_onestep = nrmse(edmdd_pred, sn_te_d)

            edmdd_rollout = evaluate_rollouts(
                test_run_s_delayed, test_run_u_delayed, edmdd_fn,
                ROLLOUT_HORIZONS, bounds, is_delayed=True,
                lag_indices=lag_indices, s_full_runs=test_run_s_full_delayed
            )
            methods["EDMD-d"] = (edmdd_onestep, edmdd_rollout)
        else:
            methods["EDMD-d"] = (np.full(9, np.nan), {h: {'fz': np.nan, 'exy': np.nan, 'diverged': 0, 'total': 0} for h in ROLLOUT_HORIZONS})

        # Print results for this mode
        for name in ["DMD", "Linear", "Linear-d", "EDMD", "EDMD-d"]:
            onestep, rollout = methods[name]
            fz_1 = onestep[FZ_IDX] if not np.all(np.isnan(onestep)) else np.nan
            exy_1 = 0.5 * (onestep[EX_IDX] + onestep[EY_IDX]) if not np.all(np.isnan(onestep)) else np.nan
            line = f"  {name:>10} | {fz_1:>10.4f} {exy_1:>10.5f} | "
            for h in ROLLOUT_HORIZONS:
                r = rollout[h]
                fz_r = r['fz']
                exy_r = r['exy']
                div_s = f"*" if r['diverged'] > 0 else ""
                if np.isnan(fz_r):
                    line += f"{'div':>10} {'div':>10} | "
                else:
                    line += f"{fz_r:>9.4f}{div_s} {exy_r:>9.4f}{div_s} | "
            print(line)


def main():
    parser = argparse.ArgumentParser(description="5-model Koopman comparison")
    parser.add_argument(
        "--data-dir", type=Path,
        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"),
    )
    parser.add_argument("--t-settle", type=float, default=0.5)
    parser.add_argument(
        "--lags", type=str, default="1,2",
        help="Comma-separated lag indices, e.g. '1,2' or '1,5,10'"
    )
    args = parser.parse_args()

    lag_indices = [int(x) for x in args.lags.split(",")]
    run_evaluation(args.data_dir, args.t_settle, lag_indices)


if __name__ == "__main__":
    main()
