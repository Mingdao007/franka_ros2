"""
evaluate.py — Full comparison: ARX vs per-mode EDMD.

Usage:
    python evaluate.py [--data-dir PATH] [--t-settle 0.5]

Runs ARX + EDMD per mode, evaluates one-step + rollout + peak metrics,
prints the locked comparison table (rows: method×mode, cols: fz/z focused).
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from preprocess import STATE_COLS, INPUT_COLS
from baselines import (
    load_table, split_by_run, get_arrays, nrmse, ModeScaler,
    fit_arx, arx_predict, persistence_predict,
)
from edmd import fit_edmd, edmd_predict, LIFT_DIM

FZ_IDX = STATE_COLS.index("fz_meas")
Z_IDX = STATE_COLS.index("z_meas")
ROLLOUT_HORIZONS = [50, 100, 500, 1000]


# ---------------------------------------------------------------------------
# Rollout
# ---------------------------------------------------------------------------

def mode_internal_rollout(
    s0: np.ndarray, u_seq: np.ndarray, s_actual: np.ndarray,
    predict_fn, horizon: int,
) -> np.ndarray:
    """
    Mode-internal rollout: fixed mode model, start within same mode.
    Returns per-variable NRMSE over the rollout window.
    """
    n_avail = min(horizon, len(u_seq), len(s_actual))
    if n_avail < 2:
        return np.full(len(STATE_COLS), np.nan)

    preds = np.zeros((n_avail, s0.shape[0]))
    s_cur = s0.copy()
    for t in range(n_avail):
        preds[t] = s_cur
        if t < n_avail - 1:
            s_cur = predict_fn(s_cur.reshape(1, -1), u_seq[t].reshape(1, -1)).ravel()

    return nrmse(preds, s_actual[:n_avail])


def full_process_rollout(
    s0: np.ndarray, u_seq: np.ndarray, s_actual: np.ndarray,
    mode_labels: np.ndarray, predict_fns: dict, horizon: int,
) -> np.ndarray:
    """
    Full-process rollout with oracle mode labels.
    Switch model at mode boundaries using ground-truth labels.
    """
    n_avail = min(horizon, len(u_seq), len(s_actual), len(mode_labels))
    if n_avail < 2:
        return np.full(len(STATE_COLS), np.nan)

    preds = np.zeros((n_avail, s0.shape[0]))
    s_cur = s0.copy()
    for t in range(n_avail):
        preds[t] = s_cur
        if t < n_avail - 1:
            mode = mode_labels[t]
            fn = predict_fns.get(mode)
            if fn is not None:
                s_cur = fn(s_cur.reshape(1, -1), u_seq[t].reshape(1, -1)).ravel()
            # If no model for this mode, hold (persistence)

    return nrmse(preds, s_actual[:n_avail])


# ---------------------------------------------------------------------------
# Peak metrics (M1 only)
# ---------------------------------------------------------------------------

def find_fz_peak(fz_series: np.ndarray, fz_des: float):
    """Find peak overshoot magnitude and timing in fz."""
    if len(fz_series) == 0:
        return np.nan, np.nan
    deviations = np.abs(fz_series - fz_des)
    peak_idx = np.argmax(deviations)
    peak_mag = deviations[peak_idx]
    peak_time = peak_idx * 0.001  # 1kHz
    return peak_mag, peak_time


def evaluate_peak_for_method(
    test_m1: pd.DataFrame, predict_fn, rollout_len: int = 500,
):
    """Peak magnitude/timing error on M1 transition segments."""
    if len(test_m1) == 0:
        return np.nan, np.nan

    s, u, sn = get_arrays(test_m1)
    run_ids = test_m1["run_id"].values
    unique_runs = np.unique(run_ids)

    mag_errors, time_errors = [], []

    for rid in unique_runs:
        mask = run_ids == rid
        s_run, u_run, sn_run = s[mask], u[mask], sn[mask]

        if len(s_run) < 10:
            continue

        fz_des = u_run[0, INPUT_COLS.index("fz_des")]

        # Actual peak
        fz_actual = np.concatenate([s_run[:, FZ_IDX], sn_run[-1:, FZ_IDX]])
        actual_mag, actual_time = find_fz_peak(fz_actual, fz_des)

        # Predicted rollout
        horizon = min(rollout_len, len(s_run))
        s_cur = s_run[0].copy()
        fz_pred_series = [s_cur[FZ_IDX]]
        for t in range(horizon - 1):
            s_next = predict_fn(s_cur.reshape(1, -1), u_run[t].reshape(1, -1)).ravel()
            fz_pred_series.append(s_next[FZ_IDX])
            s_cur = s_next
        fz_pred = np.array(fz_pred_series)
        pred_mag, pred_time = find_fz_peak(fz_pred, fz_des)

        if not (np.isnan(actual_mag) or np.isnan(pred_mag)):
            rel_mag_err = abs(pred_mag - actual_mag) / max(abs(actual_mag), 1e-6)
            mag_errors.append(rel_mag_err)
            time_errors.append(abs(pred_time - actual_time))

    if not mag_errors:
        return np.nan, np.nan
    return np.mean(mag_errors), np.mean(time_errors)


# ---------------------------------------------------------------------------
# Build prediction functions (handle standardization transparently)
# ---------------------------------------------------------------------------

def make_arx_fn(A, B, scaler, mode):
    """Return a predict_fn(s_raw, u_raw) -> s_raw_next for ARX."""
    def fn(s, u):
        s_z = scaler.transform_s(s, mode)
        u_z = scaler.transform_u(u, mode)
        pred_z = arx_predict(s_z, u_z, A, B)
        return scaler.inverse_s(pred_z, mode)
    return fn


def make_edmd_fn(K, scaler, mode):
    """Return a predict_fn(s_raw, u_raw) -> s_raw_next for EDMD."""
    def fn(s, u):
        s_z = scaler.transform_s(s, mode)
        u_z = scaler.transform_u(u, mode)
        pred_z = edmd_predict(s_z, u_z, K)
        return scaler.inverse_s(pred_z, mode)
    return fn


# ---------------------------------------------------------------------------
# Main evaluation
# ---------------------------------------------------------------------------

def run_full_evaluation(table: pd.DataFrame, t_settle: float):
    """Fit ARX + EDMD per mode, evaluate all metrics."""
    train, test, train_ids, test_ids = split_by_run(table)

    scaler = ModeScaler()
    scaler.fit(train)

    print(f"Train runs: {train_ids} ({len(train)} samples)")
    print(f"Test runs:  {test_ids} ({len(test)} samples)")
    print()

    modes = ["M0", "M1", "M2"]
    arx_fns = {}
    edmd_fns = {}
    all_results = {}

    for mode in modes:
        train_m = train[train["mode"] == mode]
        test_m = test[test["mode"] == mode]

        if len(test_m) == 0:
            print(f"  {mode}: no test data, skipping")
            continue

        s_tr, u_tr, sn_tr = get_arrays(train_m)
        s_te, u_te, sn_te = get_arrays(test_m)
        mr = {"n_train": len(train_m), "n_test": len(test_m)}

        # --- ARX ---
        if len(train_m) >= 12 and mode in scaler.stats:
            s_tr_z = scaler.transform_s(s_tr, mode)
            u_tr_z = scaler.transform_u(u_tr, mode)
            sn_tr_z = scaler.transform_s(sn_tr, mode)
            A, B = fit_arx(s_tr_z, u_tr_z, sn_tr_z)

            arx_fn = make_arx_fn(A, B, scaler, mode)
            arx_fns[mode] = arx_fn

            arx_pred = np.vstack([arx_fn(s_te[i:i+1], u_te[i:i+1]) for i in range(len(s_te))])
            mr["arx_onestep"] = nrmse(arx_pred, sn_te)
        else:
            mr["arx_onestep"] = np.full(9, np.nan)

        # --- EDMD ---
        if len(train_m) >= LIFT_DIM and mode in scaler.stats:
            s_tr_z = scaler.transform_s(s_tr, mode)
            u_tr_z = scaler.transform_u(u_tr, mode)
            sn_tr_z = scaler.transform_s(sn_tr, mode)
            K = fit_edmd(s_tr_z, u_tr_z, sn_tr_z)

            edmd_fn = make_edmd_fn(K, scaler, mode)
            edmd_fns[mode] = edmd_fn

            edmd_pred = np.vstack([edmd_fn(s_te[i:i+1], u_te[i:i+1]) for i in range(len(s_te))])
            mr["edmd_onestep"] = nrmse(edmd_pred, sn_te)
        else:
            mr["edmd_onestep"] = np.full(9, np.nan)

        # --- Mode-internal rollout ---
        first_run = test_m["run_id"].iloc[0]
        run_mask = test_m["run_id"] == first_run
        s_run, u_run, sn_run = get_arrays(test_m[run_mask])
        s_actual = np.vstack([s_run, sn_run[-1:]])

        for h in ROLLOUT_HORIZONS:
            if len(s_run) < h:
                mr[f"arx_roll{h}"] = np.full(9, np.nan)
                mr[f"edmd_roll{h}"] = np.full(9, np.nan)
                continue

            if mode in arx_fns:
                mr[f"arx_roll{h}"] = mode_internal_rollout(
                    s_run[0], u_run, s_actual, arx_fns[mode], h)
            else:
                mr[f"arx_roll{h}"] = np.full(9, np.nan)

            if mode in edmd_fns:
                mr[f"edmd_roll{h}"] = mode_internal_rollout(
                    s_run[0], u_run, s_actual, edmd_fns[mode], h)
            else:
                mr[f"edmd_roll{h}"] = np.full(9, np.nan)

        all_results[mode] = mr

    # --- Full-process rollout (oracle mode labels) ---
    test_run_ids = sorted(test["run_id"].unique())
    if test_run_ids and arx_fns and edmd_fns:
        rid = test_run_ids[0]
        run_data = test[test["run_id"] == rid].sort_index()
        s_run, u_run, sn_run = get_arrays(run_data)
        mode_labels = run_data["mode"].values
        s_actual = np.vstack([s_run, sn_run[-1:]])

        for h in ROLLOUT_HORIZONS:
            if len(s_run) < h:
                continue
            for method_name, fns in [("arx", arx_fns), ("edmd", edmd_fns)]:
                fp_nrmse = full_process_rollout(
                    s_run[0], u_run, s_actual, mode_labels, fns, h)
                # Store under a special "full" key
                all_results.setdefault("full_process", {})[f"{method_name}_roll{h}"] = fp_nrmse

    # --- Peak metrics (M1 only, in original space) ---
    test_m1 = test[test["mode"] == "M1"]
    if len(test_m1) > 0 and "M1" in all_results:
        if "M1" in arx_fns:
            mag, tim = evaluate_peak_for_method(test_m1, arx_fns["M1"])
            all_results["M1"]["arx_peak_mag"] = mag
            all_results["M1"]["arx_peak_time"] = tim

        if "M1" in edmd_fns:
            mag, tim = evaluate_peak_for_method(test_m1, edmd_fns["M1"])
            all_results["M1"]["edmd_peak_mag"] = mag
            all_results["M1"]["edmd_peak_time"] = tim

    return all_results


# ---------------------------------------------------------------------------
# Output: locked table format
# ---------------------------------------------------------------------------

def print_comparison_table(results: dict):
    """Print the locked comparison table: rows=method×mode, cols=fz/z focused."""
    print()
    print("=" * 90)
    print("MAIN RESULTS TABLE (locked format)")
    print("=" * 90)
    header = (f"{'':>12} {'fz 1-step':>10} {'z 1-step':>10} "
              f"{'fz r@500':>10} {'z r@500':>10} "
              f"{'peak mag':>10} {'peak time':>10}")
    print(header)
    print("-" * 90)

    for mode in ["M0", "M1", "M2"]:
        if mode not in results:
            continue
        r = results[mode]
        for method, key_prefix in [("ARX", "arx"), ("EDMD", "edmd")]:
            label = f"{method}-{mode}"
            onestep = r.get(f"{key_prefix}_onestep", np.full(9, np.nan))
            roll500 = r.get(f"{key_prefix}_roll500", np.full(9, np.nan))

            fz_1 = onestep[FZ_IDX] if not np.all(np.isnan(onestep)) else np.nan
            z_1 = onestep[Z_IDX] if not np.all(np.isnan(onestep)) else np.nan
            fz_r = roll500[FZ_IDX] if not np.all(np.isnan(roll500)) else np.nan
            z_r = roll500[Z_IDX] if not np.all(np.isnan(roll500)) else np.nan

            if mode == "M1":
                pk_mag = r.get(f"{key_prefix}_peak_mag", np.nan)
                pk_tim = r.get(f"{key_prefix}_peak_time", np.nan)
                pk_mag_s = f"{pk_mag:>10.4f}" if not np.isnan(pk_mag) else f"{'N/A':>10}"
                pk_tim_s = f"{pk_tim:>10.4f}" if not np.isnan(pk_tim) else f"{'N/A':>10}"
            else:
                pk_mag_s = f"{'—':>10}"
                pk_tim_s = f"{'—':>10}"

            print(f"{label:>12} {fz_1:>10.6f} {z_1:>10.6f} "
                  f"{fz_r:>10.6f} {z_r:>10.6f} "
                  f"{pk_mag_s} {pk_tim_s}")

    # Full-process rollout summary
    fp = results.get("full_process", {})
    if fp:
        print()
        print("FULL-PROCESS ROLLOUT (oracle mode labels):")
        print(f"{'':>12} {'fz NRMSE':>10} {'z NRMSE':>10}")
        print("-" * 40)
        for h in ROLLOUT_HORIZONS:
            for method in ["arx", "edmd"]:
                key = f"{method}_roll{h}"
                if key in fp:
                    v = fp[key]
                    print(f"{method.upper():>5} @{h:<5} "
                          f"{v[FZ_IDX]:>10.6f} {v[Z_IDX]:>10.6f}")


def save_report(results: dict, out_path: Path):
    """Save comparison as markdown."""
    lines = ["# Koopman v1 — ARX vs EDMD Comparison\n"]

    lines.append("## Main Results\n")
    lines.append("| | fz 1-step | z 1-step | fz roll@500 | z roll@500 | peak mag | peak time |")
    lines.append("|---|---|---|---|---|---|---|")

    for mode in ["M0", "M1", "M2"]:
        if mode not in results:
            continue
        r = results[mode]
        for method, kp in [("ARX", "arx"), ("EDMD", "edmd")]:
            label = f"{method}-{mode}"
            os = r.get(f"{kp}_onestep", np.full(9, np.nan))
            r5 = r.get(f"{kp}_roll500", np.full(9, np.nan))

            fz1 = f"{os[FZ_IDX]:.6f}" if not np.all(np.isnan(os)) else "N/A"
            z1 = f"{os[Z_IDX]:.6f}" if not np.all(np.isnan(os)) else "N/A"
            fzr = f"{r5[FZ_IDX]:.6f}" if not np.all(np.isnan(r5)) else "N/A"
            zr = f"{r5[Z_IDX]:.6f}" if not np.all(np.isnan(r5)) else "N/A"

            if mode == "M1":
                pm = r.get(f"{kp}_peak_mag", np.nan)
                pt = r.get(f"{kp}_peak_time", np.nan)
                pms = f"{pm:.4f}" if not np.isnan(pm) else "N/A"
                pts = f"{pt:.4f}" if not np.isnan(pt) else "N/A"
            else:
                pms, pts = "—", "—"

            lines.append(f"| {label} | {fz1} | {z1} | {fzr} | {zr} | {pms} | {pts} |")

    fp = results.get("full_process", {})
    if fp:
        lines.append("\n## Full-Process Rollout (oracle mode labels)\n")
        lines.append("| | fz NRMSE | z NRMSE |")
        lines.append("|---|---|---|")
        for h in ROLLOUT_HORIZONS:
            for method in ["arx", "edmd"]:
                key = f"{method}_roll{h}"
                if key in fp:
                    v = fp[key]
                    lines.append(f"| {method.upper()} @{h} | {v[FZ_IDX]:.6f} | {v[Z_IDX]:.6f} |")

    lines.append("")
    out_path.write_text("\n".join(lines))
    print(f"\nReport saved: {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Koopman full evaluation")
    parser.add_argument(
        "--data-dir", type=Path,
        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"),
    )
    parser.add_argument("--t-settle", type=float, default=0.5)
    args = parser.parse_args()

    table = load_table(args.data_dir, args.t_settle)
    results = run_full_evaluation(table, args.t_settle)
    print_comparison_table(results)

    report_path = args.data_dir / f"comparison_settle{args.t_settle}.md"
    save_report(results, report_path)


if __name__ == "__main__":
    main()
