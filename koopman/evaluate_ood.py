"""
evaluate_ood.py — OOD (out-of-distribution) validation.

Loads models trained on r=0.03m (nominal), evaluates on r=0.05m (OOD) data.
No retraining — pure generalization test.

Usage:
    # Step 1: Train and save models on nominal data
    python3 evaluate.py --save-models

    # Step 2: Collect OOD data (r=0.05m), preprocess it
    python3 preprocess.py --results-dir <ood_results_path> --out-dir data_ood

    # Step 3: Evaluate
    python3 evaluate_ood.py --ood-dir data_ood --model-dir data/trained_models
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from preprocess import STATE_COLS, INPUT_COLS, STATE_COLS_DELAYED, TARGET_COLS
from baselines import (
    load_table, split_by_run, get_arrays, nrmse, ModeScaler,
    fit_dmd, dmd_predict, fit_arx, arx_predict,
)
from edmd import fit_edmd, edmd_predict, LIFT_DIM
from evaluate import (
    FZ_IDX, Z_IDX, EX_IDX, EY_IDX, _exy, _fmt,
    M1_ROLLOUT_HORIZONS, STD_ROLLOUT_HORIZONS,
    _compute_data_bounds, _is_diverged, _shift_delayed_state,
    mode_internal_rollout, find_fz_overshoot,
    make_dmd_fn, make_linear_fn, make_edmd_nodelay_fn, make_edmd_fn,
)


def load_models_and_scaler(model_dir: Path):
    """Load saved models and scaler from evaluate.py --save-models."""
    scaler = ModeScaler()
    scaler.load(model_dir / "scaler.npz")

    data = np.load(model_dir / "models.npz", allow_pickle=True)
    models = {}
    for key in data.files:
        models[key] = data[key]

    return models, scaler


def build_predict_fns(models: dict, scaler: ModeScaler):
    """Reconstruct predict functions from saved model matrices."""
    methods = {}  # {method_name: {mode: predict_fn}}

    for method_name in ["dmd", "linear", "lineard", "edmd", "edmdd"]:
        methods[method_name] = {}

    for mode in ["M0", "M1", "M2"]:
        # DMD
        key = f"dmd_A_{mode}"
        if key in models:
            methods["dmd"][mode] = make_dmd_fn(models[key], scaler, mode)

        # Linear
        a_key, b_key = f"linear_A_{mode}", f"linear_B_{mode}"
        if a_key in models:
            methods["linear"][mode] = make_linear_fn(models[a_key], models[b_key], scaler, mode)

        # Linear-d
        a_key, b_key = f"lineard_A_{mode}", f"lineard_B_{mode}"
        if a_key in models:
            A, B = models[a_key], models[b_key]
            def _make_ld(A, B, scaler, mode):
                def fn(s, u):
                    s_z = scaler.transform_s(s, mode)
                    u_z = scaler.transform_u(u, mode)
                    pred_z = arx_predict(s_z, u_z, A, B)
                    return scaler.inverse_target(pred_z, mode)
                return fn
            methods["lineard"][mode] = _make_ld(A, B, scaler, mode)

        # EDMD (no delay)
        key = f"edmd_K_{mode}"
        if key in models:
            methods["edmd"][mode] = make_edmd_nodelay_fn(models[key], scaler, mode)

        # EDMD-d
        key = f"edmdd_K_{mode}"
        if key in models:
            methods["edmdd"][mode] = make_edmd_fn(models[key], scaler, mode)

    return methods


def evaluate_ood(ood_table: pd.DataFrame, methods: dict, scaler: ModeScaler):
    """Evaluate all methods on OOD data (no retraining)."""
    # Use ALL OOD data as test (no train split — models are pre-trained)
    bounds_lo, bounds_hi = _compute_data_bounds(ood_table)

    n_total = len(ood_table)
    print(f"OOD data: {n_total} samples")
    print()

    modes = ["M0", "M1", "M2"]
    all_results = {}

    for mode in modes:
        subset = ood_table[ood_table["mode"] == mode]
        if len(subset) == 0:
            print(f"  {mode}: no OOD data, skipping")
            continue

        s, u, sn = get_arrays(subset)
        mr = {"n_ood": len(subset)}

        # One-step for each method
        for method_name, fns in methods.items():
            if mode in fns:
                fn = fns[mode]
                pred = np.vstack([fn(s[i:i+1], u[i:i+1]) for i in range(len(s))])
                mr[f"{method_name}_onestep"] = nrmse(pred, sn)
            else:
                mr[f"{method_name}_onestep"] = np.full(9, np.nan)

        # Mode-internal rollout
        horizons = M1_ROLLOUT_HORIZONS if mode == "M1" else STD_ROLLOUT_HORIZONS

        # Use first run
        run_ids = subset["run_id"].unique()
        first_run = sorted(run_ids)[0]
        run_mask = subset["run_id"] == first_run
        s_run, u_run, sn_run = get_arrays(subset[run_mask])
        last_row = np.zeros((1, s_run.shape[1]))
        last_row[0, :9] = sn_run[-1]
        s_actual = np.vstack([s_run, last_row])

        for h in horizons:
            for method_name, fns in methods.items():
                if len(s_run) < h or mode not in fns:
                    mr[f"{method_name}_roll{h}"] = np.full(9, np.nan)
                    mr[f"{method_name}_roll{h}_horizon"] = 0
                else:
                    nr, ah = mode_internal_rollout(
                        s_run[0], u_run, s_actual, fns[mode], h,
                        bounds_lo, bounds_hi)
                    mr[f"{method_name}_roll{h}"] = nr
                    mr[f"{method_name}_roll{h}_horizon"] = ah

        all_results[mode] = mr

    # Peak metrics (M1)
    test_m1 = ood_table[ood_table["mode"] == "M1"]
    if len(test_m1) > 0 and "M1" in all_results:
        s, u, sn = get_arrays(test_m1)
        run_ids = test_m1["run_id"].values
        bounds_lo_9, bounds_hi_9 = bounds_lo, bounds_hi

        for method_name, fns in methods.items():
            if "M1" not in fns:
                continue
            fn = fns["M1"]
            unique_runs = np.unique(run_ids)
            mag_errors, time_errors = [], []

            for rid in unique_runs:
                mask = run_ids == rid
                s_run, u_run, sn_run = s[mask], u[mask], sn[mask]
                if len(s_run) < 10:
                    continue

                fz_des = u_run[0, INPUT_COLS.index("fz_des")]
                fz_actual = np.concatenate([s_run[:, FZ_IDX], sn_run[-1:, FZ_IDX]])
                actual_mag, actual_time = find_fz_overshoot(fz_actual, fz_des)

                horizon = min(100, len(s_run))
                s_cur = s_run[0].copy()
                fz_pred = [s_cur[FZ_IDX]]
                diverged = False
                for t in range(horizon - 1):
                    s_next = fn(s_cur.reshape(1, -1), u_run[t].reshape(1, -1)).ravel()
                    s_cur = _shift_delayed_state(s_cur, s_next)
                    if _is_diverged(s_cur, bounds_lo, bounds_hi):
                        diverged = True
                        break
                    fz_pred.append(s_cur[FZ_IDX])

                if diverged or len(fz_pred) < 5:
                    continue
                pred_mag, pred_time = find_fz_overshoot(np.array(fz_pred), fz_des)
                if actual_mag > 0 and pred_mag > 0:
                    mag_errors.append(abs(pred_mag - actual_mag) / max(abs(actual_mag), 1e-6))
                    if not (np.isnan(actual_time) or np.isnan(pred_time)):
                        time_errors.append(abs(pred_time - actual_time))

            if mag_errors:
                all_results["M1"][f"{method_name}_peak_mag"] = np.mean(mag_errors)
                all_results["M1"][f"{method_name}_peak_time"] = np.mean(time_errors) if time_errors else np.nan

    return all_results


def print_ood_table(results: dict):
    method_list = [("DMD", "dmd"), ("Linear", "linear"), ("Linear-d", "lineard"),
                   ("EDMD", "edmd"), ("EDMD-d", "edmdd")]

    print()
    print("=" * 95)
    print("OOD VALIDATION (trained on r=0.03m, tested on r=0.05m)")
    print("=" * 95)

    print("\n  ONE-STEP NRMSE:")
    print(f"{'':>14} {'fz':>10} {'exy':>10} {'z(aux)':>10}")
    print("-" * 48)
    for mode in ["M0", "M1", "M2"]:
        if mode not in results:
            continue
        r = results[mode]
        for method, kp in method_list:
            os = r.get(f"{kp}_onestep", np.full(9, np.nan))
            label = f"{method}-{mode}"
            print(f"{label:>14} {_fmt(os[FZ_IDX])} {_fmt(_exy(os))} {_fmt(os[Z_IDX])}")

    print("\n  MODE-INTERNAL ROLLOUT NRMSE:")
    for mode in ["M0", "M1", "M2"]:
        if mode not in results:
            continue
        r = results[mode]
        horizons = M1_ROLLOUT_HORIZONS if mode == "M1" else STD_ROLLOUT_HORIZONS
        print(f"\n  {mode}:")
        print(f"{'':>14} ", end="")
        for h in horizons:
            print(f"{'fz@'+str(h):>10} {'exy@'+str(h):>10} ", end="")
        print()
        for method, kp in method_list:
            label = f"{method}-{mode}"
            print(f"{label:>14} ", end="")
            for h in horizons:
                rv = r.get(f"{kp}_roll{h}", np.full(9, np.nan))
                ah = r.get(f"{kp}_roll{h}_horizon", 0)
                fz_v = rv[FZ_IDX] if not np.all(np.isnan(rv)) else np.nan
                exy_v = _exy(rv)
                suffix = "*" if isinstance(ah, (int, float)) and ah > 0 and ah < h else ""
                print(f"{_fmt(fz_v)}{suffix} {_fmt(exy_v)}{suffix} ", end="")
            print()

    if "M1" in results:
        r = results["M1"]
        print("\n  PEAK OVERSHOOT METRICS (M1 transition only):")
        print(f"{'':>14} {'mag err(rel)':>12} {'time err(s)':>12}")
        print("-" * 40)
        for method, kp in method_list:
            pm = r.get(f"{kp}_peak_mag", np.nan)
            pt = r.get(f"{kp}_peak_time", np.nan)
            print(f"{'  '+method:>14} {_fmt(pm, 12, 4)} {_fmt(pt, 12, 4)}")


def main():
    parser = argparse.ArgumentParser(description="OOD validation for Koopman models")
    parser.add_argument(
        "--ood-dir", type=Path, required=True,
        help="Directory with OOD preprocessed data (train_table_settle*.csv)",
    )
    parser.add_argument(
        "--model-dir", type=Path,
        default=Path("/home/andy/franka_ros2_ws/src/koopman/data/trained_models"),
        help="Directory with saved models from evaluate.py --save-models",
    )
    parser.add_argument("--t-settle", type=float, default=0.5)
    args = parser.parse_args()

    # Load pre-trained models
    models, scaler = load_models_and_scaler(args.model_dir)
    print(f"Loaded models from: {args.model_dir}")

    # Load OOD data
    ood_table = load_table(args.ood_dir, args.t_settle)
    print(f"OOD data: {len(ood_table)} samples from {args.ood_dir}")

    # Build predict functions from saved matrices
    methods = build_predict_fns(models, scaler)

    # Evaluate
    results = evaluate_ood(ood_table, methods, scaler)
    print_ood_table(results)


if __name__ == "__main__":
    main()
