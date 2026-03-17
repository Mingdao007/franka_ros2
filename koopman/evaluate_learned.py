"""
evaluate_learned.py — Compare Linear, EDMD, Learned EDMD (all no delay) + EDMD-d (with delay).

Loads hand-crafted models from evaluate.py --save-models,
loads learned models from learned_edmd.py,
runs one-step + rollout + peak on the same test set.

Usage:
    /home/andy/miniconda3/envs/isaaclab/bin/python evaluate_learned.py
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import torch

from preprocess import STATE_COLS, INPUT_COLS, STATE_COLS_DELAYED
from baselines import (
    load_table, split_by_run, get_arrays, nrmse, ModeScaler,
    fit_dmd, dmd_predict, fit_arx, arx_predict,
)
from edmd import fit_edmd, edmd_predict, LIFT_DIM
from learned_edmd import LearnedEDMD, predict_numpy, make_learned_fn, make_learned_delay_fn
from residual_learned_edmd import ResidualMLP, predict_residual, make_residual_fn
from evaluate import (
    FZ_IDX, Z_IDX, EX_IDX, EY_IDX, _exy, _fmt,
    M1_ROLLOUT_HORIZONS, STD_ROLLOUT_HORIZONS,
    _compute_data_bounds, _is_diverged, _shift_delayed_state,
    mode_internal_rollout, find_fz_overshoot,
    make_dmd_fn, make_linear_fn, make_edmd_nodelay_fn, make_edmd_fn,
)


def load_learned_models(model_dir: Path, scaler: ModeScaler, prefix: str = "learned_edmd"):
    """Load trained learned EDMD models and build predict functions."""
    fns = {}
    for mode in ["M0", "M1", "M2"]:
        pt_path = model_dir / f"{prefix}_{mode}.pt"
        cfg_path = model_dir / f"{prefix}_config_{mode}.npz"
        # Fallback: old naming convention (learned_config_M0.npz)
        if not cfg_path.exists():
            cfg_path = model_dir / f"learned_config_{mode}.npz"
        if not pt_path.exists():
            continue
        cfg = np.load(cfg_path)
        lift_dim = int(cfg["lift_dim"])
        hidden = int(cfg["hidden"])
        state_dim = int(cfg["state_dim"]) if "state_dim" in cfg else 9
        n_layers = int(cfg["n_layers"]) if "n_layers" in cfg else 2

        model = LearnedEDMD(state_dim=state_dim, input_dim=3,
                            lift_dim=lift_dim, hidden=hidden, target_dim=9,
                            n_layers=n_layers)
        model.load_state_dict(torch.load(pt_path, map_location="cpu", weights_only=True))
        model.eval()
        if state_dim == 19:
            fns[mode] = make_learned_delay_fn(model, scaler, mode)
        else:
            fns[mode] = make_learned_fn(model, scaler, mode)
    return fns


def run_comparison(table: pd.DataFrame, t_settle: float,
                   learned_model_dir: Path):
    """Fit Linear + EDMD, load Learned EDMD + EDMD-d, evaluate all."""
    train, test, train_ids, test_ids = split_by_run(table)
    scaler = ModeScaler()
    scaler.fit(train)
    bounds_lo, bounds_hi = _compute_data_bounds(table)

    print(f"Train runs: {train_ids} ({len(train)} samples)")
    print(f"Test runs:  {test_ids} ({len(test)} samples)")
    print()

    # Load learned models (no-delay and delay variants)
    learned_fns = load_learned_models(learned_model_dir, scaler, prefix="learned_edmd")
    learnedd_fns = load_learned_models(learned_model_dir, scaler, prefix="learned_edmdd")

    # Load residual learned models
    reslearn_fns = {}
    for mode in ["M0", "M1", "M2"]:
        pt_path = learned_model_dir / f"res_mlp_{mode}.pt"
        cfg_path = learned_model_dir / f"res_config_{mode}.npz"
        k_path = learned_model_dir / f"K_edmd_{mode}.npy"
        if pt_path.exists() and k_path.exists():
            cfg = np.load(cfg_path)
            mlp = ResidualMLP(input_dim=int(cfg["input_dim"]),
                              output_dim=9,
                              hidden=int(cfg["hidden"]),
                              n_layers=int(cfg["n_layers"]))
            mlp.load_state_dict(torch.load(pt_path, map_location="cpu",
                                           weights_only=True))
            mlp.eval()
            K_edmd = np.load(k_path)
            residual_gain = float(cfg["residual_gain"]) if "residual_gain" in cfg else 1.0
            reslearn_fns[mode] = make_residual_fn(K_edmd, mlp, scaler, mode,
                                                  residual_gain=residual_gain)

    # Method registry: {name: {mode: predict_fn}}
    all_methods = {
        "linear": {},
        "edmd": {},
        "learned": learned_fns,
        "edmdd": {},
        "learnedd": learnedd_fns,
        "reslearn": reslearn_fns,
    }

    modes = ["M0", "M1", "M2"]
    all_results = {}

    for mode in modes:
        train_m = train[train["mode"] == mode]
        test_m = test[test["mode"] == mode]
        if len(test_m) == 0:
            continue

        s_tr, u_tr, sn_tr = get_arrays(train_m)
        s_te, u_te, sn_te = get_arrays(test_m)
        mr = {"n_train": len(train_m), "n_test": len(test_m)}

        can_fit = len(train_m) >= 12 and mode in scaler.stats
        if not can_fit:
            continue

        s_tr_z = scaler.transform_s(s_tr, mode)
        u_tr_z = scaler.transform_u(u_tr, mode)
        sn_tr_z = scaler.transform_target(sn_tr, mode)

        # --- Linear (9d) ---
        A_lin, B_lin = fit_arx(s_tr_z[:, :9], u_tr_z, sn_tr_z)
        lin_fn = make_linear_fn(A_lin, B_lin, scaler, mode)
        all_methods["linear"][mode] = lin_fn

        # --- EDMD (9d, no delay) ---
        if len(train_m) >= 20:
            K_nd = fit_edmd(s_tr_z[:, :9], u_tr_z, sn_tr_z, alpha=0.0)
            all_methods["edmd"][mode] = make_edmd_nodelay_fn(K_nd, scaler, mode)

        # --- EDMD-d (19d, with delay) ---
        if len(train_m) >= LIFT_DIM:
            K_d = fit_edmd(s_tr_z, u_tr_z, sn_tr_z, alpha=0.0)
            all_methods["edmdd"][mode] = make_edmd_fn(K_d, scaler, mode)

        # --- One-step for all methods ---
        for mname, fns in all_methods.items():
            if mode in fns:
                fn = fns[mode]
                pred = np.vstack([fn(s_te[i:i+1], u_te[i:i+1]) for i in range(len(s_te))])
                mr[f"{mname}_onestep"] = nrmse(pred, sn_te)
            else:
                mr[f"{mname}_onestep"] = np.full(9, np.nan)

        # --- Rollout (aggregate across all test runs) ---
        horizons = M1_ROLLOUT_HORIZONS if mode == "M1" else STD_ROLLOUT_HORIZONS
        test_runs = test_m["run_id"].unique()

        for h in horizons:
            for mname, fns in all_methods.items():
                if mode not in fns:
                    mr[f"{mname}_roll{h}"] = np.full(9, np.nan)
                    mr[f"{mname}_roll{h}_horizon"] = 0
                    continue
                # Collect rollout NRMSE from each test run, then average
                run_nrmses = []
                run_horizons = []
                for rid in test_runs:
                    run_mask = test_m["run_id"] == rid
                    s_run, u_run, sn_run = get_arrays(test_m[run_mask])
                    if len(s_run) < h:
                        continue
                    last_row = np.zeros((1, s_run.shape[1]))
                    last_row[0, :9] = sn_run[-1]
                    s_actual = np.vstack([s_run, last_row])
                    nr, ah = mode_internal_rollout(
                        s_run[0], u_run, s_actual, fns[mode], h,
                        bounds_lo, bounds_hi)
                    if not np.all(np.isnan(nr)):
                        run_nrmses.append(nr)
                        run_horizons.append(ah)
                if run_nrmses:
                    mr[f"{mname}_roll{h}"] = np.mean(run_nrmses, axis=0)
                    mr[f"{mname}_roll{h}_horizon"] = min(run_horizons)
                else:
                    mr[f"{mname}_roll{h}"] = np.full(9, np.nan)
                    mr[f"{mname}_roll{h}_horizon"] = 0

        all_results[mode] = mr

    # --- Peak (M1) ---
    test_m1 = test[test["mode"] == "M1"]
    if len(test_m1) > 0 and "M1" in all_results:
        s, u, sn = get_arrays(test_m1)
        run_ids = test_m1["run_id"].values

        for mname, fns in all_methods.items():
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
                    # Penalize diverged runs instead of silently skipping
                    mag_errors.append(1.0)   # 100% relative error
                    time_errors.append(1.0)   # 1s timing error
                    continue
                pred_mag, pred_time = find_fz_overshoot(np.array(fz_pred), fz_des)
                if actual_mag > 0 and pred_mag > 0:
                    mag_errors.append(abs(pred_mag - actual_mag) / max(abs(actual_mag), 1e-6))
                    if not (np.isnan(actual_time) or np.isnan(pred_time)):
                        time_errors.append(abs(pred_time - actual_time))

            if mag_errors:
                all_results["M1"][f"{mname}_peak_mag"] = np.mean(mag_errors)
                all_results["M1"][f"{mname}_peak_time"] = np.mean(time_errors) if time_errors else np.nan

    return all_results


def print_table(results: dict):
    method_list = [("Linear", "linear"), ("EDMD", "edmd"),
                   ("Learned", "learned"), ("EDMD-d", "edmdd"),
                   ("Learned-d", "learnedd"), ("ResLrn-d", "reslearn")]

    print()
    print("=" * 95)
    print("LINEAR vs EDMD vs LEARNED vs EDMD-d vs LEARNED-d vs ResLrn-d")
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
        print("\n  PEAK OVERSHOOT (M1):")
        print(f"{'':>14} {'mag err':>10} {'time err':>10}")
        print("-" * 36)
        for method, kp in method_list:
            pm = r.get(f"{kp}_peak_mag", np.nan)
            pt = r.get(f"{kp}_peak_time", np.nan)
            print(f"{'  '+method:>14} {_fmt(pm, 10, 4)} {_fmt(pt, 10, 4)}")

    print()


def compute_m1_score(results: dict, method_key: str = "reslearn") -> float:
    """Compute combined M1 score for BO objective. Lower is better.
    score = mean(fz@5,10,20) + mean(exy@5,10,20) + divergence_penalty
    """
    if "M1" not in results:
        return 200.0

    r = results["M1"]
    horizons = [5, 10, 20]
    fz_vals, exy_vals = [], []
    for h in horizons:
        rv = r.get(f"{method_key}_roll{h}", np.full(9, np.nan))
        if np.all(np.isnan(rv)):
            return 200.0
        fz_vals.append(rv[FZ_IDX])
        exy_vals.append(_exy(rv))

    fz_score = np.mean(fz_vals)
    exy_score = np.mean(exy_vals)

    h20_horizon = r.get(f"{method_key}_roll20_horizon", 0)
    div_penalty = 100.0 if (isinstance(h20_horizon, (int, float)) and h20_horizon > 0
                            and h20_horizon < 20) else 0.0

    return fz_score + exy_score + div_penalty


def main():
    parser = argparse.ArgumentParser(description="Learned EDMD comparison")
    parser.add_argument("--data-dir", type=Path,
                        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"))
    parser.add_argument("--learned-dir", type=Path,
                        default=Path("/home/andy/franka_ros2_ws/src/koopman/data/learned_models"))
    parser.add_argument("--t-settle", type=float, default=0.5)
    args = parser.parse_args()

    table = load_table(args.data_dir, args.t_settle)
    results = run_comparison(table, args.t_settle, args.learned_dir)
    print_table(results)


if __name__ == "__main__":
    main()
