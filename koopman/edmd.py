"""
edmd.py — Per-mode EDMD with 20-dim lifting + per-mode standardization + OLS.

Usage:
    python edmd.py [--data-dir PATH] [--t-settle 0.5]

Fits per-mode EDMD, prints one-step NRMSE, saves models.
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from preprocess import STATE_COLS, INPUT_COLS
from baselines import (
    load_table, split_by_run, get_arrays, nrmse,
    ModeScaler,
)


# ---------------------------------------------------------------------------
# Lifting functions (20-dim)
# ---------------------------------------------------------------------------
# s indices: x_meas(0), y_meas(1), z_meas(2), vx(3), vy(4), vz(5),
#            ex(6), ey(7), fz_meas(8)
# u indices: x_des(0), y_des(1), fz_des(2)

def lift(s: np.ndarray, u: np.ndarray) -> np.ndarray:
    """
    Lift [s, u] -> Psi (20-dim).

    Components:
      - Linear terms (12): s (9) + u (3)
      - Squared terms (3):  fz^2, ex^2, ey^2
      - Physical cross-terms (5): ex*vx, ey*vy, z*fz, x*x_des, y*y_des
    """
    linear = np.column_stack([s, u])  # (N, 12)

    squared = np.column_stack([
        s[:, 8] ** 2,  # fz^2
        s[:, 6] ** 2,  # ex^2
        s[:, 7] ** 2,  # ey^2
    ])  # (N, 3)

    cross = np.column_stack([
        s[:, 6] * s[:, 3],  # ex * vx
        s[:, 7] * s[:, 4],  # ey * vy
        s[:, 2] * s[:, 8],  # z * fz_meas
        s[:, 0] * u[:, 0],  # x_meas * x_des
        s[:, 1] * u[:, 1],  # y_meas * y_des
    ])  # (N, 5)

    return np.column_stack([linear, squared, cross])  # (N, 20)


LIFT_DIM = 20
LIFT_NAMES = (
    list(STATE_COLS) + list(INPUT_COLS)
    + ["fz^2", "ex^2", "ey^2"]
    + ["ex*vx", "ey*vy", "z*fz", "x*x_des", "y*y_des"]
)


# ---------------------------------------------------------------------------
# EDMD fit / predict (standardized space)
# ---------------------------------------------------------------------------

def fit_edmd(s_z: np.ndarray, u_z: np.ndarray, sn_z: np.ndarray):
    """
    Fit autonomous EDMD in standardized space.
    K: (9, LIFT_DIM) — maps lifted state to next physical state.
    """
    psi = lift(s_z, u_z)  # (N, 20)
    K_T, _, _, _ = np.linalg.lstsq(psi, sn_z, rcond=None)
    K = K_T.T  # (9, 20)
    return K


def edmd_predict(s_z: np.ndarray, u_z: np.ndarray, K: np.ndarray) -> np.ndarray:
    """One-step EDMD prediction in standardized space."""
    psi = lift(s_z, u_z)
    return psi @ K.T


# ---------------------------------------------------------------------------
# Evaluate
# ---------------------------------------------------------------------------

def evaluate_edmd(table: pd.DataFrame, test_frac: float = 0.25):
    """Fit and evaluate per-mode EDMD with per-mode standardization."""
    train, test, train_ids, test_ids = split_by_run(table, test_frac)

    scaler = ModeScaler()
    scaler.fit(train)

    print(f"Train runs: {train_ids} ({len(train)} samples)")
    print(f"Test runs:  {test_ids} ({len(test)} samples)")
    print()

    modes = ["M0", "M1", "M2"]
    models = {}
    results = {}

    for mode in modes:
        train_m = train[train["mode"] == mode]
        test_m = test[test["mode"] == mode]

        if len(test_m) == 0:
            print(f"  {mode}: no test data, skipping")
            continue

        s_test, u_test, sn_test = get_arrays(test_m)

        if len(train_m) >= LIFT_DIM and mode in scaler.stats:
            s_tr, u_tr, sn_tr = get_arrays(train_m)
            s_tr_z = scaler.transform_s(s_tr, mode)
            u_tr_z = scaler.transform_u(u_tr, mode)
            sn_tr_z = scaler.transform_s(sn_tr, mode)

            K = fit_edmd(s_tr_z, u_tr_z, sn_tr_z)
            models[mode] = K

            s_te_z = scaler.transform_s(s_test, mode)
            u_te_z = scaler.transform_u(u_test, mode)
            pred_z = edmd_predict(s_te_z, u_te_z, K)
            pred = scaler.inverse_s(pred_z, mode)
            edmd_nrmse = nrmse(pred, sn_test)
        else:
            edmd_nrmse = np.full(9, np.nan)
            print(f"  {mode}: too few training samples ({len(train_m)}) for EDMD (need {LIFT_DIM})")

        results[mode] = {
            "edmd_nrmse": edmd_nrmse,
            "n_train": len(train_m),
            "n_test": len(test_m),
        }

    return results, models, scaler, train_ids, test_ids


def print_results(results: dict):
    fz_idx = STATE_COLS.index("fz_meas")
    z_idx = STATE_COLS.index("z_meas")

    print(f"{'Mode':<6} {'N_train':>8} {'N_test':>7}  "
          f"{'EDMD fz':>10} {'EDMD z':>10}")
    print("-" * 50)
    for mode, r in results.items():
        print(f"{mode:<6} {r['n_train']:>8} {r['n_test']:>7}  "
              f"{r['edmd_nrmse'][fz_idx]:>10.6f} "
              f"{r['edmd_nrmse'][z_idx]:>10.6f}")


def main():
    parser = argparse.ArgumentParser(description="Koopman EDMD")
    parser.add_argument(
        "--data-dir", type=Path,
        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"),
    )
    parser.add_argument("--t-settle", type=float, default=0.5)
    args = parser.parse_args()

    table = load_table(args.data_dir, args.t_settle)
    results, models, scaler, train_ids, test_ids = evaluate_edmd(table)
    print_results(results)

    # Save models and scaler
    save_dict = {"train_ids": train_ids, "test_ids": test_ids}
    for name, K in models.items():
        save_dict[f"edmd_K_{name}"] = K
    out_path = args.data_dir / f"edmd_models_settle{args.t_settle}.npz"
    np.savez(out_path, **save_dict)
    print(f"\nSaved models: {out_path}")

    scaler_path = args.data_dir / f"scaler_settle{args.t_settle}.npz"
    scaler.save(scaler_path)


if __name__ == "__main__":
    main()
