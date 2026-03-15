"""
baselines.py — Persistence and per-mode ARX baselines with per-mode standardization.

Usage:
    python baselines.py [--data-dir PATH] [--t-settle 0.5]

Reads train_table parquet, fits baselines, prints one-step NRMSE per mode.
Saves fitted models to data/models_settle{t}.npz.
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from preprocess import STATE_COLS, INPUT_COLS


# ---------------------------------------------------------------------------
# Shared utilities
# ---------------------------------------------------------------------------

def load_table(data_dir: Path, t_settle: float) -> pd.DataFrame:
    path = data_dir / f"train_table_settle{t_settle}.csv"
    return pd.read_csv(path)


def split_by_run(table: pd.DataFrame, test_frac: float = 0.25, seed: int = 42):
    """Split by run_id (not by row)."""
    rng = np.random.RandomState(seed)
    run_ids = sorted(table["run_id"].unique())
    n_test = max(1, int(len(run_ids) * test_frac))
    rng.shuffle(run_ids)
    test_ids = set(run_ids[:n_test])
    train_ids = set(run_ids[n_test:])
    train = table[table["run_id"].isin(train_ids)]
    test = table[table["run_id"].isin(test_ids)]
    return train, test, sorted(train_ids), sorted(test_ids)


def get_arrays(df: pd.DataFrame):
    """Extract s_t, u_t, s_next arrays from table subset."""
    s_cols = [f"s_{c}" for c in STATE_COLS]
    u_cols = [f"u_{c}" for c in INPUT_COLS]
    sn_cols = [f"sn_{c}" for c in STATE_COLS]
    s = df[s_cols].values.astype(np.float64)
    u = df[u_cols].values.astype(np.float64)
    sn = df[sn_cols].values.astype(np.float64)
    return s, u, sn


def nrmse(pred: np.ndarray, actual: np.ndarray) -> np.ndarray:
    """Per-variable NRMSE, normalized by variable range."""
    ranges = actual.max(axis=0) - actual.min(axis=0)
    ranges = np.where(ranges < 1e-12, 1.0, ranges)
    rmse = np.sqrt(np.mean((pred - actual) ** 2, axis=0))
    return rmse / ranges


# ---------------------------------------------------------------------------
# Per-mode standardization (fit on train only, reuse for test)
# ---------------------------------------------------------------------------

class ModeScaler:
    """Per-mode zero-mean unit-variance scaler. Fit on train split only."""

    def __init__(self):
        self.stats = {}  # {mode: (s_mean, s_std, u_mean, u_std)}

    def fit(self, train_df: pd.DataFrame):
        for mode in ["M0", "M1", "M2"]:
            subset = train_df[train_df["mode"] == mode]
            if len(subset) == 0:
                continue
            s, u, sn = get_arrays(subset)
            # State stats: pool s_t and s_{t+1} (both are physical state)
            all_s = np.vstack([s, sn])
            s_mean = all_s.mean(axis=0)
            s_std = all_s.std(axis=0)
            s_std = np.where(s_std < 1e-12, 1.0, s_std)
            u_mean = u.mean(axis=0)
            u_std = u.std(axis=0)
            u_std = np.where(u_std < 1e-12, 1.0, u_std)
            self.stats[mode] = (s_mean, s_std, u_mean, u_std)

    def transform_s(self, s: np.ndarray, mode: str) -> np.ndarray:
        m, sd, _, _ = self.stats[mode]
        return (s - m) / sd

    def transform_u(self, u: np.ndarray, mode: str) -> np.ndarray:
        _, _, m, sd = self.stats[mode]
        return (u - m) / sd

    def inverse_s(self, s_z: np.ndarray, mode: str) -> np.ndarray:
        m, sd, _, _ = self.stats[mode]
        return s_z * sd + m

    def save(self, path: Path):
        d = {}
        for mode, (sm, ss, um, us) in self.stats.items():
            d[f"s_mean_{mode}"] = sm
            d[f"s_std_{mode}"] = ss
            d[f"u_mean_{mode}"] = um
            d[f"u_std_{mode}"] = us
        np.savez(path, **d)

    def load(self, path: Path):
        data = np.load(path)
        for mode in ["M0", "M1", "M2"]:
            key = f"s_mean_{mode}"
            if key in data:
                self.stats[mode] = (
                    data[f"s_mean_{mode}"],
                    data[f"s_std_{mode}"],
                    data[f"u_mean_{mode}"],
                    data[f"u_std_{mode}"],
                )


# ---------------------------------------------------------------------------
# Baselines
# ---------------------------------------------------------------------------

def persistence_predict(s: np.ndarray) -> np.ndarray:
    """s(t+1) = s(t)"""
    return s.copy()


def fit_arx(s_z: np.ndarray, u_z: np.ndarray, sn_z: np.ndarray):
    """
    Fit s_z(t+1) = A @ s_z(t) + B @ u_z(t) via OLS.
    All inputs/outputs in standardized space.
    """
    X = np.column_stack([s_z, u_z])  # (N, 12)
    W, _, _, _ = np.linalg.lstsq(X, sn_z, rcond=None)
    A = W[:9].T   # (9, 9)
    B = W[9:].T   # (9, 3)
    return A, B


def arx_predict(s_z: np.ndarray, u_z: np.ndarray,
                A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Predict in standardized space."""
    return s_z @ A.T + u_z @ B.T


# ---------------------------------------------------------------------------
# Main: fit and evaluate
# ---------------------------------------------------------------------------

def evaluate_baselines(table: pd.DataFrame, test_frac: float = 0.25):
    """Fit and evaluate persistence + per-mode ARX with standardization."""
    train, test, train_ids, test_ids = split_by_run(table, test_frac)

    # Fit scaler on training data
    scaler = ModeScaler()
    scaler.fit(train)

    print(f"Train runs: {train_ids} ({len(train)} samples)")
    print(f"Test runs:  {test_ids} ({len(test)} samples)")
    print()

    modes = ["M0", "M1", "M2"]
    arx_models = {}
    results = {}

    for mode in modes:
        train_m = train[train["mode"] == mode]
        test_m = test[test["mode"] == mode]

        if len(test_m) == 0:
            print(f"  {mode}: no test data, skipping")
            continue

        s_test, u_test, sn_test = get_arrays(test_m)

        # Persistence (operates in original space, no standardization needed)
        pers_pred = persistence_predict(s_test)
        pers_nrmse = nrmse(pers_pred, sn_test)

        # Per-mode ARX with standardization
        if len(train_m) >= 12 and mode in scaler.stats:
            s_train, u_train, sn_train = get_arrays(train_m)
            s_tr_z = scaler.transform_s(s_train, mode)
            u_tr_z = scaler.transform_u(u_train, mode)
            sn_tr_z = scaler.transform_s(sn_train, mode)

            A, B = fit_arx(s_tr_z, u_tr_z, sn_tr_z)
            arx_models[mode] = (A, B)

            s_te_z = scaler.transform_s(s_test, mode)
            u_te_z = scaler.transform_u(u_test, mode)
            arx_pred_z = arx_predict(s_te_z, u_te_z, A, B)
            arx_pred = scaler.inverse_s(arx_pred_z, mode)
            arx_nrmse = nrmse(arx_pred, sn_test)
        else:
            arx_nrmse = np.full(9, np.nan)
            if len(train_m) < 12:
                print(f"  {mode}: too few training samples ({len(train_m)}) for ARX")

        results[mode] = {
            "persistence_nrmse": pers_nrmse,
            "arx_nrmse": arx_nrmse,
            "n_train": len(train_m),
            "n_test": len(test_m),
        }

    return results, arx_models, scaler, train_ids, test_ids


def print_results(results: dict):
    # Focused on fz and z
    fz_idx = STATE_COLS.index("fz_meas")
    z_idx = STATE_COLS.index("z_meas")

    print(f"{'Mode':<6} {'N_train':>8} {'N_test':>7}  "
          f"{'Pers fz':>10} {'ARX fz':>10} {'Pers z':>10} {'ARX z':>10}")
    print("-" * 70)
    for mode, r in results.items():
        print(f"{mode:<6} {r['n_train']:>8} {r['n_test']:>7}  "
              f"{r['persistence_nrmse'][fz_idx]:>10.6f} "
              f"{r['arx_nrmse'][fz_idx]:>10.6f} "
              f"{r['persistence_nrmse'][z_idx]:>10.6f} "
              f"{r['arx_nrmse'][z_idx]:>10.6f}")


def main():
    parser = argparse.ArgumentParser(description="Koopman baselines")
    parser.add_argument(
        "--data-dir", type=Path,
        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"),
    )
    parser.add_argument("--t-settle", type=float, default=0.5)
    args = parser.parse_args()

    table = load_table(args.data_dir, args.t_settle)
    results, arx_models, scaler, train_ids, test_ids = evaluate_baselines(table)
    print_results(results)

    # Save models and scaler
    save_dict = {"train_ids": train_ids, "test_ids": test_ids}
    for mode, (A, B) in arx_models.items():
        save_dict[f"arx_A_{mode}"] = A
        save_dict[f"arx_B_{mode}"] = B
    out_path = args.data_dir / f"models_settle{args.t_settle}.npz"
    np.savez(out_path, **save_dict)
    print(f"\nSaved models: {out_path}")

    scaler_path = args.data_dir / f"scaler_settle{args.t_settle}.npz"
    scaler.save(scaler_path)
    print(f"Saved scaler: {scaler_path}")


if __name__ == "__main__":
    main()
