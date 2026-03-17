"""
evaluate_narx.py — Compare Linear-d, EDMD-d, and NARX-d on M1 rollout.

All three use the same 19-dim delay-embedded state and the same standardization.
Standardization: current 9-dim pooled with target, delay 10-dim separate, input 3-dim separate.

Usage:
    /home/andy/miniconda3/envs/isaaclab/bin/python evaluate_narx.py
"""

from pathlib import Path
import numpy as np
import torch

from preprocess import STATE_COLS, INPUT_COLS
from baselines import load_table, split_by_run, get_arrays, nrmse, ModeScaler
from narx import NARX, predict_narx, get_arrays_delayed, DELAY_COLS

FZ_IDX = STATE_COLS.index("fz_meas")
EX_IDX = STATE_COLS.index("ex")
EY_IDX = STATE_COLS.index("ey")

_DELAY_SRC = [STATE_COLS.index(c) for c in DELAY_COLS]

M1_HORIZONS = [5, 10, 20, 50, 100]


class DelayedScaler:
    """Standardize 19-dim delayed state consistently with ModeScaler.

    Current 9 dims: pooled stats from ModeScaler (same as s_t and s_{t+1}).
    Delay 10 dims: computed separately from training data.
    Input 3 dims: from ModeScaler.
    Target 9 dims: same pooled stats as current 9 dims.
    """

    def __init__(self, scaler: ModeScaler, train: dict):
        self.stats = {}
        for mode in ["M0", "M1", "M2"]:
            if mode not in scaler.stats:
                continue
            s9_mean, s9_std, u_mean, u_std = scaler.stats[mode]

            # Compute delay stats from training table
            train_m = train[train["mode"] == mode]
            s19, _, _ = get_arrays_delayed(train_m)
            delay = s19[:, 9:]  # 10-dim
            d_mean = delay.mean(axis=0)
            d_std = delay.std(axis=0)
            d_std[d_std < 1e-12] = 1.0

            # Full 19-dim stats
            s19_mean = np.concatenate([s9_mean, d_mean])
            s19_std = np.concatenate([s9_std, d_std])

            self.stats[mode] = {
                "s_mean": s19_mean, "s_std": s19_std,
                "u_mean": u_mean, "u_std": u_std,
                "sn_mean": s9_mean, "sn_std": s9_std,  # target = pooled current
            }

    def transform_s(self, s, mode):
        st = self.stats[mode]
        return (s - st["s_mean"]) / st["s_std"]

    def transform_u(self, u, mode):
        st = self.stats[mode]
        return (u - st["u_mean"]) / st["u_std"]

    def transform_sn(self, sn, mode):
        st = self.stats[mode]
        return (sn - st["sn_mean"]) / st["sn_std"]

    def inverse_sn(self, sn_z, mode):
        st = self.stats[mode]
        return sn_z * st["sn_std"] + st["sn_mean"]


def lift_delayed(s_z, u_z):
    """Lift 19-dim delayed state + 3-dim input to 30-dim.
    Same lifting as edmd.py but on 19-dim input."""
    nl = np.column_stack([
        s_z[:, 8] ** 2,
        s_z[:, 6] ** 2,
        s_z[:, 7] ** 2,
        s_z[:, 6] * s_z[:, 3],
        s_z[:, 7] * s_z[:, 4],
        s_z[:, 2] * s_z[:, 8],
        s_z[:, 0] * u_z[:, 0],
        s_z[:, 1] * u_z[:, 1],
    ])
    return np.column_stack([s_z, u_z, nl])  # (N, 30)


def shift_delayed_state(s19, s_new_9):
    """Shift 19-dim delayed state in raw space."""
    out = np.zeros(19)
    out[:9] = s_new_9
    out[14:19] = s19[9:14]
    for i, src in enumerate(_DELAY_SRC):
        out[9 + i] = s19[src]
    return out


def rollout_multi_run(test_m, fn, horizon):
    """Rollout over all test runs, return mean fz and exy NRMSE."""
    fz_all, exy_all = [], []
    for rid in test_m["run_id"].unique():
        run_mask = test_m["run_id"] == rid
        s_run, u_run, sn_run = get_arrays_delayed(test_m[run_mask])
        if len(s_run) < horizon:
            continue

        s_cur = s_run[0].copy()
        preds = []
        diverged = False
        for t in range(horizon):
            s_next_9 = fn(s_cur.reshape(1, -1), u_run[t].reshape(1, -1)).ravel()
            preds.append(s_next_9)
            s_cur = shift_delayed_state(s_cur, s_next_9)
            if np.any(np.abs(s_cur[:9]) > 1e6):
                diverged = True
                break

        if diverged or len(preds) < horizon:
            fz_all.append(np.nan)
            exy_all.append(np.nan)
            continue

        preds = np.array(preds)
        actual = sn_run[:horizon]
        os = nrmse(preds, actual)
        fz_all.append(os[FZ_IDX])
        exy_all.append((os[EX_IDX] + os[EY_IDX]) / 2)

    fz_valid = [v for v in fz_all if not np.isnan(v)]
    exy_valid = [v for v in exy_all if not np.isnan(v)]
    n_div = sum(1 for v in fz_all if np.isnan(v))
    if fz_valid:
        return np.mean(fz_valid), np.mean(exy_valid), n_div
    return np.nan, np.nan, n_div


def main():
    data_dir = Path("/home/andy/franka_ros2_ws/src/koopman/data")
    table = load_table(data_dir, 0.5)
    train, test, train_ids, test_ids = split_by_run(table)

    scaler = ModeScaler()
    scaler.fit(train)
    dscaler = DelayedScaler(scaler, train)

    print(f"Train: {train_ids} ({len(train)})")
    print(f"Test:  {test_ids} ({len(test)})")
    print()

    mode = "M1"
    train_m = train[train["mode"] == mode]
    test_m = test[test["mode"] == mode]

    s_tr, u_tr, sn_tr = get_arrays_delayed(train_m)
    s_te, u_te, sn_te = get_arrays_delayed(test_m)

    s_tr_z = dscaler.transform_s(s_tr, mode)
    u_tr_z = dscaler.transform_u(u_tr, mode)
    sn_tr_z = dscaler.transform_sn(sn_tr, mode)
    s_te_z = dscaler.transform_s(s_te, mode)
    u_te_z = dscaler.transform_u(u_te, mode)

    # --- Linear-d: [s̃(19), u(3)] -> s_next(9), OLS ---
    X_tr = np.hstack([s_tr_z, u_tr_z])
    W_lin, _, _, _ = np.linalg.lstsq(X_tr, sn_tr_z, rcond=None)

    def lineard_fn(s, u):
        s_z = dscaler.transform_s(s, mode)
        u_z = dscaler.transform_u(u, mode)
        pred_z = np.hstack([s_z, u_z]) @ W_lin
        return dscaler.inverse_sn(pred_z, mode)

    # --- EDMD-d: [s̃(19), u(3), φ_nl(8)] -> s_next(9), OLS ---
    psi_tr = lift_delayed(s_tr_z, u_tr_z)
    W_edmd, _, _, _ = np.linalg.lstsq(psi_tr, sn_tr_z, rcond=None)

    def edmdd_fn(s, u):
        s_z = dscaler.transform_s(s, mode)
        u_z = dscaler.transform_u(u, mode)
        psi = lift_delayed(s_z, u_z)
        pred_z = psi @ W_edmd
        return dscaler.inverse_sn(pred_z, mode)

    # --- NARX-d: MLP([s̃(19), u(3)]) -> s_next(9) ---
    narx_dir = data_dir / "narx_models"
    cfg = np.load(narx_dir / f"narx_config_{mode}.npz")
    model = NARX(input_dim=int(cfg["input_dim"]), output_dim=9,
                 hidden=int(cfg["hidden"]), n_layers=int(cfg["n_layers"]))
    model.load_state_dict(torch.load(narx_dir / f"narx_{mode}.pt",
                                     map_location="cpu", weights_only=True))
    model.eval()

    # NARX was trained with its own standardization - load those stats
    narx_s_mean = cfg["s_mean"]
    narx_s_std = cfg["s_std"]
    narx_u_mean = cfg["u_mean"]
    narx_u_std = cfg["u_std"]
    narx_sn_mean = cfg["sn_mean"]
    narx_sn_std = cfg["sn_std"]

    def narx_fn(s, u):
        s_z = (s - narx_s_mean) / narx_s_std
        u_z = (u - narx_u_mean) / narx_u_std
        pred_z = predict_narx(model, s_z, u_z)
        return pred_z * narx_sn_std + narx_sn_mean

    # --- One-step ---
    methods = [("Linear-d", lineard_fn), ("EDMD-d", edmdd_fn), ("NARX-d", narx_fn)]

    print("ONE-STEP NRMSE (M1):")
    print(f"{'Method':<12} {'fz':>10} {'exy':>10}")
    print("-" * 35)
    for name, fn in methods:
        pred = np.vstack([fn(s_te[i:i+1], u_te[i:i+1]) for i in range(len(s_te))])
        os = nrmse(pred, sn_te)
        exy = (os[EX_IDX] + os[EY_IDX]) / 2
        print(f"{name:<12} {os[FZ_IDX]:>10.6f} {exy:>10.6f}")

    # --- Rollout ---
    print(f"\nM1 ROLLOUT NRMSE (multi-run avg):")
    print(f"{'Method':<12}", end="")
    for h in M1_HORIZONS:
        print(f" {'fz@'+str(h):>8} {'exy@'+str(h):>8}", end="")
    print("  div")
    print("-" * (12 + 17 * len(M1_HORIZONS) + 5))

    for name, fn in methods:
        print(f"{name:<12}", end="")
        for h in M1_HORIZONS:
            fz, exy, n_div = rollout_multi_run(test_m, fn, h)
            if np.isnan(fz):
                print(f" {'div':>8} {'div':>8}", end="")
            else:
                sfx = "*" if n_div > 0 else ""
                print(f" {fz:>7.4f}{sfx} {exy:>7.4f}{sfx}", end="")
        _, _, total_div = rollout_multi_run(test_m, fn, 100)
        print(f"  {total_div}/{len(test_m['run_id'].unique())}")


if __name__ == "__main__":
    main()
