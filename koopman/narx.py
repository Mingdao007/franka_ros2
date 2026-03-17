"""
narx.py — NARX baseline: MLP directly maps [s̃_t, u_t] -> s_{t+1}.

Same delay-embedded input as EDMD-d but no Koopman structure.
Uses the same 19-dim delayed state + 3-dim input.
Trained with one-step MSE, per-mode standardization.

Usage:
    python narx.py [--hidden 64] [--n-layers 2] [--epochs 200]
"""

import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

from preprocess import STATE_COLS, INPUT_COLS
from baselines import load_table, split_by_run, get_arrays, nrmse, ModeScaler

# Delay embedding columns (same as EDMD-d)
DELAY_COLS = ["z_meas", "vz", "fz_meas", "ex", "ey"]
LAG_COLS = [f"{c}_lag{l}" for l in [1, 2] for c in DELAY_COLS]
STATE_COLS_DELAYED = STATE_COLS + LAG_COLS  # 9 + 10 = 19


def get_arrays_delayed(df):
    """Extract 19-dim delayed state, 3-dim input, 9-dim target."""
    s_cols = [f"s_{c}" for c in STATE_COLS_DELAYED]
    u_cols = [f"u_{c}" for c in INPUT_COLS]
    sn_cols = [f"sn_{c}" for c in STATE_COLS]
    s = df[s_cols].values.astype(np.float64)
    u = df[u_cols].values.astype(np.float64)
    sn = df[sn_cols].values.astype(np.float64)
    return s, u, sn


class NARX(nn.Module):
    """MLP: [s̃(19), u(3)] -> s_next(9). No Koopman structure."""

    def __init__(self, input_dim=22, output_dim=9, hidden=64, n_layers=2):
        super().__init__()
        layers = [nn.Linear(input_dim, hidden), nn.Tanh()]
        for _ in range(n_layers - 1):
            layers += [nn.Linear(hidden, hidden), nn.Tanh()]
        layers.append(nn.Linear(hidden, output_dim))
        self.mlp = nn.Sequential(*layers)

    def forward(self, x):
        return self.mlp(x)


def train_narx(model, s_z, u_z, sn_z, epochs=200, lr=1e-3,
               batch_size=4096, verbose=True):
    """Train NARX with one-step MSE."""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    use_amp = device.type == "cuda"
    model.to(device)

    su = torch.tensor(np.hstack([s_z, u_z]), dtype=torch.float32, device=device)
    target = torch.tensor(sn_z, dtype=torch.float32, device=device)
    N = su.shape[0]

    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    grad_scaler = torch.amp.GradScaler(enabled=use_amp)

    for epoch in range(epochs):
        model.train()
        total_loss, n_batches = 0.0, 0
        perm = torch.randperm(N, device=device)
        for start in range(0, N, batch_size):
            idx = perm[start:start + batch_size]
            su_b = su[idx]
            t_b = target[idx]
            with torch.autocast(device_type="cuda", dtype=torch.bfloat16, enabled=use_amp):
                pred = model(su_b)
                loss = nn.functional.mse_loss(pred, t_b)
            optimizer.zero_grad()
            grad_scaler.scale(loss).backward()
            grad_scaler.step(optimizer)
            grad_scaler.update()
            total_loss += loss.item()
            n_batches += 1
        scheduler.step()

        if verbose and (epoch + 1) % 50 == 0:
            print(f"  epoch {epoch+1:4d}/{epochs}: loss={total_loss/n_batches:.8f}")

    return model


def predict_narx(model, s_z, u_z):
    """One-step prediction, numpy in/out."""
    model.eval()
    device = next(model.parameters()).device
    with torch.no_grad():
        su = torch.tensor(np.hstack([s_z, u_z]), dtype=torch.float32, device=device)
        pred = model(su)
    return pred.cpu().numpy()


def make_narx_fn(model, s_mean, s_std, u_mean, u_std, sn_mean, sn_std):
    """Return predict_fn(s_raw_19d, u_raw) -> s_raw_next_9d."""
    def fn(s, u):
        s_z = (s - s_mean) / s_std
        u_z = (u - u_mean) / u_std
        pred_z = predict_narx(model, s_z, u_z)
        return pred_z * sn_std + sn_mean
    return fn


def main():
    parser = argparse.ArgumentParser(description="NARX baseline training")
    parser.add_argument("--data-dir", type=Path,
                        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"))
    parser.add_argument("--t-settle", type=float, default=0.5)
    parser.add_argument("--hidden", type=int, default=64)
    parser.add_argument("--n-layers", type=int, default=2)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    table = load_table(args.data_dir, args.t_settle)
    train, test, train_ids, test_ids = split_by_run(table)

    scaler = ModeScaler()
    scaler.fit(train)

    FZ_IDX = STATE_COLS.index("fz_meas")
    EX_IDX = STATE_COLS.index("ex")
    EY_IDX = STATE_COLS.index("ey")

    print(f"Train runs: {train_ids} ({len(train)} samples)")
    print(f"Test runs:  {test_ids} ({len(test)} samples)")
    print(f"NARX: hidden={args.hidden}, layers={args.n_layers}, epochs={args.epochs}")
    print()

    save_dir = args.data_dir / "narx_models"
    save_dir.mkdir(parents=True, exist_ok=True)

    for mode in ["M0", "M1", "M2"]:
        train_m = train[train["mode"] == mode]
        test_m = test[test["mode"] == mode]
        if len(train_m) < 50 or len(test_m) == 0:
            print(f"  {mode}: skipping")
            continue

        # Use 19-dim delayed state (same history window as EDMD-d)
        s_tr, u_tr, sn_tr = get_arrays_delayed(train_m)
        s_te, u_te, sn_te = get_arrays_delayed(test_m)

        # Per-mode standardization (manual, since scaler only handles 9-dim)
        s_mean = s_tr.mean(axis=0)
        s_std = s_tr.std(axis=0)
        s_std[s_std < 1e-12] = 1.0
        u_mean = u_tr.mean(axis=0)
        u_std = u_tr.std(axis=0)
        u_std[u_std < 1e-12] = 1.0
        # Pool s_t[:9] and sn for target standardization (same as ModeScaler)
        pooled = np.vstack([s_tr[:, :9], sn_tr])
        sn_mean = pooled.mean(axis=0)
        sn_std = pooled.std(axis=0)
        sn_std[sn_std < 1e-12] = 1.0

        s_tr_z = (s_tr - s_mean) / s_std
        u_tr_z = (u_tr - u_mean) / u_std
        sn_tr_z = (sn_tr - sn_mean) / sn_std
        s_te_z = (s_te - s_mean) / s_std
        u_te_z = (u_te - u_mean) / u_std

        print(f"  {mode}: training NARX (n={len(train_m)}, input_dim={s_tr_z.shape[1]+u_tr_z.shape[1]})...")
        input_dim = s_tr_z.shape[1] + u_tr_z.shape[1]  # 19 + 3 = 22
        model = NARX(input_dim=input_dim, output_dim=9,
                     hidden=args.hidden, n_layers=args.n_layers)
        model = train_narx(model, s_tr_z, u_tr_z, sn_tr_z,
                           epochs=args.epochs, lr=args.lr)

        # Evaluate one-step
        pred_z = predict_narx(model, s_te_z, u_te_z)
        pred = pred_z * sn_std + sn_mean
        os_nrmse = nrmse(pred, sn_te)

        exy = (os_nrmse[EX_IDX] + os_nrmse[EY_IDX]) / 2
        print(f"  {mode}: one-step NRMSE  fz={os_nrmse[FZ_IDX]:.6f}  exy={exy:.6f}")

        # Save
        torch.save(model.state_dict(), save_dir / f"narx_{mode}.pt")
        np.savez(save_dir / f"narx_config_{mode}.npz",
                 hidden=args.hidden, n_layers=args.n_layers,
                 input_dim=input_dim,
                 s_mean=s_mean, s_std=s_std,
                 u_mean=u_mean, u_std=u_std,
                 sn_mean=sn_mean, sn_std=sn_std)

    print(f"\nModels saved to: {save_dir}")


if __name__ == "__main__":
    main()
