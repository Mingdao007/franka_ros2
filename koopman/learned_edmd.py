"""
learned_edmd.py — Learned EDMD: MLP lifting + linear readout.

Model:
    φ_θ = MLP([s_t, u_t])           # learned nonlinear features
    z_t = [s_t, u_t, φ_θ(s_t,u_t)]  # augmented state
    s_{t+1} = W @ z_t               # linear readout

Supports both 9-dim (no delay) and 19-dim (delay-embedded) state.
Per-mode training with one-step MSE loss or EMA-balanced loss.

Usage:
    python3 learned_edmd.py [--use-delay] [--balance-loss] [--lift-dim 16]
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader

from preprocess import STATE_COLS, INPUT_COLS
from baselines import load_table, split_by_run, get_arrays, nrmse, ModeScaler


class LearnedLifting(nn.Module):
    """MLP that produces learned nonlinear features from [s, u]."""

    def __init__(self, input_dim: int = 12, lift_dim: int = 16, hidden: int = 64,
                 n_layers: int = 2):
        super().__init__()
        layers = [nn.Linear(input_dim, hidden), nn.Tanh()]
        for _ in range(n_layers - 1):
            layers += [nn.Linear(hidden, hidden), nn.Tanh()]
        layers.append(nn.Linear(hidden, lift_dim))
        self.mlp = nn.Sequential(*layers)

    def forward(self, s_u: torch.Tensor) -> torch.Tensor:
        return self.mlp(s_u)


class LearnedEDMD(nn.Module):
    """
    s_{t+1} = W @ [s, u, φ_θ(s,u)]

    Linear readout on augmented state (original + learned features).
    state_dim=9 for no-delay, 19 for delay-embedded.
    target_dim always 9 (predict next current state).
    """

    def __init__(self, state_dim: int = 9, input_dim: int = 3,
                 lift_dim: int = 16, hidden: int = 64, target_dim: int = 9,
                 n_layers: int = 2):
        super().__init__()
        self.state_dim = state_dim
        self.input_dim = input_dim
        self.lift_dim = lift_dim
        self.target_dim = target_dim

        su_dim = state_dim + input_dim
        self.lifting = LearnedLifting(su_dim, lift_dim, hidden, n_layers)
        self.W = nn.Linear(su_dim + lift_dim, target_dim, bias=False)

    def forward(self, s: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        su = torch.cat([s, u], dim=-1)
        phi = self.lifting(su)
        z = torch.cat([s, u, phi], dim=-1)
        return self.W(z)


def train_model(model: LearnedEDMD, s_train: np.ndarray, u_train: np.ndarray,
                sn_train: np.ndarray, epochs: int = 200, lr: float = 1e-3,
                batch_size: int = 512, verbose: bool = True,
                loss_weights: np.ndarray | None = None,
                balance_loss: bool = False, ema_beta: float = 0.99):
    """Train learned EDMD with one-step MSE loss.

    balance_loss: EMA-based dynamic loss normalization between fz and exy.
                  scale_exy = ema_fz / (ema_exy + eps), so exy gets same
                  gradient magnitude as fz. Other dims unweighted.
    """
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)

    s_t = torch.tensor(s_train, dtype=torch.float32, device=device)
    u_t = torch.tensor(u_train, dtype=torch.float32, device=device)
    sn_t = torch.tensor(sn_train, dtype=torch.float32, device=device)

    if loss_weights is not None and not balance_loss:
        w_t = torch.tensor(loss_weights, dtype=torch.float32, device=device)
    else:
        w_t = None

    # EMA trackers for balanced loss (3-way: fz / exy / z)
    ema_fz = 0.0
    ema_exy = 0.0
    ema_z = 0.0
    eps = 1e-8
    SCALE_CLIP = (0.1, 100.0)
    FZ = STATE_COLS.index("fz_meas")
    Z = STATE_COLS.index("z_meas")
    EXY = [STATE_COLS.index("ex"), STATE_COLS.index("ey")]

    dataset = TensorDataset(s_t, u_t, sn_t)
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)

    for epoch in range(epochs):
        model.train()
        total_loss = 0.0
        n_batches = 0
        for s_b, u_b, sn_b in loader:
            pred = model(s_b, u_b)

            if balance_loss:
                sq = (pred - sn_b) ** 2
                loss_fz = sq[:, FZ].mean()
                loss_exy = sq[:, EXY].mean()
                loss_z = sq[:, Z].mean()
                rest_idx = [i for i in range(pred.shape[1])
                            if i != FZ and i != Z and i not in EXY]
                loss_rest = sq[:, rest_idx].mean()

                # Update EMA
                ema_fz = ema_beta * ema_fz + (1 - ema_beta) * loss_fz.item()
                ema_exy = ema_beta * ema_exy + (1 - ema_beta) * loss_exy.item()
                ema_z = ema_beta * ema_z + (1 - ema_beta) * loss_z.item()

                # Dynamic scales clipped to [0.1, 100]
                scale_exy = max(SCALE_CLIP[0], min(SCALE_CLIP[1],
                                ema_fz / (ema_exy + eps)))
                scale_z = max(SCALE_CLIP[0], min(SCALE_CLIP[1],
                              ema_fz / (ema_z + eps)))
                loss = loss_fz + scale_exy * loss_exy + scale_z * loss_z + loss_rest
            elif w_t is not None:
                loss = (w_t * (pred - sn_b) ** 2).mean()
            else:
                loss = nn.functional.mse_loss(pred, sn_b)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total_loss += loss.item()
            n_batches += 1
        scheduler.step()

        if verbose and (epoch + 1) % 50 == 0:
            avg = total_loss / n_batches
            extra = ""
            if balance_loss:
                s_exy = max(SCALE_CLIP[0], min(SCALE_CLIP[1],
                            ema_fz / (ema_exy + eps)))
                s_z = max(SCALE_CLIP[0], min(SCALE_CLIP[1],
                          ema_fz / (ema_z + eps)))
                extra = f"  s_exy={s_exy:.4f} s_z={s_z:.4f}"
            print(f"  epoch {epoch+1:4d}/{epochs}: loss={avg:.8f}{extra}")

    return model


def predict_numpy(model: LearnedEDMD, s: np.ndarray, u: np.ndarray) -> np.ndarray:
    """One-step prediction, numpy in/out. Handles GPU model."""
    model.eval()
    device = next(model.parameters()).device
    with torch.no_grad():
        s_t = torch.tensor(s, dtype=torch.float32, device=device)
        u_t = torch.tensor(u, dtype=torch.float32, device=device)
        pred = model(s_t, u_t)
    return pred.cpu().numpy()


def make_learned_fn(model: LearnedEDMD, scaler: ModeScaler, mode: str):
    """Return predict_fn(s_raw, u_raw) -> s_raw_next for learned EDMD.
    Uses only current 9-dim state (no delay), same as hand-crafted EDMD."""
    def fn(s, u):
        s9 = s[:, :9] if s.ndim == 2 else s[:9].reshape(1, -1)
        s9_mean, s9_std = scaler.stats[mode][0][:9], scaler.stats[mode][1][:9]
        s9_z = (s9 - s9_mean) / s9_std
        u_z = scaler.transform_u(u, mode)
        pred_z = predict_numpy(model, s9_z, u_z)
        return scaler.inverse_target(pred_z, mode)
    return fn


def make_learned_delay_fn(model: LearnedEDMD, scaler: ModeScaler, mode: str):
    """Return predict_fn(s_raw_19d, u_raw) -> s_raw_next_9d for learned EDMD-d.
    Uses full 19-dim delayed state. Rollout shifting handled externally."""
    def fn(s, u):
        s_z = scaler.transform_s(s, mode)
        u_z = scaler.transform_u(u, mode)
        pred_z = predict_numpy(model, s_z, u_z)
        return scaler.inverse_target(pred_z, mode)
    return fn


def main():
    parser = argparse.ArgumentParser(description="Learned EDMD training")
    parser.add_argument("--data-dir", type=Path,
                        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"))
    parser.add_argument("--t-settle", type=float, default=0.5)
    parser.add_argument("--lift-dim", type=int, default=16)
    parser.add_argument("--hidden", type=int, default=64)
    parser.add_argument("--n-layers", type=int, default=2,
                        help="Number of hidden layers in MLP (default 2)")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--exy-weight", type=float, default=1.0,
                        help="Loss weight for ex/ey dims (default 1.0 = uniform)")
    parser.add_argument("--balance-loss", action="store_true",
                        help="EMA-based dynamic fz/exy loss normalization")
    parser.add_argument("--ema-beta", type=float, default=0.99,
                        help="EMA decay for balanced loss (default 0.99)")
    parser.add_argument("--use-delay", action="store_true",
                        help="Use 19-dim delay-embedded state instead of 9-dim")
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

    # Build per-dim loss weights: ex(6), ey(7) get exy_weight; rest = 1.0
    loss_weights = np.ones(9)
    loss_weights[EX_IDX] = args.exy_weight
    loss_weights[EY_IDX] = args.exy_weight

    state_dim = 19 if args.use_delay else 9
    tag = "learned EDMD-d" if args.use_delay else "learned EDMD"
    save_prefix = "learned_edmdd" if args.use_delay else "learned_edmd"

    print(f"Train runs: {train_ids} ({len(train)} samples)")
    print(f"Test runs:  {test_ids} ({len(test)} samples)")
    print(f"Model: {tag} (state_dim={state_dim})")
    print(f"Lift dim: {args.lift_dim}, Hidden: {args.hidden}, Layers: {args.n_layers}, Epochs: {args.epochs}")
    if args.balance_loss:
        print(f"Loss: EMA-balanced fz/exy (beta={args.ema_beta})")
    elif args.exy_weight != 1.0:
        print(f"Loss weights: exy={args.exy_weight}, rest=1.0")
    print()

    save_dir = args.data_dir / "learned_models"
    save_dir.mkdir(parents=True, exist_ok=True)

    for mode in ["M0", "M1", "M2"]:
        train_m = train[train["mode"] == mode]
        test_m = test[test["mode"] == mode]
        if len(train_m) < 50 or len(test_m) == 0:
            print(f"  {mode}: skipping (too few samples)")
            continue

        s_tr, u_tr, sn_tr = get_arrays(train_m)
        s_te, u_te, sn_te = get_arrays(test_m)

        # Standardize state
        if args.use_delay:
            s_tr_z = scaler.transform_s(s_tr, mode)    # full 19-dim
            s_te_z = scaler.transform_s(s_te, mode)
        else:
            s_tr_z = (s_tr[:, :9] - scaler.stats[mode][0][:9]) / scaler.stats[mode][1][:9]
            s_te_z = (s_te[:, :9] - scaler.stats[mode][0][:9]) / scaler.stats[mode][1][:9]
        u_tr_z = scaler.transform_u(u_tr, mode)
        sn_tr_z = scaler.transform_target(sn_tr, mode)
        u_te_z = scaler.transform_u(u_te, mode)

        print(f"  {mode}: training {tag} (n={len(train_m)})...")
        model = LearnedEDMD(state_dim=state_dim, input_dim=3,
                            lift_dim=args.lift_dim, hidden=args.hidden,
                            target_dim=9, n_layers=args.n_layers)
        model = train_model(model, s_tr_z, u_tr_z, sn_tr_z,
                            epochs=args.epochs, lr=args.lr,
                            loss_weights=loss_weights,
                            balance_loss=args.balance_loss,
                            ema_beta=args.ema_beta)

        # Evaluate one-step
        pred_z = predict_numpy(model, s_te_z, u_te_z)
        pred = scaler.inverse_target(pred_z, mode)
        os_nrmse = nrmse(pred, sn_te)

        exy = (os_nrmse[EX_IDX] + os_nrmse[EY_IDX]) / 2
        print(f"  {mode}: one-step NRMSE  fz={os_nrmse[FZ_IDX]:.6f}  exy={exy:.6f}")

        # Save model
        torch.save(model.state_dict(), save_dir / f"{save_prefix}_{mode}.pt")
        np.savez(save_dir / f"{save_prefix}_config_{mode}.npz",
                 lift_dim=args.lift_dim, hidden=args.hidden,
                 state_dim=state_dim, n_layers=args.n_layers)

    print(f"\nModels saved to: {save_dir}")
    scaler.save(save_dir / "scaler.npz")


if __name__ == "__main__":
    main()
