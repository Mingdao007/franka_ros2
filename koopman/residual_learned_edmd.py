"""
residual_learned_edmd.py — Residual Learned EDMD-d with simulation error.

Model:
    s_{t+1} = EDMD-d(s̃_t, u_t) + r_θ(s̃_t, u_t)

Hand-crafted EDMD-d provides a stable base prediction.
MLP r_θ learns the residual.

Training modes:
    --one-step:   standard one-step MSE on residuals (default)
    --sim-error K: multi-step simulation error, rollout K steps and
                   backprop through entire trajectory

Usage:
    python3 residual_learned_edmd.py --balance-loss
    python3 residual_learned_edmd.py --balance-loss --sim-error 5
"""

import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

from preprocess import STATE_COLS, INPUT_COLS, DELAY_COLS
from baselines import load_table, split_by_run, get_arrays, nrmse, ModeScaler
from edmd import fit_edmd, edmd_predict, lift


# Indices for delay shifting (same as evaluate.py)
_DELAY_SRC_INDICES = [STATE_COLS.index(c) for c in DELAY_COLS]


class ResidualMLP(nn.Module):
    """MLP that predicts residual correction r_θ(s̃, u) -> Δs (9-dim)."""

    def __init__(self, input_dim: int = 22, output_dim: int = 9,
                 hidden: int = 128, n_layers: int = 3):
        super().__init__()
        layers = [nn.Linear(input_dim, hidden), nn.Tanh()]
        for _ in range(n_layers - 1):
            layers += [nn.Linear(hidden, hidden), nn.Tanh()]
        layers.append(nn.Linear(hidden, output_dim))
        self.mlp = nn.Sequential(*layers)

    def forward(self, s_u: torch.Tensor) -> torch.Tensor:
        return self.mlp(s_u)


def _lift_torch(s: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
    """PyTorch version of edmd.lift() for backprop through rollout.
    s: (B, 19), u: (B, 3) -> (B, 30)"""
    # Nonlinear features (current-time only, indices into 9-dim current)
    fz2 = s[:, 8:9] ** 2
    ex2 = s[:, 6:7] ** 2
    ey2 = s[:, 7:8] ** 2
    ex_vx = s[:, 6:7] * s[:, 3:4]
    ey_vy = s[:, 7:8] * s[:, 4:5]
    z_fz = s[:, 2:3] * s[:, 8:9]
    x_xdes = s[:, 0:1] * u[:, 0:1]
    y_ydes = s[:, 1:2] * u[:, 1:2]
    nl = torch.cat([fz2, ex2, ey2, ex_vx, ey_vy, z_fz, x_xdes, y_ydes], dim=-1)
    return torch.cat([s, u, nl], dim=-1)  # (B, 30)


def _build_shift_params(scaler, mode, device):
    """Precompute scale/offset tensors for standardized delay shifting.

    When shifting current→lag1, we need:
      val_lag1_z = val_current_z * (std_current / std_lag1) + (mean_current - mean_lag1) / std_lag1
    Similarly lag1→lag2.
    Returns (scale_c2l1, offset_c2l1, scale_l1l2, offset_l1l2), each (5,).
    """
    m, sd = scaler.stats[mode][0], scaler.stats[mode][1]
    # DELAY_SRC_INDICES maps current-state indices [2,5,8,6,7] to delay cols
    # lag1 = dims 9:14, lag2 = dims 14:19
    cur_idx = _DELAY_SRC_INDICES  # [2, 5, 8, 6, 7]

    scale_c2l1 = sd[cur_idx] / sd[9:14]
    offset_c2l1 = (m[cur_idx] - m[9:14]) / sd[9:14]

    scale_l1l2 = sd[9:14] / sd[14:19]
    offset_l1l2 = (m[9:14] - m[14:19]) / sd[14:19]

    return (torch.tensor(scale_c2l1, dtype=torch.float32, device=device),
            torch.tensor(offset_c2l1, dtype=torch.float32, device=device),
            torch.tensor(scale_l1l2, dtype=torch.float32, device=device),
            torch.tensor(offset_l1l2, dtype=torch.float32, device=device))


def _shift_delayed_torch(s_19, s_new_9, shift_params):
    """Shift delay state in standardized space with correct rescaling.
    s_19: (B, 19), s_new_9: (B, 9) -> (B, 19)
    shift_params: (scale_c2l1, offset_c2l1, scale_l1l2, offset_l1l2) from _build_shift_params.
    """
    scale_c2l1, offset_c2l1, scale_l1l2, offset_l1l2 = shift_params
    # lag2 = old lag1, rescaled from lag1-stats to lag2-stats
    lag2 = s_19[:, 9:14] * scale_l1l2 + offset_l1l2
    # lag1 = old current's delay-relevant vars, rescaled from current-stats to lag1-stats
    lag1 = s_19[:, _DELAY_SRC_INDICES] * scale_c2l1 + offset_c2l1
    return torch.cat([s_new_9, lag1, lag2], dim=-1)


def _balanced_loss(sq, ema_fz, ema_exy, ema_z, ema_beta, eps=1e-8,
                   clip=(0.1, 100.0)):
    """Compute 3-way balanced loss. Returns loss, updated EMAs."""
    FZ = STATE_COLS.index("fz_meas")
    Z = STATE_COLS.index("z_meas")
    EXY = [STATE_COLS.index("ex"), STATE_COLS.index("ey")]

    loss_fz = sq[:, FZ].mean()
    loss_exy = sq[:, EXY].mean()
    loss_z = sq[:, Z].mean()
    rest_idx = [i for i in range(sq.shape[1]) if i != FZ and i != Z and i not in EXY]
    loss_rest = sq[:, rest_idx].mean()

    ema_fz = ema_beta * ema_fz + (1 - ema_beta) * loss_fz.item()
    ema_exy = ema_beta * ema_exy + (1 - ema_beta) * loss_exy.item()
    ema_z = ema_beta * ema_z + (1 - ema_beta) * loss_z.item()

    scale_exy = max(clip[0], min(clip[1], ema_fz / (ema_exy + eps)))
    scale_z = max(clip[0], min(clip[1], ema_fz / (ema_z + eps)))
    loss = loss_fz + scale_exy * loss_exy + scale_z * loss_z + loss_rest

    return loss, ema_fz, ema_exy, ema_z, scale_exy, scale_z


def train_residual_onestep(mlp, s_z, u_z, residuals, epochs, lr,
                           batch_size, verbose, balance_loss, ema_beta,
                           weight_decay=1e-4):
    """One-step residual training (original method)."""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    use_amp = device.type == "cuda"
    mlp.to(device)

    su_t = torch.tensor(np.hstack([s_z, u_z]), dtype=torch.float32, device=device)
    r_t = torch.tensor(residuals, dtype=torch.float32, device=device)
    N = su_t.shape[0]

    ema_fz, ema_exy, ema_z = 0.0, 0.0, 0.0

    optimizer = torch.optim.AdamW(mlp.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    grad_scaler = torch.amp.GradScaler(enabled=use_amp)

    for epoch in range(epochs):
        mlp.train()
        total_loss, n_batches = 0.0, 0
        perm = torch.randperm(N, device=device)
        for start in range(0, N, batch_size):
            idx = perm[start:start + batch_size]
            su_b = su_t[idx]
            r_b = r_t[idx]
            with torch.autocast(device_type="cuda", dtype=torch.bfloat16, enabled=use_amp):
                pred = mlp(su_b)
                if balance_loss:
                    sq = (pred - r_b) ** 2
                    loss, ema_fz, ema_exy, ema_z, s_exy, s_z_val = _balanced_loss(
                        sq, ema_fz, ema_exy, ema_z, ema_beta)
                else:
                    loss = nn.functional.mse_loss(pred, r_b)
            optimizer.zero_grad()
            grad_scaler.scale(loss).backward()
            grad_scaler.step(optimizer)
            grad_scaler.update()
            total_loss += loss.item()
            n_batches += 1
        scheduler.step()

        if verbose and (epoch + 1) % 50 == 0:
            avg = total_loss / n_batches
            print(f"  epoch {epoch+1:4d}/{epochs}: loss={avg:.8f}")

    return mlp


def build_trajectory_batches(table_mode, scaler, mode, seq_len, stride=1):
    """Build (s_seq, u_seq, sn_seq) trajectory chunks per run.
    Each chunk: consecutive seq_len steps from one run.
    stride=1 for maximum overlap (important for short M1 runs).
    Returns lists of numpy arrays, each (seq_len, dim)."""
    s_seqs, u_seqs, sn_seqs = [], [], []
    for rid in table_mode["run_id"].unique():
        run = table_mode[table_mode["run_id"] == rid]
        s, u, sn = get_arrays(run)
        s_z = scaler.transform_s(s, mode)
        u_z = scaler.transform_u(u, mode)
        sn_z = scaler.transform_target(sn, mode)
        T = len(s_z)
        for start in range(0, T - seq_len + 1, stride):
            s_seqs.append(s_z[start:start + seq_len])
            u_seqs.append(u_z[start:start + seq_len])
            sn_seqs.append(sn_z[start:start + seq_len])
    return s_seqs, u_seqs, sn_seqs


def train_residual_simerror(mlp, K_edmd, train_mode, scaler, mode,
                            sim_steps, epochs, lr, batch_size, verbose,
                            balance_loss, ema_beta, weight_decay=1e-4):
    """Simulation error training: rollout K steps, backprop through trajectory."""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    use_amp = device.type == "cuda"
    mlp.to(device)
    K_t = torch.tensor(K_edmd.T, dtype=torch.float32, device=device)  # (30, 9)

    # Precompute delay shift scale/offset for correct standardized-space shifting
    shift_params = _build_shift_params(scaler, mode, device)

    # Build trajectory chunks (stride=1 for max overlap)
    s_seqs, u_seqs, sn_seqs = build_trajectory_batches(
        train_mode, scaler, mode, sim_steps, stride=1)
    if not s_seqs:
        print(f"    WARNING: no sequences of length {sim_steps}, skipping")
        return mlp

    # Stack into tensors: (N_chunks, sim_steps, dim)
    # Cap at 50k chunks to avoid excessive iteration on large modes (M2)
    MAX_CHUNKS = 50000
    if len(s_seqs) > MAX_CHUNKS:
        print(f"    subsampling {len(s_seqs)} → {MAX_CHUNKS} chunks (MAX_CHUNKS)")
        rng = np.random.RandomState(42)
        idx = rng.choice(len(s_seqs), MAX_CHUNKS, replace=False)
        s_seqs = [s_seqs[i] for i in idx]
        u_seqs = [u_seqs[i] for i in idx]
        sn_seqs = [sn_seqs[i] for i in idx]
    S = torch.tensor(np.stack(s_seqs), dtype=torch.float32, device=device)
    U = torch.tensor(np.stack(u_seqs), dtype=torch.float32, device=device)
    SN = torch.tensor(np.stack(sn_seqs), dtype=torch.float32, device=device)
    n_chunks = S.shape[0]
    print(f"    {n_chunks} chunks, batch_size={batch_size}, "
          f"{(n_chunks + batch_size - 1) // batch_size} batches/epoch")

    ema_fz, ema_exy, ema_z = 0.0, 0.0, 0.0

    optimizer = torch.optim.AdamW(mlp.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    grad_scaler = torch.amp.GradScaler(enabled=use_amp)

    for epoch in range(epochs):
        mlp.train()
        total_loss, n_batches = 0.0, 0

        # Shuffle chunks
        perm = torch.randperm(n_chunks, device=device)
        for batch_start in range(0, n_chunks, batch_size):
            idx = perm[batch_start:batch_start + batch_size]
            s_batch = S[idx]   # (B, K, 19)
            u_batch = U[idx]   # (B, K, 3)
            sn_batch = SN[idx] # (B, K, 9)

            B = s_batch.shape[0]
            s_cur = s_batch[:, 0]  # (B, 19) — initial state

            with torch.autocast(device_type="cuda", dtype=torch.bfloat16, enabled=use_amp):
                all_sq = []
                alive = torch.ones(B, dtype=torch.bool, device=device)
                for t in range(sim_steps):
                    u_cur = u_batch[:, t]  # (B, 3)

                    # EDMD-d base prediction (differentiable via _lift_torch)
                    psi = _lift_torch(s_cur, u_cur)  # (B, 30)
                    base_pred = psi @ K_t            # (B, 9)

                    # MLP residual
                    su = torch.cat([s_cur, u_cur], dim=-1)
                    res = mlp(su)                    # (B, 9)

                    s_next_9 = base_pred + res       # (B, 9)
                    target_9 = sn_batch[:, t]        # (B, 9)

                    # Divergence guard: mark samples where any dim > 10 std
                    diverged = (s_next_9.abs() > 10.0).any(dim=1)
                    alive = alive & ~diverged

                    sq = (s_next_9 - target_9) ** 2  # (B, 9)
                    sq = sq * alive.unsqueeze(1).float()
                    all_sq.append(sq)

                    # Shift delay for next step (with correct rescaling)
                    s_cur = _shift_delayed_torch(s_cur, s_next_9, shift_params)

                # Average over time steps; only count alive samples
                all_sq = torch.stack(all_sq, dim=1)  # (B, K, 9)
                n_alive = alive.sum().clamp(min=1)
                sq_mean = (all_sq.sum(dim=(0, 1)) / (n_alive * sim_steps)).unsqueeze(0)

                if balance_loss:
                    loss, ema_fz, ema_exy, ema_z, s_exy, s_z_val = _balanced_loss(
                        sq_mean, ema_fz, ema_exy, ema_z, ema_beta)
                else:
                    loss = sq_mean.mean()

            optimizer.zero_grad()
            grad_scaler.scale(loss).backward()
            grad_scaler.unscale_(optimizer)
            nn.utils.clip_grad_norm_(mlp.parameters(), 1.0)
            grad_scaler.step(optimizer)
            grad_scaler.update()
            total_loss += loss.item()
            n_batches += 1
        scheduler.step()

        if verbose and (epoch + 1) % 50 == 0:
            avg = total_loss / n_batches
            print(f"  epoch {epoch+1:4d}/{epochs}: loss={avg:.8f}")

    return mlp


def predict_residual(mlp: ResidualMLP, s_z: np.ndarray,
                     u_z: np.ndarray) -> np.ndarray:
    """Predict residual, numpy in/out."""
    mlp.eval()
    device = next(mlp.parameters()).device
    with torch.no_grad():
        su = torch.tensor(np.hstack([s_z, u_z]), dtype=torch.float32, device=device)
        pred = mlp(su)
    return pred.cpu().numpy()


def make_residual_fn(K_edmd, mlp: ResidualMLP, scaler: ModeScaler, mode: str,
                     residual_gain: float = 1.0):
    """Return predict_fn(s_raw_19d, u_raw) -> s_raw_next_9d.
    Combines EDMD-d base + residual_gain * MLP residual correction."""
    def fn(s, u):
        s_z = scaler.transform_s(s, mode)
        u_z = scaler.transform_u(u, mode)
        base_z = edmd_predict(s_z, u_z, K_edmd)
        res_z = predict_residual(mlp, s_z, u_z)
        return scaler.inverse_target(base_z + residual_gain * res_z, mode)
    return fn


def main():
    parser = argparse.ArgumentParser(description="Residual Learned EDMD-d")
    parser.add_argument("--data-dir", type=Path,
                        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"))
    parser.add_argument("--t-settle", type=float, default=0.5)
    parser.add_argument("--hidden", type=int, default=128)
    parser.add_argument("--n-layers", type=int, default=3)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--balance-loss", action="store_true",
                        help="EMA-based 3-way (fz/exy/z) loss normalization")
    parser.add_argument("--ema-beta", type=float, default=0.99)
    parser.add_argument("--sim-error", type=int, default=0,
                        help="Simulation error rollout length (0=one-step only)")
    parser.add_argument("--residual-gain", type=float, default=1.0,
                        help="Scale factor for MLP residual (default 1.0)")
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
    print(f"Model: Residual Learned EDMD-d (EDMD-d base + MLP residual)")
    print(f"Hidden: {args.hidden}, Layers: {args.n_layers}, Epochs: {args.epochs}")
    if args.sim_error > 0:
        pretrain = args.epochs // 2
        finetune = args.epochs - pretrain
        print(f"Training: one-step pretrain ({pretrain}ep) → sim-error finetune ({finetune}ep, K={args.sim_error})")
    else:
        print(f"Training: one-step residual")
    if args.balance_loss:
        print(f"Loss: EMA-balanced fz/exy/z (beta={args.ema_beta})")
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

        s_tr_z = scaler.transform_s(s_tr, mode)
        u_tr_z = scaler.transform_u(u_tr, mode)
        sn_tr_z = scaler.transform_target(sn_tr, mode)
        s_te_z = scaler.transform_s(s_te, mode)
        u_te_z = scaler.transform_u(u_te, mode)

        # Step 1: Fit EDMD-d (frozen base)
        K_edmd = fit_edmd(s_tr_z, u_tr_z, sn_tr_z, alpha=0.0)

        # Step 2: Compute one-step residual stats
        edmd_pred_z = edmd_predict(s_tr_z, u_tr_z, K_edmd)
        residuals = sn_tr_z - edmd_pred_z
        res_std = np.std(residuals, axis=0)
        print(f"  {mode}: residual std  fz={res_std[FZ_IDX]:.6f}"
              f"  exy={(res_std[EX_IDX]+res_std[EY_IDX])/2:.6f}"
              f"  z={res_std[STATE_COLS.index('z_meas')]:.6f}")

        # Step 3: Train MLP
        input_dim = s_tr_z.shape[1] + u_tr_z.shape[1]  # 22
        mlp = ResidualMLP(input_dim=input_dim, output_dim=9,
                          hidden=args.hidden, n_layers=args.n_layers)

        if args.sim_error > 0:
            # Phase 1: one-step pretrain
            print(f"  {mode}: phase 1 — one-step pretrain ({pretrain} ep)...")
            mlp = train_residual_onestep(
                mlp, s_tr_z, u_tr_z, residuals,
                epochs=pretrain, lr=args.lr, batch_size=65536,
                verbose=True, balance_loss=args.balance_loss,
                ema_beta=args.ema_beta, weight_decay=args.weight_decay)
            # Phase 2: sim-error finetune
            print(f"  {mode}: phase 2 — sim-error finetune ({finetune} ep, K={args.sim_error})...")
            mlp = train_residual_simerror(
                mlp, K_edmd, train_m, scaler, mode,
                sim_steps=args.sim_error, epochs=finetune, lr=args.lr * 0.1,
                batch_size=2048, verbose=True,
                balance_loss=args.balance_loss, ema_beta=args.ema_beta,
                weight_decay=args.weight_decay)
        else:
            print(f"  {mode}: training residual MLP (one-step, n={len(train_m)})...")
            mlp = train_residual_onestep(
                mlp, s_tr_z, u_tr_z, residuals,
                epochs=args.epochs, lr=args.lr, batch_size=65536,
                verbose=True, balance_loss=args.balance_loss,
                ema_beta=args.ema_beta, weight_decay=args.weight_decay)

        # Step 4: Evaluate combined one-step (with residual_gain)
        edmd_te_z = edmd_predict(s_te_z, u_te_z, K_edmd)
        res_te_z = predict_residual(mlp, s_te_z, u_te_z)
        combined_z = edmd_te_z + args.residual_gain * res_te_z
        combined = scaler.inverse_target(combined_z, mode)
        os_nrmse = nrmse(combined, sn_te)

        base_pred = scaler.inverse_target(edmd_te_z, mode)
        base_nrmse = nrmse(base_pred, sn_te)

        exy = (os_nrmse[EX_IDX] + os_nrmse[EY_IDX]) / 2
        base_exy = (base_nrmse[EX_IDX] + base_nrmse[EY_IDX]) / 2
        print(f"  {mode}: EDMD-d base   fz={base_nrmse[FZ_IDX]:.6f}  exy={base_exy:.6f}")
        print(f"  {mode}: +residual     fz={os_nrmse[FZ_IDX]:.6f}  exy={exy:.6f}")

        # Save
        torch.save(mlp.state_dict(), save_dir / f"res_mlp_{mode}.pt")
        np.savez(save_dir / f"res_config_{mode}.npz",
                 hidden=args.hidden, n_layers=args.n_layers,
                 input_dim=input_dim,
                 residual_gain=args.residual_gain)
        np.save(save_dir / f"K_edmd_{mode}.npy", K_edmd)

    print(f"\nModels saved to: {save_dir}")
    scaler.save(save_dir / "scaler.npz")


if __name__ == "__main__":
    main()
