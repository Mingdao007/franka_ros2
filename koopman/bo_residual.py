"""
bo_residual.py — BOHB-style hyperparameter optimization for Residual Learned EDMD-d.

Uses Optuna with HyperbandPruner. Trains incrementally (not from scratch per budget).
Objective: M1 combined fz + exy rollout at @5/@10/@20 (lower is better).

Usage:
    python bo_residual.py --n-trials 30
    python bo_residual.py --n-trials 2 --max-epochs 50   # smoke test
"""

import argparse
from pathlib import Path
import json

import numpy as np
import optuna
import torch

from preprocess import STATE_COLS, INPUT_COLS
from baselines import load_table, split_by_run, get_arrays, nrmse, ModeScaler
from edmd import fit_edmd, edmd_predict
from residual_learned_edmd import (
    ResidualMLP, predict_residual, make_residual_fn,
)
from evaluate import (
    _compute_data_bounds, mode_internal_rollout,
    M1_ROLLOUT_HORIZONS, FZ_IDX, EX_IDX, EY_IDX,
)

# Preload data once (set by main, used by objective)
_DATA = {}


def evaluate_m1_score(mlp, K_edmd, scaler, test_m, bounds_lo, bounds_hi,
                      residual_gain: float) -> float:
    """Compute M1 rollout score: mean(fz, exy) at @5/@10/@20 + divergence penalty."""
    fn = make_residual_fn(K_edmd, mlp, scaler, "M1", residual_gain=residual_gain)
    test_runs = test_m["run_id"].unique()

    scores = {h: {"fz": [], "exy": []} for h in [5, 10, 20]}
    diverged_20 = False

    for rid in test_runs:
        run_mask = test_m["run_id"] == rid
        s_run, u_run, sn_run = get_arrays(test_m[run_mask])

        for h in [5, 10, 20]:
            if len(s_run) < h:
                continue
            last_row = np.zeros((1, s_run.shape[1]))
            last_row[0, :9] = sn_run[-1]
            s_actual = np.vstack([s_run, last_row])
            nr, ah = mode_internal_rollout(
                s_run[0], u_run, s_actual, fn, h, bounds_lo, bounds_hi)
            if not np.all(np.isnan(nr)):
                scores[h]["fz"].append(nr[FZ_IDX])
                scores[h]["exy"].append((nr[EX_IDX] + nr[EY_IDX]) / 2)
                if h == 20 and ah > 0 and ah < 20:
                    diverged_20 = True

    # Check we have results
    for h in [5, 10, 20]:
        if not scores[h]["fz"]:
            return 200.0

    fz = np.mean([np.mean(scores[h]["fz"]) for h in [5, 10, 20]])
    exy = np.mean([np.mean(scores[h]["exy"]) for h in [5, 10, 20]])
    div_penalty = 100.0 if diverged_20 else 0.0

    return fz + exy + div_penalty


def train_onestep_incremental(mlp, s_z, u_z, residuals, total_epochs,
                              lr, weight_decay, device, use_amp):
    """Train one-step for total_epochs. Returns mlp. Supports incremental calling."""
    import torch.nn as nn
    from residual_learned_edmd import _balanced_loss

    mlp.to(device)
    su_t = torch.tensor(np.hstack([s_z, u_z]), dtype=torch.float32, device=device)
    r_t = torch.tensor(residuals, dtype=torch.float32, device=device)
    N = su_t.shape[0]
    batch_size = min(65536, N)

    optimizer = torch.optim.AdamW(mlp.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=total_epochs)
    grad_scaler = torch.amp.GradScaler(enabled=use_amp)

    ema_fz, ema_exy, ema_z = 0.0, 0.0, 0.0

    for epoch in range(total_epochs):
        mlp.train()
        perm = torch.randperm(N, device=device)
        for start in range(0, N, batch_size):
            idx = perm[start:start + batch_size]
            su = su_t[idx]
            r_b = r_t[idx]
            with torch.autocast(device_type="cuda", dtype=torch.bfloat16, enabled=use_amp):
                pred = mlp(su)
                sq = (pred - r_b) ** 2
                loss, ema_fz, ema_exy, ema_z, _, _ = _balanced_loss(
                    sq, ema_fz, ema_exy, ema_z, 0.99)
            optimizer.zero_grad()
            grad_scaler.scale(loss).backward()
            grad_scaler.step(optimizer)
            grad_scaler.update()
        scheduler.step()

    return mlp


def objective(trial: optuna.Trial) -> float:
    """Optuna objective with Hyperband-style incremental budget."""
    d = _DATA

    params = {
        "hidden": trial.suggest_categorical("hidden", [64, 128, 256]),
        "n_layers": trial.suggest_categorical("n_layers", [2, 3, 4]),
        "weight_decay": trial.suggest_float("weight_decay", 1e-6, 1e-2, log=True),
        "lr": trial.suggest_float("lr", 1e-4, 3e-3, log=True),
        "residual_gain": trial.suggest_float("residual_gain", 0.05, 1.0),
        "sim_error": trial.suggest_categorical("sim_error", [0, 5, 10]),
    }

    torch.manual_seed(42)
    np.random.seed(42)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    use_amp = device.type == "cuda"

    # Build MLP once, train incrementally
    mlp = ResidualMLP(input_dim=22, output_dim=9,
                      hidden=params["hidden"], n_layers=params["n_layers"])

    budgets = [b for b in [50, 100, 200, 400] if b <= d["max_epochs"]]
    prev_budget = 0

    for budget in budgets:
        incremental_epochs = budget - prev_budget

        if params["sim_error"] > 0 and prev_budget == 0:
            # First half: one-step pretrain, second half: sim-error
            pretrain_ep = budget // 2
            finetune_ep = budget - pretrain_ep

            from residual_learned_edmd import train_residual_onestep, train_residual_simerror
            mlp = train_residual_onestep(
                mlp, d["s_tr_z"], d["u_tr_z"], d["residuals"],
                epochs=pretrain_ep, lr=params["lr"], batch_size=65536,
                verbose=False, balance_loss=True, ema_beta=0.99,
                weight_decay=params["weight_decay"])
            mlp = train_residual_simerror(
                mlp, d["K_edmd"], d["train_m"], d["scaler"], "M1",
                sim_steps=params["sim_error"], epochs=finetune_ep,
                lr=params["lr"] * 0.1, batch_size=2048,
                verbose=False, balance_loss=True, ema_beta=0.99,
                weight_decay=params["weight_decay"])
        elif params["sim_error"] > 0 and prev_budget > 0:
            # Continue with sim-error only for subsequent budgets
            from residual_learned_edmd import train_residual_simerror
            mlp = train_residual_simerror(
                mlp, d["K_edmd"], d["train_m"], d["scaler"], "M1",
                sim_steps=params["sim_error"], epochs=incremental_epochs,
                lr=params["lr"] * 0.1, batch_size=2048,
                verbose=False, balance_loss=True, ema_beta=0.99,
                weight_decay=params["weight_decay"])
        else:
            # Pure one-step: train incrementally
            mlp = train_onestep_incremental(
                mlp, d["s_tr_z"], d["u_tr_z"], d["residuals"],
                total_epochs=incremental_epochs, lr=params["lr"],
                weight_decay=params["weight_decay"],
                device=device, use_amp=use_amp)

        prev_budget = budget

        # Evaluate at this budget
        mlp.eval()
        score = evaluate_m1_score(
            mlp, d["K_edmd"], d["scaler"], d["test_m"],
            d["bounds_lo"], d["bounds_hi"],
            residual_gain=params["residual_gain"])

        trial.report(score, step=budget)
        if trial.should_prune():
            raise optuna.TrialPruned()

    return score


def main():
    parser = argparse.ArgumentParser(description="BOHB for Residual Learned EDMD-d")
    parser.add_argument("--data-dir", type=Path,
                        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"))
    parser.add_argument("--t-settle", type=float, default=0.5)
    parser.add_argument("--n-trials", type=int, default=30)
    parser.add_argument("--max-epochs", type=int, default=400)
    args = parser.parse_args()

    table = load_table(args.data_dir, args.t_settle)
    train, test, train_ids, test_ids = split_by_run(table)
    scaler = ModeScaler()
    scaler.fit(train)
    bounds_lo, bounds_hi = _compute_data_bounds(table)

    # Precompute M1 training data
    train_m = train[train["mode"] == "M1"]
    test_m = test[test["mode"] == "M1"]
    s_tr, u_tr, sn_tr = get_arrays(train_m)

    s_tr_z = scaler.transform_s(s_tr, "M1")
    u_tr_z = scaler.transform_u(u_tr, "M1")
    sn_tr_z = scaler.transform_target(sn_tr, "M1")

    K_edmd = fit_edmd(s_tr_z, u_tr_z, sn_tr_z, alpha=0.0)
    residuals = sn_tr_z - edmd_predict(s_tr_z, u_tr_z, K_edmd)

    # Store in global for objective
    _DATA.update({
        "scaler": scaler, "bounds_lo": bounds_lo, "bounds_hi": bounds_hi,
        "train_m": train_m, "test_m": test_m,
        "s_tr_z": s_tr_z, "u_tr_z": u_tr_z, "residuals": residuals,
        "K_edmd": K_edmd, "max_epochs": args.max_epochs,
    })

    print(f"Train: {len(train_m)} M1 samples, Test: {len(test_m)} M1 samples")
    print(f"BOHB: {args.n_trials} trials, budget 50→{args.max_epochs} epochs")
    print(f"Objective: mean(fz, exy) @ {{5, 10, 20}} + div penalty")
    print()

    study = optuna.create_study(
        direction="minimize",
        pruner=optuna.pruners.HyperbandPruner(
            min_resource=50, max_resource=args.max_epochs, reduction_factor=2),
        study_name="reslearn_bohb",
    )

    study.optimize(objective, n_trials=args.n_trials, show_progress_bar=True)

    # Report
    best = study.best_trial
    print("\n" + "=" * 60)
    print("BEST TRIAL")
    print("=" * 60)
    print(f"  Score: {best.value:.6f}")
    for k, v in best.params.items():
        print(f"  {k}: {v}")
    print()

    # Save
    results_dir = args.data_dir / "bo_results"
    results_dir.mkdir(parents=True, exist_ok=True)
    with open(results_dir / "best_params.json", "w") as f:
        json.dump({"score": best.value, "params": best.params}, f, indent=2)

    trials_data = [
        {"number": t.number, "value": t.value, "params": t.params, "state": t.state.name}
        for t in study.trials
    ]
    with open(results_dir / "all_trials.json", "w") as f:
        json.dump(trials_data, f, indent=2)

    print(f"Results saved to: {results_dir}")


if __name__ == "__main__":
    main()
