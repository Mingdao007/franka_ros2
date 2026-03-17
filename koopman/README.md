# Learned Koopman Extensions — Shared Index for Closed Experimental Branches

This README is a shared index for 7 closed experimental branches that attempted to improve on hand-crafted EDMD by learning the lifting function or adding residual corrections. **None of these approaches outperformed hand-crafted EDMD** on M1 rollout, which is why all branches are closed.

> **Note:** The scripts listed below (`learned_edmd.py`, `residual_learned_edmd.py`, etc.) are **not committed to any of these branches** — they exist as working-tree files. This README documents the experimental arc, not a runnable branch state.

## Outcome Summary

| Branch | Approach | Result |
|--------|----------|--------|
| `feature/learned-edmd` | MLP lifting, no delay | Worse than EDMD — learned features overfit without delay context |
| `feature/balance-loss` | EMA-balanced loss for learned EDMD | Marginal improvement in exy, still worse than EDMD-d overall |
| `feature/learned-edmd-d` | MLP lifting + delay (19-dim) | Competitive one-step, but unstable in long rollout |
| `feature/learned-edmd-d-3way-loss` | Delay + 3-way balanced loss | Better exy balance, still no rollout win over EDMD-d |
| `feature/residual-learned-edmd-d` | EDMD-d base + MLP residual | Residual sometimes helps short horizon, hurts long horizon |
| `feature/residual-simerror` | Multi-step rollout backprop for residual | More stable than one-step residual, but added complexity not justified |
| `feature/bohb-residual` | Bayesian optimization of residual HPs | Best residual config still can't beat hand-crafted EDMD-d reliably |

**Conclusion:** Hand-crafted EDMD-d with domain-informed lifting (squared fz/ex/ey + cross-terms) and delay embedding is hard to beat for this problem size and data regime. The 20-dim lifting captures the essential nonlinearities; additional learned parameters overfit or destabilize rollouts.

## Architecture Overview

### Learned Lifting (`feature/learned-edmd`, `balance-loss`, `learned-edmd-d`, `3way-loss`)

```
s_{t+1} = W @ [s, u, φ_θ(s, u)]
```

- `φ_θ`: 2–4 layer MLP, configurable hidden dims and lift-dim
- Trained end-to-end with OLS-closed-form K update or gradient descent
- Balance-loss variants add EMA-based per-dimension loss scaling (fz / exy / z)

### Residual (`feature/residual-learned-edmd-d`, `residual-simerror`, `bohb-residual`)

```
s_{t+1} = EDMD-d_base(s̃, u) + r_θ(s̃, u)
```

- EDMD-d base is frozen (hand-crafted K matrix)
- `r_θ`: MLP learning the correction term
- `residual-simerror` adds multi-step rollout backprop (Phase 1: one-step pretrain, Phase 2: K-step simulation error)
- `bohb-residual` uses Optuna + Hyperband to search over hidden/layers/lr/weight_decay/sim_error

## Scripts (not committed — working-tree only)

| Script | Role |
|--------|------|
| `learned_edmd.py` | Learned lifting training (MLP + optional delay + optional balanced loss) |
| `residual_learned_edmd.py` | Residual MLP training (one-step + optional sim-error) |
| `bo_residual.py` | Bayesian optimization of residual hyperparameters |
| `evaluate_learned.py` | Unified evaluation of all learned models vs baselines |
| `plot_results.py` | Comparison figures |

## Committed Files (inherited from v1)

| File | Role |
|------|------|
| `preprocess.py` | Shared preprocessing |
| `baselines.py` | Shared baselines |
| `edmd.py` | Hand-crafted EDMD (the baseline that won) |
| `evaluate.py` | In-distribution evaluation |

## Lessons Learned

1. **Hand-crafted lifting is hard to beat at small data scale.** Domain-informed features (fz², ex·vx, etc.) encode physics that MLPs need much more data to discover.
2. **Delay embedding matters more than lifting complexity.** Linear-d already beats EDMD (no delay). Adding learned lifting on top of delay gives diminishing returns.
3. **Residual corrections help short horizons but hurt long ones.** The MLP residual can reduce one-step error, but errors compound faster during rollout.
4. **Loss balancing helps but doesn't change the ranking.** EMA-balanced loss improves exy tracking but doesn't close the gap with EDMD-d on the primary fz metric.

## See Also

- `feature/koopman-v1` — the winning approach (hand-crafted EDMD-d)
- `feature/narx-comparison` — black-box MLP baseline (also loses to EDMD-d)
