# NARX Comparison — MLP Baseline vs EDMD-d

NARX (Nonlinear AutoRegressive with eXogenous inputs) MLP baseline for head-to-head comparison against EDMD-d on M1 rollout. Tests whether Koopman structure adds value over black-box learning with the same delay-embedded input.

**Status:** Done

## NARX Architecture

- Input: 22-dim `[s̃(19), u(3)]` — same 19-dim delay embedding as EDMD-d + 3-dim input
- Output: 9-dim (next physical state)
- Hidden: 64 units, 2 layers (default)
- Training: one-step MSE, AdamW + cosine annealing, per-mode, 200 epochs
- Mixed precision (bfloat16 on GPU)

Delay embedding: `{z, vz, fz, ex, ey}` at lag-1 and lag-2 → 10 extra dims → 19-dim state.

## Key Files

| File | Role |
|------|------|
| `narx.py` | NARX model training — per-mode MLP, delay embedding, AdamW |
| `evaluate_narx.py` | Compare Linear-d vs EDMD-d vs NARX-d on M1 rollout |
| `data/narx_models/` | Trained `.pt` weights + `.npz` config per mode |
| `preprocess.py` | Shared preprocessing |
| `edmd.py` | Shared EDMD (for comparison) |

## How to Run

```bash
# 1 — Train NARX per mode
python /home/andy/franka_ros2_ws/src/koopman/narx.py

# 2 — Compare all three methods on M1
python /home/andy/franka_ros2_ws/src/koopman/evaluate_narx.py
```

## Key Results

EDMD-d retains structural advantage over NARX despite NARX having more parameters. The Koopman linear operator (with hand-crafted lifting) provides better long-horizon rollout stability than a black-box MLP with the same input features.

Evaluation horizons: [5, 10, 20, 50, 100] (stricter than v1's [50, 100, 500, 1000]).

## See Also

- `feature/koopman-v1` — parent branch, full 5-method ablation
