# Koopman v1 — Out-of-Distribution Evaluation

OOD evaluation of Koopman v1 models on held-out data from unseen conditions (e.g., different frequency sweeps).

**Status:** WIP — branch created but not yet differentiated from `feature/koopman-v1` (currently same commit). OOD evaluation scripts exist as untracked files.

## Purpose

Test whether per-mode EDMD-d generalizes beyond the training distribution. The in-distribution results on `feature/koopman-v1` showed EDMD-d as best, but OOD robustness is unknown.

## Key Files

| File | Role |
|------|------|
| `evaluate_ood.py` | OOD evaluation — tests models on held-out frequency sweeps |
| `data_ood/` | OOD test data (separate from training distribution) |
| `preprocess.py` | Shared preprocessing pipeline |
| `baselines.py` | Shared baseline models |
| `edmd.py` | Shared EDMD implementation |

## How to Run

```bash
# Preprocess OOD data
python /home/andy/franka_ros2_ws/src/koopman/preprocess.py --data-dir data_ood

# Run OOD evaluation
python /home/andy/franka_ros2_ws/src/koopman/evaluate_ood.py
```

## See Also

- `feature/koopman-v1` — in-distribution ablation (parent branch)
