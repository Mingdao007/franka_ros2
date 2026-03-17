# Koopman v1 — Out-of-Distribution Evaluation

OOD evaluation of Koopman v1 models on held-out data from unseen conditions (e.g., different circle radius).

**Status:** WIP — evaluation script committed, OOD data collection not yet run.

## Purpose

Test whether per-mode EDMD generalizes beyond the training distribution. The in-distribution results on `feature/koopman-v1` showed EDMD outperforms ARX, but OOD robustness is untested.

## Key Files

| File | Role |
|------|------|
| `evaluate_ood.py` | OOD evaluation — tests models on held-out frequency sweeps |
| `../run_ood_experiment.sh` | Launches Gazebo with r=0.05m circle for OOD data collection |
| `preprocess.py` | Shared preprocessing pipeline |
| `baselines.py` | Shared baseline models |
| `edmd.py` | Shared EDMD implementation |
| `evaluate.py` | In-distribution evaluation (from v1) |

## Not Yet Available

| Item | Status |
|------|--------|
| `data_ood/` | Planned — OOD data not yet collected (ignored in .gitignore, too large for git) |

## How to Run

```bash
# 1 — Collect OOD data (runs Gazebo with r=0.05m circle)
bash /home/andy/franka_ros2_ws/src/run_ood_experiment.sh

# 2 — Run OOD evaluation (after data collection)
python /home/andy/franka_ros2_ws/src/koopman/evaluate_ood.py
```

## See Also

- `feature/koopman-v1` — in-distribution comparison (parent branch)
