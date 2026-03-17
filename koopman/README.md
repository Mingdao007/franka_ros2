# Koopman v1 — Out-of-Distribution Evaluation

Placeholder branch for OOD evaluation of Koopman v1 models on held-out data from unseen conditions.

**Status:** WIP — documentation-only branch state. No OOD-specific scripts or data have been committed yet.

## Purpose

Test whether per-mode EDMD generalizes beyond the training distribution. The in-distribution results on `feature/koopman-v1` showed EDMD outperforms ARX, but OOD robustness is untested.

## Current Branch State

This branch shares the same base code as `feature/koopman-v1` (Persistence / ARX / EDMD pipeline). The following OOD-specific files are **planned but not yet committed**:

| Planned File | Role |
|--------------|------|
| `evaluate_ood.py` | OOD evaluation on held-out frequency sweeps |
| `data_ood/` | OOD test data (separate from training distribution) |

## Committed Files (inherited from v1)

| File | Role |
|------|------|
| `preprocess.py` | Shared preprocessing pipeline |
| `baselines.py` | Shared baseline models |
| `edmd.py` | Shared EDMD implementation |
| `evaluate.py` | In-distribution evaluation (from v1) |

## See Also

- `feature/koopman-v1` — in-distribution comparison (parent branch)
