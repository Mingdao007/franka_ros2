# Koopman v1 — Switched-Mode EDMD for Hybrid Contact Dynamics

5-method ablation comparing Koopman operator methods for predicting FR3 hybrid contact dynamics across three operating modes (free-space, transition, steady contact).

**Status:** Done (paper branch)

## Methods

| Method | Lifting | Delay | Description |
|--------|---------|-------|-------------|
| Persistence | — | — | Baseline: predict `s_{t+1} = s_t` |
| ARX | — | — | Per-mode linear autoregression |
| Linear | — | No | OLS on raw 9-dim state + 3-dim input |
| Linear-d | — | Yes | OLS with delay embedding (19-dim) |
| EDMD | 20-dim | No | Hand-crafted lifting (squared + cross terms) |
| EDMD-d | 20-dim | Yes | EDMD + delay embedding. **Best overall.** |

## State and Modes

```
State (9-dim):  [x_meas, y_meas, z_meas, vx, vy, vz, ex, ey, fz_meas]
Input (3-dim):  [x_des, y_des, fz_des]

M0 (Pre-contact):      phase==0, free-space impedance
M1 (Transition):       phase==1, t_since_contact < 0.5s
M2 (Steady contact):   phase==1, t_since_contact >= 0.5s
```

EDMD lifting (20-dim): 12 linear + 3 squared (fz², ex², ey²) + 5 cross-terms (ex·vx, ey·vy, z·fz, x·x_des, y·y_des).

## Key Files

| File | Role |
|------|------|
| `preprocess.py` | Load CSV logs, label modes (M0/M1/M2), generate training tables |
| `baselines.py` | Persistence + per-mode ARX baselines, ModeScaler |
| `edmd.py` | Per-mode EDMD with 20-dim lifting (OLS fit) |
| `evaluate.py` | One-step NRMSE, mode-internal rollout, full-process rollout, peak metrics |
| `plot_spectrum.py` | Eigenvalue spectrum visualization (unit circle) |
| `koopman_report.tex` | LaTeX paper draft |
| `data/` | Trained models + training tables |
| `figures/` | Paper figures |

## How to Run

```bash
# 1 — Preprocess raw data
python /home/andy/franka_ros2_ws/src/koopman/preprocess.py

# 2 — Train baselines
python /home/andy/franka_ros2_ws/src/koopman/baselines.py

# 3 — Train EDMD
python /home/andy/franka_ros2_ws/src/koopman/edmd.py

# 4 — Evaluate all methods
python /home/andy/franka_ros2_ws/src/koopman/evaluate.py

# 5 — Plot eigenvalue spectra
python /home/andy/franka_ros2_ws/src/koopman/plot_spectrum.py
```

## Key Results

EDMD-d is the best method overall. Per-mode switching + delay embedding captures the hybrid contact dynamics that global linear models miss. See `evaluate.py` output and `koopman_report.tex` for full comparison tables.

## Training Rules

- Per-mode standardization: fit on train split, reuse for val/test
- Split by `run_id` (not random row split)
- Standardize before lifting, un-standardize predictions for NRMSE
