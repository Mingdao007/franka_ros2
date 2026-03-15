---
name: koopman-contact
description: Koopman operator analysis for hybrid contact dynamics on the FR3 platform. Use when collecting contact data, implementing EDMD/DMD, training Koopman models, evaluating predictions, or analyzing eigenvalue spectra. Trigger phrases include Koopman, EDMD, DMD, lifting, eigenfunction, contact prediction, 混杂动力学, 谱分析.
metadata:
  author: mingdao-lab
  version: 1.0.0
compatibility: claude-code
---

# Koopman Contact Dynamics — v1 Experiment Spec

## Goal

Build a switched autonomous EDMD model that predicts hybrid contact dynamics (pre-contact → transition → steady contact) better than linear baselines. This is the modeling/prediction layer above the existing hybrid force/position controller.

## Architecture Position

```
Koopman model (this skill) ← predicts state evolution, NOT a controller
    ↕ data
Hybrid force/position PID  ← unchanged base controller (fr3-force-control skill)
```

Koopman does NOT replace the controller. It consumes logged data and produces predictions.

## State Vector

From `internal.csv` (15 columns):

```
time, x_des, y_des, x_meas, y_meas, z_meas, ex, ey, vx, vy, vz, fz_des, fz_meas, fz_cmd, phase
```

**Koopman state (9-dim):**
```
s = [x_meas, y_meas, z_meas, vx, vy, vz, ex, ey, fz_meas]
```

**Input (3-dim):** `u = [x_des, y_des, fz_des]`

## Mode Definitions

| Mode | Label | Condition | Dynamics character |
|---|---|---|---|
| M0 | Pre-contact | `phase == 0` | Free-space impedance, z decreasing, fz ≈ 0 |
| M1 | Transition | `phase == 1` AND `t_since_contact < T_settle` | Force overshoot, settling, < 0.5s |
| M2 | Steady contact | `phase == 1` AND `t_since_contact >= T_settle` | Quasi-periodic circle tracking + force regulation |

**T_settle:** primary value **0.5s**. Sensitivity analysis at {0.3, 0.5, 0.8}s.

## Method: Autonomous Switched EDMD

### Formulation

Autonomous EDMD: reference signals (u) are absorbed into the input vector, so the model predicts the full 9-dim physical state as a self-contained dynamical system.

For each mode m, collect data pairs `(s_t, u_t) → s_{t+1}` and solve:

```
K_m = argmin ||Ψ(s_{t+1}) - K_m · Ψ([s_t; u_t])||²
```

via ordinary least squares (OLS).

**Output:** K_m predicts the 9-dim physical state `s_{t+1}`, NOT the lifted state.

### Lifting Functions

Start with a controlled, interpretable set:

1. **Linear terms (12-dim):** `[s; u]` — the 9 state + 3 input variables
2. **Squared terms (12-dim):** `[s_i², u_j²]` — all individual squares
3. **Physical cross-terms (3-dim):** `ex·vx, ey·vy, z·fz_meas`

**Total lifted dimension: 27.** Keep it small for v1. Increase only if one-step NRMSE is poor.

### Baselines

| # | Method | Description |
|---|---|---|
| B1 | Persistence | `s(t+1) = s(t)` |
| B2 | Per-mode ARX | `s(t+1) = A_m·s(t) + B_m·u(t)` per mode (OLS) |
| B3 | Global EDMD | Single K for all modes (same lifting) |
| B4 | Per-mode EDMD | Per-mode K_m (our method) |

## Data Pipeline

### Collection
```bash
cd /home/andy/franka_ros2_ws/src
source /home/andy/franka_ros2_ws/install/setup.bash
for i in $(seq 1 25); do
  bash run_hybrid_circle_force_experiment.sh
  sleep 2
done
```

Each run produces `run_<N>/internal.csv` with ~15k rows at 1kHz.

### Training Data Format

One-step supervised table per mode:

| Column | Description |
|---|---|
| `run_id` | Which experimental run |
| `mode` | M0 / M1 / M2 |
| `s_t` (9 cols) | Current state |
| `u_t` (3 cols) | Current input |
| `s_{t+1}` (9 cols) | Next-step state (target) |

### Data Splitting

**Split by run, not by row.** Train/test split is at the run level to prevent data leakage from temporal correlation within a single run.

Example: 20 runs → 15 train + 5 test.

## Evaluation

### Metrics

| Metric | Description |
|---|---|
| One-step NRMSE | Per-variable, per-mode, normalized by variable range |
| Rollout NRMSE | Multi-step prediction error at {50, 100, 500, 1000} steps |
| Peak magnitude error | `|peak_predicted - peak_actual| / peak_actual` for fz overshoot |
| Peak timing error | `|t_peak_predicted - t_peak_actual|` in seconds |

### Rollout Horizons

- 50 steps (0.05s) — immediate prediction
- 100 steps (0.1s) — short-term
- 500 steps (0.5s) — medium-term, covers transition
- 1000 steps (1.0s) — long-term stability

## File Structure

```
src/koopman/
├── preprocess.py       # Load internal.csv, mode label, generate training table
├── baselines.py        # Persistence + per-mode ARX
├── edmd.py             # Per-mode EDMD (lifting + OLS)
├── evaluate.py         # One-step/rollout/peak metrics + comparison tables
└── plot_spectrum.py    # Eigenvalue visualization
```

## Current Status

- [x] CSV logging extended with z_meas, vx, vy, vz (verified 2026-03-15)
- [x] v1 experiment spec locked (this document)
- [ ] First data collection batch (20-25 runs)
- [ ] preprocess.py
- [ ] baselines.py
- [ ] edmd.py
- [ ] evaluate.py
- [ ] plot_spectrum.py
- [ ] EDMD vs ARX comparison report

## Rules

- Do not modify the base controller for Koopman purposes
- All Koopman code goes in `src/koopman/`, not in the controller package
- Data comes from `internal.csv` only — do not add ROS subscribers for Koopman
- Validate predictions offline first, before any online/feedforward integration
- T_settle sensitivity: always report results for {0.3, 0.5, 0.8}s, not just one value
