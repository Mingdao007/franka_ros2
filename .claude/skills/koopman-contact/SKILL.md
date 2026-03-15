---
name: koopman-contact
description: Koopman operator analysis for hybrid contact dynamics on the FR3 platform. Use when collecting contact data, implementing EDMD/DMD, training Koopman models, evaluating predictions, or analyzing eigenvalue spectra. Trigger phrases include Koopman, EDMD, DMD, lifting, eigenfunction, contact prediction, 混杂动力学, 谱分析.
metadata:
  author: mingdao-lab
  version: 1.1.0
compatibility: claude-code
---

# Koopman Contact Dynamics — v1 Experiment Spec

## Goal

Build a switched autonomous EDMD model that predicts hybrid contact dynamics (pre-contact → transition → steady contact) better than linear baselines. This is the modeling/prediction layer above the existing hybrid force/position controller.

**Paper scope:** IEEE double-column, 6 pages. Narrow problem, few experiments, strong narrative. Focus on transition (M1), not steady-state micro-improvements.

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

v1 principle: **enough expressiveness + don't explode dimensions + interpretable.**

1. **Linear terms (12-dim):** `[s; u]` — the 9 state + 3 input variables
2. **Squared terms (3-dim):** `fz², ex², ey²` — only physically motivated nonlinearities
3. **Physical cross-terms (5-dim):** `ex·vx, ey·vy, z·fz_meas, x·x_des, y·y_des`

**Total lifted dimension: 20.** Do not add more without data-driven justification.

### Training Rules (locked)

- Each mode trained separately with its own K_m
- **Per-mode standardization**: fit mean/std on train split only, reuse for val/test
- Data split by run, never randomly by row
- Standardization before lifting, un-standardize predictions for evaluation

### Baselines

| # | Method | Description |
|---|---|---|
| B1 | Persistence | `s(t+1) = s(t)` |
| B2 | Per-mode ARX | `s(t+1) = A_m·s(t) + B_m·u(t)` per mode (OLS) |

v1 drops global EDMD (B3) to keep comparison focused: ARX vs per-mode EDMD.

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

### Metrics (focused on fz and z)

| Metric | Description |
|---|---|
| One-step NRMSE | Per-variable, per-mode, normalized by variable range |
| Rollout NRMSE | Multi-step prediction error at {50, 100, 500, 1000} steps |
| Peak magnitude error | `|peak_predicted - peak_actual| / peak_actual` for fz overshoot |
| Peak timing error | `|t_peak_predicted - t_peak_actual|` in seconds |

### Rollout Methodology (locked)

**Type 1: Mode-internal rollout**
- Start point within same mode
- Fixed mode model throughout rollout
- Tests "local structure prediction capability"

**Type 2: Full-process rollout (oracle mode labels)**
- Start from real sample
- Switch K_m at mode boundaries using ground-truth labels
- Tests "if mode is known, how far can Koopman predict the full hybrid process"

**v1 does NOT do:**
- Model-based online mode classification
- Automatic operator switching
- These conflate "mode classifier problem" with "dynamics model problem"

### Rollout Horizons

- 50 steps (0.05s) — immediate prediction
- 100 steps (0.1s) — short-term
- 500 steps (0.5s) — medium-term, covers transition
- 1000 steps (1.0s) — long-term stability

## Paper Deliverables (IEEE 6-page, locked)

### 3 Figures + 1 Table

**Figure 1: Mode partition + fz on representative run**
- X-axis: phase-local time
- Curve: fz_meas
- Background bands: M0 / M1 / M2
- Purpose: define mode structure for reader

**Figure 2: One-step / rollout prediction comparison**
- Transition-heavy window
- True vs ARX vs EDMD for fz and z
- Sub-panels for different horizons
- Purpose: show prediction quality

**Figure 3: Transition peak event errors**
- Bar or box plot
- fz peak magnitude error + fz peak timing error
- Purpose: quantify the hybrid contact story

**Table 1: Main results**

| | fz 1-step | z 1-step | fz roll@500 | z roll@500 | peak mag | peak time |
|---|---|---|---|---|---|---|
| ARX-M0 | | | | | — | — |
| ARX-M1 | | | | | | |
| ARX-M2 | | | | | — | — |
| EDMD-M0 | | | | | — | — |
| EDMD-M1 | | | | | | |
| EDMD-M2 | | | | | — | — |

T_settle sensitivity (0.3/0.5/0.8) goes in supplementary or a compact sub-table, not in main narrative.

### Allowed conclusions (exactly one of these three)

1. **EDMD clearly outperforms ARX** — across modes and metrics
2. **EDMD outperforms ARX only on transition** — acceptable, likely the real story
3. **EDMD shows no advantage** — must explain why (insufficient observables, task too smooth, etc.)

**Forbidden:** "therefore we should replace the base controller with Koopman"

### Story focus

The real highlight is almost certainly **M1 transition**, not M2 steady-state micro-improvements. Implementation and writing must emphasize:
- Force settling process
- Short/medium-term rollout
- Peak prediction accuracy

## Literature Context

| Paper | Relevance | Key takeaway |
|---|---|---|
| O'Neill & Asada 2024 (IROS) | Making/breaking contact Koopman | Transition focus correct; observable design must cover fast force changes |
| Shi, Liu, Karydis 2023 (Springer review) | Koopman in robotics survey | Mechanics-inspired lifting > deep methods for small data; v1 approach validated |
| Folkestad & Burdick 2021 (Caltech) | Control-affine Koopman | v1 autonomous OK; bilinear/control form is v2 territory |

## File Structure

```
src/koopman/
├── preprocess.py       # Load internal.csv, mode label, standardize, generate training table
├── baselines.py        # Persistence + per-mode ARX (with per-mode standardization)
├── edmd.py             # Per-mode EDMD (lifting + OLS + per-mode standardization)
├── evaluate.py         # One-step/rollout/peak metrics + comparison table
└── plot_spectrum.py    # Eigenvalue visualization
```

## Current Status

- [x] CSV logging extended with z_meas, vx, vy, vz (verified 2026-03-15)
- [x] v1 experiment spec locked (this document)
- [x] Pipeline code written (preprocess/baselines/edmd/evaluate/plot_spectrum)
- [ ] First data collection batch (20-25 runs)
- [ ] Pipeline validation on collected data
- [ ] EDMD vs ARX comparison report
- [ ] Paper figures and tables

## Rules

- Do not modify the base controller for Koopman purposes
- All Koopman code goes in `src/koopman/`, not in the controller package
- Data comes from `internal.csv` only — do not add ROS subscribers for Koopman
- Validate predictions offline first, before any online/feedforward integration
- T_settle sensitivity: always report results for {0.3, 0.5, 0.8}s, not just one value
- Do not add more lifting functions without data-driven justification
- Do not implement online mode classification in v1
- Comparison table must follow the locked format (rows: method×mode, cols: fz/z focused)
