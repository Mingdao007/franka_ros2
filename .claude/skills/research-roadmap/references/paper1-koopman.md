# Paper 1: Koopman for Hybrid Contact Dynamics

## Paper Info
- **Title**: An operator-theoretic viewpoint to non-smooth dynamical systems: Koopman analysis of a hybrid pendulum
- **Authors**: Govindarajan, Arbabi, van Blargian, Matchen, Tegling, Mezić
- **Venue**: CDC 2016
- **Zotero**: `~/Zotero/storage/CDBLZPKN/`

## Core Ideas to Transfer

### From the paper
1. Hybrid automaton: continuous flow + guard conditions + reset maps
2. Koopman operator linearizes the dynamics in observable space, even for nonsmooth systems
3. Eigenfunctions/eigenvalues reveal geometric structure: invariant sets, mode boundaries
4. Projection operator (Laplace average) extracts eigenfunctions from data
5. Damped vs undamped: spectral properties differ — damped case has decaying modes

### Transfer to contact dynamics
| Pendulum concept | Contact dynamics analogue |
|---|---|
| Continuous flow (swing) | Free-space motion, steady contact sliding |
| Guard condition (angle = ±θ*) | Force threshold (|Fz| > F_contact) |
| Reset map (velocity kick) | Contact impact (velocity discontinuity, force spike) |
| Damped case | Energy dissipation through contact friction/compliance |
| Eigenfunction level sets | Regions in (x, v, F) space with similar evolution |

## Experiment Design (First Implementation)

### State vector for Koopman
```
s = [x_meas, y_meas, z_meas, vx, vy, vz, ex, ey, fz_meas]
```
- 9-dimensional state (after logging is extended)
- Input: reference trajectory (x_des, y_des, fz_des)
- Output for prediction: next-step s

### Mode definition (for switched EDMD)
| Mode | Condition | Expected dynamics |
|---|---|---|
| Descent | phase=0, |Fz| < F_contact | Free-space impedance, z decreasing |
| Transition | phase=1, elapsed < T_settle | Force overshoot, settling, ramp-up |
| Steady contact | phase=1, elapsed >= T_settle | Quasi-periodic (circle tracking + force regulation) |

T_settle to be determined from data (e.g., 2-3s after contact).

### Lifting functions (EDMD)
Start with:
- Original state: s (9 dim)
- Quadratic terms: s_i * s_j (45 dim)
- Optional: thin-plate RBF centered on mode-representative states

Total lifted dimension: ~50-100. Tune based on reconstruction error.

### Comparison baselines
1. **Persistence model**: s(t+1) = s(t)
2. **Linear ARX**: s(t+1) = A*s(t) + B*u(t)
3. **Single-mode EDMD**: one Koopman operator for all modes
4. **Switched EDMD**: per-mode Koopman operators (our method)

### Metrics
- One-step prediction RMSE (per state variable, per mode)
- Multi-step rollout RMSE at horizons: 10, 50, 100 steps (10ms, 50ms, 100ms)
- Transition prediction: can the model predict force settling time and overshoot amplitude?
- Eigenvalue spectrum: do eigenvalues separate modes? Do damped modes match physical intuition?

### Data collection plan
**Option A: Many single-transition experiments**
- Run N=20-50 experiments, each with 1 descent → contact → circle
- Pro: simple, uses existing pipeline
- Con: slow, only 1 transition per run

**Option B: Multi-transition within single experiment**
- Modify controller to do: descent → contact → lift → re-contact → lift → ...
- Pro: many transitions per run, more data-efficient
- Con: requires controller modification (add lift-off phase)

**Recommendation**: Start with Option A using existing pipeline. If data is insufficient, implement Option B.

### Minimum data requirements (rough estimate)
- Per mode: ~1000-5000 samples for stable EDMD
- At 1kHz: 1s of data = 1000 samples
- Descent phase: ~3s per run → 3000 samples/run
- Transition phase: ~2-3s per run → 2000-3000 samples/run
- Steady contact: ~10s per run → 10000 samples/run
- With 20 runs: 60k descent, 50k transition, 200k steady — should be sufficient for EDMD

## Implementation Plan

### Step 1: Platform preparation
- Extend CSV logging (add z_meas, vx, vy, vz)
- Verify data quality with one test run
- No controller changes

### Step 2: Data collection
- Run 20+ baseline experiments (existing controller, 0.1Hz)
- Concatenate internal.csv files
- Label modes from phase column + elapsed time

### Step 3: EDMD implementation
- Python script: load data → construct lifted state → solve EDMD
- Per-mode: separate Koopman matrices K_descent, K_transition, K_steady
- Validate: eigenvalue spectrum, reconstruction error

### Step 4: Prediction evaluation
- One-step and multi-step rollout
- Compare against baselines
- Focus on transition phase (hardest, most valuable)

### Step 5: Analysis and reporting
- Eigenfunction visualization (level sets in state space)
- Spectral analysis per mode
- Physical interpretation of dominant modes
- Write up for report

## Key Risks
1. **Insufficient transition data**: transition phase is short (~2s), may need Option B
2. **Lifting function selection**: wrong choice → poor EDMD approximation → iterate
3. **Gazebo contact not hybrid enough**: compliant contact may be smooth, not truly nonsmooth → spectral properties may be boring. Mitigation: compare with higher-stiffness contact.
4. **Computational**: EDMD with 100-dim lifting on 200k samples is manageable, but deep Koopman would be harder
