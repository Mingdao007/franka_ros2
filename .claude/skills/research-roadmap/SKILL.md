---
name: research-roadmap
description: Guide the research direction across three target papers and the ultimate goal (precision force control on delicate surfaces). Use when planning experiments, choosing methods, evaluating paper relevance, or deciding what to learn next. Trigger phrases include 研究方向, roadmap, 论文, paper plan, 下一步做什么, 鸡蛋, 化妆, next paper, literature.
metadata:
  author: mingdao-lab
  version: 0.1.0
compatibility: claude-code
---

# Research Roadmap

## Ultimate Goal

Precision force control on **compliant, curved, unknown surfaces**:
- Writing on eggs with pencil (~1-3N, fragile shell, curved)
- Applying makeup on human faces (safe, adaptive, curved + deformable)

Both require: trajectory tracking in tangential plane + constant normal force + real-time adaptation to unknown surface geometry + safety guarantees.

## User Profile

Mingdao Lin — controls/robotics researcher. First author of PG-GRU feedforward paper (IEEE TCST 2026). Background in control systems theory, physics-guided neural networks, inversion-based feedforward. Moving from mechatronics (mass-spring-damper) into robotic manipulation.

Strengths to leverage: control theory rigor, Lyapunov/convergence analysis, physics-guided learning.
Areas to build: robot dynamics, contact mechanics, sim-to-real transfer, Gazebo/ROS2 platform.

## Current Platform

- FR3 robot in Gazebo (Ignition Fortress), ROS 2 Humble
- Hybrid force/position controller: XY position PID + Z force PID
- Descent → circle state machine with cosine soft-start
- Batch experiment + report pipeline
- Baseline: Fz RMSE ~0.42N, XY NRMSE ~7.5% at 0.1Hz, 0.03m radius

## Three Papers — Execution Order

### Paper 1: Koopman for Hybrid Contact Dynamics (CURRENT PRIORITY)

**Source**: Govindarajan et al., "An operator-theoretic viewpoint to non-smooth dynamical systems: Koopman analysis of a hybrid pendulum," CDC 2016.
**Zotero**: `~/Zotero/storage/CDBLZPKN/`

**What to take from it**: NOT the pendulum model. The core ideas:
- Hybrid/nonsmooth systems can be analyzed via Koopman spectral theory
- Lifting to observable space linearizes nonlinear + nonsmooth dynamics
- Eigenfunctions recover geometric structure (level sets, invariant sets)

**Application in our context**: Contact dynamics modeling and prediction layer.
- Robot contact is inherently hybrid: approach → first contact → steady contact → lift-off
- Koopman can model how fz, z, ex, ey evolve across mode transitions
- Short-term prediction enables feedforward correction and force spike suppression

**Architecture**:
```
┌─────────────────────────────────────────────────┐
│ Task Layer: trajectory + safety constraints       │
├─────────────────────────────────────────────────┤
│ Mid Layer: Koopman hybrid model                   │
│   - mode identification                           │
│   - short-term state/force prediction             │
│   - feedforward reference correction              │
├─────────────────────────────────────────────────┤
│ Base Layer: hybrid force/position control (PID)   │
│   - XY position tracking                          │
│   - Z force regulation                            │
│   - unchanged from current baseline               │
└─────────────────────────────────────────────────┘
```

**Method selection for first implementation**: Switched EDMD
- Per-mode Koopman operator (descent, transition, steady contact)
- Hand-crafted lifting functions (polynomials, RBF) on observable state
- Mode labels from phase column + force threshold
- Compare against: standard DMD, local linear ARX

**Key research questions**:
1. Can switched EDMD predict the contact transition (descent → steady) better than linear baselines?
2. Does Koopman short-rollout prediction capture force overshoot/settling dynamics?
3. What lifting functions and observable set give the best finite-dimensional approximation for contact dynamics?

**Platform requirements** (before starting):
- [ ] Extend CSV log: add z_meas, vx, vy, vz (code has p_ee(2) and v_ee, just not logged)
- [ ] Decide experiment design for sufficient transition samples
- [ ] Python pipeline: EDMD training, prediction evaluation, comparison plots

**Success criteria**:
- One-step prediction RMSE of fz, ex, ey significantly better than linear ARX in transition phase
- Short rollout (10-50 steps) prediction captures overshoot dynamics qualitatively
- Clear mode separation visible in Koopman eigenfunction/eigenvalue spectrum

**Not doing in this paper**:
- Replacing the base controller with Koopman
- Deep Koopman (autoencoder lifting) — save for later if EDMD is insufficient
- Curved surfaces — stay on flat surface for isolation
- End-to-end learning

See: [`references/paper1-koopman.md`](references/paper1-koopman.md)

---

### Paper 3: Impedance Learning + DRL for Unknown Terrains (NEXT)

**Source**: Li et al., "Impedance Learning-Based Adaptive Force Tracking for Robot on Unknown Terrains," IEEE TRO 2025.
**Zotero**: `~/Zotero/storage/X3TVDG6I/`

**What to take from it**:
- DRL (actor-critic) learns impedance parameter adjustment policy in simulation
- "Couch model" for sim-to-real safe transfer (Lipschitz continuity bounds)
- Neural network feedforward + variable impedance feedback combination
- Lyapunov stability and convergence proofs

**Application**: Replace or augment the fixed-gain PID force controller with learned adaptive impedance. Train in Gazebo with randomized surfaces, deploy on real FR3.

**When to start**: After Paper 1 Koopman modeling is validated. Paper 1's prediction model can inform Paper 3's DRL reward shaping or state representation.

**Platform requirements**:
- [ ] Randomized terrain generation in Gazebo (varying height, stiffness)
- [ ] Variable impedance controller implementation (replace fixed PID gains with adjustable K, B)
- [ ] DRL training pipeline (likely stable-baselines3 or similar)
- [ ] Sim-to-real transfer validation (future, requires real FR3)

---

### Paper 2: PG-GRU Feedforward for Force Control (AFTER Paper 3)

**Source**: Lin, Bolderman, Lazar, "Physics-Guided Gated Recurrent Units for Inversion-Based Feedforward Control," IEEE TCST 2026.
**Zotero**: `~/Zotero/storage/56SW5KEI/`

**What to take from it** (this is your own paper):
- PG-GRU: physics model (linear) + GRU (nonlinear residual) for inverse dynamics
- Two-step: stable linear inversion first, then GRU on residual
- Proven on mass-spring-damper, needs extension to robotic force control

**Application**: Learn the inverse contact dynamics (given desired force trajectory → required impedance/position commands). The physics model is the linearized robot+contact model; GRU learns the nonlinear residual (contact nonlinearity, friction, surface uncertainty).

**Synergy with Papers 1 and 3**:
- Paper 1's Koopman model provides the linear component for PG-GRU's physics layer
- Paper 3's impedance framework provides the control structure that PG-GRU feedforward augments

**When to start**: After Paper 3 gives a working adaptive force controller. PG-GRU adds feedforward to improve transient response.

---

## Long-Term Research Direction

### Force-First (current focus)
Direct force control + learning methods to handle unknown surfaces.
- Fixed control structure (hybrid force/position or variable impedance)
- Learning layer on top for prediction, adaptation, feedforward

### Impedance-First (future)
Full impedance framework where force emerges from impedance + environment.
- More suitable for 6-DOF contact, deformable surfaces
- Needed when moving from flat/rigid to egg-shell/face-skin
- Paper 3 is the bridge between force-first and impedance-first

### Curved Surface Progression
1. **Flat rigid** (current) → validate baseline + Koopman
2. **Flat compliant** → validate force control on soft surface
3. **Known curved** (cylinder, sphere) → surface-normal force control
4. **Egg-shaped** → curved + fragile + small radius
5. **Deformable** (face model) → curved + compliant + safety

### Key Capability Gaps to Close
| Gap | Needed for | Which paper helps |
|---|---|---|
| Contact dynamics prediction | All | Paper 1 (Koopman) |
| Adaptive impedance on unknown surfaces | Egg, face | Paper 3 (DRL) |
| Feedforward from learned inverse model | Transient performance | Paper 2 (PG-GRU) |
| Surface-normal force control (not world-Z) | Curved surfaces | Platform upgrade |
| Sim-to-real transfer | Real robot | Paper 3 (couch model) |
| Safety constraints (max force, rate limit) | Egg, face | All papers |

## When to use this skill

- Planning next experiment or research step
- Evaluating whether a new paper/method fits the roadmap
- Deciding what platform capability to build next
- Writing reports or proposals — reference the architecture and progression
- Assessing what knowledge/skills the user should study next

## Literature monitoring

Periodically search for new work in:
- Koopman + contact dynamics / manipulation
- DRL + variable impedance + force control
- Physics-guided neural networks for robot control
- Safe sim-to-real transfer for contact tasks
- Force control on deformable/fragile objects

Use Zotero local-first skill for paper access. Organize into force-first vs impedance-first in Zotero.

## Learning recommendations for user

Based on current knowledge profile and research direction:

**Priority (needed for Paper 1)**:
- Dynamic Mode Decomposition (DMD) and Extended DMD — Brunton & Kutz textbook, or the Koopman review paper already in Zotero (storage/2CZAYEKG)
- Hybrid dynamical systems basics — guard conditions, reset maps, hybrid automaton

**Medium-term (for Papers 2-3)**:
- Deep reinforcement learning for control — focus on actor-critic (SAC/TD3), not general RL
- Variable impedance control theory — Hogan's original impedance control papers
- Sim-to-real transfer techniques — domain randomization, system identification

**Long-term (for curved surfaces + real robot)**:
- Differential geometry for surface-normal control (moving frames, Gauss map)
- Contact mechanics (Hertz contact, compliant contact models)
- Franka real-robot API (libfranka) — different from Gazebo interface
