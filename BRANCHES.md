# Branch Index

This fork (`Mingdao007/franka_ros2`) extends the upstream Franka ROS 2 stack with force-control experiments and Koopman operator modelling for hybrid contact dynamics.

## Controller Branches

| Branch | Status | Description | Key Files |
|--------|--------|-------------|-----------|
| `feature/pid-force-control` | Active | XY PID + Z force PID. Main controller for Koopman data collection. | `franka_example_controllers/src/hybrid_circle_force_controller.cpp` |
| `feature/impedance-pid` | Active | XY impedance (KD) + Z force PID. Softer XY tracking. | `franka_example_controllers/src/hybrid_circle_force_controller.cpp` |
| `feature/xyzIMPEDANCE` | WIP | Full impedance on all axes (Z channel changed from force PID to impedance). | `franka_example_controllers/src/hybrid_circle_force_controller.cpp` |
| `feature/ft-sensor` | Closed | Force-torque sensor integration (Ignition FT plugin). | `franka_gazebo/franka_ign_ros2_control/ign_hardware_plugins.xml` |
| `feature/ft-sensor-v2` | Closed | FT sensor v2 — refinement of sensor bridge. | same as above |
| `feature/analytical-d-term` | Active | Analytical velocity derivative for force PID D-term. | `franka_example_controllers/src/hybrid_circle_force_controller.cpp` |

All controllers share the `HybridCircleForceController` base with Pinocchio dynamics, Jacobian-based Cartesian force, and two-phase entry (impedance descent → circle tracking).

## Koopman Branches

| Branch | Status | Description | Key Files |
|--------|--------|-------------|-----------|
| `feature/koopman-v1` | Done | Per-mode comparison: Persistence vs ARX vs EDMD. Paper branch. | `koopman/{preprocess,baselines,edmd,evaluate}.py` |
| `feature/koopman-v1-ood` | WIP | OOD evaluation (placeholder — scripts not yet committed). | `koopman/evaluate_ood.py` (planned) |
| `feature/narx-comparison` | Done | NARX MLP baseline vs EDMD on M1 rollout. | `koopman/{narx,evaluate_narx}.py` |

## Learned-Extension Branches (all closed)

Experimental extensions to EDMD with learned lifting / residual correction. All branch from `feature/koopman-v1`. **None outperformed hand-crafted EDMD.** See shared README on `feature/learned-edmd`.

| Branch | Description |
|--------|-------------|
| `feature/learned-edmd` | MLP lifting function, no delay embedding. |
| `feature/balance-loss` | EMA-balanced loss (fz / exy / z) for learned EDMD. |
| `feature/learned-edmd-d` | MLP lifting with delay embedding (19-dim state). |
| `feature/learned-edmd-d-3way-loss` | Delay + 3-way balanced loss. |
| `feature/residual-learned-edmd-d` | Hand-crafted EDMD-d base + MLP residual correction. |
| `feature/residual-simerror` | Simulation-error (multi-step rollout) training for residual. |
| `feature/bohb-residual` | Bayesian optimization (Optuna + Hyperband) of residual hyperparameters. |

Key scripts (working-tree only, not committed to branches): `koopman/{learned_edmd,residual_learned_edmd,bo_residual,evaluate_learned}.py`

## Quick Reference

- **Upstream:** `frankarobotics/franka_ros2` (origin)
- **Fork:** `Mingdao007/franka_ros2` (fork)
- **Base branch:** `main` (tracks `origin/humble`)
- **Controller code:** `franka_example_controllers/`
- **Koopman code:** `koopman/`
