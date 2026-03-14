# Current State (FR3 Hybrid Circle Force Project)

## Verified status
| Item | Status | Evidence |
|---|---|---|
| Core pipeline (controller + launch + script + report) | ✅ Done | `hybrid_circle_force_controller.cpp`, `gazebo_hybrid_circle_force.launch.py`, `run_hybrid_circle_force_experiment.sh`, `generate_hybrid_experiment_report.py` |
| Baseline acceptance gate pass | ✅ Done | `results/hybrid_circle_force/20260315_032904/report.md` |
| Launch startup speed (3s activation) | ✅ Fixed | Reduced `TimerAction` delays from 5.0/6.0s to 0.5/1.5s in launch file |
| Gazebo ignition dependencies | ✅ Removed | CMakeLists.txt no longer depends on `ignition-msgs8` / `ignition-transport11` |
| RViz ink trail (measured + desired) | ✅ Done | Blue (measured, force-gated) + Red (desired, always) LINE_STRIP markers via `visualization_msgs` |
| Descent phase (impedance → force) | ✅ Done | High-stiffness impedance descent, transitions to hybrid force/position on contact detection |
| Cosine soft-start ramp | ✅ Done | C1-continuous radius ramp, no velocity discontinuity at transition |
| Dedicated RViz config | ✅ Done | `hybrid_circle_force.rviz` with world frame, top-down view, both marker displays |
| Orientation control (redundancy) | ❌ Reverted | Attempted with cross-product error, caused instability. Needs proper sign analysis and testing |
| Periodic stutter at one angle | ❌ Open | Visible in Gazebo, not trajectory-related (cos/sin is smooth). Likely contact dynamics |
| Frequency sweep (0.1/0.2/0.3 Hz) | ❌ Pending | |
| Multi-day repeatability | ❌ Pending | |
| Real robot transfer SOP | ❌ Pending | |

## Controller architecture
```
Phase 1: kDescent
  - XY: high-stiffness impedance (kp=6000, kd=200), hold initial position
  - Z: impedance control, descend at 0.02 m/s
  - Transition: |Fz| > descent_contact_force (2.0N)

Phase 2: kCircle
  - XY: PD tracking of circle trajectory (cosine soft-start ramp)
  - Z: force PID (track force_desired)
  - Null-space: joint damping
  - Torque rate saturation: 1.0 Nm/cycle (safety, do not relax)
```

## Important note on consistency
- Always report exact file path and timestamped run directory.
- If report text and runtime parameter files disagree, treat controller config and run artifacts as source of truth for executed behavior.
- Metric policy: do not use MSE in reports; use RMSE and normalized RMSE variants.
