# Hybrid Circle Force Controller — PID Force Control

XY position PID + Z-axis force PID for circular contact tracking on FR3. Primary controller used for Koopman data collection.

**Status:** Active

## Control Law

| Axis | Mode | Law |
|------|------|-----|
| XY | Circle tracking | `F = Kp·e + Ki·∫e + Kd·ė` (filtered derivative) |
| Z | Force regulation | `F = Kp_f·e_f + Ki_f·∫e_f + Kd_f·ė_f` (filtered derivative) |

Two-phase entry: high-stiffness impedance descent until force threshold, then soft-start ramp to circle.

Hybrid output: `τ = J^T · F_cmd + N · (−D · dq)` (Cartesian force via pseudo-inverse + null-space damping).

## Key Files

| File | Role |
|------|------|
| `src/hybrid_circle_force_controller.cpp` | Controller implementation (PID on XY + force PID on Z) |
| `include/franka_example_controllers/hybrid_circle_force_controller.hpp` | Header with state variables, gain declarations |
| `franka_example_controllers.xml` | Plugin registration |
| `../franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` | Gain tuning (kp_xy, kd_xy, ki_xy, force_kp/ki/kd, filter alphas) |
| `../franka_gazebo/franka_gazebo_bringup/launch/gazebo_hybrid_circle_force.launch.py` | Launch file (Gazebo + controller + RViz ink trails) |

## How to Run

```bash
# Terminal 1 — launch Gazebo + controller
ros2 launch franka_gazebo_bringup gazebo_hybrid_circle_force.launch.py

# Terminal 2 — record data for Koopman
ros2 topic echo /hybrid_circle_force_controller/internal --csv > data.csv
```

## Dependencies

- Pinocchio (kinematic/dynamic model)
- Ignition Gazebo (simulation)
- franka_hardware (state interfaces)
