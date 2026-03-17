# Hybrid Circle Force Controller — Analytical D-Term

Replaces finite-difference derivative in force PID with analytical velocity derivative from Jacobian. Reduces noise and phase lag in the D-term.

**Status:** Active

## Key Change

Standard D-term: `ė = (e − e_prev) / dt` — noisy, lags by one step.

Analytical D-term: computes `d/dt[f_z]` from Jacobian velocity `ẋ = J · dq`, avoiding numerical differentiation entirely. Cleaner signal → better high-frequency stability.

## Key Files

| File | Role |
|------|------|
| `src/hybrid_circle_force_controller.cpp` | Controller with analytical derivative computation |
| `include/franka_example_controllers/hybrid_circle_force_controller.hpp` | Header |
| `../franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` | Gains |
| `../franka_gazebo/franka_gazebo_bringup/launch/gazebo_hybrid_circle_force.launch.py` | Launch file |

## How to Run

```bash
ros2 launch franka_gazebo_bringup gazebo_hybrid_circle_force.launch.py
```

## See Also

- `feature/pid-force-control` — parent branch (finite-difference D-term)
