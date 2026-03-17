# Hybrid Circle Force Controller — Full XYZ Impedance

Pure impedance (spring-damper) on all three axes — no force PID. Z-axis switched from force regulation to impedance position tracking.

**Status:** WIP

## Control Law

| Axis | Mode | Law |
|------|------|-----|
| XY | Circle tracking | `F = Kp·(x_des − x) + Kd·(0 − ẋ)` |
| Z | Position tracking | `F = Kp_z·(z_des − z) + Kd_z·(0 − ż)` |

No integral term on any axis. Trades force tracking precision for structural simplicity and uniform compliance.

## Key Files

| File | Role |
|------|------|
| `src/hybrid_circle_force_controller.cpp` | Controller (impedance on all axes) |
| `include/franka_example_controllers/hybrid_circle_force_controller.hpp` | Header |
| `../franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` | Gains |
| `../franka_gazebo/franka_gazebo_bringup/launch/gazebo_hybrid_circle_force.launch.py` | Launch file |

## How to Run

```bash
ros2 launch franka_gazebo_bringup gazebo_hybrid_circle_force.launch.py
```

## Notes

WIP — likely needs further tuning. Without integral term on Z, force tracking during contact may drift. This branch tests whether pure impedance is sufficient for the contact task.
