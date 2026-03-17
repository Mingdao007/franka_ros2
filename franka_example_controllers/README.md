# Hybrid Circle Force Controller — Impedance + PID

XY impedance (spring-damper) + Z-axis force PID for circular contact tracking on FR3. Softer XY compliance compared to the full-PID variant.

**Status:** Active

## Control Law

| Axis | Mode | Law |
|------|------|-----|
| XY | Circle tracking | `F = Kp·(x_des − x) + Kd·(0 − ẋ)` (impedance, no integral) |
| Z | Force regulation | `F = Kp_f·e_f + Ki_f·∫e_f + Kd_f·ė_f` (PID, filtered derivative) |

Impedance on XY maps position/velocity error directly to force without integrating — more compliant but no steady-state error correction. Z-axis keeps full PID for force tracking.

Two-phase entry: impedance descent → circle tracking (same as PID variant).

## Key Files

| File | Role |
|------|------|
| `src/hybrid_circle_force_controller.cpp` | Controller (impedance KD on XY, force PID on Z) |
| `include/franka_example_controllers/hybrid_circle_force_controller.hpp` | Header |
| `../franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` | Gains (descent_kp_xy/kd_xy reused for circle phase XY) |
| `../franka_gazebo/franka_gazebo_bringup/launch/gazebo_hybrid_circle_force.launch.py` | Launch file |

## How to Run

```bash
ros2 launch franka_gazebo_bringup gazebo_hybrid_circle_force.launch.py
```

## Comparison with PID Variant

| | `pid-force-control` | `impedance-pid` |
|---|---|---|
| XY | Full PID (Kp + Ki + Kd) | Impedance (Kp + Kd only) |
| Z | Force PID | Force PID |
| XY steady-state error | Corrected (integral) | Not corrected |
| XY compliance | Stiffer | Softer |
