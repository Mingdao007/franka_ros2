# Architecture Snapshot (2026-03-15)

## System Overview

```
┌─────────────────────────────────────────────────────────┐
│ Ignition Gazebo 6 (Fortress)                            │
│  ┌──────────┐  ┌───────────┐  ┌───────────────────────┐ │
│  │ Physics   │  │ ForceTorque│  │ FR3 + ball_tip       │ │
│  │ (ODE/DART)│  │ System     │  │ 7-DOF + fixed EE     │ │
│  └──────────┘  └─────┬─────┘  └───────────────────────┘ │
│                      │ gz.msgs.Wrench                    │
└──────────────────────┼──────────────────────────────────┘
                       │
              ┌────────┴────────┐
              │ ros_gz_bridge   │
              │ parameter_bridge│
              └────────┬────────┘
                       │ /ft_sensor_data (geometry_msgs/Wrench)
                       ▼
┌──────────────────────────────────────────────────────────┐
│ hybrid_circle_force_controller (1 kHz)                   │
│                                                          │
│  ┌──────────────┐    ┌──────────────┐                    │
│  │ Force Sensing │    │ Pinocchio    │                    │
│  │ FT sub or     │    │ FK/Jacobian  │                    │
│  │ J-based est.  │    │              │                    │
│  └──────┬───────┘    └──────┬───────┘                    │
│         │                   │                            │
│  ┌──────▼───────────────────▼──────┐                     │
│  │ State Machine                    │                    │
│  │  Descent (impedance) → Circle   │                    │
│  │  transition: |Fz| > threshold   │                    │
│  └──────┬──────────────────────────┘                     │
│         │                                                │
│  ┌──────▼──────────────────────────┐                     │
│  │ Hybrid Control Law              │                    │
│  │  XY: PID position tracking      │                    │
│  │  Z:  PID force regulation       │                    │
│  │  τ = J^T F_cmd + N(-D dq)      │                    │
│  └──────┬──────────────────────────┘                     │
│         │ effort commands                                │
└─────────┼────────────────────────────────────────────────┘
          ▼
    7 joint effort interfaces
```

## Key Files

| File | Role |
|------|------|
| `franka_example_controllers/.../hybrid_circle_force_controller.hpp` | Controller header |
| `franka_example_controllers/.../hybrid_circle_force_controller.cpp` | Controller implementation (~750 lines) |
| `franka_gazebo/franka_gazebo_bringup/launch/gazebo_hybrid_circle_force.launch.py` | Launch (URDF injection, Gazebo, bridge, controllers) |
| `franka_gazebo/franka_gazebo_bringup/worlds/direct_force_world.sdf` | World (table, pad, paper, plugins) |
| `franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` | Controller params |

## Control Law Details

### Descent Phase
- Pure impedance control: `F = Kp * e + Ki * ∫e + Kd * ė` in all 3 axes
- Fixed downward velocity reference: `z_des = z0 - v_descent * t`
- Transition condition: `|Fz_filtered| > descent_contact_force`

### Circle Phase
- XY: PID position tracking of circular trajectory `(cx + r*cos(ωt), cy + r*sin(ωt))`
- Z: PID force regulation `u = Kp*(Fd - Fm) + Ki*∫e + Kd*ė`, applied as `Fz_cmd = sign * (Fd + u)`
- Null-space: joint damping `N * (-D * dq)`
- Torque rate saturation: `Δτ_max = 1.0 Nm/step`

### Force Estimation (dual mode)
1. **Jacobian-based**: `F = (JJ^T + λ²I)^{-1} J (τ_meas - τ_gravity - bias)`
2. **FT sensor**: Subscribe to `/ft_sensor_data`, rotate from child frame to world: `F_world = R * F_local`

## Simulation Environment

- Ignition Gazebo 6 (Fortress) on Ubuntu 22.04 / ROS 2 Humble
- Physics: default (ODE), 1ms step
- End-effector: custom ball tip (fixed joint, 0.1 kg, sphere r=15mm)
- Contact surface: compliant pad (kp=20000, kd=100) + paper layer on table (z=0.416m)
- FT sensor: on ball_tip_joint, child frame, child_to_parent, 1kHz update
