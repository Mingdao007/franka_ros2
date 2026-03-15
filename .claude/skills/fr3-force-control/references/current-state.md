# Current State (FR3 Hybrid Circle Force Project)

## Architecture

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
- Cosine soft-start ramp: C1-continuous radius ramp

### Force Estimation (dual mode)
1. **Jacobian-based** (default): `F = (JJ^T + λ²I)^{-1} J (τ_meas - τ_gravity - bias)`
2. **FT sensor**: Subscribe to `/ft_sensor_data`, rotate from child frame to world: `F_world = R * F_local`

## Simulation Environment
- Ignition Gazebo 6 (Fortress) on Ubuntu 22.04 / ROS 2 Humble
- Physics: default (ODE), 1ms step
- End-effector: custom ball tip (fixed joint, 0.1 kg, sphere r=15mm)
- Contact surface: compliant pad (kp=20000, kd=100) + paper layer on table (z=0.416m)

## Verified status
| Item | Status | Evidence |
|---|---|---|
| Core pipeline (controller + launch + script + report) | ✅ Done | Full experiment chain verified |
| Baseline gate pass (0.1Hz) | ✅ Done | Fz RMSE ~0.42N, XY NRMSE ~7.5% |
| 0.2Hz also passes gates | ✅ Done | |
| Launch startup speed (3s activation) | ✅ Fixed | `TimerAction` 0.5/1.5s |
| RViz ink trail (measured + desired) | ✅ Done | Blue + Red LINE_STRIP markers |
| Descent → circle state machine | ✅ Done | Cosine soft-start, phase column in CSV |
| Top-level package synced | ✅ Done | symlink-install, runtime = top-level source YAML |
| Experiment infra: config path + meta snapshot | ✅ Done | A1+A2 fix verified 2026-03-15 |
| Analytical D-term (feature/analytical-d-term) | ✅ Feasible | Equivalent to numerical D at 0.1Hz, not merged |
| FT sensor | ❌ Abandoned | ign-gazebo-6 plugin compat issues |
| Orientation control | ❌ Reverted | Sign error caused instability, queued for retry |
| Frequency sweep 0.3+ Hz | ❌ Pending | |

## Important note on consistency
- Runtime config: `src/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` (symlink-installed)
- Inner dev config (`franka_gazebo/franka_gazebo_bringup/...`) is NOT read at runtime
- YAML changes take effect immediately, only C++ changes need rebuild
