# EMA Filter Alpha Sweep Results

## Background

The hybrid_circle_force_controller uses an EMA (Exponential Moving Average) low-pass
filter on the Jacobian-estimated Fz measurement before feeding it to the force PID.

The original code had the EMA operands reversed from the upstream franka_ros1/ros2
convention. This was fixed to match upstream:

```
// Upstream convention (franka_hw_sim.cpp, joint_impedance_example_controller.cpp):
// alpha=1.0: no filtering (use raw), alpha=0.0: frozen (max smoothing)
f_meas_filtered = alpha * raw + (1 - alpha) * f_meas_filtered;
```

## Sweep Results (constant Fz=5N setpoint, circle phase steady-state t>3s, 1kHz)

| alpha | raw weight | time constant | Fz RMSE (N) | eXY RMSE (mm) | stability |
|-------|-----------|---------------|-------------|---------------|-----------|
| 0.00001 | 0.001% | 100s | 2.753 | 2.644 | stable but slow contact detect |
| 0.05 | 5% | 20ms | 0.299 | 2.391 | stable, good balance |
| 0.50 | 50% | 2ms | 0.535 | 2.873 | stable, noisier |
| 0.80 | 80% | 1.25ms | 1.042 | 3.106 | robot fell over |
| 1.0 | 100% | 0 | 1.596 | 3.747 | Gazebo glitches |

## Key Finding

The relationship between alpha and Fz RMSE is U-shaped:
- Too low (alpha -> 0): filter is nearly frozen, can't track real force changes
- Sweet spot (~0.05): enough smoothing to suppress Jacobian estimation noise,
  fast enough to track the constant setpoint
- Too high (alpha -> 1): raw Jacobian noise dominates, PID oscillates, robot unstable

## Implication for FT Sensor Paper

Jacobian-based force estimation **requires** heavy low-pass filtering (alpha=0.05,
i.e. 20ms time constant) to produce usable force feedback. This filtering introduces
latency that is acceptable for constant-force tasks but problematic for:
- Fast contact/release transitions
- Dynamic force tracking (time-varying setpoints)
- Near-singular configurations (where Jacobian noise amplifies)

A direct FT sensor does not need this trade-off — its signal is clean enough to
use with minimal or no filtering.

## Config

Selected `force_filter_alpha: 0.05` as default for Gazebo simulation.
