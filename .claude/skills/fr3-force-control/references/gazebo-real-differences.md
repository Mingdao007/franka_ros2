# Gazebo vs Real Robot (Force Control Differences)

| Aspect | Real Franka (libfranka) | Gazebo simulation |
|---|---|---|
| Torque command | Driver applies internal gravity handling | Command is applied in simulation dynamics path |
| Measured torque quality | Sensor drift and startup bias may exist | Cleaner synthetic measurement in many setups |
| Bias calibration (`tau_ext_initial`) | Often needed | Can be harmful if it injects offset |
| Gravity compensation in control logic | Usually aligned with hardware stack assumptions | Must be validated carefully for simulation model assumptions |

## Practical rule used in this project
- For Gazebo runs in this repository, `use_bias_calibration: false` is currently preferred.
- Keep the gravity-compensation treatment consistent with the controller implementation and experiment evidence.

## Why this matters
- Porting real-robot templates directly to Gazebo may degrade metrics.
- Always verify with objective gates instead of assuming official defaults are universally optimal.
