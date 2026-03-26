# Gazebo vs Real Robot (Force Control Differences)

| Aspect | Real Franka (libfranka) | Gazebo simulation |
|---|---|---|
| Torque command | Driver applies internal gravity handling | Command is applied in simulation dynamics path |
| Measured torque quality | Sensor drift and startup bias may exist | Cleaner synthetic measurement in many setups |
| Bias calibration (`tau_ext_initial`) | Often needed | Can be harmful if it injects offset |
| Gravity compensation in control logic | Usually aligned with hardware stack assumptions | Must be validated carefully for simulation model assumptions |

## CRITICAL: franka_ign_ros2_control Hidden Gravity Compensation

**`libfranka_ign_ros2_control-system.so` silently adds KDL gravity compensation to every effort command before sending it to Gazebo.** This was confirmed by inspecting the binary:

```
strings libfranka_ign_ros2_control-system.so | grep gravity
→ KDL::ChainDynParam::JntToGravity
→ ModelKDL::gravity
```

Consequences for controller design:
- **NEVER add Pinocchio/KDL gravity comp in your controller** when running on this Gazebo stack. It will double-count gravity and cause the arm to fling upward (confirmed in ITR-009, exy went from 380mm to 0.22mm by disabling controller-side gravity comp).
- **Coriolis is NOT added by the plugin** (no `JntToCoriolis` symbol found). Controller-side Coriolis comp will not double-count, but may still hurt if the Pinocchio model diverges from Gazebo's internal model.
- **Feedforward acceleration is NOT added by the plugin.** Safe to add in controller.
- Always set `enable_gravity_comp: false` in YAML for any effort-mode controller on this stack.

**Verification command** (run this when the ign_ros2_control plugin changes):
```bash
strings $(find /home/andy/franka_ros2_ws/install/franka_ign_ros2_control -name "*.so") \
  | grep -iE 'gravity|coriolis|JntTo'
```

## Practical rule used in this project
- For Gazebo runs in this repository, `use_bias_calibration: false` is currently preferred.
- `enable_gravity_comp: false` is MANDATORY for all effort-mode controllers on this Gazebo stack.
- Keep the gravity-compensation treatment consistent with the controller implementation and experiment evidence.

## Why this matters
- Porting real-robot templates directly to Gazebo may degrade metrics.
- Always verify with objective gates instead of assuming official defaults are universally optimal.
- Hidden compensation in the hardware interface layer can silently break controllers that assume raw effort passthrough.
