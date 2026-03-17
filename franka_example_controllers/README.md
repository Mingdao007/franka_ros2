# Hybrid Circle Force Controller — FT Sensor v2

Refinement of `feature/ft-sensor` — addresses sensor calibration, bridge latency, and parameter tuning for FT sensor integration.

**Status:** Closed

## Key Files

| File | Role |
|------|------|
| `src/hybrid_circle_force_controller.cpp` | Controller with FT sensor subscription |
| `include/franka_example_controllers/hybrid_circle_force_controller.hpp` | Header |
| `../franka_gazebo/franka_ign_ros2_control/ign_hardware_plugins.xml` | FT sensor plugin registration |

## See Also

- `feature/ft-sensor` — original FT sensor proof-of-concept
- `feature/pid-force-control` — same control law, Jacobian-based force estimation
