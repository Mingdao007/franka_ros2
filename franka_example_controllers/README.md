# Hybrid Circle Force Controller — FT Sensor Integration

Replaces Jacobian-based force estimation with direct force-torque sensor measurement via Ignition Gazebo FT plugin + ros_gz_bridge.

**Status:** Closed (proof-of-concept, superseded by v2)

## Key Files

| File | Role |
|------|------|
| `src/hybrid_circle_force_controller.cpp` | Controller with FT sensor subscription |
| `include/franka_example_controllers/hybrid_circle_force_controller.hpp` | Header |
| `../franka_gazebo/franka_ign_ros2_control/ign_hardware_plugins.xml` | FT sensor plugin registration |
| `../franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` | Config (ft_sensor_topic) |

## Approach

Same control law as `feature/pid-force-control`, but `F_hat` (Cartesian force estimate) comes from the FT sensor message instead of `J^T · τ_ext` estimation. Requires Ignition FT sensor plugin and ros_gz_bridge relay.

## See Also

- `feature/ft-sensor-v2` — refinement of this branch
