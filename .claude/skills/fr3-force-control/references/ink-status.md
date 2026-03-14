# Ink Visualization Status

## Current approach: RViz markers
**Two `visualization_msgs::msg::Marker` LINE_STRIP publishers** in `HybridCircleForceController::update()`:

| Topic | Color | Content | Gating |
|---|---|---|---|
| `~/ink_trail` | Blue (0.1, 0.1, 0.8) | Measured EE position | `\|Fz\| > ink_force_threshold` |
| `~/ink_desired` | Red (0.8, 0.1, 0.1) | Desired circle position | Always (circle phase) |

- `ink_enabled: true` in YAML activates trail accumulation + RViz publish.
- Line width: `ink_line_width * 0.5` (half of configured width).
- QoS: transient-local, depth 1 (late-joining RViz subscribers get the latest marker).
- Dedicated RViz config: `hybrid_circle_force.rviz` (world frame, top-down camera, both displays pre-configured).
- Points projected to `ink_surface_z + ink_surface_epsilon` for visual alignment with paper surface.

## Gazebo native markers: fully removed
All `ignition-transport11` / `ignition-msgs8` dependencies have been removed from:
- Header (no `ignition/transport/Node.hh`)
- Source (no `ignition/msgs/marker.pb.h`, no `gz_node_`, no service calls)
- CMakeLists.txt (no `find_package`, no `target_link_libraries`)
- YAML (no `ink_marker_service`, no `ink_marker_timeout_ms`)

**Why removed:** Fortress MarkerManager `/marker` is one-way (no-reply), protobuf type mismatch caused silent rejection, and the blocking service call in the 1kHz control loop caused periodic stuttering as trail_points grew.

## Verification
```bash
ros2 launch franka_gazebo_bringup gazebo_hybrid_circle_force.launch.py use_rviz:=true
```
1. Red circle (desired) appears immediately when circle phase starts.
2. Blue trail (measured) appears when contact force exceeds threshold.
3. `ros2 topic echo /hybrid_circle_force_controller/ink_trail` shows marker data.
4. `ros2 topic echo /hybrid_circle_force_controller/ink_desired` shows desired marker data.
