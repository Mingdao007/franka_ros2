# File Map (What can be used for what)

| Path | Purpose |
|---|---|
| `franka_example_controllers/include/.../hybrid_circle_force_controller.hpp` | Controller header: phase state machine, RViz marker publishers, descent/circle parameters. |
| `franka_example_controllers/src/hybrid_circle_force_controller.cpp` | Main controller: descent impedance → circle hybrid force/position, RViz ink trail publish. |
| `franka_example_controllers/CMakeLists.txt` | Build config. Uses `visualization_msgs`. No ignition/gz dependencies. |
| `franka_gazebo_bringup/launch/gazebo_hybrid_circle_force.launch.py` | Top-level launch: robot description (ball-tip), Gazebo, RViz (optional), controller spawning. |
| `franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` | **Runtime config** (symlink-installed). All tunable parameters: gains, descent, circle, ink trail, force PID. |
| `franka_gazebo_bringup/config/hybrid_circle_force.rviz` | RViz config: world frame, InkTrail (blue) + InkDesired (red) marker displays, top-down view. |
| `franka_gazebo_bringup/config/gz_gui.config` | Gazebo GUI camera layout. |
| `franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` | Inner dev config. **NOT read at runtime.** No known consumer. |
| `run_hybrid_circle_force_experiment.sh` | Batch experiment runner with cleanup, activation polling, data collection, report trigger. |
| `collect_force_data.py` | Captures commanded and estimated wrench topics into `wrench.csv`. |
| `generate_hybrid_experiment_report.py` | Computes gates and metrics, outputs markdown report + plots. Reads normalization params from `run_meta.txt` first, fallback to YAML. |
| `generate_pseudo_ink.py` | Pseudo-ink trajectory plot from internal.csv (force-weighted line width). |
| `results/hybrid_circle_force/<timestamp>/` | Run artifacts: logs, csv files, plots, and report. |

## Runtime config loading chain
```
xacro $(find franka_gazebo_bringup)/config/franka_gazebo_controllers.yaml
  → resolves to install/share (symlink) → src/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml
```
YAML changes take effect immediately (no rebuild). Only C++ changes need `colcon build`.

## RViz marker topics
| Topic | Color | Content | Condition |
|---|---|---|---|
| `~/ink_trail` | Blue | Measured EE position | Force above threshold |
| `~/ink_desired` | Red | Desired circle trajectory | Always (during circle phase) |

## Canonical acceptance gates
From `generate_hybrid_experiment_report.py`:
- `|mean(Fz error)| < 0.300 N`
- `Fz RMSE < 0.600 N`
- `XY RMS < 0.005 m`
