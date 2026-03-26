---
name: simulation
description: Run, build, and verify FR3 Gazebo simulation end-to-end. Handles colcon build, launch commands, topic inspection, and output interpretation. Use whenever the user needs to start a simulation, test a code change in Gazebo, check sensor topics, or debug launch failures. Trigger phrases include 仿真, simulation, gazebo, 跑一下, launch, ign topic, 编译, build and run, 试试.
metadata:
  author: mingdao-lab
  version: 1.0.0
compatibility: claude-code
---

# Simulation Skill

Owns the full build-launch-verify cycle for FR3 Gazebo simulation. Every time simulation is involved, this skill's rules apply.

## Core Rule: Build It, Command It, Explain It

Every simulation-related response MUST include:

1. **Build** — run `colcon build` yourself (never ask the user to build)
2. **Commands** — give complete, copy-paste-ready terminal blocks
3. **Output guide** — tell the user exactly what to look for in the output

No exceptions. If you skip any of these three, the user wastes a round-trip.

## Build Protocol

Always build before giving launch commands. Use absolute paths:

```bash
source /opt/ros/humble/setup.bash
source /home/andy/franka_ros2_ws/install/setup.bash
colcon build \
  --build-base /home/andy/franka_ros2_ws/build \
  --install-base /home/andy/franka_ros2_ws/install \
  --base-paths /home/andy/franka_ros2_ws/src \
  --packages-select <PACKAGE_NAMES>
```

After build, verify the installed files match the source:
- Launch files: diff source vs install
- Config files: grep for the key parameter you just changed
- SDF/xacro: grep for the element you added

**Known trap:** This workspace has duplicate directories:
- `src/franka_gazebo_bringup/` — the real colcon package (this is what gets built)
- `src/franka_gazebo/franka_gazebo_bringup/` — git subdir, NOT a separate package

Always edit files in `src/franka_gazebo_bringup/` (or whatever `package.xml` belongs to). After build, always verify the installed file has your changes.

## Launch Commands — Packaged Run Mode (preferred)

Default: give the user a **single script** that:
1. Cleans residual processes
2. Creates a timestamped log folder in the current directory
3. Launches simulation with `timeout` auto-shutdown
4. Captures all relevant logs and topic samples
5. Prints a summary of what was collected

**Log directory convention:**

```
sim_logs/YYYYMMDD_HHMMSS/
├── launch.log          # Full launch stdout+stderr
├── ign_topics.log      # ign topic -l snapshot
├── ft_sensor.log       # FT sensor data sample (if applicable)
├── ros2_topics.log     # ros2 topic list snapshot
├── estimated_wrench.log # Controller wrench publisher (if applicable)
└── summary.txt         # Auto-generated: duration, exit code, key grep results
```

**Template for packaged run script:**

Every script MUST start with a build step. The user should never need to build separately.

```bash
#!/bin/bash
# — Auto-generated simulation test —
# Run from anywhere. Logs saved to sim_logs/<timestamp>/

set -e
LOG_DIR=/home/andy/franka_ros2_ws/sim_logs/$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOG_DIR"
echo "Logs → $LOG_DIR"

# Build first (always)
source /opt/ros/humble/setup.bash
source /home/andy/franka_ros2_ws/install/setup.bash
colcon build \
  --build-base /home/andy/franka_ros2_ws/build \
  --install-base /home/andy/franka_ros2_ws/install \
  --base-paths /home/andy/franka_ros2_ws/src \
  --packages-select <PACKAGE_NAMES> \
  > "$LOG_DIR/build.log" 2>&1
source /home/andy/franka_ros2_ws/install/setup.bash  # re-source after build

# Clean residual processes
pkill -f "ign gazebo" 2>/dev/null || true
pkill -f "controller_manager" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
sleep 2

source /home/andy/franka_ros2_ws/install/setup.bash

# Start simulation with timeout (background)
timeout <DURATION>s \
  ros2 launch <PACKAGE> <LAUNCH_FILE> <ARGS> \
  > "$LOG_DIR/launch.log" 2>&1 &
SIM_PID=$!

# Wait for controller to activate (poll launch.log)
echo "Waiting for controller activation..."
for i in $(seq 1 60); do
  if grep -q "Configured and activated" "$LOG_DIR/launch.log" 2>/dev/null; then
    echo "Controller active after ${i}s"
    break
  fi
  sleep 1
done

# Capture topic snapshots
ign topic -l > "$LOG_DIR/ign_topics.log" 2>&1 || true
ros2 topic list > "$LOG_DIR/ros2_topics.log" 2>&1 || true

# Capture topic data samples (run for a few seconds then kill)
# <ADD TOPIC-SPECIFIC CAPTURES HERE>

# Wait for simulation to finish (timeout auto-kills)
wait $SIM_PID 2>/dev/null || true
EXIT_CODE=$?

# Generate summary
{
  echo "=== Simulation Run Summary ==="
  echo "Time: $(date)"
  echo "Duration: <DURATION>s (timeout)"
  echo "Exit code: $EXIT_CODE"
  echo ""
  echo "=== Key Log Lines ==="
  grep -E "Configured and activated|Contact detected|Failed to load|\\[Err\\]|ft_sensor" \
    "$LOG_DIR/launch.log" 2>/dev/null || echo "(no key lines found)"
  echo ""
  echo "=== Ignition Topics ==="
  cat "$LOG_DIR/ign_topics.log" 2>/dev/null || echo "(empty)"
  echo ""
  echo "=== ROS 2 Topics ==="
  cat "$LOG_DIR/ros2_topics.log" 2>/dev/null || echo "(empty)"
} > "$LOG_DIR/summary.txt"

cat "$LOG_DIR/summary.txt"
echo ""
echo "Full logs: $LOG_DIR/"
```

When generating this script, always fill in `<DURATION>`, `<PACKAGE>`, `<LAUNCH_FILE>`, and add test-specific topic captures.

After the user says "跑完了", read `sim_logs/` for the latest run's `summary.txt` and relevant logs.

## Launch Commands — Manual Mode (fallback)

Only use when the user explicitly wants interactive control. Always give full blocks:

```bash
# Terminal 1 — start simulation
source /home/andy/franka_ros2_ws/install/setup.bash
ros2 launch franka_gazebo_bringup gazebo_hybrid_circle_force.launch.py
```

For topic inspection:

```bash
# Terminal 2
source /home/andy/franka_ros2_ws/install/setup.bash
ros2 topic list | grep <PATTERN>
ros2 topic echo <FULL_TOPIC_NAME>
```

For Ignition-side topics:

```bash
ign topic -l | grep <PATTERN>
ign topic -e -t <FULL_TOPIC_PATH>
```

## Output Specification

After giving commands, always tell the user:

1. **Success indicators** — what lines in the log mean it worked
   - Example: "should see `Configured and activated hybrid_circle_force_controller`"
   - Example: "should NOT see `Failed to load system plugin`"
2. **Failure indicators** — what to watch for
   - Example: "if you see `timed out`, wait 10 more seconds, it's normal for large URDFs"
3. **Data to paste back** — what output to copy if something goes wrong
   - Example: "if it fails, paste the first error line with `[Err]`"

## Ignition Fortress Rules

This workspace uses Ignition Gazebo Fortress (version 6). Critical naming differences:

| What | Fortress (us) | Garden+ |
|------|---------------|---------|
| Namespace | `ignition::gazebo::systems::` | `gz::sim::systems::` |
| Plugin prefix | `ignition-gazebo-` | `gz-sim-` |
| CLI command | `ign topic` | `gz topic` |
| Plugin class example | `ignition::gazebo::systems::ForceTorque` | `gz::sim::systems::ForceTorque` |

**Before writing any Gazebo plugin/sensor/system XML:**
1. Find the actual `.so` file: `ls /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/`
2. Check the registered name: `strings <file>.so | grep "ignition::gazebo"`
3. Cross-reference with official franka_ros2 source (jazzy branch has Fortress examples)

## Reference Official Code First

For any Gazebo/ROS integration (plugins, sensors, bridges, URDF extensions):

1. Check `franka_ros2` official repo — same branch first, then jazzy
2. Check `franka_ros` (ROS 1) — has mature Gazebo Classic implementations
3. Check `ros_gz` examples — for bridge syntax and message type mappings
4. Only then write new code

## Batch Experiment Script

For the standard force-control experiment:

```bash
cd /home/andy/franka_ros2_ws/src
source /home/andy/franka_ros2_ws/install/setup.bash
bash run_hybrid_circle_force_experiment.sh
```

This runs 1 sim (15s), collects data, generates `report.md`.

## Process Cleanup

Before starting a new simulation, always clean residual processes:

```bash
# Kill leftover Gazebo / controller processes
pkill -f "ign gazebo" 2>/dev/null
pkill -f "controller_manager" 2>/dev/null
pkill -f "robot_state_publisher" 2>/dev/null
sleep 2
```

## CRITICAL: Hidden Gravity Compensation in franka_ign_ros2_control

`libfranka_ign_ros2_control-system.so` silently adds KDL gravity compensation (`KDL::ChainDynParam::JntToGravity`) to every effort command before sending it to Gazebo. **No Coriolis or feedforward terms are added — only gravity.**

**Hard rule:** Any effort-mode controller on this stack MUST set `enable_gravity_comp: false`. Adding controller-side gravity comp will double-count and cause catastrophic arm fling. This was proven in ITR-009 (cheating-contact branch).

## Shutdown Crash

The `terminate called ... Node needs to be associated with an executor` crash on Ctrl+C is a known bug in `IgnitionROS2ControlPlugin` destructor. It is harmless — ignore it.

## Clipboard Auto-Copy

After generating a simulation run command or script:

1. **First explain** what the script does (3-5 bullet points: build what, launch what, capture what, how long, where logs go)
2. Then copy to clipboard: `echo "<command>" | xclip -selection clipboard`
3. Tell the user: "已复制到剪切板，直接粘贴运行。"

User must be able to trust the command before running it. Never just say "已复制" without explaining what it does.

## Checklist Before Handing Off to User

Before giving the user any simulation command:

- [ ] Did I run `colcon build` myself?
- [ ] Did I verify the installed files have my changes?
- [ ] Did I give complete copy-paste commands with `source` included?
- [ ] Did I explain what success/failure looks like in the output?
- [ ] Did I check official source code for any Gazebo plugin/sensor syntax?
- [ ] Did I copy the command to clipboard via `xclip`?
