---
name: fr3-force-control
description: Validate and iterate FR3 Gazebo force-control experiments with repeatable gates and fast feedback. Use when users ask to run or debug hybrid circle force control, tune Fz PID, inspect RMSE or XY gates, or summarize simulation progress in this repository. Trigger phrases include FR3 力控仿真验证, hybrid_circle_force_controller, Fz RMSE, run_hybrid_circle_force_experiment, and Gazebo force control.
metadata:
  author: mingdao-lab
  version: 0.3.0
  primary_repo: franka_ros2_ws/src
  mirror_path: docs/skills/fr3-force-control
compatibility: claude-code, claude-ai, copilot-cli
---

# FR3 Force Control Validation Skill

## Purpose
Use this skill to run, validate, and debug FR3 force-control simulation in Gazebo with a fast, reproducible workflow.

This skill is optimized for:
- Hybrid task: XY circle trajectory + Z-axis constant force.
- Descent → circle phase state machine (impedance descent, then hybrid force/position).
- RViz-based ink trail visualization (measured + desired trajectories).
- Rapid iteration cycles (avoid long idle waits around short experiments).
- Evidence-based decisions using report metrics and run logs.

## Collaboration model
- **Claude edits code**, user runs simulation and verifies behavior manually.
- Do not run simulation launches or long-running experiments autonomously.
- Focus on code edits, build verification, and providing correct `source + launch` commands.
- When proposing speculative or safety-relaxing changes, suggest them but wait for user approval before implementing.
- After `colcon build`, always remind user to `source install/setup.bash` before runtime verification.

## Use this skill when
- User asks to validate FR3 force control in Gazebo.
- User asks to run or tune `hybrid_circle_force_controller`.
- User asks why force tracking failed or why gates did not pass.
- User asks for current project progress and what can be done now.

## Do not use this skill for
- Real robot safety-critical deployment without additional review.
- Non-FR3 platforms or non-Gazebo simulators unless adapted.
- Tasks unrelated to force-control simulation validation.

## Current progress snapshot
- ✅ Core pipeline: controller + launch + batch experiment + report generation.
- ✅ Baseline gate pass: Fz RMSE 0.40N, XY NRMSE 7.2% (0.1Hz).
- ✅ 0.2Hz also passes gates.
- ✅ Launch startup speed fixed (3s activation).
- ✅ RViz ink trail: measured (blue) + desired (red) LINE_STRIP.
- ✅ Descent → circle state machine with cosine soft-start radius ramp.
- ✅ CSV log with phase column for descent/circle analysis.
- ✅ Top-level package synced (launch, world, config, rviz all installed).
- ✅ FT sensor attempted and abandoned (ign-gazebo-6 plugin compat issues).
- 🔄 Phase 2 improvements in progress (force ramp, dwell-time, D-term, orientation).
- ❌ Orientation control: attempted, sign error caused instability, reverted. Queued for retry.
- ❌ Frequency sweep at 0.3+ Hz not fully analyzed.

For details and evidence, read:
- `references/current-state.md`
- `references/file-map.md`

## Execution protocol (fast loop)
1. Make one scoped change, build.
2. Verify by running the automated experiment script (NOT manual `ros2 launch`):
   ```bash
   cd /home/andy/franka_ros2_ws/src
   source /home/andy/franka_ros2_ws/install/setup.bash
   bash run_hybrid_circle_force_experiment.sh
   ```
   This runs 1 sim (15s), collects data, generates `report.md` with metrics.
3. User shares report. Claude interprets: compare against baseline, flag regressions.
4. If gates fail, adjust only the most likely parameter. One variable per run.
5. Commit + push when verified.

**Baseline reference (0.1Hz, 0.03m radius):**
- Fz RMSE: 0.40 N (8.0% of 5N target)
- |mean(Fz error)|: 0.12 N
- XY NRMSE: 7.2% of radius

**Acceptance gates:**
- `|mean(Fz error)| < 0.300 N`
- `Fz RMSE < 0.600 N`
- `XY RMS < 0.005 m`

Detailed commands and interpretation:
- `references/validated-workflow.md`

## Mandatory guardrails
- **NEVER enable Pinocchio/KDL gravity comp in effort-mode controllers.** `franka_ign_ros2_control` already adds KDL gravity comp internally. Double-counting causes catastrophic arm fling (proven ITR-009). Always `enable_gravity_comp: false`.
- One simulation at a time. Always clean residual Gazebo/controller processes before a new run.
- Prefer short experiments for tuning loops (for example 15-20 seconds) to maximize iteration speed.
- Never claim success without evidence from report files and logs.
- Keep changes scoped. Do not rewrite unrelated controller logic.
- Preserve failure artifacts (`launch.log`, `collector.log`, `wrench.csv`, `internal.csv`) for diagnosis.
- Do not relax safety parameters (e.g., `k_delta_tau_max`) to mask symptoms. Diagnose root cause first.
- When switching technical approaches, fully remove all traces of the old approach from code, config, and build deps.

## Decision rules
- If `|mean(Fz error)|` fails gate: prioritize force integral/preload consistency checks.
- If `Fz RMSE` fails gate: reduce tangential-normal coupling (trajectory aggressiveness or XY gains).
- If `XY RMS` fails gate: increase damping or reduce trajectory aggressiveness before touching force loop.
- If controller activation fails: check dependency/build/runtime loading first, then controller config.
- Do not use MSE as a reporting metric. Use RMSE, and include normalized variants when possible.

## Response contract
When using this skill, return:
1. **Full experimental conditions** before any results table:
   - Trajectory: frequency, radius
   - Force: force_desired
   - XY gains: kp_xy, kd_xy, ki_xy
   - Force gains: force_kp, force_ki, force_kd
   - Any parameter that differs from baseline must be explicitly called out
2. What was run (exact command and duration).
3. What evidence was used (report path, run path, key logs).
4. Gate results and key metrics.
5. Root-cause hypothesis (if failed).
6. Next single best action.

## Reference index
- `references/current-state.md`
- `references/file-map.md`
- `references/validated-workflow.md`
- `references/gazebo-real-differences.md`
- `references/troubleshooting.md`
- `references/usage-and-handoff.md`
- `references/ink-status.md`
- `references/metric-standard.md`
