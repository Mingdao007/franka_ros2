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
- ✅ Baseline gate pass observed in retained reports.
- ✅ Launch startup speed fixed (3s activation).
- ✅ RViz ink trail: measured (blue) + desired (red) LINE_STRIP via `visualization_msgs::msg::Marker`.
- ✅ Descent phase: high-stiffness impedance control descends to surface, transitions to circle on contact.
- ✅ Cosine soft-start ramp: C1-continuous radius ramp (no velocity discontinuity).
- ✅ Gazebo ignition dependencies fully removed (was Fortress-incompatible, now pure ROS 2).
- ❌ Orientation control (redundancy exploitation): attempted, sign error caused instability, reverted. Needs proper analysis.
- ❌ Small periodic stutter in Gazebo at one angle: under investigation, likely contact dynamics.
- ❌ Frequency sweep matrix (0.1/0.2/0.3 Hz) not fully completed.
- ❌ Real-robot transfer procedure not completed.

For details and evidence, read:
- `references/current-state.md`
- `references/file-map.md`

## Execution protocol (fast loop)
1. Confirm current status and constraints from `references/current-state.md`.
2. Run one short baseline experiment first, then inspect generated report.
3. If gates fail, use troubleshooting mapping to adjust only the most likely parameters.
4. Re-run quickly with minimal changes. Keep one variable change per run when possible.
5. Summarize outcome with metrics, evidence paths, and next action.

Detailed commands and interpretation:
- `references/validated-workflow.md`

## Mandatory guardrails
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
1. What was run (exact command and duration).
2. What evidence was used (report path, run path, key logs).
3. Gate results and key metrics.
4. Root-cause hypothesis (if failed).
5. Next single best action.

## Reference index
- `references/current-state.md`
- `references/file-map.md`
- `references/validated-workflow.md`
- `references/gazebo-real-differences.md`
- `references/troubleshooting.md`
- `references/usage-and-handoff.md`
- `references/ink-status.md`
- `references/metric-standard.md`
