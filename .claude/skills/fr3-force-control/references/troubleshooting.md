# Troubleshooting Playbook

## 1) Controller did not become active
Symptoms:
- `run_hybrid_circle_force_experiment.sh` reports activation timeout.
- `ros2 control list_controllers` does not show `hybrid_circle_force_controller` active.

Check:
1. Open `run_*/launch.log`.
2. Confirm plugin loading and dependency status.
3. Rebuild related packages if binary dependency mismatch is suspected.

Actions:
- Rebuild:
  ```bash
  cd <your_workspace>
  source /opt/ros/humble/setup.bash
  colcon build --packages-select franka_semantic_components franka_hardware franka_example_controllers --symlink-install
  source install/setup.bash
  ```
- Re-run single short test.

## 2) Gates fail with high Fz RMSE
Symptoms:
- `Fz RMSE >= 0.600 N`

Likely causes:
- Tangential-normal coupling too strong.
- XY loop too aggressive for current contact condition.

Actions:
- Lower trajectory aggressiveness (frequency/radius) first.
- If needed, reduce `kp_xy` or increase damping (`kd_xy`) cautiously.

## 3) Mean force error too large
Symptoms:
- `|mean(Fz error)| >= 0.300 N`

Likely causes:
- Integral action insufficient.
- Contact preload inconsistency at run start.

Actions:
- Slightly increase `force_ki`.
- Improve start contact consistency.

## 4) XY RMS too large
Symptoms:
- `XY RMS >= 0.005 m`

Actions:
- Increase damping (`kd_xy`) first.
- Reduce trajectory aggressiveness before changing force loop gains.

## 5) Robot crashes or diverges on activation
Likely causes:
- Sign error in control law (e.g., orientation control with wrong cross-product order).
- Safety parameter relaxed (e.g., `k_delta_tau_max` increased beyond 1.0 Nm/cycle).

Actions:
- Revert to last known working configuration.
- Check all force/torque signs carefully before re-testing.
- Never relax `k_delta_tau_max` to mask tracking issues — diagnose root cause.

## 6) Periodic stutter at specific angle in circle
Possible causes:
- Gazebo contact dynamics artifact (penalty-based solver force oscillation).
- Joint torque rate saturation at specific configuration.
- Heavy computation in control loop (e.g., blocking service calls — now removed).

Diagnosis:
- Check if red (desired) trail is smooth at that angle in RViz. If yes, it's tracking/physics, not trajectory.
- Log joint torques and check if any joint hits the rate limit at that angle.

## 7) Ink trail not visible in RViz
Check:
- `ink_enabled: true` in YAML.
- Launch with `use_rviz:=true`.
- RViz config has Marker displays for both topics.
- Fixed Frame is `world` (not `base`).

## 8) Simulation process residue causes unstable behavior
Symptoms:
- Repeated controller load/config errors on subsequent runs.

Actions:
- Ensure previous simulation is fully terminated before next run.
- Keep one simulation session at a time.
