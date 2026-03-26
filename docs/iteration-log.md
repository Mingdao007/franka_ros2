# Cheating Force Controller — Iteration Log

Baseline: hybrid_circle_force_controller (exy=1.67mm, fz_err=0.30N, z_std=0.41mm)

---

## ITR-001 — Remove gain_ramp from gravity comp only

**Date:** 2026-03-26 19:00
**Branch:** `feature/cheating-control`
**Base commit:** `832c41b`
**Status:** FAILED

### Hypothesis
gain_ramp multiplies gravity comp by ~0 at startup → arm gets no gravity support → falls. Removing gain_ramp from gravity comp should let arm hold from frame 1.

### Changes
| File | Change | Why |
|------|--------|-----|
| `cheating_force_controller.cpp:420` | `gain_ramp * tau_g` → `tau_g` | Gravity comp unramped |

### Results
Arm still dropped. Screenshot showed vertical blue ink trail (Z drop).

### Conclusion
**Verdict:** Failed — PD gains still ramped from 0, so even with gravity comp, no position correction for model mismatch.

### Do Not Repeat
- [ ] Unramping gravity comp alone is insufficient if PD is also zero

---

## ITR-002 — Remove gain_ramp from PD + gravity comp + Coriolis

**Date:** 2026-03-26 19:30
**Branch:** `feature/cheating-control`
**Status:** FAILED

### Hypothesis
Full PD + full gravity comp from frame 1. At t=0 error=0 so PD doesn't cause shock, but immediately corrects any drift.

### Changes
| File | Change | Why |
|------|--------|-----|
| `cheating_force_controller.cpp` | Removed gain_ramp from PD, gravity, Coriolis | All torques full from t=0 |

### Results
User reported arm still flung. Likely same gz_ros2_control zero-torque first cycle issue.

### Conclusion
**Verdict:** Failed — removing gain_ramp alone doesn't solve the hold_joints→effort transition.

---

## ITR-003 — Settle phase (gravity comp only, no PD, 15s)

**Date:** 2026-03-26 20:00
**Branch:** `feature/cheating-control` (stashed)
**Status:** FAILED

### Hypothesis
During 15s settle, only gravity comp + joint damping → arm reaches equilibrium without PD oscillation. Re-capture center at 15s.

### Changes
Added settle_duration, settled_ flag, PD=0 during settle.

### Results
exy=380mm, ez=-660mm. Arm completely collapsed. No task-space damping → gravity comp can't arrest motion.

### Do Not Repeat
- [ ] PD=0 during settle without task-space damping → catastrophic collapse

---

## ITR-004 — Settle with velocity damping + rate limiter

**Date:** 2026-03-26 20:30
**Branch:** `feature/cheating-control` (stashed)
**Status:** FAILED

### Hypothesis
Task-space velocity damping (Kd only) during settle + rate limiter for safety.

### Results
Rate limiter (1 Nm/step) prevented gravity comp from ramping fast enough after gz_ros2_control's zero-torque first cycle. Arm collapsed.

### Do Not Repeat
- [ ] Rate limiter with delta_tau_max=1.0 is incompatible with effort startup — blocks gravity comp ramp

---

## ITR-005 — Warmstart: joint_impedance → switch to cheating

**Date:** 2026-03-26 21:00
**Branch:** `feature/cheating-warmstart`
**Status:** FAILED

### Hypothesis
joint_impedance_example_controller holds arm via effort, then atomic switch to cheating (effort→effort, no zero-torque gap).

### Results
joint_impedance has no gravity comp (pure PD, k=600) AND actively moves joints 4+5 (cosine ramp demo). Arm collapsed during warmstart phase.

### Do Not Repeat
- [ ] joint_impedance_example_controller is a DEMO, not a static hold — never use as warmstart

---

## ITR-006 — Warmstart: hybrid → switch to cheating

**Date:** 2026-03-26 21:30
**Branch:** `feature/cheating-warmstart`
**Status:** PARTIAL

### Hypothesis
hybrid_circle_force_controller is proven stable in Gazebo. Use it as warmstart, switch to cheating at 10s.

### Changes
| File | Change | Why |
|------|--------|-----|
| `gazebo_cheating_force.launch.py` | hybrid warmstart → switch_controllers at 10s | Bypass hold_joints→effort gap |
| `cheating_force_controller.cpp` | Removed all gain_ramp | Full control from frame 1 |
| `franka_gazebo_controllers.yaml` | log_file_path set | Enable CSV logging |

### Results
- Hybrid phase: z_meas 545→417mm (normal descent), phases 0+1 seen ✓
- Switch to cheating: exy=0mm at handover ✓
- CSV post-switch: exy mean=0.57mm, ez mean=-5.5mm
- But user reports visual "fling" at startup

**Unexpected:** CSV data looks good (exy<3.5mm) but user sees fling. May be the hybrid descent itself looking violent, or brief transient not captured in 100ms-sampled view.

### Conclusion
**Verdict:** Partial — data OK but user not satisfied with visual behavior.

### Next
Lower PD gains (kp=600, kd=50) per Codex recommendation. Confirm arm can stand before increasing.

---

## ITR-007 — Lower PD gains to kp=600

**Date:** 2026-03-26 22:30
**Branch:** `feature/cheating-warmstart`
**Status:** FAILED

### Hypothesis
kp=6000 is 250x higher than official Gazebo examples. Lower to kp=600, kd=50, confirm arm holds.

### Changes
| File | Change | Why |
|------|--------|-----|
| `franka_gazebo_controllers.yaml` | kp 6000→600, kd 200→50 | Match official magnitude |

### Results
| Metric | Baseline (hybrid) | This ITR | Delta |
|--------|-------------------|----------|-------|
| exy mean (mm) | 1.67 | 22.24 | 13x worse |
| ez mean (mm) | 0.41 | 78.07 | 190x worse |

### Conclusion
**Verdict:** Failed — gains too low. PD can't compensate gravity comp model mismatch. kp=6000 was actually correct for this use case.

### Do Not Repeat
- [ ] kp=600 is insufficient for Cartesian cheating controller — gravity comp mismatch needs high PD to compensate

### Next
Revert to kp=6000. The real problem is the startup, not the gains. Focus on understanding why hybrid warmstart still "flings" visually despite good CSV data.

---

## ITR-009 — Disable Pinocchio gravity comp (KDL double-counting fix)

**Date:** 2026-03-26 23:00
**Branch:** `feature/cheating-warmstart`
**Base commit:** `832c41b`
**Status:** PASSED

### Hypothesis
Sonnet audit found gz_ros2_control silently adds KDL gravity comp to every effort command in ign_system::write(). Controller was also adding Pinocchio gravity comp → double-counting → arm pushed up → "fling."

### Changes
| File | Change | Why |
|------|--------|-----|
| `franka_gazebo_controllers.yaml` | `enable_gravity_comp: false` | Let gz_ros2_control KDL handle gravity |
| `gazebo_cheating_force.launch.py` | Removed warmstart/pause/unpause, back to direct launch | Double-counting was the real root cause, not startup timing |
| `direct_force_world.sdf` | Removed drawing_paper model | Decoration only, user requested removal |

### Results
| Metric | Baseline (hybrid) | This ITR | Delta |
|--------|-------------------|----------|-------|
| exy mean (mm) | 1.67 | 0.22 | 7.5x better |
| ez mean (mm) | 0.41 | 0.03 | 13x better |
| Startup stable? | — | YES (peak 0.20mm) | No fling |

### Conclusion
**Verdict:** PASSED — Gate 1 AND Gate 2 both cleared. Root cause was double gravity compensation.

### Do Not Repeat
- [ ] NEVER enable Pinocchio gravity comp when using gz_ros2_control effort mode — KDL already does it internally
- [ ] Warmstart/pause hacks were unnecessary once the real bug was found

### Next
Add force control (fz tracking) to make this a true contact cheating controller, not just free-space circle.
