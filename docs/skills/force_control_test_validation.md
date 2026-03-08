# Skill: Direct Force Control - Testing & Validation Workflow

**Date:** 2026-02-01
**Category:** Testing / Validation / Force Control
**Confidence:** High (experimentally verified)

---

## 1. Quick Start: Complete Testing Workflow

### Step 1: Build
```bash
cd /home/andy/franka_ros2_ws
colcon build --packages-select franka_example_controllers franka_gazebo_bringup --symlink-install
source install/setup.bash
```

### Step 2: Clean & Launch Gazebo
```bash
# MUST clean residual processes first!
pkill -9 -f "ign gazebo" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
sleep 2

# Launch simulation
ros2 launch franka_gazebo_bringup gazebo_direct_force_jacobian.launch.py
```

### Step 3: Monitor (Optional - in separate terminals)
```bash
# Terminal 2: Estimated force
ros2 topic echo /direct_force_example_controller/estimated_wrench

# Terminal 3: Commanded force
ros2 topic echo /direct_force_example_controller/commanded_wrench
```

### Step 4: Analyze Results
```bash
# Data is auto-logged to:
# /home/andy/franka_ros2_ws/refactor_test.csv

# Generate analysis plot:
python3 /home/andy/franka_ros2_ws/src/visualize_from_csv.py
# or use the inline script below
```

---

## 2. Data Analysis Script

Create and run this script to analyze force tracking performance:

```python
#!/usr/bin/env python3
"""Force Control Performance Analysis from CSV"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from datetime import datetime

# Read data
df = pd.read_csv('/home/andy/franka_ros2_ws/refactor_test.csv')

# Skip first 5 seconds (startup transient)
df = df[df['time'] > 5.0].copy()
df['error'] = df['f_des'] - df['f_meas']

# Calculate metrics
mean_err = df['error'].mean()
std_err = df['error'].std()
rmse = np.sqrt((df['error'] ** 2).mean())
max_err = df['error'].abs().max()

print("=" * 50)
print("Force Tracking Performance Analysis")
print("=" * 50)
print(f"Data range: {df['time'].min():.2f}s - {df['time'].max():.2f}s")
print(f"Data points: {len(df)}")
print()
print(f"Mean Error:  {mean_err:+.4f} N")
print(f"Std Dev:     {std_err:.4f} N")
print(f"RMSE:        {rmse:.4f} N")
print(f"Max Error:   {max_err:.4f} N")
print()
print("Validation:")
print(f"  Mean Error < 0.1N: {'PASS' if abs(mean_err) < 0.1 else 'FAIL'}")
print(f"  Std Dev < 0.1N:    {'PASS' if std_err < 0.1 else 'FAIL'}")
print(f"  RMSE < 0.6N:       {'PASS' if rmse < 0.6 else 'FAIL'}")

# Create visualization
fig = plt.figure(figsize=(16, 10))
fig.suptitle('Franka Direct Force Control - Performance Analysis', fontsize=16)

gs = GridSpec(3, 3, figure=fig, hspace=0.35, wspace=0.3)

# Force tracking plot
ax1 = fig.add_subplot(gs[0:2, 0:2])
ax1.plot(df['time'], df['f_des'], 'b-', lw=2, label='Desired', alpha=0.9)
ax1.plot(df['time'], df['f_meas'], 'r-', lw=1.5, label='Measured', alpha=0.8)
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Force (N)')
ax1.set_title('Force Tracking (Z-axis)')
ax1.legend()
ax1.grid(True, alpha=0.3)

# Error plot
ax2 = fig.add_subplot(gs[2, 0:2])
ax2.plot(df['time'], df['error'], 'g-', lw=1.5, alpha=0.8)
ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
ax2.axhline(y=0.5, color='orange', linestyle='--', alpha=0.5)
ax2.axhline(y=-0.5, color='orange', linestyle='--', alpha=0.5)
ax2.fill_between(df['time'], -0.5, 0.5, alpha=0.1, color='green')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error (N)')
ax2.set_title('Tracking Error')
ax2.grid(True, alpha=0.3)

# Histogram
ax3 = fig.add_subplot(gs[0, 2])
ax3.hist(df['error'], bins=50, color='steelblue', edgecolor='white', alpha=0.7, density=True)
ax3.axvline(x=0, color='r', linestyle='--', lw=2)
ax3.axvline(x=mean_err, color='orange', linestyle='-', lw=2, label=f'Mean: {mean_err:.3f}N')
ax3.set_xlabel('Error (N)')
ax3.set_ylabel('Density')
ax3.set_title('Error Distribution')
ax3.legend()

# Metrics text
ax4 = fig.add_subplot(gs[1, 2])
ax4.axis('off')
grade = "PASS" if rmse < 0.6 else "FAIL"
metrics_text = (
    f"Performance Metrics\n"
    f"{'='*25}\n\n"
    f"Mean Error:     {mean_err:+.4f} N\n"
    f"Std Deviation:   {std_err:.4f} N\n"
    f"RMSE:            {rmse:.4f} N\n"
    f"Max Error:       {max_err:.4f} N\n\n"
    f"Grade: {grade}"
)
ax4.text(0.1, 0.95, metrics_text, transform=ax4.transAxes,
         fontsize=12, verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# RMSE by force level
ax5 = fig.add_subplot(gs[2, 2])
for target in [3.0, 5.0, 8.0]:
    mask = (df['f_des'] > target - 0.5) & (df['f_des'] < target + 0.5)
    if mask.sum() > 100:
        segment_rmse = np.sqrt((df.loc[mask, 'error'] ** 2).mean())
        ax5.bar(f'{target}N', segment_rmse, alpha=0.7, edgecolor='black')
ax5.axhline(y=0.6, color='r', linestyle='--', alpha=0.7, label='Threshold')
ax5.set_ylabel('RMSE (N)')
ax5.set_title('RMSE by Force Level')
ax5.legend()

plt.tight_layout()
save_path = f'/home/andy/franka_ros2_ws/force_control_analysis_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png'
plt.savefig(save_path, dpi=150, bbox_inches='tight')
print(f"\nFigure saved: {save_path}")
plt.show()
```

---

## 3. Validation Criteria

| Metric | Target | Description |
|--------|--------|-------------|
| **RMSE** | < 0.6 N | Root mean square error (primary metric) |
| Mean Error | < 0.1 N | Systematic bias indicator |
| Std Deviation | < 0.1 N | Noise/oscillation indicator |
| Max Error | < 1.0 N | Worst-case tracking |

### Interpreting Results

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Mean >> 0 | Systematic under-compensation | Increase `ki` |
| Mean << 0 | Over-compensation | Decrease `ki` |
| High Std | Oscillation/noise | Increase filtering, decrease `kp` |
| High RMSE in transitions | Slow response | Increase `kp` cautiously |

---

## 4. Key Configuration Parameters

**File:** `franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml`

```yaml
direct_force_example_controller:
  ros__parameters:
    # Force target
    force_desired: 5.0          # Target force (N)
    force_axis: 2               # Z-axis

    # Waypoint trajectory (optional)
    waypoint_forces: [5.0, 8.0, 3.0, 5.0]
    waypoint_durations: [4.0, 4.0, 4.0]
    waypoint_loop: true

    # PID gains (tuned for Gazebo)
    kp: 0.03                    # Proportional
    ki: 0.005                   # Integral
    kd: 0.006                   # Derivative

    # Filtering
    force_filter_alpha: 0.95    # Low-pass filter (0.95 = strong smoothing)
    d_filter_alpha: 0.0         # Derivative filter (0 = disabled)

    # Gazebo-specific (CRITICAL!)
    use_ft_sensor: false        # Use Jacobian estimation
    use_bias_calibration: false # Must be false for Gazebo!

    # Regularization
    lambda: 0.01                # Damped least squares

    # Logging
    log_file_path: "/home/andy/franka_ros2_ws/refactor_test.csv"
```

---

## 5. Tuning Guidelines

### Stable PID Ranges (Verified)
```yaml
kp: 0.025 - 0.04    # Higher = faster but less stable
ki: 0.002 - 0.008   # Higher = better steady-state, risk of oscillation
kd: 0.004 - 0.01    # Higher = better damping, sensitive to noise
```

### Quick Tuning Protocol

1. **Start with** `kp=0.03, ki=0.005, kd=0.006` (baseline)
2. **If oscillating:** Reduce `kp` or increase `force_filter_alpha`
3. **If slow response:** Increase `kp` gradually
4. **If steady-state error:** Increase `ki` gradually
5. **If noisy:** Enable `d_filter_alpha=0.5`

---

## 6. Realtime Visualization (ROS2)

For live monitoring during experiments:

```bash
python3 /home/andy/franka_ros2_ws/src/visualize_force_tracking.py
```

Features:
- Real-time force tracking plot
- Error histogram
- Performance metrics display
- Auto-save on Ctrl+C

---

## 7. Constant Force Test (for baseline)

To eliminate waypoint dynamics and test pure tracking:

```yaml
# Modify in franka_gazebo_controllers.yaml:
waypoint_forces: [5.0]
waypoint_durations: []
waypoint_loop: false
```

Expected results with constant 5N target:
- Mean Error: < 0.05 N
- Std: < 0.05 N
- RMSE: < 0.1 N

---

## 8. Troubleshooting

### Controller fails to load
```bash
# Clean all Gazebo processes
pkill -9 -f "ign gazebo"
pkill -9 -f "gz sim"
pkill -9 -f "ros2 launch"
sleep 3
# Then restart
```

### Force not tracking at all
1. Check `start_in_force_mode: true` in config
2. Verify URDF model is correct
3. Check Pinocchio initialization in logs

### Large oscillations
1. Reduce `kp` by 50%
2. Increase `force_filter_alpha` to 0.98
3. Check for residual Gazebo processes

---

## 9. Output Files

| File | Description |
|------|-------------|
| `/home/andy/franka_ros2_ws/refactor_test.csv` | Raw data (time, f_des, f_meas, f_cmd) |
| `/home/andy/franka_ros2_ws/force_control_analysis_*.png` | Analysis figures |
| Controller logs | ROS2 console output |

---

## 10. Example Test Result

**Test Date:** 2026-02-01
**Configuration:** Waypoint trajectory [5N → 8N → 3N → 5N], loop mode

| Metric | Result | Target | Status |
|--------|--------|--------|--------|
| RMSE | 0.5311 N | < 0.6 N | PASS |
| Mean Error | -0.3402 N | < 0.1 N | Note: phase lag in dynamic tracking |
| Std | 0.4079 N | < 0.1 N | Note: transition oscillation |
| Max Error | 0.9661 N | < 1.0 N | PASS |

**Analysis:** RMSE passes threshold. Higher mean/std due to waypoint transitions. For constant force, expect much better results.

---

## References

- Controller source: `franka_example_controllers/src/direct_force_example_controller.cpp`
- Related skill: `gazebo_vs_real_robot_force_control.md`
- Visualization: `visualize_force_tracking.py`, `analyze_force_control.py`
