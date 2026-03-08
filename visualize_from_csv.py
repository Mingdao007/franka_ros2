#!/usr/bin/env python3
"""
Generate Force Control Visualization from CSV Log - Simplified Version
"""
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# Load data using numpy
csv_path = '/home/andy/franka_ros2_ws/internal_force_log.csv'
print(f"Loading data from: {csv_path}")
data = np.genfromtxt(csv_path, delimiter=',', skip_header=1)
print(f"Loaded {len(data)} data points")

t = data[:, 0]
f_des = data[:, 1]
f_meas = data[:, 2]

print(f"Time range: {t.min():.2f} - {t.max():.2f} s")

# Calculate error
error = f_des - f_meas

# Create figure with 3 subplots
try:
    plt.style.use('seaborn-whitegrid')
except:
    try:
        plt.style.use('ggplot')
    except:
        pass

fig, axes = plt.subplots(3, 1, figsize=(12, 10))
fig.suptitle('Franka end-effector force control tracking results', fontsize=16, fontweight='bold')

# --- Plot 1: Z-Axis Force Tracking Performance ---
ax1 = axes[0]
ax1.plot(t, f_des, 'b-', linewidth=2, label='Desired force')
ax1.plot(t, f_meas, 'r-', linewidth=1.5, alpha=0.8, label='Measured force')
ax1.fill_between(t, f_des, f_meas, alpha=0.2, color='purple')
ax1.set_ylabel('Force z (N)', fontsize=11)
ax1.set_title('Z-axis force tracking performance', fontweight='bold', fontsize=12)
ax1.legend(loc='upper right', fontsize=10)
ax1.grid(True, alpha=0.3)

# --- Plot 2: Tracking Error ---
ax2 = axes[1]
ax2.plot(t, error, 'g-', linewidth=1)
ax2.axhline(y=0, color='k', linestyle='-', linewidth=1.5)
ax2.set_ylabel('Error (N)', fontsize=11)
ax2.set_title('Force tracking error (desired - measured)', fontweight='bold', fontsize=12)
ax2.grid(True, alpha=0.3)

# --- Plot 3: Zoomed View (Last 3 seconds) ---
ax3 = axes[2]
zoom_start = max(0, t[-1] - 3)
mask = t >= zoom_start
ax3.plot(t[mask], f_des[mask], 'b-', linewidth=2.5, label='Desired force')
ax3.plot(t[mask], f_meas[mask], 'r-', linewidth=2, alpha=0.8, label='Measured force')
ax3.fill_between(t[mask], f_des[mask], f_meas[mask], alpha=0.3, color='purple')
ax3.set_xlabel('Time (s)', fontsize=11)
ax3.set_ylabel('Force z (N)', fontsize=11)
ax3.set_title('Zoomed view - steady state (last 3 seconds)', fontweight='bold', fontsize=12)
ax3.legend(loc='upper right', fontsize=10)
ax3.grid(True, alpha=0.3)

# Add timestamp
timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
fig.text(0.99, 0.01, f'Generated: {timestamp}', ha='right', fontsize=9, color='gray')

plt.tight_layout()

# Save
save_path = '/home/andy/franka_ros2_ws/force_tracking_results.png'
plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor='white')

print(f"\n{'='*50}")
print(f"Plot saved to: {save_path}")
print(f"{'='*50}")

plt.show()
print("\nVisualization complete!")
