#!/usr/bin/env python3
"""
Force Control Tracking - Data Collection & Visualization
Automatically collects data and saves the final plot
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
from collections import deque
import threading
from datetime import datetime
import time
import sys

class ForceDataCollector(Node):
    def __init__(self):
        super().__init__('force_data_collector')
        
        self.max_len = 2000
        self.time_data = deque(maxlen=self.max_len)
        
        self.cmd_fx = deque(maxlen=self.max_len)
        self.cmd_fy = deque(maxlen=self.max_len)
        self.cmd_fz = deque(maxlen=self.max_len)
        
        self.est_fx = deque(maxlen=self.max_len)
        self.est_fy = deque(maxlen=self.max_len)
        self.est_fz = deque(maxlen=self.max_len)
        
        self.start_time = None
        self.lock = threading.Lock()
        self.data_count = 0
        
        self.cmd_sub = self.create_subscription(
            WrenchStamped,
            '/direct_force_example_controller/commanded_wrench',
            self.cmd_callback, 10)
        
        self.est_sub = self.create_subscription(
            WrenchStamped,
            '/direct_force_example_controller/estimated_wrench',
            self.est_callback, 10)
        
        self.get_logger().info('Force Data Collector Started')
        
    def cmd_callback(self, msg):
        with self.lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.time_data.append(current_time)
            
            self.cmd_fx.append(msg.wrench.force.x)
            self.cmd_fy.append(msg.wrench.force.y)
            self.cmd_fz.append(msg.wrench.force.z)
            self.data_count += 1
    
    def est_callback(self, msg):
        with self.lock:
            self.est_fx.append(msg.wrench.force.x)
            self.est_fy.append(msg.wrench.force.y)
            self.est_fz.append(msg.wrench.force.z)
    
    def get_data(self):
        with self.lock:
            n = min(len(self.time_data), len(self.cmd_fz), len(self.est_fz))
            return {
                'time': np.array(list(self.time_data)[:n]),
                'cmd_fx': np.array(list(self.cmd_fx)[:n]),
                'cmd_fy': np.array(list(self.cmd_fy)[:n]),
                'cmd_fz': np.array(list(self.cmd_fz)[:n]),
                'est_fx': np.array(list(self.est_fx)[:n]),
                'est_fy': np.array(list(self.est_fy)[:n]),
                'est_fz': np.array(list(self.est_fz)[:n]),
                'count': self.data_count
            }


def create_visualization(data, save_path):
    """Create and save the force tracking visualization"""
    
    if len(data['time']) < 10:
        print("ERROR: Not enough data collected!")
        return False
    
    t = data['time']
    
    # Calculate errors
    error_fz = data['cmd_fz'] - data['est_fz']
    error_fx = data['cmd_fx'] - data['est_fx']
    error_fy = data['cmd_fy'] - data['est_fy']
    
    # Calculate metrics
    mean_err = np.mean(error_fz)
    std_err = np.std(error_fz)
    max_err = np.max(np.abs(error_fz))
    rms_err = np.sqrt(np.mean(error_fz**2))
    
    # Steady-state (last 30%)
    n = len(error_fz)
    ss_data = error_fz[int(n*0.7):]
    ss_err = np.mean(np.abs(ss_data))
    
    # Create figure
    try:
        plt.style.use('seaborn-whitegrid')
    except:
        try:
            plt.style.use('ggplot')
        except:
            pass
    
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('Franka End-Effector Force Control Tracking Results', fontsize=18, fontweight='bold', y=0.98)
    
    gs = GridSpec(4, 3, figure=fig, hspace=0.4, wspace=0.3)
    
    # --- Row 1: Force Tracking ---
    ax1 = fig.add_subplot(gs[0, :2])
    ax1.plot(t, data['cmd_fz'], 'b-', linewidth=2, label='Desired Fz')
    ax1.plot(t, data['est_fz'], 'r-', linewidth=1.5, alpha=0.8, label='Actual Fz')
    ax1.fill_between(t, data['cmd_fz'], data['est_fz'], alpha=0.3, color='purple')
    ax1.set_xlabel('Time (s)', fontsize=11)
    ax1.set_ylabel('Force Z (N)', fontsize=11)
    ax1.set_title('Z-Axis Force Tracking (Main Control Direction)', fontweight='bold', fontsize=12)
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    # --- Row 1 Right: Metrics ---
    ax_metrics = fig.add_subplot(gs[0, 2])
    ax_metrics.axis('off')
    
    if ss_err < 0.2:
        rating = "EXCELLENT"
        color = 'green'
    elif ss_err < 1.0:
        rating = "PASS"
        color = 'blue'
    else:
        rating = "NEEDS TUNING"
        color = 'red'
    
    metrics_text = (
        f"Performance Metrics\n"
        f"{'='*25}\n\n"
        f"Data Points:     {data['count']}\n"
        f"Duration:        {t[-1]:.1f} s\n\n"
        f"Z-Axis Force Control:\n"
        f"  Mean Error:    {mean_err:+.4f} N\n"
        f"  Std Dev:       {std_err:.4f} N\n"
        f"  Max Error:     {max_err:.4f} N\n"
        f"  RMS Error:     {rms_err:.4f} N\n\n"
        f"Steady-State:    {ss_err:.4f} N\n\n"
        f"Rating: {rating}"
    )
    
    ax_metrics.text(0.1, 0.95, metrics_text, transform=ax_metrics.transAxes,
                   fontsize=11, verticalalignment='top', fontfamily='monospace',
                   bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9, edgecolor='gray'))
    
    # --- Row 2: X and Y Force ---
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t, data['cmd_fx'], 'b-', linewidth=1.5, label='Desired')
    ax2.plot(t, data['est_fx'], 'r-', linewidth=1.2, alpha=0.8, label='Actual')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Force X (N)')
    ax2.set_title('X-Axis Force', fontweight='bold')
    ax2.legend(loc='upper right', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(t, data['cmd_fy'], 'b-', linewidth=1.5, label='Desired')
    ax3.plot(t, data['est_fy'], 'r-', linewidth=1.2, alpha=0.8, label='Actual')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Force Y (N)')
    ax3.set_title('Y-Axis Force', fontweight='bold')
    ax3.legend(loc='upper right', fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # --- Row 2 Right: 3D scatter ---
    ax3d = fig.add_subplot(gs[1, 2])
    ax3d.scatter(data['cmd_fz'], data['est_fz'], c=t, cmap='viridis', s=10, alpha=0.6)
    min_f = min(np.min(data['cmd_fz']), np.min(data['est_fz']))
    max_f = max(np.max(data['cmd_fz']), np.max(data['est_fz']))
    ax3d.plot([min_f, max_f], [min_f, max_f], 'r--', linewidth=2, label='Ideal')
    ax3d.set_xlabel('Desired Fz (N)')
    ax3d.set_ylabel('Actual Fz (N)')
    ax3d.set_title('Desired vs Actual', fontweight='bold')
    ax3d.legend()
    ax3d.grid(True, alpha=0.3)
    ax3d.set_aspect('equal', adjustable='box')
    
    # --- Row 3: Error over time ---
    ax4 = fig.add_subplot(gs[2, :2])
    ax4.plot(t, error_fz, 'g-', linewidth=1.5)
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax4.axhline(y=0.2, color='blue', linestyle='--', alpha=0.5, label='Excellent (+/-0.2N)')
    ax4.axhline(y=-0.2, color='blue', linestyle='--', alpha=0.5)
    ax4.axhline(y=1.0, color='orange', linestyle='--', alpha=0.5, label='Pass (+/-1.0N)')
    ax4.axhline(y=-1.0, color='orange', linestyle='--', alpha=0.5)
    ax4.fill_between(t, -0.2, 0.2, alpha=0.1, color='green')
    ax4.set_xlabel('Time (s)', fontsize=11)
    ax4.set_ylabel('Error (N)', fontsize=11)
    ax4.set_title('Z-Axis Tracking Error Over Time', fontweight='bold', fontsize=12)
    ax4.legend(loc='upper right', fontsize=9)
    ax4.grid(True, alpha=0.3)
    
    # --- Row 3 Right: Error histogram ---
    ax5 = fig.add_subplot(gs[2, 2])
    ax5.hist(error_fz, bins=40, color='steelblue', edgecolor='white', alpha=0.7, density=True)
    ax5.axvline(x=0, color='r', linestyle='--', linewidth=2)
    ax5.axvline(x=mean_err, color='orange', linestyle='-', linewidth=2, label=f'Mean={mean_err:.3f}')
    ax5.set_xlabel('Error (N)')
    ax5.set_ylabel('Density')
    ax5.set_title('Error Distribution', fontweight='bold')
    ax5.legend(fontsize=9)
    ax5.grid(True, alpha=0.3)
    
    # --- Row 4: Zoomed view ---
    ax6 = fig.add_subplot(gs[3, :])
    zoom_start = max(0, t[-1] - 5)  # Last 5 seconds
    mask = t >= zoom_start
    ax6.plot(t[mask], data['cmd_fz'][mask], 'b-', linewidth=2.5, label='Desired Fz')
    ax6.plot(t[mask], data['est_fz'][mask], 'r-', linewidth=2, alpha=0.8, label='Actual Fz')
    ax6.fill_between(t[mask], data['cmd_fz'][mask], data['est_fz'][mask], alpha=0.3, color='purple')
    ax6.set_xlabel('Time (s)', fontsize=11)
    ax6.set_ylabel('Force Z (N)', fontsize=11)
    ax6.set_title('Zoomed View (Last 5 seconds)', fontweight='bold', fontsize=12)
    ax6.legend(loc='upper right', fontsize=10)
    ax6.grid(True, alpha=0.3)
    
    # Add timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    fig.text(0.99, 0.01, f'Generated: {timestamp}', ha='right', fontsize=9, color='gray')
    
    plt.tight_layout(rect=[0, 0.02, 1, 0.96])
    
    # Save
    plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor='white')
    print(f"\n{'='*50}")
    print(f"Plot saved to: {save_path}")
    print(f"{'='*50}")
    
    return True


def main():
    rclpy.init()
    
    collector = ForceDataCollector()
    
    # Spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(collector,), daemon=True)
    spin_thread.start()
    
    collection_time = 15  # seconds
    
    print("\n" + "="*50)
    print("  Force Control Data Collection & Visualization")
    print("="*50)
    print(f"\nCollecting data for {collection_time} seconds...")
    print("Please wait...\n")
    
    # Progress bar
    for i in range(collection_time):
        time.sleep(1)
        data = collector.get_data()
        progress = int((i+1) / collection_time * 40)
        bar = '=' * progress + '>' + ' ' * (40 - progress)
        sys.stdout.write(f'\r[{bar}] {i+1}/{collection_time}s | Data points: {data["count"]}')
        sys.stdout.flush()
    
    print("\n\nData collection complete!")
    
    # Get final data
    data = collector.get_data()
    print(f"Total data points collected: {data['count']}")
    
    # Generate plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = f'/home/andy/franka_ros2_ws/force_tracking_results_{timestamp}.png'
    
    print("\nGenerating visualization...")
    success = create_visualization(data, save_path)
    
    if success:
        print("\nVisualization complete!")
        # Also show the plot
        plt.show()
    
    # Cleanup
    collector.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
