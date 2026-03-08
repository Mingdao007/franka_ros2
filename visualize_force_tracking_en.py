#!/usr/bin/env python3
"""
Force Tracking Visualization (English Version)
===============================================
Real-time + final static plot for force control experiments.
Auto-closes after specified duration and generates a clean final plot.

Usage:
    python3 visualize_force_tracking_en.py [--duration 15] [--output force_result.png]
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import threading
import argparse
import time
from datetime import datetime
import sys


class ForceTrackingVisualizer(Node):
    def __init__(self):
        super().__init__('force_tracking_visualizer')
        
        # Data storage (keep last 3000 samples for ~30s at 100Hz)
        self.max_len = 3000
        self.time_data = deque(maxlen=self.max_len)
        
        # Commanded force (desired)
        self.cmd_fx = deque(maxlen=self.max_len)
        self.cmd_fy = deque(maxlen=self.max_len)
        self.cmd_fz = deque(maxlen=self.max_len)
        
        # Estimated force (measured)
        self.est_fx = deque(maxlen=self.max_len)
        self.est_fy = deque(maxlen=self.max_len)
        self.est_fz = deque(maxlen=self.max_len)
        
        self.start_time = None
        self.lock = threading.Lock()
        self.data_count = 0
        
        # Subscribe to topics
        self.cmd_sub = self.create_subscription(
            WrenchStamped,
            '/direct_force_example_controller/commanded_wrench',
            self.cmd_callback,
            10
        )
        
        self.est_sub = self.create_subscription(
            WrenchStamped,
            '/direct_force_example_controller/estimated_wrench',
            self.est_callback,
            10
        )
        
        self.get_logger().info('Force tracking visualizer started')
        
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
        """Get snapshot of current data"""
        with self.lock:
            return {
                'time': np.array(self.time_data),
                'cmd_fx': np.array(self.cmd_fx),
                'cmd_fy': np.array(self.cmd_fy),
                'cmd_fz': np.array(self.cmd_fz),
                'est_fx': np.array(self.est_fx),
                'est_fy': np.array(self.est_fy),
                'est_fz': np.array(self.est_fz),
                'count': self.data_count
            }


def run_realtime_visualization(node, duration):
    """Run real-time visualization for specified duration"""
    plt.style.use('default')
    plt.rcParams.update({
        'font.size': 16,
        'axes.titlesize': 18,
        'axes.labelsize': 16,
        'legend.fontsize': 14,
        'xtick.labelsize': 14,
        'ytick.labelsize': 14,
    })
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    ax_fx, ax_fy, ax_fz = axes
    
    fig.suptitle('Force Tracking (Real-Time)', fontsize=20, fontweight='bold')
    
    # Setup lines
    line_cmd_fx, = ax_fx.plot([], [], 'b-', linewidth=2.5, label='Commanded')
    line_est_fx, = ax_fx.plot([], [], 'r-', linewidth=2, alpha=0.8, label='Estimated')
    ax_fx.set_ylabel('Fx (N)')
    ax_fx.legend(loc='upper right')
    ax_fx.grid(True, alpha=0.3)
    ax_fx.set_xlim(0, duration + 5)
    ax_fx.set_ylim(-5, 5)
    
    line_cmd_fy, = ax_fy.plot([], [], 'b-', linewidth=2.5, label='Commanded')
    line_est_fy, = ax_fy.plot([], [], 'r-', linewidth=2, alpha=0.8, label='Estimated')
    ax_fy.set_ylabel('Fy (N)')
    ax_fy.legend(loc='upper right')
    ax_fy.grid(True, alpha=0.3)
    ax_fy.set_xlim(0, duration + 5)
    ax_fy.set_ylim(-5, 5)
    
    line_cmd_fz, = ax_fz.plot([], [], 'b-', linewidth=2.5, label='Commanded')
    line_est_fz, = ax_fz.plot([], [], 'r-', linewidth=2, alpha=0.8, label='Estimated')
    ax_fz.set_xlabel('Time (s)')
    ax_fz.set_ylabel('Fz (N)')
    ax_fz.legend(loc='upper right')
    ax_fz.grid(True, alpha=0.3)
    ax_fz.set_xlim(0, duration + 5)
    ax_fz.set_ylim(-15, 5)
    
    timer_text = fig.text(0.02, 0.02, '', fontsize=14)
    
    plt.tight_layout(rect=[0, 0.04, 1, 0.95])
    
    start_time = time.time()
    
    def update(frame):
        elapsed = time.time() - start_time
        remaining = max(0, duration - elapsed)
        
        data = node.get_data()
        timer_text.set_text(f'Remaining: {remaining:.1f}s | Samples: {data["count"]}')
        
        if elapsed >= duration:
            plt.close(fig)
            return
        
        if len(data['time']) < 2:
            return
        
        t = data['time']
        
        # Update curves
        min_len = min(len(t), len(data['cmd_fx']), len(data['est_fx']))
        if min_len > 0:
            line_cmd_fx.set_data(t[:min_len], data['cmd_fx'][:min_len])
            line_est_fx.set_data(t[:min_len], data['est_fx'][:min_len])
        
        min_len = min(len(t), len(data['cmd_fy']), len(data['est_fy']))
        if min_len > 0:
            line_cmd_fy.set_data(t[:min_len], data['cmd_fy'][:min_len])
            line_est_fy.set_data(t[:min_len], data['est_fy'][:min_len])
        
        min_len = min(len(t), len(data['cmd_fz']), len(data['est_fz']))
        if min_len > 0:
            line_cmd_fz.set_data(t[:min_len], data['cmd_fz'][:min_len])
            line_est_fz.set_data(t[:min_len], data['est_fz'][:min_len])
        
        # Dynamic Y-axis
        for ax, cmd_key, est_key in [
            (ax_fx, 'cmd_fx', 'est_fx'),
            (ax_fy, 'cmd_fy', 'est_fy'),
            (ax_fz, 'cmd_fz', 'est_fz')
        ]:
            if len(data[cmd_key]) > 0 and len(data[est_key]) > 0:
                all_d = np.concatenate([data[cmd_key], data[est_key]])
                y_min, y_max = np.min(all_d), np.max(all_d)
                margin = max(1, (y_max - y_min) * 0.2)
                ax.set_ylim(y_min - margin, y_max + margin)
    
    ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()


def generate_final_plot(data, output_file):
    """Generate clean final static plot"""
    if data['count'] < 10:
        print('Not enough data for final plot')
        return None
    
    plt.style.use('default')
    plt.rcParams.update({
        'font.size': 16,
        'axes.titlesize': 18,
        'axes.labelsize': 16,
        'legend.fontsize': 14,
        'xtick.labelsize': 14,
        'ytick.labelsize': 14,
    })
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    ax_fx, ax_fy, ax_fz = axes
    
    fig.suptitle('Franka End-Effector Force Tracking Result', 
                fontsize=20, fontweight='bold')
    
    t = data['time']
    
    # Plot Fx
    min_len = min(len(t), len(data['cmd_fx']), len(data['est_fx']))
    if min_len > 0:
        ax_fx.plot(t[:min_len], data['cmd_fx'][:min_len], 'b-', 
                  linewidth=2.5, label='Commanded Fx')
        ax_fx.plot(t[:min_len], data['est_fx'][:min_len], 'r-', 
                  linewidth=2, alpha=0.8, label='Estimated Fx')
    ax_fx.set_ylabel('Force X (N)')
    ax_fx.legend(loc='upper right')
    ax_fx.grid(True, alpha=0.3)
    ax_fx.set_xlim(0, t[-1] if len(t) > 0 else 20)
    
    # Plot Fy
    min_len = min(len(t), len(data['cmd_fy']), len(data['est_fy']))
    if min_len > 0:
        ax_fy.plot(t[:min_len], data['cmd_fy'][:min_len], 'b-', 
                  linewidth=2.5, label='Commanded Fy')
        ax_fy.plot(t[:min_len], data['est_fy'][:min_len], 'r-', 
                  linewidth=2, alpha=0.8, label='Estimated Fy')
    ax_fy.set_ylabel('Force Y (N)')
    ax_fy.legend(loc='upper right')
    ax_fy.grid(True, alpha=0.3)
    
    # Plot Fz
    min_len = min(len(t), len(data['cmd_fz']), len(data['est_fz']))
    if min_len > 0:
        ax_fz.plot(t[:min_len], data['cmd_fz'][:min_len], 'b-', 
                  linewidth=2.5, label='Commanded Fz')
        ax_fz.plot(t[:min_len], data['est_fz'][:min_len], 'r-', 
                  linewidth=2, alpha=0.8, label='Estimated Fz')
    ax_fz.set_xlabel('Time (s)')
    ax_fz.set_ylabel('Force Z (N)')
    ax_fz.legend(loc='upper right')
    ax_fz.grid(True, alpha=0.3)
    
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    
    # Save
    if output_file:
        output_path = output_file
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f'/home/andy/franka_ros2_ws/force_tracking_{timestamp}.png'
    
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f'\n[SAVED] {output_path}')
    
    plt.show()
    return output_path


def main(args=None):
    parser = argparse.ArgumentParser(description='Force Tracking Visualization')
    parser.add_argument('--duration', type=float, default=15.0,
                       help='Duration for real-time visualization (seconds)')
    parser.add_argument('--output', type=str, default=None,
                       help='Output filename for final plot')
    cli_args = parser.parse_args()
    
    rclpy.init(args=args)
    node = ForceTrackingVisualizer()
    
    # Spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print('\n' + '='*50)
    print('  Force Tracking Visualization')
    print('='*50)
    print(f'  Duration: {cli_args.duration}s')
    print('='*50 + '\n')
    
    try:
        # Real-time visualization
        run_realtime_visualization(node, cli_args.duration)
        
        # Get final data and generate static plot
        print('\nGenerating final plot...')
        final_data = node.get_data()
        generate_final_plot(final_data, cli_args.output)
        
    except KeyboardInterrupt:
        print('\nStopped by user')
        final_data = node.get_data()
        if final_data['count'] > 10:
            generate_final_plot(final_data, cli_args.output)
    
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('Done')


if __name__ == '__main__':
    main()
