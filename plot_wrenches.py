#!/usr/bin/env python3
"""
Plot commanded and estimated wrenches in real-time from ROS2 topics.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import threading

class WrenchPlotter(Node):
    def __init__(self):
        super().__init__('wrench_plotter')
        
        # Data storage (keep last 500 samples)
        self.max_len = 500
        self.time_data = deque(maxlen=self.max_len)
        
        # Commanded wrench
        self.cmd_fx = deque(maxlen=self.max_len)
        self.cmd_fy = deque(maxlen=self.max_len)
        self.cmd_fz = deque(maxlen=self.max_len)
        
        # Estimated wrench
        self.est_fx = deque(maxlen=self.max_len)
        self.est_fy = deque(maxlen=self.max_len)
        self.est_fz = deque(maxlen=self.max_len)
        
        self.start_time = None
        self.lock = threading.Lock()
        
        # Subscribers
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
        
        self.get_logger().info('Wrench plotter started. Collecting data...')
        
    def cmd_callback(self, msg):
        with self.lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.time_data.append(current_time)
            
            self.cmd_fx.append(msg.wrench.force.x)
            self.cmd_fy.append(msg.wrench.force.y)
            self.cmd_fz.append(msg.wrench.force.z)
    
    def est_callback(self, msg):
        with self.lock:
            self.est_fx.append(msg.wrench.force.x)
            self.est_fy.append(msg.wrench.force.y)
            self.est_fz.append(msg.wrench.force.z)
    
    def plot(self):
        with self.lock:
            if len(self.time_data) == 0:
                self.get_logger().warn('No data collected yet!')
                return
            
            time = np.array(self.time_data)
            
            # Commanded wrench
            cmd_fx = np.array(self.cmd_fx)
            cmd_fy = np.array(self.cmd_fy)
            cmd_fz = np.array(self.cmd_fz)
            
            # Estimated wrench (pad if shorter)
            n_samples = len(time)
            est_fx = np.array(list(self.est_fx)[-n_samples:])
            est_fy = np.array(list(self.est_fy)[-n_samples:])
            est_fz = np.array(list(self.est_fz)[-n_samples:])
        
        # Create figure with 6 subplots (3x2)
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle('Force Control Wrenches', fontsize=16, fontweight='bold')
        
        # Commanded wrench plots
        axes[0, 0].plot(time, cmd_fx, 'b-', linewidth=1.5, label='Commanded Fx')
        axes[0, 0].set_ylabel('Force X (N)', fontsize=11)
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend(loc='upper right')
        axes[0, 0].set_title('Commanded Wrench - X Axis', fontweight='bold')
        
        axes[1, 0].plot(time, cmd_fy, 'b-', linewidth=1.5, label='Commanded Fy')
        axes[1, 0].set_ylabel('Force Y (N)', fontsize=11)
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].legend(loc='upper right')
        axes[1, 0].set_title('Commanded Wrench - Y Axis', fontweight='bold')
        
        axes[2, 0].plot(time, cmd_fz, 'b-', linewidth=1.5, label='Commanded Fz')
        axes[2, 0].axhline(y=-2.0, color='r', linestyle='--', linewidth=1, label='Target: -2.0 N')
        axes[2, 0].set_xlabel('Time (s)', fontsize=11)
        axes[2, 0].set_ylabel('Force Z (N)', fontsize=11)
        axes[2, 0].grid(True, alpha=0.3)
        axes[2, 0].legend(loc='upper right')
        axes[2, 0].set_title('Commanded Wrench - Z Axis (Control Output)', fontweight='bold')
        
        # Estimated wrench plots
        axes[0, 1].plot(time, est_fx, 'r-', linewidth=1.5, label='Estimated Fx')
        axes[0, 1].set_ylabel('Force X (N)', fontsize=11)
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend(loc='upper right')
        axes[0, 1].set_title('Estimated Wrench - X Axis', fontweight='bold')
        
        axes[1, 1].plot(time, est_fy, 'r-', linewidth=1.5, label='Estimated Fy')
        axes[1, 1].set_ylabel('Force Y (N)', fontsize=11)
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].legend(loc='upper right')
        axes[1, 1].set_title('Estimated Wrench - Y Axis', fontweight='bold')
        
        axes[2, 1].plot(time, est_fz, 'r-', linewidth=1.5, label='Estimated Fz (external)')
        axes[2, 1].axhline(y=0.0, color='g', linestyle='--', linewidth=1, label='No contact: 0 N')
        axes[2, 1].set_xlabel('Time (s)', fontsize=11)
        axes[2, 1].set_ylabel('Force Z (N)', fontsize=11)
        axes[2, 1].grid(True, alpha=0.3)
        axes[2, 1].legend(loc='upper right')
        axes[2, 1].set_title('Estimated Wrench - Z Axis (After Gravity Comp.)', fontweight='bold')
        
        plt.tight_layout()
        
        # Save figure
        filename = f'/home/andy/franka_ros2_ws/wrench_plots_{int(time[-1])}.png'
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        self.get_logger().info(f'Plot saved to: {filename}')
        
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    
    plotter = WrenchPlotter()
    
    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(plotter,), daemon=True)
    spin_thread.start()
    
    try:
        print("\nCollecting wrench data for 10 seconds...")
        print("Press Ctrl+C to stop and generate plots.\n")
        
        import time
        time.sleep(10)  # Collect data for 10 seconds
        
        print("\nGenerating plots...")
        plotter.plot()
        
    except KeyboardInterrupt:
        print("\nGenerating plots from collected data...")
        plotter.plot()
    
    finally:
        plotter.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
