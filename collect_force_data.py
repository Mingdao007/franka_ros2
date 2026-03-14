#!/usr/bin/env python3
"""
Force Data Collector
====================
Records force tracking data from ROS2 topics and saves to CSV file.

Usage:
    python3 collect_force_data.py [--duration 20] [--output data.csv]
                                 [--controller-name direct_force_example_controller]
                                 [--commanded-topic /foo/commanded_wrench]
                                 [--estimated-topic /foo/estimated_wrench]
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import csv
import argparse
import time
import threading
from datetime import datetime


class ForceDataCollector(Node):
    def __init__(self, output_file, duration, commanded_topic, estimated_topic):
        super().__init__('force_data_collector')
        
        self.output_file = output_file
        self.duration = duration
        self.commanded_topic = commanded_topic
        self.estimated_topic = estimated_topic
        self.data = []
        self.start_time = None
        self.lock = threading.Lock()
        
        # Latest values from each topic
        self.latest_cmd = {'fx': 0.0, 'fy': 0.0, 'fz': 0.0}
        self.latest_est = {'fx': 0.0, 'fy': 0.0, 'fz': 0.0}
        
        self.cmd_sub = self.create_subscription(
            WrenchStamped,
            self.commanded_topic,
            self.cmd_callback,
            10
        )
        
        self.est_sub = self.create_subscription(
            WrenchStamped,
            self.estimated_topic,
            self.est_callback,
            10
        )
        
        self.get_logger().info(
            f'Recording for {duration}s -> {output_file} | cmd={self.commanded_topic} est={self.estimated_topic}'
        )
        
    def cmd_callback(self, msg):
        with self.lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            
            self.latest_cmd = {
                'fx': msg.wrench.force.x,
                'fy': msg.wrench.force.y,
                'fz': msg.wrench.force.z
            }
            
            # Record data point
            self.data.append({
                'time': t,
                'cmd_fx': self.latest_cmd['fx'],
                'cmd_fy': self.latest_cmd['fy'],
                'cmd_fz': self.latest_cmd['fz'],
                'est_fx': self.latest_est['fx'],
                'est_fy': self.latest_est['fy'],
                'est_fz': self.latest_est['fz'],
            })
    
    def est_callback(self, msg):
        with self.lock:
            self.latest_est = {
                'fx': msg.wrench.force.x,
                'fy': msg.wrench.force.y,
                'fz': msg.wrench.force.z
            }
    
    def save_to_csv(self):
        if not self.data:
            self.get_logger().warn('No data collected!')
            return False
        
        with open(self.output_file, 'w', newline='') as f:
            fieldnames = ['time', 'cmd_fx', 'cmd_fy', 'cmd_fz', 
                         'est_fx', 'est_fy', 'est_fz']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.data)
        
        self.get_logger().info(f'Saved {len(self.data)} samples to {self.output_file}')
        return True


def main():
    parser = argparse.ArgumentParser(description='Force Data Collector')
    parser.add_argument('--duration', type=float, default=20.0,
                       help='Recording duration in seconds')
    parser.add_argument('--output', type=str, default=None,
                       help='Output CSV file path')
    parser.add_argument('--controller-name', type=str, default='direct_force_example_controller',
                       help='Controller name to build default wrench topics')
    parser.add_argument('--commanded-topic', type=str, default=None,
                       help='Override commanded wrench topic')
    parser.add_argument('--estimated-topic', type=str, default=None,
                       help='Override estimated wrench topic')
    args = parser.parse_args()

    if args.commanded_topic is None:
        args.commanded_topic = f'/{args.controller_name}/commanded_wrench'
    if args.estimated_topic is None:
        args.estimated_topic = f'/{args.controller_name}/estimated_wrench'
    
    # Default output filename with timestamp
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f'/home/andy/franka_ros2_ws/force_data_{timestamp}.csv'
    
    rclpy.init()
    node = ForceDataCollector(args.output, args.duration, args.commanded_topic, args.estimated_topic)
    
    # Spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print(f'\n{"="*50}')
    print(f'  Force Data Collector')
    print(f'{"="*50}')
    print(f'  Duration: {args.duration}s')
    print(f'  Output:   {args.output}')
    print(f'  CmdTopic: {args.commanded_topic}')
    print(f'  EstTopic: {args.estimated_topic}')
    print(f'{"="*50}\n')
    
    try:
        start = time.time()
        while time.time() - start < args.duration:
            elapsed = time.time() - start
            remaining = args.duration - elapsed
            print(f'\rRecording... {remaining:.1f}s remaining, {len(node.data)} samples', end='')
            time.sleep(0.5)
        
        print('\n')
        node.save_to_csv()
        
    except KeyboardInterrupt:
        print('\n\nStopped by user')
        node.save_to_csv()
    
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('Done')


if __name__ == '__main__':
    main()
