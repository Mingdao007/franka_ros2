#!/usr/bin/env python3
"""
测试脚本：在不接触桌子的情况下检查重力补偿效果
"""
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import time

class WrenchChecker(Node):
    def __init__(self):
        super().__init__('wrench_checker')
        self.estimated_sub = self.create_subscription(
            WrenchStamped,
            '/direct_force_example_controller/estimated_wrench',
            self.estimated_callback,
            10
        )
        self.commanded_sub = self.create_subscription(
            WrenchStamped,
            '/direct_force_example_controller/commanded_wrench',
            self.commanded_callback,
            10
        )
        self.estimated_z = None
        self.commanded_z = None
        self.sample_count = 0
        self.max_samples = 50

    def estimated_callback(self, msg):
        self.estimated_z = msg.wrench.force.z
        
    def commanded_callback(self, msg):
        self.commanded_z = msg.wrench.force.z
        self.sample_count += 1
        
        if self.sample_count % 10 == 0 and self.estimated_z is not None:
            self.get_logger().info(
                f"Sample {self.sample_count}: "
                f"estimated_z={self.estimated_z:.3f} N, "
                f"commanded_z={self.commanded_z:.3f} N"
            )
        
        if self.sample_count >= self.max_samples:
            self.get_logger().info("=" * 60)
            self.get_logger().warn(
                f"平均estimated_z = {self.estimated_z:.3f} N "
                f"(应该≈-2N如果重力补偿正确)"
            )
            self.get_logger().info("=" * 60)
            rclpy.shutdown()

def main():
    rclpy.init()
    node = WrenchChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
