#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import csv, os, time

class Logger(Node):
    def __init__(self):
        super().__init__('impedance_logger')
        # 只订阅真实来源的 /joint_states（来自 joint_state_broadcaster）
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 100)
        self.start = None
        self.duration = 100.0  
        ts = time.strftime('%Y%m%d_%H%M%S')
        self.out = os.path.expanduser(f'~/impedance_log_{ts}.csv')
        self.f = open(self.out, 'w', newline='')
        self.w = csv.writer(self.f)
        
        self.w.writerow(['t'] +
                        [f'q{i+1}' for i in range(7)] +
                        [f'dq{i+1}' for i in range(7)] +
                        [f'effort{i+1}' for i in range(7)])
        self.get_logger().info(f'Logging to {self.out}')

    def _to_sec(self, stamp: Time):
        return stamp.sec + stamp.nanosec * 1e-9

    def cb(self, msg: JointState):
        t = self._to_sec(msg.header.stamp) if msg.header.stamp else self.get_clock().now().nanoseconds * 1e-9
        if self.start is None:
            self.start = t
        if t - self.start <= self.duration:
            # 有些仿真只给 position/velocity，effort 可能缺省；防守式补零
            pos = list(msg.position) + [0.0] * (7 - len(msg.position))
            vel = list(msg.velocity) + [0.0] * (7 - len(msg.velocity))
            eff = list(msg.effort)   + [0.0] * (7 - len(msg.effort))
            self.w.writerow([t] + pos[:7] + vel[:7] + eff[:7])
        else:
            self.get_logger().info('Done (100 s). Closing CSV and shutting down.')
            self.f.close()
            rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(Logger())

if __name__ == '__main__':
    main()
