#!/usr/bin/env python3
"""
末端力控跟踪结果可视化
=======================
实时订阅ROS2话题可视化期望力与实际力的跟踪效果
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
import numpy as np
from collections import deque
import threading
from datetime import datetime

class ForceTrackingVisualizer(Node):
    def __init__(self):
        super().__init__('force_tracking_visualizer')
        
        # 数据存储 (保存最近1000个采样点)
        self.max_len = 1000
        self.time_data = deque(maxlen=self.max_len)
        
        # 期望力 (commanded/desired)
        self.cmd_fx = deque(maxlen=self.max_len)
        self.cmd_fy = deque(maxlen=self.max_len)
        self.cmd_fz = deque(maxlen=self.max_len)
        
        # 实际力 (estimated/measured)
        self.est_fx = deque(maxlen=self.max_len)
        self.est_fy = deque(maxlen=self.max_len)
        self.est_fz = deque(maxlen=self.max_len)
        
        # 误差历史
        self.error_fx = deque(maxlen=self.max_len)
        self.error_fy = deque(maxlen=self.max_len)
        self.error_fz = deque(maxlen=self.max_len)
        
        self.start_time = None
        self.lock = threading.Lock()
        self.data_count = 0
        
        # 订阅话题
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
        
        self.get_logger().info('🚀 力控跟踪可视化节点启动')
        self.get_logger().info('📡 订阅话题:')
        self.get_logger().info('   - /direct_force_example_controller/commanded_wrench')
        self.get_logger().info('   - /direct_force_example_controller/estimated_wrench')
        
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
            
            # 计算误差 (如果有对应的期望值)
            if len(self.cmd_fz) > 0:
                self.error_fx.append(self.cmd_fx[-1] - msg.wrench.force.x if len(self.cmd_fx) > 0 else 0)
                self.error_fy.append(self.cmd_fy[-1] - msg.wrench.force.y if len(self.cmd_fy) > 0 else 0)
                self.error_fz.append(self.cmd_fz[-1] - msg.wrench.force.z if len(self.cmd_fz) > 0 else 0)
    
    def get_data(self):
        """获取当前数据的快照"""
        with self.lock:
            return {
                'time': np.array(self.time_data),
                'cmd_fx': np.array(self.cmd_fx),
                'cmd_fy': np.array(self.cmd_fy),
                'cmd_fz': np.array(self.cmd_fz),
                'est_fx': np.array(self.est_fx),
                'est_fy': np.array(self.est_fy),
                'est_fz': np.array(self.est_fz),
                'error_fx': np.array(self.error_fx),
                'error_fy': np.array(self.error_fy),
                'error_fz': np.array(self.error_fz),
                'count': self.data_count
            }
    
    def calculate_metrics(self, data):
        """计算性能指标"""
        metrics = {}
        
        if len(data['error_fz']) > 10:
            # Z轴力跟踪性能 (主要控制方向)
            error_z = data['error_fz']
            metrics['mean_error_z'] = np.mean(error_z)
            metrics['std_error_z'] = np.std(error_z)
            metrics['max_error_z'] = np.max(np.abs(error_z))
            metrics['rms_error_z'] = np.sqrt(np.mean(error_z**2))
            
            # 稳态误差 (最后20%数据)
            n = len(error_z)
            steady_state = error_z[int(n*0.8):]
            metrics['steady_state_error'] = np.mean(np.abs(steady_state))
        
        return metrics


class RealTimePlotter:
    def __init__(self, node):
        self.node = node
        self.fig = None
        self.axes = None
        self.lines = {}
        
    def setup_plot(self):
        """设置图形布局"""
        # 使用兼容的样式
        try:
            plt.style.use('seaborn-whitegrid')
        except:
            plt.style.use('ggplot')
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('🤖 Franka 末端力控跟踪实时可视化', fontsize=16, fontweight='bold')
        
        gs = GridSpec(3, 3, figure=self.fig, hspace=0.35, wspace=0.3)
        
        # 力跟踪图 (左侧 2列)
        self.ax_fx = self.fig.add_subplot(gs[0, 0:2])
        self.ax_fy = self.fig.add_subplot(gs[1, 0:2])
        self.ax_fz = self.fig.add_subplot(gs[2, 0:2])
        
        # 误差图 (右侧)
        self.ax_error = self.fig.add_subplot(gs[0, 2])
        self.ax_metrics = self.fig.add_subplot(gs[1, 2])
        self.ax_histogram = self.fig.add_subplot(gs[2, 2])
        
        # 初始化线条
        self.lines['cmd_fx'], = self.ax_fx.plot([], [], 'b-', linewidth=2, label='期望 Fx')
        self.lines['est_fx'], = self.ax_fx.plot([], [], 'r-', linewidth=1.5, alpha=0.8, label='实际 Fx')
        self.ax_fx.set_ylabel('Force X (N)', fontsize=11)
        self.ax_fx.set_title('X轴力跟踪', fontweight='bold')
        self.ax_fx.legend(loc='upper right')
        self.ax_fx.set_xlim(0, 30)
        self.ax_fx.set_ylim(-5, 5)
        
        self.lines['cmd_fy'], = self.ax_fy.plot([], [], 'b-', linewidth=2, label='期望 Fy')
        self.lines['est_fy'], = self.ax_fy.plot([], [], 'r-', linewidth=1.5, alpha=0.8, label='实际 Fy')
        self.ax_fy.set_ylabel('Force Y (N)', fontsize=11)
        self.ax_fy.set_title('Y轴力跟踪', fontweight='bold')
        self.ax_fy.legend(loc='upper right')
        self.ax_fy.set_xlim(0, 30)
        self.ax_fy.set_ylim(-5, 5)
        
        self.lines['cmd_fz'], = self.ax_fz.plot([], [], 'b-', linewidth=2, label='期望 Fz')
        self.lines['est_fz'], = self.ax_fz.plot([], [], 'r-', linewidth=1.5, alpha=0.8, label='实际 Fz')
        self.ax_fz.set_xlabel('时间 (s)', fontsize=11)
        self.ax_fz.set_ylabel('Force Z (N)', fontsize=11)
        self.ax_fz.set_title('Z轴力跟踪 (主控方向)', fontweight='bold')
        self.ax_fz.legend(loc='upper right')
        self.ax_fz.set_xlim(0, 30)
        self.ax_fz.set_ylim(-10, 10)
        
        # 误差时间曲线
        self.lines['error_z'], = self.ax_error.plot([], [], 'g-', linewidth=1.5, label='误差 Fz')
        self.ax_error.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        self.ax_error.axhline(y=1.0, color='orange', linestyle='--', alpha=0.5, label='±1.0N')
        self.ax_error.axhline(y=-1.0, color='orange', linestyle='--', alpha=0.5)
        self.ax_error.set_ylabel('Error (N)', fontsize=11)
        self.ax_error.set_title('Z轴跟踪误差', fontweight='bold')
        self.ax_error.legend(loc='upper right', fontsize=9)
        self.ax_error.set_xlim(0, 30)
        self.ax_error.set_ylim(-3, 3)
        
        # 性能指标文本
        self.ax_metrics.axis('off')
        self.ax_metrics.set_title('📊 实时性能指标', fontweight='bold')
        self.metrics_text = self.ax_metrics.text(0.1, 0.9, '', transform=self.ax_metrics.transAxes,
                                                   fontsize=11, verticalalignment='top',
                                                   fontfamily='monospace',
                                                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # 误差直方图
        self.ax_histogram.set_xlabel('Error (N)', fontsize=10)
        self.ax_histogram.set_ylabel('频次', fontsize=10)
        self.ax_histogram.set_title('误差分布', fontweight='bold')
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
    def update(self, frame):
        """动画更新函数"""
        data = self.node.get_data()
        
        if len(data['time']) < 2:
            return list(self.lines.values())
        
        t = data['time']
        
        # 更新力跟踪曲线
        min_len = min(len(t), len(data['cmd_fx']), len(data['est_fx']))
        if min_len > 0:
            self.lines['cmd_fx'].set_data(t[:min_len], data['cmd_fx'][:min_len])
            self.lines['est_fx'].set_data(t[:min_len], data['est_fx'][:min_len])
        
        min_len = min(len(t), len(data['cmd_fy']), len(data['est_fy']))
        if min_len > 0:
            self.lines['cmd_fy'].set_data(t[:min_len], data['cmd_fy'][:min_len])
            self.lines['est_fy'].set_data(t[:min_len], data['est_fy'][:min_len])
        
        min_len = min(len(t), len(data['cmd_fz']), len(data['est_fz']))
        if min_len > 0:
            self.lines['cmd_fz'].set_data(t[:min_len], data['cmd_fz'][:min_len])
            self.lines['est_fz'].set_data(t[:min_len], data['est_fz'][:min_len])
        
        # 更新误差曲线
        if len(data['error_fz']) > 0:
            min_len = min(len(t), len(data['error_fz']))
            self.lines['error_z'].set_data(t[:min_len], data['error_fz'][:min_len])
        
        # 动态调整X轴范围
        if len(t) > 0:
            t_max = t[-1]
            for ax in [self.ax_fx, self.ax_fy, self.ax_fz, self.ax_error]:
                if t_max > 25:
                    ax.set_xlim(max(0, t_max - 30), t_max + 5)
                else:
                    ax.set_xlim(0, max(30, t_max + 5))
        
        # 动态调整Y轴范围
        if len(data['cmd_fz']) > 0:
            fz_all = np.concatenate([data['cmd_fz'], data['est_fz']]) if len(data['est_fz']) > 0 else data['cmd_fz']
            fz_min, fz_max = np.min(fz_all), np.max(fz_all)
            margin = max(1, (fz_max - fz_min) * 0.2)
            self.ax_fz.set_ylim(fz_min - margin, fz_max + margin)
        
        # 更新性能指标
        metrics = self.node.calculate_metrics(data)
        if metrics:
            metrics_str = (
                f"数据点数: {data['count']}\n"
                f"─────────────────\n"
                f"Z轴力控性能:\n"
                f"  平均误差: {metrics.get('mean_error_z', 0):.3f} N\n"
                f"  标准差:   {metrics.get('std_error_z', 0):.3f} N\n"
                f"  最大误差: {metrics.get('max_error_z', 0):.3f} N\n"
                f"  RMS误差:  {metrics.get('rms_error_z', 0):.3f} N\n"
                f"─────────────────\n"
                f"稳态误差:   {metrics.get('steady_state_error', 0):.3f} N\n"
            )
            # 判断性能等级
            ss_err = metrics.get('steady_state_error', float('inf'))
            if ss_err < 0.2:
                metrics_str += "评级: ✅ 优秀"
            elif ss_err < 1.0:
                metrics_str += "评级: ✅ 合格"
            else:
                metrics_str += "评级: ⚠️ 需改进"
            
            self.metrics_text.set_text(metrics_str)
        
        # 更新误差直方图
        if len(data['error_fz']) > 10:
            self.ax_histogram.clear()
            self.ax_histogram.hist(data['error_fz'], bins=30, color='steelblue', 
                                   edgecolor='white', alpha=0.7)
            self.ax_histogram.axvline(x=0, color='r', linestyle='--', linewidth=2)
            self.ax_histogram.set_xlabel('Error (N)', fontsize=10)
            self.ax_histogram.set_ylabel('频次', fontsize=10)
            self.ax_histogram.set_title('误差分布', fontweight='bold')
        
        return list(self.lines.values())
    
    def run(self):
        """运行实时绘图"""
        self.setup_plot()
        ani = animation.FuncAnimation(self.fig, self.update, interval=100, 
                                       blit=False, cache_frame_data=False)
        plt.show()
        return ani
    
    def save_final_plot(self, filename=None):
        """保存最终图像"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f'/home/andy/franka_ros2_ws/force_tracking_{timestamp}.png'
        
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"📁 图像已保存: {filename}")


def main(args=None):
    rclpy.init(args=args)
    
    node = ForceTrackingVisualizer()
    plotter = RealTimePlotter(node)
    
    # 在后台线程运行ROS2 spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        print("\n" + "="*50)
        print("🎯 末端力控跟踪结果可视化")
        print("="*50)
        print("正在收集数据并实时显示...")
        print("按 Ctrl+C 停止并保存图像")
        print("="*50 + "\n")
        
        # 运行实时绘图
        ani = plotter.run()
        
    except KeyboardInterrupt:
        print("\n⏹️ 停止可视化...")
        plotter.save_final_plot()
    
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        print("✅ 可视化完成")


if __name__ == '__main__':
    main()
