#!/usr/bin/env python3
"""
验证脚本：检查重力补偿是否真实有效，还是只是"减去一个数字"来隐藏误差
"""
import numpy as np

print("=" * 80)
print("重力补偿验证报告")
print("=" * 80)
print()

# 从实测数据来看
estimated_z = -9.698  # estimated_wrench的Z轴分量（减去重力后）
commanded_z = -2.117  # commanded_wrench的Z轴分量

print("【实测数据】")
print(f"  estimated_wrench.force.z = {estimated_z:.3f} N  (这是F_hat = F_total - F_gravity)")
print(f"  commanded_wrench.force.z = {commanded_z:.3f} N  (这是PID输出的命令)")
print()

print("【关键问题】")
print("  1. estimated_wrench显示 -9.7 N，但我们期望的外力应该是 ~-2 N")
print("  2. 这意味着重力补偿后仍然残留了约 -7.7 N 的误差")
print("  3. PID看到的测量值f_meas ≈ -9.7 N，而期望值f_desired = 2.0 N")
print("  4. 所以误差 e = 2.0 - (-9.7) = 11.7 N")
print()

print("【数学验证】")
f_desired = 2.0
f_meas = estimated_z  # PID看到的是滤波后的estimated值
e = f_desired - f_meas
kp = 0.005
ki = 0.0
kd = 0.01
u = kp * e + ki * 0 + kd * 0  # 稳态时微分项≈0
f_cmd_calc = -1.0 * (f_desired + u)  # command_sign = -1

print(f"  误差 e = {f_desired:.3f} - ({f_meas:.3f}) = {e:.3f} N")
print(f"  PID输出 u = {kp:.3f} × {e:.3f} = {u:.4f} N")
print(f"  命令力 f_cmd = -1 × ({f_desired:.3f} + {u:.4f}) = {f_cmd_calc:.4f} N")
print(f"  实测命令 = {commanded_z:.3f} N")
print(f"  计算误差 = {abs(f_cmd_calc - commanded_z):.4f} N ✓ 吻合")
print()

print("【问题诊断】")
print("  ❌ 重力补偿失败！")
print(f"  - 期望：重力补偿后，无外力时应该≈0 N，有-2N外力时应该≈-2 N")
print(f"  - 实际：重力补偿后，显示 {estimated_z:.1f} N（残留约{estimated_z+2:.1f} N误差）")
print()
print("  可能原因：")
print("  1. Pinocchio模型参数（质量、惯量、重心）与Gazebo模型不匹配")
print("  2. 雅可比矩阵计算的参考系不一致（LOCAL vs WORLD）")
print("  3. 伪逆求解 (JJ^T)^{-1}J 的数值误差（lambda=0.1可能太大）")
print("  4. 末端执行器质量没有正确建模（夹爪？传感器？）")
print()

print("【是否存在\"作弊\"？】")
print("  ✓ 没有隐藏数据 - estimated_wrench如实发布了F_total - F_gravity")
print("  ✓ 没有篡改命令 - commanded_wrench是真实的PID输出")
print("  ✗ 但重力补偿确实失效 - 残留误差高达 7.7 N (理论外力应该只有-2N)")
print()
print("  结论：代码逻辑正确，但重力补偿精度不足，导致PID看到大误差。")
print("       这不是\"骗人\"，而是物理模型不匹配造成的系统误差。")
print()

print("【系统如何仍然工作？】")
print(f"  虽然测量有 {estimated_z+2:.1f} N 误差，但通过低kp=0.005抑制了过度修正：")
print(f"  - 如果kp=1.0: u = 1.0 × 11.7 = 11.7 N → f_cmd = -(2+11.7) = -13.7 N (严重偏离)")
print(f"  - 实际kp=0.005: u = 0.005 × 11.7 = 0.058 N → f_cmd = -(2+0.058) = -2.06 N (勉强可用)")
print()
print("  这是用\"低增益\"掩盖\"大偏差\"的权宜之计，不是根本解决方案。")
print("=" * 80)
