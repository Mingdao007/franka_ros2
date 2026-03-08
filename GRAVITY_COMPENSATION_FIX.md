# 重力补偿修复说明 (Gravity Compensation Fix)

## 📋 修改日期
2026-01-30

## ⚠️ 发现的问题

### 错误实现 (Before)
```cpp
// ❌ 错误：在笛卡尔空间分别映射后再相减
F_gravity = (J*J^T + λ²I)^{-1} * J * tau_gravity;  // 先映射重力
F_total = (J*J^T + λ²I)^{-1} * J * tau_measured;   // 再映射总测量力
F_ext = F_total - F_gravity;                        // 最后相减
```

**问题：**
- 雅可比矩阵 J 是位置依赖的，在不同计算步骤中可能不完全一致
- 阻尼最小二乘法引入的非线性项使得分别映射再相减 ≠ 先相减再映射
- 导致残留约 -7.7N 的重力补偿误差

### 正确实现 (After)
```cpp
// ✅ 正确：在关节空间先相减，再映射到笛卡尔空间
tau_ext = tau_measured - tau_gravity;               // 关节空间相减
F_ext = (J*J^T + λ²I)^{-1} * J * tau_ext;          // 然后映射
```

## 🔧 具体修改

**文件：** `franka_example_controllers/src/direct_force_example_controller.cpp`

**修改位置：** Line 378-413

### 关键变更
1. **创建 `tau_ext` 向量**：先复制 `tau_meas_`，然后在关节空间减去重力
2. **条件性重力补偿**：
   - **Gazebo 仿真**：使用 Pinocchio RNEA 计算重力力矩并减去
   - **真实机器人**：不处理（libfranka 已经自动补偿）
3. **单次映射**：只进行一次从关节空间到笛卡尔空间的映射

## 📐 数学原理

### 正确公式
$$
\begin{align}
\boldsymbol{\tau}_{\text{ext}} &= \boldsymbol{\tau}_{\text{measured}} - \boldsymbol{\tau}_{\text{gravity}} \quad \text{(关节空间)} \\
\mathbf{F}_{\text{ext}} &= (\mathbf{J}\mathbf{J}^T + \lambda^2\mathbf{I})^{-1} \mathbf{J} \boldsymbol{\tau}_{\text{ext}} \quad \text{(映射到笛卡尔)}
\end{align}
$$

### 为什么原来的方法错误？
当 $\mathbf{A} = \mathbf{J}\mathbf{J}^T + \lambda^2\mathbf{I}$ 时：
$$
\mathbf{A}^{-1}\mathbf{J}(\boldsymbol{\tau}_a - \boldsymbol{\tau}_b) \neq \mathbf{A}^{-1}\mathbf{J}\boldsymbol{\tau}_a - \mathbf{A}^{-1}\mathbf{J}\boldsymbol{\tau}_b
$$
因为 $\mathbf{A}$ 包含 $\mathbf{J}$，而 $\mathbf{J}$ 是时变的。

## 🎯 官方参考

根据 Franka Robotics 官方控制器实现：
- `joint_impedance_example_controller.cpp`：不显式补偿重力
- `cartesian_pose_example_controller.cpp`：不显式补偿重力
- `gravity_compensation_example_controller.cpp`：命令全 0（依赖硬件补偿）

**结论：**
- ✅ **真实机器人**：`tau_measured` 已由 libfranka 自动减去重力
- ✅ **Gazebo 仿真**：需要显式补偿，但必须在关节空间进行

## 🧪 预期效果

修改后应该看到：
- ✅ `estimated_wrench.force.z` 在悬空时接近 **-2N**（末端工具重量）
- ✅ 重力补偿误差从 -7.7N 降低到 < 0.5N
- ✅ PID 控制器看到正确的外力测量值
- ✅ 力跟踪性能显著提升

## 📝 测试步骤

1. 重新编译：
```bash
cd ~/franka_ros2_ws
colcon build --packages-select franka_example_controllers
source install/setup.bash
```

2. 运行控制器并检查数据：
```bash
# Terminal 1: 启动控制器
ros2 launch franka_bringup franka.launch.py

# Terminal 2: 检查 wrench 数据
ros2 run franka_ros2 check_wrench_live.py
```

3. 预期结果：
   - `estimated_z` ≈ -2.0 N (instead of -9.7 N)
   - 力跟踪误差 < 1.0 N

## 🔍 进一步诊断

如果问题仍然存在，检查：
1. **URDF 模型**：末端工具质量、惯性参数是否正确？
2. **Pinocchio 模型加载**：`pin_model_` 是否包含正确的动力学参数？
3. **坐标系对齐**：雅可比矩阵是否在正确的参考系下计算？
4. **传感器校准**：关节力矩传感器零点是否校准？

---
**Author:** Mingdao (PhD Student)  
**Reference:** Franka Robotics Official Controllers & libfranka API
