# Skill: Gazebo vs Real Robot - Force Control Differences

**Date:** 2026-02-01  
**Category:** Force Control / Simulation / Debugging  
**Confidence:** High (experimentally verified)

---

## 1. Core Finding: Bias Calibration is Harmful in Gazebo

### Problem
Implementing the official Franka `tau_ext_initial` bias calibration pattern **degrades performance in Gazebo** simulation.

### Evidence
| Configuration | RMSE | Mean Bias | Std |
|--------------|------|-----------|-----|
| With bias calibration | 0.6664 N | 0.5279 N | 0.4066 N |
| **Without bias calibration** | **0.5341 N** | **0.3488 N** | **0.4045 N** |

**Result:** Disabling bias calibration improved RMSE by **20%**.

### Root Cause
```cpp
// Official Franka pattern (for REAL robot):
tau_ext = tau_measured - tau_gravity - tau_ext_initial;

// Problem in Gazebo:
// - tau_meas - tau_gravity is already clean (no sensor drift)
// - tau_ext_initial captures noise at activation time
// - Subtracting it introduces systematic offset error
```

### Solution
```yaml
# In franka_gazebo_controllers.yaml:
use_bias_calibration: false  # CRITICAL: Only for Gazebo!
```

---

## 2. Gazebo vs Real Robot: Key Differences Summary

| Aspect | Real Franka (libfranka) | Gazebo Simulation |
|--------|------------------------|-------------------|
| **Torque Command** | `τ_actual = τ_user + τ_gravity` (auto-compensated) | `τ_actual = τ_user` (raw) |
| **Measured Torque** | Contains sensor drift, needs calibration | Clean, no drift |
| **Gravity Compensation** | Built into driver | Must compute via Pinocchio RNEA |
| **Bias Calibration** | **Required** (`tau_ext_initial`) | **Harmful** (introduces error) |
| **Coriolis Compensation** | Often beneficial | Negligible effect in tests |
| **D-term Filtering** | Useful for noise | Negligible effect (clean signal) |

---

## 3. What NOT to Copy from Official Examples

### ❌ Don't blindly copy these patterns to Gazebo:

1. **Bias Calibration** (`tau_ext_initial`)
   - Real robot: Removes sensor drift at startup
   - Gazebo: Adds unwanted offset → **disable it**

2. **Strict Torque Rate Limits** (`kDeltaTauMax = 1.0 Nm`)
   - Real robot: Protects hardware
   - Gazebo: May cause unnecessary saturation in some cases

### ✅ DO keep these patterns:

1. **Gravity Compensation** (via Pinocchio RNEA)
   - Essential for both, but implementation differs

2. **Jacobian-based Force Estimation**
   - Works well in both environments

3. **PID Structure**
   - Same tuning approach works

---

## 4. Debugging Methodology: RMSE Decomposition

When performance degrades, decompose RMSE into **Mean** and **Std**:

```
RMSE² = Mean² + Std²
```

| Symptom | Likely Cause |
|---------|--------------|
| Mean ↑, Std same | Calibration/offset issue (bias) |
| Std ↑, Mean same | Noise/filtering issue |
| Both ↑ | Fundamental control problem |

**Today's case:** Mean increased while Std stayed constant → pointed directly to bias calibration.

---

## 5. Recommended Configuration for Gazebo Force Control

```yaml
direct_force_example_controller:
  ros__parameters:
    # Core settings
    gazebo: true
    use_ft_sensor: false  # Jacobian estimation is better tuned
    
    # PID (well-tuned for Gazebo)
    kp: 0.03
    ki: 0.005
    kd: 0.006
    
    # CRITICAL: Gazebo-specific settings
    use_bias_calibration: false  # ← Key finding!
    coriolis_factor: 0.0         # Negligible effect
    d_filter_alpha: 0.0          # Signal already clean
    
    # Regularization
    lambda: 0.01  # Damped least squares for Jacobian
```

---

## 6. Performance Benchmark

| Method | RMSE | Notes |
|--------|------|-------|
| Jacobian + no bias cal | **0.5341 N** | ✅ Best (rigid surface) |
| Jacobian + bias cal | 0.6664 N | ❌ 20% worse |
| F/T Sensor direct | 0.7581 N | Needs more tuning |

### Compliant Surface Tests (more realistic)

| Surface Stiffness | RMSE | Mean | Std | Notes |
|------------------|------|------|-----|-------|
| Rigid (∞) | 0.5341 N | 0.3488 N | 0.4045 N | Baseline |
| Hard rubber (20k N/m) | ~0.54 N | ~0.35 N | ~0.41 N | Recommended |
| Medium (10k N/m) | 0.5430 N | 0.3499 N | 0.4151 N | +1.7% |
| Soft foam (5k N/m) | 0.6172 N | 0.4509 N | 0.4214 N | +15.6% |

**Insight:** Compliant surfaces increase control difficulty, which is more realistic. The PID gains may need retuning for softer surfaces.

---

## 7. Key Takeaways

1. **Don't assume "official" = "universal"** - Patterns designed for real hardware may not apply to simulation
2. **Decompose errors** - Mean vs Std analysis quickly identifies bias issues
3. **Simulation is cleaner** - Many real-robot compensations are unnecessary or harmful
4. **Document findings** - Future self will thank you

---

## 8. CRITICAL: Gazebo Session Management

### Problem
**每次实验结束后必须完全关闭 Gazebo！** 残留进程会严重干扰下一次实验：
- Controller 加载失败 ("Controller already loaded")
- Controller 配置失败 ("Could not configure controller")
- 状态不一致导致数据错误

### Symptoms of Residual Process Interference
```
[spawner] Controller already loaded, skipping load_controller
[spawner] Failed to configure controller
[controller_manager] Could not configure controller with name 'xxx' because no controller with this name exists
```

### Solution: Clean Shutdown Procedure
```bash
# 方法1: 使用 Ctrl+C 在 launch 终端 (推荐)

# 方法2: 强制清理所有相关进程
pkill -9 -f "ros2 launch"
pkill -9 -f "gz sim"
pkill -9 -f "ign gazebo"
pkill -9 -f "ruby"  # Gazebo uses ruby for some components
sleep 3  # Wait for cleanup

# 方法3: 重启系统 (最彻底)
```

### Best Practice
1. **启动仿真前 (MUST)**: 必须先执行 `pkill -9 -f "ign gazebo"` 清理所有残留进程，确认清理干净后才能启动新仿真！
2. **实验后**: 用 Ctrl+C 优雅关闭，或用上述命令强制清理
3. **调试时**: 如遇到奇怪的 controller 错误，先清理进程再重试

### 启动仿真的正确流程
```bash
# Step 1: 清理所有残留进程 (必须！)
pkill -9 -f "ign gazebo" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
sleep 2

# Step 2: 确认清理干净
ps aux | grep -E "(gz|gazebo)" | grep -v grep || echo "Clean!"

# Step 3: 启动仿真
ros2 launch franka_gazebo_bringup gazebo_direct_force_jacobian.launch.py
```

---

## References

- Official Franka ROS: `frankarobotics/franka_ros/force_example_controller.cpp`
- Pinocchio RNEA: Used for gravity/Coriolis computation
- Test data: `/home/andy/franka_ros2_ws/no_bias_best_0.5341.csv`
