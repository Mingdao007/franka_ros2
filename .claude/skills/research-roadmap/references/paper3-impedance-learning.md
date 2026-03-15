# Paper 3: Impedance Learning + DRL for Unknown Terrains

## Paper Info
- **Title**: Impedance Learning-Based Adaptive Force Tracking for Robot on Unknown Terrains
- **Authors**: Li, Zheng, Wang, Dong, Zhang (USTC)
- **Venue**: IEEE Transactions on Robotics, Vol. 41, 2025
- **Zotero**: `~/Zotero/storage/X3TVDG6I/`

## Key Contributions
1. DRL (actor-critic) learns impedance parameter adjustment policy: ΔK, ΔB, Δx_r
2. Neural network feedforward (uf) + variable impedance feedback (ub) — eq. (12)
3. "Couch model": simplified dynamic contact model satisfying Lipschitz continuity for safe sim-to-real
4. Lyapunov stability proof + contraction mapping convergence proof
5. Validated on real UR5 robot with unknown terrain profiles

## Architecture
```
Feedforward (DRL policy, pretrained in sim)
  → ΔI = [ΔK, ΔB, Δx_r]
  → adjusts impedance parameters online

Feedback (variable impedance controller)
  → M(ẍ_m - ẍ_r) + B(ẋ_m - ẋ_r) + K(x_m - x_r) = f_m - f_d
  → uses adjusted K(t), B(t) from DRL

Combined: u = u_f + u_b
```

## What to Adapt for Our Platform
- Replace our fixed PID force gains with variable impedance (K, B adjustable)
- Train DRL policy in Gazebo with randomized surface heights/stiffnesses
- Use Koopman model (Paper 1) for state representation in DRL observations
- Validate on flat → curved surfaces

## Prerequisite Platform Work
- Randomized terrain generation in Gazebo SDF
- Variable impedance controller (modify hybrid_circle_force_controller)
- DRL training infrastructure (Python, likely stable-baselines3)
- Episode management for sim training

## Not immediately needed
- Real robot (sim-only for now, per user preference)
- Couch model sim-to-real transfer (future work)
