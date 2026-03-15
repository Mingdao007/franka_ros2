# Paper 2: PG-GRU Feedforward for Force Control

## Paper Info
- **Title**: Physics-Guided Gated Recurrent Units for Inversion-Based Feedforward Control
- **Authors**: Lin, Bolderman, Lazar (TU Eindhoven → EIT Ningbo / HK PolyU)
- **Venue**: IEEE Transactions on Control Systems Technology, 2026
- **Zotero**: `~/Zotero/storage/56SW5KEI/`
- **Note**: This is the user's own paper.

## Key Contributions
1. PG-GRU = physics model (linear stable inverse) + GRU (nonlinear residual)
2. Two-step approach: first design stable linear inverse, then train GRU on residual
3. ~2x improvement over linear feedforward and preview-based GRU on mass-spring-damper
4. Stability proofs exist for GRU (unlike Transformers)

## Transfer to Force Control

### Original system: SISO mass-spring-damper
- Linear physics model is dominant, GRU captures parasitic nonlinearity
- Inverse is well-defined: given desired output → compute input

### Target system: robot + contact surface
- Physics model: linearized impedance model (M, B, K) + contact stiffness
- GRU residual: contact nonlinearity, friction, surface geometry variation
- Inverse: given desired force trajectory → compute position/impedance reference

### Key challenge
Original system: nonlinearity is a small residual on a linear system.
Force control: contact nonlinearity is the MAIN effect, not residual.
→ The physics model (linear part) needs to be strong enough to capture most dynamics.
→ Paper 1's Koopman linear model could serve as the physics layer.

## Synergy with Other Papers
- **Paper 1 (Koopman)** → provides the linear model for PG-GRU's physics layer
- **Paper 3 (DRL impedance)** → provides the control structure; PG-GRU adds feedforward on top

## Implementation Plan (future)
1. After Paper 3 gives working adaptive impedance controller
2. Collect input-output data from closed-loop force control runs
3. Build PG-GRU: Koopman linear model + GRU on residual
4. Train on offline data, validate as feedforward compensator
5. Compare: with/without PG-GRU feedforward, measure transient improvement
