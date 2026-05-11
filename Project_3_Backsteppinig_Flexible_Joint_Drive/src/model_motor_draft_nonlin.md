
---

# 1. Nonlinear State-Space Model (Briefly)

**State vector:**
$$
x = \begin{bmatrix} \theta_l \\ \omega_l \\ \theta_m \\ \omega_m \end{bmatrix} \in \mathbb{R}^4
$$
| Component | Physical Meaning | Units |
|:---|:---|:---|
| $\theta_l$ | Load angular position | rad |
| $\omega_l$ | Load angular velocity | rad/s |
| $\theta_m$ | Motor angular position | rad |
| $\omega_m$ | Motor angular velocity | rad/s |

**Control input:**
$$
u = \tau_m \in \mathbb{R}
$$
| Component | Physical Meaning | Units |
|:---|:---|:---|
| $\tau_m$ | Motor electromagnetic torque | N·m |

**Nonlinear dynamics:**
$$
\dot{x} = f(x) + g(x)u + d(t)
$$
Explicit vector field:
$$
\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= \frac{k}{J_l}(x_3 - x_1) + \frac{k_3}{J_l}(x_3 - x_1)^3 + \frac{b}{J_l}(x_4 - x_2) - \frac{1}{J_l}T_{f,l}(x_2) + d_l(t) \\
\dot{x}_3 &= x_4 \\
\dot{x}_4 &= \frac{1}{J_m}u - \frac{k}{J_m}(x_3 - x_1) - \frac{k_3}{J_m}(x_3 - x_1)^3 - \frac{b}{J_m}(x_4 - x_2) - \frac{1}{J_m}T_{f,m}(x_4) + d_m(t)
\end{aligned}
$$

---

# 2. Derivation of Nonlinear Flexible-Joint Dynamics

## 2.1 Newton–Euler Equations
The system comprises a DC servo motor inertially coupled to a rotational load via a compliant shaft. Applying rotational dynamics to each inertia:
$$
J_l \ddot{\theta}_l = \tau_{\text{coupling}} - T_{f,l}(\dot{\theta}_l) - T_{\text{ext}}(t)
$$
$$
J_m \ddot{\theta}_m = \tau_m - \tau_{\text{coupling}} - T_{f,m}(\dot{\theta}_m)
$$
where $\tau_{\text{coupling}}$ is the torque transmitted through the flexible element, and $T_{f}(\cdot)$ denotes bearing/gear friction.

## 2.2 Nonlinear Torsional Coupling (Cubic Hardening)
Real couplings exhibit stiffness that increases with deflection due to geometric nonlinearity and material hardening. The transmitted torque is modeled as:
$$
\tau_{\text{coupling}} = k(\theta_m - \theta_l) + k_3(\theta_m - \theta_l)^3 + b(\dot{\theta}_m - \dot{\theta}_l)
$$
- $k$: Linear torsional stiffness [N·m/rad]
- $k_3$: Cubic hardening coefficient [N·m/rad³] ($k_3 > 0$ for hardening, $k_3 < 0$ for softening)
- $b$: Structural damping [N·m·s/rad]

## 2.3 Smooth Stribeck Friction Model
To preserve differentiability for Lyapunov-based synthesis, Coulomb + viscous friction is approximated by a smooth hyperbolic tangent:
$$
T_f(\omega) = F_c \tanh\left(\frac{\omega}{v_s}\right) + B_v \omega
$$
- $F_c$: Coulomb friction level [N·m]
- $v_s$: Stribeck velocity threshold [rad/s]
- $B_v$: Viscous friction coefficient [N·m·s/rad]

## 2.4 Strict-Feedback Structure for Backstepping
Defining $x_1=\theta_l, x_2=\omega_l, x_3=\theta_m, x_4=\omega_m$, the dynamics naturally cascade:
$$
\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= f_2(x_1,x_2,x_3,x_4) \\
\dot{x}_3 &= x_4 \\
\dot{x}_4 &= \frac{1}{J_m}u + f_4(x_1,x_2,x_3,x_4)
\end{aligned}
$$
This **triangular strict-feedback form** is the canonical structure for recursive backstepping. Each state acts as a virtual control for the preceding subsystem, enabling step-by-step Lyapunov stabilization without Jacobian inversion or global linearization.

---

# 3. Why Linear Control Methods Fail

| Method | Limitation on Nonlinear Flexible Joint |
|:---|:---|
| **PID / Classical Loop Shaping** | Assumes small-signal linearity around a fixed operating point. Fails under large deflections where $k_3(\Delta\theta)^3$ dominates. Integral windup and derivative noise amplify near resonance. |
| **LQR / Pole Placement** | Requires exact linearization $\dot{\delta x} = A\delta x + B\delta u$. Gains optimized at trim become destabilizing when $\Delta\theta$ or $\omega$ exceed $\pm 5\%$ of nominal. Ignores friction nonlinearity and stiffness hardening, causing limit cycles or sustained oscillations. |
| **Gain-Scheduled Control** | Requires offline identification of multiple linear models and smooth switching logic. Still suffers from performance degradation during fast transients or parameter drift (e.g., temperature-dependent $k$, load changes). |
| **Backstepping** | **Explicitly handles** cubic stiffness, Stribeck friction, and non-collocated actuation in a single recursive framework. Guarantees large-region asymptotic stability via Lyapunov design, automatically shapes closed-loop damping to suppress resonance, and adapts naturally to parameter uncertainty. |

**Key takeaway:** The flexible-joint system is fundamentally **non-affine in large-signal operation** and exhibits state-dependent resonance. Linear methods only guarantee local stability; backstepping provides **global/large-domain stability** while preserving physical energy flow constraints.

---

# 4. Complete Symbol Dictionary

| Symbol | Meaning | Units | Role |
|:---|:---|:---|:---|
| $x$ | State vector | – | $[\theta_l, \omega_l, \theta_m, \omega_m]^\top$ |
| $u$ | Control input | N·m | Motor torque command $\tau_m$ |
| $J_l, J_m$ | Load & motor inertia | kg·m² | Rotational mass |
| $k$ | Linear torsional stiffness | N·m/rad | Coupling baseline elasticity |
| $k_3$ | Cubic stiffness coefficient | N·m/rad³ | Hardening/softening nonlinearity |
| $b$ | Coupling damping | N·m·s/rad | Structural dissipation |
| $T_{f,l}, T_{f,m}$ | Load & motor friction | N·m | Stribeck + viscous loss |
| $F_c$ | Coulomb friction magnitude | N·m | Static/dry friction level |
| $v_s$ | Stribeck velocity | rad/s | Transition speed threshold |
| $B_v$ | Viscous friction coefficient | N·m·s/rad | Speed-proportional loss |
| $d_l, d_m$ | Disturbances | rad/s² | Unmodeled torque, payload variation, noise |
| $\Delta\theta$ | Relative displacement | rad | $\theta_m - \theta_l$ |
| $\Delta\omega$ | Relative velocity | rad/s | $\omega_m - \omega_l$ |

---

# 5. System Schematic & Physical Interpretation

```
                  ↑ τ_m (Motor Torque, u)
                  │
          ┌───────┴───────┐
          │   DC Motor    │ ← J_m, ω_m, θ_m
          │  (Inertia Jm) │   Friction: T_{f,m}(ω_m)
          └───────┬───────┘
                  │ Δθ, Δω
        ┌─────────┴─────────┐
        │ Flexible Coupling │ ← k + k₃(Δθ)³ + b (Nonlinear Spring-Damper)
        └─────────┬─────────┘
                  │
          ┌───────┴───────┐
          │   Payload     │ ← J_l, ω_l, θ_l (Controlled Output)
          │ (Inertia Jl)  │   Friction: T_{f,l}(ω_l)
          └───────────────┘
```

**Mapping to dynamics:**
- **Motor Block:** Generates torque $u$. Dynamics governed by $J_m \dot{\omega}_m = u - \tau_{\text{coupling}} - T_{f,m}$.
- **Flexible Coupling:** Transmits nonlinear torque $\tau_{\text{coupling}} = k\Delta\theta + k_3(\Delta\theta)^3 + b\Delta\omega$. Cubic term captures material hardening at large twist.
- **Load Block:** Receives coupling torque. Dynamics: $J_l \dot{\omega}_l = \tau_{\text{coupling}} - T_{f,l}$. This is the non-collocated output stabilized via backstepping.
- **Cascade Structure:** Physical energy flow ($u \to \theta_m \to \text{coupling} \to \theta_l$) creates a chain of integrators with cross-coupling. Backstepping treats $\theta_m$ as virtual control for $\theta_l$, then $\omega_m$ for $\theta_m$, finally synthesizing $u$ to close the loop with guaranteed stability.
- **Disturbances $d(t)$:** Lumped terms representing unmodeled Coulomb friction, backlash, variable payload, or sensor noise entering acceleration channels.

---
