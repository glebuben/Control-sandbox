# 1. State-Space Model (Briefly)

State vector:
$$
x = \begin{bmatrix} \theta_l \\ \omega_l \\ \theta_m \\ \omega_m \end{bmatrix} \in \mathbb{R}^4
$$
| Component | Physical Meaning | Units |
|:---|:---|:---|
| $\theta_l$ | Load angular position | rad |
| $\omega_l$ | Load angular velocity | rad/s |
| $\theta_m$ | Motor angular position | rad |
| $\omega_m$ | Motor angular velocity | rad/s |

Control (action) vector:
$$
u = \tau \in \mathbb{R}
$$
| Component | Physical Meaning | Units | Feasible Range |
|:---|:---|:---|:---|
| $\tau$ | Motor driving torque | N·m | $[-\tau_{\max}, +\tau_{\max}]$ |

Nonlinear Dynamics:
$$\dot{x} = f(x) + g(x)u + d(t)$$
Explicit form:
$$
\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= -\frac{k}{J_l}(x_1 - x_3) - \frac{b}{J_l}(x_2 - x_4) + d_l(t) \\
\dot{x}_3 &= x_4 \\
\dot{x}_4 &= \frac{1}{J_m}\left[ u - k(x_3 - x_1) - b(x_4 - x_2) \right] + d_m(t)
\end{aligned}
$$

---

# 2. Derivation of Flexible-Joint Dynamics

## 2.1 Physical Setup & Free-Body Diagrams
The system consists of a DC servo motor rigidly coupled to a torsional spring-damper, which in turn drives a rotational load. The coupling compliance introduces a relative angular displacement $\Delta\theta = \theta_m - \theta_l$ and relative velocity $\Delta\omega = \omega_m - \omega_l$.

Applying Newton-Euler rotational dynamics to each inertial element:
$$
J_l \ddot{\theta}_l = \underbrace{k(\theta_m - \theta_l)}_{\text{Spring torque}} + \underbrace{b(\dot{\theta}_m - \dot{\theta}_l)}_{\text{Viscous damping}} - T_{\text{ext}}(t)
$$
$$
J_m \ddot{\theta}_m = \tau - k(\theta_m - \theta_l) - b(\dot{\theta}_m - \dot{\theta}_l) - T_{\text{fric}}(\omega_m)
$$

## 2.2 Dimensional Scaling & Parameter Grouping
The torsional stiffness $k$ [N·m/rad] and damping $b$ [N·m·s/rad] are identified experimentally or derived from coupling geometry:
$$
k = \frac{G J_p}{L_c}, \quad b = \eta \sqrt{k J_{\text{eq}}}
$$
where $G$ is shear modulus, $J_p$ polar moment of inertia of the shaft, $L_c$ coupling length, and $\eta$ structural damping ratio.

For control-oriented modeling, external disturbances and unmodeled friction are aggregated into additive terms $d_l(t), d_m(t)$. The torque input $\tau$ is assumed to be generated instantaneously by a fast inner current loop ($i_a \propto \tau$), which is standard for backstepping design at the mechanical level.

## 2.3 Strict-Feedback Form Conversion
Defining the state vector $x = [\theta_l, \omega_l, \theta_m, \omega_m]^\top$, the dynamics naturally cascade:
$$
\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= f_2(x_1,x_2,x_4) + g_2(\cdot)x_3 \\
\dot{x}_3 &= x_4 \\
\dot{x}_4 &= f_4(x_1,x_2,x_3,x_4) + g_4(\cdot)u
\end{aligned}
$$
This **triangular strict-feedback structure** is a necessary condition for recursive backstepping. Each subsystem depends only on its own state, preceding states, and the next state/control, enabling sequential Lyapunov-based stabilization without inversion of the full Jacobian.

---

# 3. Complete Symbol Dictionary

| Symbol | Meaning | Units | Role in Model |
|:---|:---|:---|:---|
| $x$ | State vector | – | $x = [\theta_l, \omega_l, \theta_m, \omega_m]^\top$ |
| $u$ | Control input | N·m | Motor torque command $\tau$ |
| $t$ | Time | s | Independent variable |
| $f(x), g(x)$ | Drift & input vector fields | varies | $\dot{x} = f(x) + g(x)u + d(t)$ |
| $\theta_l$ | Load position | rad | Controlled output |
| $\omega_l$ | Load velocity | rad/s | $= \dot{\theta}_l$ |
| $\theta_m$ | Motor position | rad | Actuator side state |
| $\omega_m$ | Motor velocity | rad/s | $= \dot{\theta}_m$ |
| $J_l$ | Load inertia | kg·m² | Rotational mass of payload |
| $J_m$ | Motor inertia | kg·m² | Rotor + attached hardware inertia |
| $k$ | Torsional stiffness | N·m/rad | Coupling spring constant |
| $b$ | Torsional damping | N·m·s/rad | Structural/viscous coupling loss |
| $\tau$ | Applied torque | N·m | Control action from motor |
| $d_l, d_m$ | Disturbances | rad/s² | Unmodeled friction, load torque, noise |
| $T_{\text{ext}}$ | External load torque | N·m | Process disturbance (lumped in $d_l$) |
| $\Delta\theta$ | Relative displacement | rad | $\theta_m - \theta_l$, drives spring torque |
| $\Delta\omega$ | Relative velocity | rad/s | $\omega_m - \omega_l$, drives damping torque |

---

# 4. System Schematic & Physical Interpretation

```
                  ↑ τ (Motor Torque)
                  │
          ┌───────┴───────┐
          │   DC Motor    │ ← J_m, ω_m, θ_m
          │  (Inertia Jm) │
          └───────┬───────┘
                  │ Δθ, Δω
        ┌─────────┴─────────┐
        │ Flexible Coupling │ ← k, b (Spring-Damper)
        └─────────┬─────────┘
                  │
          ┌───────┴───────┐
          │   Payload     │ ← J_l, ω_l, θ_l (Controlled Output)
          │ (Inertia Jl)  │
          └───────────────┘
```

**How the diagram maps to the model:**
- **Motor Block:** Generates torque $\tau$. Its dynamics are governed by $J_m \ddot{\theta}_m = \tau - \tau_{\text{coupling}}$.
- **Flexible Coupling:** Acts as a torsional spring-damper. Transmits torque $\tau_{\text{coupling}} = k(\theta_m - \theta_l) + b(\omega_m - \omega_l)$ to the load.
- **Load Block:** Receives coupling torque. Dynamics: $J_l \ddot{\theta}_l = \tau_{\text{coupling}}$. This is the non-collocated output we aim to stabilize.
- **Cascade Structure:** The physical energy flow ($\tau \to \theta_m \to \text{spring} \to \theta_l$) creates a chain of integrators with cross-coupling. Backstepping exploits this by treating $\theta_m$ as a virtual control for $\theta_l$, then $\omega_m$ for $\theta_m$, and finally synthesizing $\tau$ to close the loop.
- **Disturbances $d(t)$:** Represent unmodeled Coulomb friction, backlash, sensor noise, or variable payload inertia entering additively into acceleration channels.

---

# 5. Representative Parameters Table (Lab-Scale Flexible Joint)

| Parameter | Symbol | Typical Value | Notes |
|:---|:---|:---|:---|
| Load inertia | $J_l$ | $0.025$ kg·m² | Aluminum disk + added weights |
| Motor inertia | $J_m$ | $0.0032$ kg·m² | Rotor + encoder hub |
| Coupling stiffness | $k$ | $12.5$ N·m/rad | Steel torsion shaft |
| Coupling damping | $b$ | $0.15$ N·m·s/rad | Structural + bearing losses |
| Max torque | $\tau_{\max}$ | $2.4$ N·m | Continuous rating (thermal limit) |
| Peak torque | $\tau_{\text{peak}}$ | $7.2$ N·m | Short-duration overload (3s) |
| Resonant frequency | $\omega_n$ | $\approx 22.3$ rad/s | $\sqrt{k(1/J_l + 1/J_m)}$ |
| Damping ratio | $\zeta$ | $0.03$–$0.08$ | Lightly damped resonance |
| Encoder resolution | – | $4096$ PPR | Typical for $x_1, x_3$ measurement |
| Sampling rate | $f_s$ | $1000$ Hz | Discrete implementation for backstepping |