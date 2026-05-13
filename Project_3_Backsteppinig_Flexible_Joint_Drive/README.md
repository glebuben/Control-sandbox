# 🔌 Project 3 – Backstepping Control of a nonlinear Flexible-Joint Drive
**A simulation of a nonlinear motor system with cubic torsional stiffness and smooth friction, controlled via a recursive Lyapunov-based backstepping law.**

The project includes a 4-DOF nonlinear plant model, nominal backstepping controller, baseline comparison, static plots (time-series, phase portraits, Lyapunov function evolution), and an interactive `pygame` visualisation with GIF export.


📋 Brief Description    
 The backstepping controller recursively stabilises load position by treating transmitted torque as a virtual control, guaranteeing asymptotic tracking without linearisation or gain scheduling. 
![alt text](animations/animation_backstepping.gif)

| Component | Description |
|-----------|-------------|
| **Baseline controller** | PID and PD tuned at small-signal equilibrium; loses damping during large twist or friction saturation |
| **Backstepping controller** | Recursive Lyapunov design using coupling torque as virtual control; exact cancellation of $k_3\delta^3$ and $T_f(\omega)$|
| **Flexible-joint dynamics** | Nonlinear non-collocated mechanical system with cubic stiffness & smooth Coulomb-viscous friction |

The complete derivation of the nonlinear state-space model, possible implementation current-actuator mapping, energy passivity analysis, and recursive backstepping control law with stability proofs is provided in [model_motor_draftV3.md](model_motor_draftV3.md) and [readme_flexible_joint_backstepping_theory_v2.md](readme_flexible_joint_backstepping_theory_v2.md).

**Run Project 3:**
```bash
cd Project_3_Backstepping_Flexible_Joint_Drive/src
python main.py         
```

🔧 Architecture of Project
```
Project_3_Backstepping_Flexible_Joint_Drive/
├── src/
│   ├── system.py          # nonlinear flexible-joint plant model
│   ├── controller.py      # backstepping & baseline controllers
│   ├── simulation.py      # data collection & numerical integration
│   ├── visualization.py   # matplotlib plots & pygame animation
│   └── main.py            # entry point & CLI parser
├── configs/               # YAML/JSON parameter files
├── figures/               # static plots (PNG/PDF)
├── animations/            # GIF exports
├── code_description.md    # repository guide & CLI reference
├── model_motor_draftV3.md # detailed nonlinear plant derivation
├── readme_flexible_joint_backstepping_theory_v2.md # control law, current actuation & Lyapunov proofs
└── README.md
```

## 1. System Description & Symbol Dictionary
Full mathematical derivation: [model_motor_draftV3.md](model_motor_draftV3.md) §1-2, [readme_flexible_joint_backstepping_theory_v2.md](readme_flexible_joint_backstepping_theory_v2.md) §1

*Scheme with meanings only for main idea reference. Description of symbols below.*
![alt text](figures/643f7356-6309-46ce-a87d-aaad20e8a6ec.jpg)

**State & Control Vectors**
$$
s = \begin{bmatrix} \theta_l \\ \omega_l \\ \theta_m \\ \omega_m \end{bmatrix} \in \mathbb{R}^4
$$
$$
a = \tau_m  \in \mathbb{R}
$$
| Symbol | Meaning | Units |
|--------|---------|-------|
| $s$ | State vector | – |
| $\theta_l$ | Load angular position | rad |
| $\omega_l$ | Load angular velocity | rad/s |
| $\theta_m$ | Motor angular position | rad |
| $\omega_m$ | Motor angular velocity | rad/s |
| $a$ | Action- Control input | N·m  |
| $\tau_m $ | Motor electromagnetic torque | N·m |

**Nonlinear Dynamics**
$$\boxed{\dot{s} = f(s) + g a + d(t)}$$

where $g = [0, 0, 0, 1/J_m]^\top$ and $d(t) = [0, d_l(t), 0, d_m(t)]^\top$ represents bounded acceleration disturbances.

**NOTE:** The terms $d_l(t)$ and $d_m(t)$ represent bounded acceleration disturbances due to unmodeled payload variations, external torque perturbations, and parameter identification errors. For the nominal backstepping synthesis, we assume $d_l(t) \equiv d_m(t) \equiv 0$ to focus on exact nonlinear cancellation.

Explicit form:
$$
\begin{aligned}
\dot{s}_1 &= s_2 \\
\dot{s}_2 &= \frac{1}{J_l}\Bigl[ k(s_3-s_1) + k_3(s_3-s_1)^3 + b(s_4-s_2) - T_{f,l}(s_2) \Bigr] + d_l(t) \\
\dot{s}_3 &= s_4 \\
\dot{s}_4 &= \frac{1}{J_m}\Bigl[ K_t u - k(s_3-s_1) - k_3(s_3-s_1)^3 - b(s_4-s_2) - T_{f,m}(s_4) \Bigr] + d_m(t)
\end{aligned}
$$
| Symbol | Meaning | Units |
|--------|---------|-------|
| $\dot{s}$ | State derivative | varies |
| $f(s), g$ | Drift & input vector fields | varies |
| $J_l, J_m$ | Load & motor inertia | kg·m² |
| $k, k_3, b$ | Linear/cubic stiffness, damping | N·m/rad, N·m/rad³, N·m·s/rad |
| $T_{f,l}, T_{f,m}$ | Smooth friction torques | N·m |
| $d_l, d_m$ | Bounded acceleration disturbances | rad/s² |
| $K_t$ | Motor torque constant | N·m/A |

## 2. Nonlinear Coupling & Friction Model
Full derivation: [model_motor_draftV3.md](model_motor_draftV3.md) §2, [readme_flexible_joint_backstepping_theory_v2.md](readme_flexible_joint_backstepping_theory_v2.md) §1-4

**Relative Coordinates & Transmitted Torque**   
From kinematic coupling and constitutive shaft behaviour:
$$\delta \triangleq \theta_m - \theta_l = s_3 - s_1, \qquad \nu \triangleq \omega_m - \omega_l = s_4 - s_2$$
The elastic-dissipative torque transmitted through the flexible shaft is:
$$\tau_c = k\delta + k_3\delta^3 + b\nu$$
| Symbol | Meaning | Units |
|--------|---------|-------|
| $\delta, \nu$ | Shaft twist & relative velocity | rad, rad/s |
| $k$ | Linear torsional stiffness | N·m/rad |
| $k_3$ | Cubic hardening coefficient | N·m/rad³ |
| $b$ | Structural damping | N·m·s/rad |
| $\tau_c$ | Coupling torque | N·m |

Elastic potential energy: $U(\delta) = \frac{1}{2}k\delta^2 + \frac{1}{4}k_3\delta^4$, confirming $\partial U/\partial \delta = k\delta + k_3\delta^3$.

**Smooth Friction Approximation**
To preserve differentiability for Lyapunov synthesis, discontinuous Coulomb friction is approximated by a hyperbolic tangent:
$$T_f(\omega) = F_c \tanh\left(\frac{\omega}{v_s}\right) + B_v \omega$$
| Symbol | Meaning | Units |
|--------|---------|-------|
| $F_c$ | Coulomb friction level | N·m |
| $v_s$ | Stribeck smoothing threshold | rad/s |
| $B_v$ | Viscous friction coefficient | N·m·s/rad |

The $\tanh(\cdot)$ function is $C^\infty$ everywhere, with derivative $\frac{d}{d\omega}T_f = \frac{F_c}{v_s}\operatorname{sech}^2(\omega/v_s) + B_v$. This guarantees smooth recursive backstepping differentiation and avoids Filippov nonsmooth analysis. Near zero velocity, $T_f(\omega) \approx (F_c/v_s + B_v)\omega$, acting as enhanced viscous damping.

## 3. Control Strategy & Lyapunov Proofs
Full derivation: [readme_flexible_joint_backstepping_theory_v2.md](readme_flexible_joint_backstepping_theory_v2.md) §7-10, §13

### 3.1 Error Coordinates & Virtual Controls
Define tracking error and first virtual control:
$$z_1 = \theta_l - \theta_d, \quad \alpha_1 = \dot{\theta}_d - c_1 z_1, \quad z_2 = \omega_l - \alpha_1$$
The load velocity error dynamics:
$$\dot{z}_2 = \frac{1}{J_l}\left(\tau_c - T_{f,l}\right) - \dot{\alpha}_1$$

### 3.2 Desired Coupling Torque 
Treat $\tau_c$ as intermediate virtual control. Choose:
$$\tau_c^* = J_l\left(\dot{\alpha}_1 - c_2 z_2 - z_1\right) + T_{f,l}$$
Define torque tracking error: $z_3 = \tau_c - \tau_c^*$. Then:
$$\dot{z}_2 = -c_2 z_2 - z_1 + \frac{1}{J_l}z_3$$

### 3.3 Final Backstepping Law & Current Mapping 
The coupling torque derivative is $\dot{\tau}_c = \frac{b }{J_m}a + \Phi(x)$, where $\Phi(x)$ contains known nonlinearities and:     

$$\Phi(x) = (k+3k_3\delta^2)\nu - b\left(\frac{1}{J_m}+\frac{1}{J_l}\right)\tau_c - \frac{b}{J_m}T_{f,m} + \frac{b}{J_l}T_{f,l}$$

 The backstepping design with motor torque:
$$\boxed{a = \frac{J_m}{b}\left[-\Phi(x) + \dot{\tau}_c^* - c_3 z_3 - \frac{1}{J_l}z_2\right]}$$
For practical actuation, torque is usually realised via motor current $i$:
$$\boxed{i^* = \frac{\tau_m^*}{K_t}}$$
Under ideal current tracking ($K_t i \approx \tau_m^*$=a), the mechanical closed-loop dynamics remain unchanged. But in our model we were mainly focused on how to cope with a lot of nonlinearities, in that case we just showed that we know how to control motor in real life but in our exact example we control straightly torque.

| Symbol | Meaning | Units |
|--------|---------|-------|
| $z_1, z_2, z_3$ | Tracking & torque errors | rad, rad/s, N·m |
| $c_1, c_2, c_3 > 0$ | Backstepping convergence gains | s⁻¹ |
| $\Phi(x)$ | Known nonlinear drift | N·m/s |
| $\tau_c^*$ | Desired elastic torque | N·m |
| $K_t$ | Torque constant | N·m/A |

### 3.4 Lyapunov Stability Proof
Composite Lyapunov candidate:
$$\boxed{\mathcal{V} = \frac{1}{2}z_1^2 + \frac{1}{2}z_2^2 + \frac{1}{2}z_3^2}$$
Derivative under closed-loop dynamics:
$$\dot{\mathcal{V}} = -c_1 z_1^2 - c_2 z_2^2 - c_3 z_3^2 \leq 0$$
Since $\dot{\mathcal{V}}$ is negative definite, $z_1(t), z_2(t), z_3(t) \to 0$ asymptotically. The internal shaft dynamics $b\dot{\delta} + k\delta + k_3\delta^3 = \tau_c$ are input-to-state stable (ISS), guaranteeing bounded twist under bounded control.  
 Not in our case, but in addition about how to cope with current: Current saturation $|i| \leq i_{\max}$ introduces local stability margins but preserves convergence within actuator authority.

## 4. Parameters Reference
Source: [model_motor_draftV3.md](model_motor_draftV3.md) §7, [readme_flexible_joint_backstepping_theory_v2.md](readme_flexible_joint_backstepping_theory_v2.md) §13

**System Parameters (Lab-Scale Flexible Joint)**
| Symbol | Value | Units | Meaning |
|--------|-------|-------|---------|
| $J_l$ | 0.025 | kg·m² | Load inertia |
| $J_m$ | 0.0032 | kg·m² | Motor rotor inertia |
| $k$ | 12.5 | N·m/rad | Linear shaft stiffness |
| $k_3$ | 300 | N·m/rad³ | Cubic hardening coefficient |
| $b$ | 0.15 | N·m·s/rad | Coupling damping |
| $F_{c,l}, F_{c,m}$ | 0.08, 0.04 | N·m | Coulomb friction levels |
| $v_{s,l}, v_{s,m}$ | 0.05, 0.03 | rad/s | Stribeck thresholds |
| $B_{v,l}, B_{v,m}$ | 0.02, 0.015 | N·m·s/rad | Viscous friction |
| $\tau_{\text{peak}}$ | 7.2 | N·m | Motor torque limit |
| $K_t$ | 0.5 | N·m/A | Motor torque constant |
| $i_{\max}$ | 14.4 | A | Peak current limit ($\approx 7.2$ N·m) |

**Controller Parameters**
| Symbol | Value | Units | Meaning |
|--------|-------|-------|---------|
| $c_1$ | 5.0 | s⁻¹ | Position loop gain |
| $c_2$ | 10.0 | s⁻¹ | Velocity loop gain |
| $c_3$ | 15.0 | s⁻¹ | Torque tracking gain |
| $\theta_d(t)$ | – | rad | Reference trajectory (step/sinusoid) |
| $f_{\text{filter}}$ | 40 | Hz | Command filter cutoff for $\dot{\tau}_c^*$ |

## 5. Additional interaction control
Interactive Pygame Controls:
| Key | Action | Key | Action |
|-----|--------|-----|--------|
| SPACE / → | Step forward | ← | Step backward |
| R | Restart | A | Toggle auto-play |
| PgUp / PgDown | Speed control | S / G | Save PNG / Export GIF |
| Q / Esc | Quit | Click scrubber | Seek to time |

The technical content is mostly sound but there are a few issues to address:

1. **The Lyapunov claim is slightly overstated.** The brief oscillation at the start of V(t) means it is *not* strictly monotonically decreasing from t=0 — it decreases after a short transient. The theoretical guarantee holds asymptotically, not pointwise from the first sample. Claiming strict monotonicity when the plot visibly shows a dip-and-rise at t≈0 would invite criticism.

2. **"command filtering" is not in your model.** Your system has torque saturation clipping, not a filter. Don't claim something you haven't implemented.

3. **The discussion section undersells the result.** "The only visible advantage is convergence speed" is too weak — the tail plot shows persistent oscillation and failure to regulate, which is a qualitative difference, not just a speed difference.

4. **Minor language issues** throughout — passive constructions, informal phrasing, missing articles.

Here is a revised version:

---

## 6. Results

### Backstepping

![alt text](animations/animation_backstepping.gif)

The backstepping controller exhibits a brief transient during initialisation, then rapidly drives $\theta_l(t) \to \theta_d$ with zero steady-state error. The recursive design analytically cancels the cubic stiffness term $k_3\delta^3$ and the smooth friction model $T_f(\omega)$, explicitly shapes the elastic potential energy stored in the coupling, and injects virtual damping into the torque transmission channel at each step of the Lyapunov construction.

We verify closed-loop stability by examining the composite Lyapunov function $\mathcal{V} = \tfrac{1}{2}e_1^2 + \tfrac{1}{2}e_2^2 + \tfrac{1}{2}e_3^2$. After an initial transient — during which the large initial shaft twist causes the error coordinates to evolve rapidly — $\mathcal{V}$ decreases monotonically to zero, consistent with the theoretical guarantee $\dot{\mathcal{V}} \leq 0$ derived in the backstepping design.

![alt text](figures/ctrl_compare_eq_lyapunov.png)

### PD

![alt text](animations/animation_pd.gif)

### PID

![alt text](animations/animation_pid.gif)

### Discussion

At first glance, all three controllers appear to reach the vicinity of the desired position. The full simulation plot suggests the primary advantage of backstepping is faster convergence.

![alt text](figures/ctrl_compare_eq_states.png)

However, the steady-state detail plot — showing only the final second of the simulation — reveals a more subtle but structurally important difference. Both PD and PID converge to a stationary state, but that state is not $\theta_d = 1.0\,\text{rad}$. The PD controller stabilises at a constant positive offset above the setpoint, which is the expected behaviour for a proportional-derivative controller acting on a plant with Coulomb friction and nonlinear stiffness: with no integral action, the proportional term can only balance the steady-state disturbance torque at a nonzero position error. The PID controller, while theoretically capable of eliminating steady-state error through integration, shows a persistent and slowly drifting offset over the simulation horizon, suggesting the integral state has not yet converged — a consequence of the slow time constant introduced by the low integral gain $K_i = 1$ relative to the plant stiffness. In both cases the residual tracking error $e_1 = \theta_l - \theta_d$ remains visibly nonzero at $t = 10\,\text{s}$, while the backstepping controller achieves $e_1 \approx 0$ to numerical precision.

![alt text](figures/ctrl_compare_eq_states_tail.png)

Phase portraits confirm these observations. All three trajectories converge to a fixed point (zero velocity), but the final positions differ: PD and PID land slightly above $\theta_d$, while the backstepping trajectory terminates exactly at the target. The backstepping controller achieves this because it explicitly accounts for the shaft compliance, friction, and load dynamics in the control law — the recursive Lyapunov construction guarantees convergence to the exact equilibrium rather than to a bias point determined by the balance between the proportional gain and the unmodelled internal torques.

![alt text](figures/ctrl_compare_eq_phase_portraits.png)


***AI guidance***   
Acknowledgement of AI usage for theoretical information research, structural formatting of the documentation, controller tuning guidance.
