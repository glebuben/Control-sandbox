# Mathematical Model of the Nonlinear Flexible-Joint Drive

## 1. State-Space Representation

**State vector:**
$$
x = \begin{bmatrix} \theta_l \\ \omega_l \\ \theta_m \\ \omega_m \end{bmatrix} \triangleq \begin{bmatrix} x_1 \\ x_2 \\ x_3 \\ x_4 \end{bmatrix} \in \mathbb{R}^4
$$
| Component | Physical Meaning | Units |
|:---|:---|:---|
| $x_1 = \theta_l$ | Load angular position | rad |
| $x_2 = \omega_l$ | Load angular velocity | rad/s |
| $x_3 = \theta_m$ | Motor angular position | rad |
| $x_4 = \omega_m$ | Motor angular velocity | rad/s |

**Control input:**
$$
u = \tau_m \in \mathbb{R} \quad [\text{N}\cdot\text{m}]
$$

**Nonlinear dynamics:**
$$
\dot{x} = f(x) + g u + d(t)
$$
where $g = [0, 0, 0, 1/J_m]^\top$ and $d(t) = [0, d_l(t), 0, d_m(t)]^\top$ represents bounded acceleration disturbances.

The terms $d_l(t)$ and $d_m(t)$ represent bounded acceleration disturbances due to unmodeled payload variations, external torque perturbations, and parameter identification errors. For the nominal backstepping synthesis, we assume $d_l(t) \equiv d_m(t) \equiv 0$ to focus on exact nonlinear cancellation.

---

## 2. Derivation of Nonlinear Flexible-Joint Dynamics

### 2.1 Relative Coordinates & Coupling Torque
Define the shaft twist and relative angular velocity:
$$
\delta = \theta_m - \theta_l = x_3 - x_1, \qquad \nu = \omega_m - \omega_l = x_4 - x_2
$$
The torque transmitted through the compliant coupling is modeled as:
$$
\tau_c(\delta, \nu) = k\,\delta + k_3\,\delta^3 + b\,\nu
$$
- $k > 0$: Linear torsional stiffness [N·m/rad]
- $k_3 \ge 0$: Cubic hardening coefficient [N·m/rad$^3$]. Captures amplitude-dependent stiffness growth.
- $b > 0$: Structural damping [N·m·s/rad]. Dissipates relative motion energy.

The corresponding elastic potential energy is $U(\delta) = \frac{1}{2}k\delta^2 + \frac{1}{4}k_3\delta^4$, confirming that the conservative spring torque is a gradient field: $\partial U/\partial \delta = k\delta + k_3\delta^3$.

### 2.2 Smooth Friction Approximation
Bearing and gear friction on both inertias is modeled using a smooth Coulomb-viscous approximation:
$$
T_{f}(\omega) = F_c \tanh\!\left(\frac{\omega}{v_s}\right) + B_v \omega
$$
| Parameter | Meaning | Units |
|:---|:---|:---|
| $F_c \ge 0$ | Coulomb friction level | N·m |
| $v_s > 0$ | Stribeck smoothing threshold | rad/s |
| $B_v \ge 0$ | Viscous friction coefficient | N·m·s/rad |

**Why $\tanh(\cdot)$?**  
The hyperbolic tangent is analytic ($C^\infty$) everywhere, unlike the discontinuous $\operatorname{sign}(\omega)$. Its derivative $\frac{d}{d\omega}\tanh(\omega/v_s) = \frac{1}{v_s}\operatorname{sech}^2(\omega/v_s)$ exists globally, which is essential for recursive Lyapunov/backstepping differentiation and avoids nonsmooth Filippov analysis. Near zero velocity, $\tanh(\omega/v_s) \approx \omega/v_s$, so the friction behaves as an additional viscous damping term $B_{\text{eff}} = F_c/v_s + B_v$, ensuring smooth small-signal dynamics.

### 2.3 Newton–Euler Equations
Applying rotational dynamics to each inertia yields:
$$
J_l \dot{\omega}_l = \tau_c(\delta,\nu) - T_{f,l}(\omega_l) + \tau_{d,l}(t)
$$
$$
J_m \dot{\omega}_m = u - \tau_c(\delta,\nu) - T_{f,m}(\omega_m) + \tau_{d,m}(t)
$$
Here $\tau_{d,l}$ and $\tau_{d,m}$ are external disturbance torques. In acceleration form, define normalized disturbances $d_l = \tau_{d,l}/J_l$ and $d_m = \tau_{d,m}/J_m$. The acceleration channels become:
$$
\dot{\omega}_l = \frac{1}{J_l}\left[ \tau_c - T_{f,l} \right] + d_l(t)
$$
$$
\dot{\omega}_m = \frac{1}{J_m}\left[ u - \tau_c - T_{f,m} \right] + d_m(t)
$$

### 2.4 Explicit State-Space Dynamics
Substituting the coupling and friction expressions into the kinematic and dynamic relations gives the complete nonlinear vector field:
$$
\begin{aligned}
\dot{x}_1 &= x_2 \\[4pt]
\dot{x}_2 &= \frac{1}{J_l}\Bigl[ k(x_3-x_1) + k_3(x_3-x_1)^3 + b(x_4-x_2) - T_{f,l}(x_2) \Bigr] + d_l(t) \\[4pt]
\dot{x}_3 &= x_4 \\[4pt]
\dot{x}_4 &= \frac{1}{J_m}\Bigl[ u - k(x_3-x_1) - k_3(x_3-x_1)^3 - b(x_4-x_2) - T_{f,m}(x_4) \Bigr] + d_m(t)
\end{aligned}
$$

---

## 3. Mathematical Properties Relevant to Control Synthesis

### 3.1 Input-Affine Structure & Regularity
The system is **input-affine**: $u$ appears linearly only in $\dot{x}_4$. The drift vector field $f(x)$ consists of polynomial stiffness terms and $C^\infty$ friction terms, making $f(x)$ **locally Lipschitz** on $\mathbb{R}^4$. For bounded $u(t)$ and finite initial energy, solutions are forward-complete (no finite-time escape) due to the radially unbounded elastic potential $U(\delta)$. While the cubic term prevents global Lipschitz continuity, the energy inequality $\dot{E} \le \omega_m u + c_1 E + c_2$ guarantees that states remain bounded over finite horizons under physically realistic actuator limits.

### 3.2 Cascade Structure & Backstepping Compatibility
In the original coordinates, $\dot{x}_2$ depends on both $x_3$ and $x_4$ through $\tau_c$, breaking the strict triangular form $\dot{x}_i = f_i(x_{1:i}) + g_i x_{i+1}$. However, by introducing the **transmitted torque** $\tau_c$ as an intermediate virtual control variable, the dynamics reveal a recursive cascade:
$$
\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= \frac{1}{J_l}(\tau_c - T_{f,l}) + d_l \\
\dot{\tau}_c &= \frac{b}{J_m}u + \Phi(x) \quad \text{(where $\Phi(x)$ contains known nonlinearities)}
\end{aligned}
$$
This structure enables **torque-shaped backstepping**: stabilize position error → shape desired load velocity → shape desired coupling torque $\tau_c^*$ → synthesize motor torque $u$ to track $\tau_c^*$. The cubic stiffness mapping $\delta \mapsto k\delta + k_3\delta^3$ is strictly monotonic for $k_3 \ge 0$, guaranteeing a unique inverse for computing the required shaft twist $\delta_d$. The recursive Lyapunov design explicitly shapes the elastic potential and damping distribution, guaranteeing asymptotic tracking without linearization or gain scheduling.

### 3.3 Energy & Passivity Characteristics
Define the total mechanical energy:
$$
E(x) = \frac{1}{2}J_l x_2^2 + \frac{1}{2}J_m x_4^2 + \frac{1}{2}k(x_3-x_1)^2 + \frac{1}{4}k_3(x_3-x_1)^4
$$
For the nominal disturbance-free system ($d(t)=0$), the time derivative satisfies:
$$
\dot{E} = \omega_m u - b(\omega_m-\omega_l)^2 - \omega_l T_{f,l}(\omega_l) - \omega_m T_{f,m}(\omega_m) \leq \omega_m u
$$
This proves the plant is **passive** from motor torque $u$ to motor velocity $\omega_m$. The coupling damping $b>0$ and smooth friction terms are strictly dissipative, providing natural energy decay that backstepping can exploit for large-domain stability. The internal shaft twist dynamics $b\dot{\delta} + k\delta + k_3\delta^3 = \tau_c$ are input-to-state stable (ISS) with respect to $\tau_c$, ensuring bounded internal states under bounded control actions.

---

## 4. System Schematic & Physical Interpretation

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

**Mapping to the mathematical model:**  
The physical architecture directly dictates the cascade structure of the differential equations. The motor inertia ($J_m$) receives the electromagnetic torque $u$ and converts it into angular acceleration $\dot{\omega}_m$. This acceleration is opposed by two forces: the transmitted coupling torque $\tau_c$ and motor-side friction $T_{f,m}$. The coupling acts as a torsional spring-damper, storing energy elastically ($k\delta + k_3\delta^3$) and dissipating it viscoelastically ($b\nu$). Crucially, $\tau_c$ depends on the *relative* displacement and velocity between motor and load, which creates the cross-coupling terms $(x_3-x_1)$ and $(x_4-x_2)$ in the state equations. The load inertia ($J_l$) is not directly actuated; it only responds to $\tau_c$ minus load-side friction. This **non-collocated actuation** means the control input $u$ must travel through the flexible element to influence the controlled output $\theta_l$, introducing a resonance pair that shifts with amplitude due to $k_3$. The smooth $\tanh(\cdot)$ friction models the transition from static/dry friction near zero speed to velocity-proportional losses at higher speeds, avoiding discontinuities that would break Lyapunov differentiation. Backstepping exploits this energy flow by treating $\tau_c$ as a virtual control: the controller first computes the exact elastic torque needed to stabilize $\theta_l$, then determines the required shaft twist $\delta_d$, and finally synthesizes $u$ to drive the physical coupling toward that torque while damping relative oscillations. This preserves the physical energy constraints and guarantees global tracking without linearizing the plant.

---

## 5. Why Linear Control Methods Fail

Linear control techniques such as PID tuning, LQR, or pole placement rely on a first-order Taylor expansion of the dynamics around a fixed equilibrium point. While they perform adequately for small deviations, they fundamentally fail to address the structural nonlinearities inherent in this flexible-joint system. 

**Amplitude-dependent stiffness:** The cubic term $k_3\delta^3$ causes the effective torsional stiffness to grow with shaft deflection. This shifts the natural frequency $\omega_n \approx \sqrt{(k + 3k_3\delta^2)/J_{\text{eq}}}$ during large transients. A linear controller tuned at $\delta \approx 0$ becomes detuned as $\delta$ increases, leading to degraded damping, overshoot, or even sustained oscillations when the resonance moves into the controller bandwidth.

**Non-collocated actuation & non-minimum phase-like behavior:** Because the input acts on the motor while the output is the load, the transfer function from $u$ to $\theta_l$ contains a pair of lightly damped complex poles. Linear PID controllers struggle to damp this resonance without severely limiting the control bandwidth, resulting in sluggish response. LQR can place poles arbitrarily, but the optimal gains are computed for a single linearized $A,B$ matrix; they do not adapt when the operating point changes or when $\delta$ becomes large enough to activate $k_3$.

**State-dependent friction dissipation:** The $\tanh(\omega/v_s)$ friction model transitions from high effective damping near zero velocity to saturated Coulomb behavior at higher speeds. A linear controller uses constant gains and cannot compensate for this velocity-dependent energy loss, causing steady-state errors during slow tracking or excessive control effort during reversals.

**Structural mismatch with backstepping requirements:** Backstepping does not treat nonlinearities as perturbations; it uses them explicitly in the control law. The recursive Lyapunov design cancels $k_3\delta^3$ and $T_f(\omega)$ analytically, shapes the elastic potential energy, and distributes damping exactly where it is needed in the cascade. Linear methods cannot perform this energy shaping because they discard higher-order terms by definition. Consequently, backstepping guarantees large-domain asymptotic stability, preserves physical passivity, and maintains performance across wide operating ranges without gain scheduling or heuristic anti-windup modifications.

---

## 6. Complete Symbol Dictionary

| Symbol | Meaning | Units | Role |
|:---|:---|:---|:---|
| $x$ | State vector | – | $[\theta_l, \omega_l, \theta_m, \omega_m]^\top$ |
| $u$ | Control input | N·m | Motor electromagnetic torque $\tau_m$ |
| $J_l, J_m$ | Load & motor inertia | kg·m² | Rotational mass parameters |
| $k$ | Linear torsional stiffness | N·m/rad | Baseline coupling elasticity |
| $k_3$ | Cubic stiffness coefficient | N·m/rad³ | Amplitude-dependent hardening |
| $b$ | Coupling damping | N·m·s/rad | Structural dissipation |
| $\delta, \nu$ | Shaft twist & relative velocity | rad, rad/s | $\delta=x_3-x_1, \nu=x_4-x_2$ |
| $\tau_c$ | Transmitted coupling torque | N·m | $k\delta + k_3\delta^3 + b\nu$ |
| $T_{f,l}, T_{f,m}$ | Friction torques | N·m | Smooth Coulomb-viscous model |
| $F_c, v_s, B_v$ | Friction parameters | N·m, rad/s, N·m·s/rad | $\tanh$-based smooth approximation |
| $d_l, d_m$ | Normalized disturbances | rad/s² | Unmodeled load/payload/noise |
| $E(x)$ | Total mechanical energy | J | Lyapunov/storage function candidate |
| $U(\delta)$ | Elastic potential energy | J | $\frac{1}{2}k\delta^2 + \frac{1}{4}k_3\delta^4$ |
| $\Phi(x)$ | Nonlinear drift in $\dot{\tau}_c$ | N·m/s | Coupling derivative terms for backstepping |

