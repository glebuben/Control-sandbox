Here is a complete, self-contained mathematical model of an aircraft's longitudinal dynamics, written in English, using your requested notation (`s`, `a`, `P(s,a)`, etc.), with full derivations, a symbol dictionary, a schematic explanation, and a parameter table.

---
##  1. State-Space Model (Briefly)

**State vector:**  
$$s = \begin{bmatrix} V \\ \alpha \\ q \\ \theta \end{bmatrix} \in \mathbb{R}^4$$

**Control (action) vector:**  
$$a = \begin{bmatrix} \delta_e \\ \delta_t \end{bmatrix} \in \mathbb{R}^2$$

**Dynamics:**  
$$\dot{s} = P(s, a) + d(t)$$

**Explicit form of $P(s,a)$:**
$$
P(s,a) = \begin{bmatrix}
\frac{1}{m}\left[ T(\delta_t)\cos\alpha - D(V,\alpha,\delta_e) - mg\sin\theta \right] \\[6pt]
q - \frac{1}{mV}\left[ T(\delta_t)\sin\alpha + L(V,\alpha,\delta_e) - mg\cos\theta \right] \\[6pt]
\frac{M(V,\alpha,q,\delta_e)}{I_y} \\[6pt]
q
\end{bmatrix}
$$


---

#### **Observation Model**
The measured output is:
$$y = h(s)$$
For a rigid-body aircraft, sensors do not measure the abstract state $s$ directly; they measure physical quantities that can be mapped to $s$ via a known function $h$:
- **Full-state feedback:** $h(s) = s \implies y = [V, \alpha, q, \theta]^\top$ (requires pitot, AoA vane, gyro, inclinometer)
- **Partial feedback (typical):** $h(s) = C s$ where $C \in \mathbb{R}^{p \times 4}$ selects measurable states. Example:
  $$C = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \implies y = \begin{bmatrix} V \\ \theta \end{bmatrix}$$
- **Nonlinear observation:** If sensors have biases or nonlinear scaling, $h(s)$ can include calibration maps: $y = h(s) = C s + b_{\text{sensor}} + \eta_{\text{noise}}(s)$.

#### **Disturbance Model**
The additive term $d(t) \in \mathbb{R}^4$ captures all unmodeled dynamics, environmental effects, and sensor/actuator imperfections:
$$d(t) = \begin{bmatrix} d_V(t) \\ d_\alpha(t) \\ d_q(t) \\ d_\theta(t) \end{bmatrix}$$
| Component | Physical Origin | Typical Magnitude |
|-----------|----------------|-------------------|
| $d_V$ | Headwind/tailwind gusts, propulsion lag | $\pm 2\text{–}5 \text{ m/s}^2$ |
| $d_\alpha$ | Vertical gusts, wake turbulence, AoA sensor lag | $\pm 0.02\text{–}0.05 \text{ rad/s}$ |
| $d_q$ | Atmospheric turbulence, unsteady aerodynamics, gyro noise | $\pm 0.01\text{–}0.03 \text{ rad/s}^2$ |
| $d_\theta$ | IMU drift, magnetic variation, numerical integration error | $\pm 10^{-4}\text{–}10^{-3} \text{ rad/s}$ |

In the lecture's ISS framework, the full dynamics become:
$$\dot{s} = P(s,a) + d(t)$$
This structure is explicitly used for robustness analysis: $\|s(t)\| \leq \beta(\|s_0\|, t) + \gamma(\|d\|_\infty)$.

---
###  2. Derivation of Aerodynamic Forces & Moments

#### **2.1 Dynamic Pressure & Dimensional Scaling**
From Bernoulli's equation for steady, incompressible, inviscid flow along a streamline:
$$p + \frac{1}{2}\rho V^2 = \text{constant}$$
The term $\frac{1}{2}\rho V^2$ represents the kinetic energy per unit volume of the airstream, defined as the **dynamic pressure**:
$$\bar{q} \triangleq \frac{1}{2}\rho V^2 \quad [\text{Pa} = \text{N/m}^2]$$

Using the **Buckingham $\pi$ Theorem**, any aerodynamic force $F$ depends on:
- Fluid properties: $\rho, \mu$ (viscosity)
- Flow properties: $V, p_\infty$
- Geometry: $S$ (reference area), $b$ (span), shape parameters
- Orientation: $\alpha, \beta$

Non-dimensionalizing yields:
$$\frac{F}{\bar{q} S} = f(\text{Re}, \text{Ma}, \text{shape}, \alpha, \dots) \triangleq C_F$$
Thus, the universal scaling law for aerodynamic forces is:
$$F = \bar{q} S C_F = \frac{1}{2}\rho V^2 S C_F$$

#### **2.2 Lift & Drag Forces**
Lift ($L$) and drag ($D$) are defined relative to the **wind axes** (perpendicular and parallel to $V$):
$$L = \bar{q} S C_L(\alpha, \delta_e), \quad D = \bar{q} S C_D(\alpha, \delta_e)$$

For control design, coefficients are linearized around a trim condition $(\alpha_0, \delta_{e0})$ using a first-order Taylor expansion:
$$C_L(\alpha, \delta_e) \approx C_{L_0} + C_{L_\alpha}(\alpha - \alpha_0) + C_{L_{\delta_e}}(\delta_e - \delta_{e0})$$
$$C_D(\alpha, \delta_e) \approx C_{D_0} + C_{D_\alpha}(\alpha - \alpha_0) + C_{D_{\delta_e}}(\delta_e - \delta_{e0}) + C_{D_i}$$

**Induced Drag Derivation:**
From lifting-line theory, the downwash velocity $w$ creates an induced angle $\alpha_i \approx \frac{w}{V} = \frac{C_L}{\pi e \text{AR}}$. The induced drag coefficient is:
$$C_{D_i} = C_L \sin\alpha_i \approx \frac{C_L^2}{\pi e \text{AR}}$$
where:
- $\text{AR} = \frac{b^2}{S}$ (wing aspect ratio)
- $e \in [0.7, 0.95]$ (Oswald efficiency factor, accounts for non-elliptical lift distribution)

Final drag polar:
$$C_D(\alpha, \delta_e) = C_{D_0} + K(C_L - C_{L_0})^2, \quad K = \frac{1}{\pi e \text{AR}}$$

#### **2.3 Pitching Moment**
Moments require a reference length to convert force $\times$ distance into torque. In longitudinal dynamics, the **mean aerodynamic chord** $\bar{c}$ is used:
$$M = \bar{q} S \bar{c} \, C_m(\alpha, q, \delta_e)$$

The pitching moment coefficient is expanded as:
$$C_m(\alpha, q, \delta_e) \approx C_{m_0} + C_{m_\alpha}\alpha + C_{m_q}\underbrace{\left(\frac{q\bar{c}}{2V}\right)}_{\hat{q}} + C_{m_{\delta_e}}\delta_e$$

**Why $\hat{q} = \frac{q\bar{c}}{2V}$?**
- $q$ has units $[\text{rad/s}]$, but aerodynamic damping depends on the ratio of rotational speed to freestream speed.
- $\frac{\bar{c}}{2V}$ is the characteristic time for air to travel half the chord.
- $\hat{q}$ is **dimensionless**, making $C_{m_q}$ invariant to aircraft scale and flight speed.

**Stability & Damping Conditions:**
- $C_{m_\alpha} < 0 \implies$ **Static stability** (CG ahead of neutral point; $\alpha \uparrow \Rightarrow$ nose-down moment)
- $C_{m_q} < 0 \implies$ **Dynamic damping** (pitch rate $\uparrow \Rightarrow$ restoring moment opposes rotation)

#### **2.4 Thrust Model**
Engine thrust is fundamentally a function of mass flow and exhaust velocity: $T = \dot{m}(V_e - V)$. For control-oriented rigid-body modeling, we assume:
1. Engine dynamics are faster than airframe dynamics ($\tau_{\text{engine}} \ll \tau_{\text{airframe}}$)
2. Thrust line is aligned with the body $x$-axis
3. Throttle command $\delta_t \in [0,1]$ scales linearly

Thus:
$$T(\delta_t) = T_{\text{max}}(\rho, \text{Ma}) \cdot \delta_t \approx T_{\text{max}} \cdot \delta_t$$
*(Altitude/Mach dependencies are lumped into $T_{\text{max}}$ or treated as slow-varying parameters in $d(t)$)*

If actuator lag must be modeled:
$$\tau_T \dot{T} + T = T_{\text{max}} \delta_t \implies T(s) = \frac{T_{\text{max}}}{\tau_T s + 1} \delta_t(s)$$
For $\dot{s} = P(s,a)$, the algebraic form $T = T_{\text{max}}\delta_t$ is standard.

#### **2.5 Closing the Loop: From Physics to $P(s,a)$**
Substitute $L, D, M, T$ into the longitudinal Newton-Euler equations. Using wind axes for forces and body axes for moments:

**Force Balance (along & normal to $V$):**
$$m\dot{V} = T\cos\alpha - D - mg\sin\theta$$
$$mV\dot{\alpha} = mqV + T\sin\alpha + L - mg\cos\theta$$

**Moment Balance (about $y$-axis through CG):**
$$I_y \dot{q} = M$$

**Kinematic Coupling:**
$$\dot{\theta} = q$$

Solving for derivatives and substituting aerodynamic expressions:
$$
P(s,a) = \begin{bmatrix}
\frac{1}{m} \left[ T_{\text{max}}\delta_t \cos\alpha - \frac{1}{2}\rho V^2 S C_D(\alpha,\delta_e) - mg\sin\theta \right] \\[8pt]
q - \frac{1}{mV} \left[ T_{\text{max}}\delta_t \sin\alpha + \frac{1}{2}\rho V^2 S C_L(\alpha,\delta_e) - mg\cos\theta \right] \\[8pt]
\frac{1}{I_y} \left[ \frac{1}{2}\rho V^2 S \bar{c} \, C_m(\alpha,q,\delta_e) \right] \\[8pt]
q
\end{bmatrix}
$$

Expanding coefficients explicitly:
$$
\begin{aligned}
C_L &= C_{L_0} + C_{L_\alpha}\alpha + C_{L_{\delta_e}}\delta_e \\
C_D &= C_{D_0} + \frac{(C_L - C_{L_0})^2}{\pi e \text{AR}} \\
C_m &= C_{m_0} + C_{m_\alpha}\alpha + C_{m_q}\frac{q\bar{c}}{2V} + C_{m_{\delta_e}}\delta_e
\end{aligned}
$$

# This yields the complete, non-linear vector field $P(s,a)$ that exactly matches your lecture's form $\dot{s} = P(s,a)$. Every state derivative is explicitly defined, all control inputs are separated, and the model is ready for:
- Lyapunov candidate construction $L(s)$
- ISS robustness bounds with $d(t)$
- Barrier certificate synthesis for flight envelope protection
- Linearization $A = \frac{\partial P}{\partial s}, B = \frac{\partial P}{\partial a}$ for LQR/MPC


---
##  3. Complete Symbol Dictionary

| Symbol | Meaning | Units | Role in Model |
|--------|---------|-------|---------------|
| $s$ | State vector | – | $s = [V, \alpha, q, \theta]^\top$ |
| $a$ | Control (action) vector | – | $a = [\delta_e, \delta_t]^\top$ |
| $t$ | Time | s | Independent variable |
| $d(t)$ | Disturbance/uncertainty | varies | Additive unmodeled effects (gusts, bias, etc.) |
| $P(s,a)$ | Dynamics function | varies | Nonlinear vector field: $\dot{s} = P(s,a) + d(t)$ |
| $y$ | Measured output | varies | Sensor readings |
| $h(s)$ | Observation map | – | $y = h(s)$, often $h(s)=s$ |
| $V$ | Airspeed | m/s | Magnitude of velocity relative to air |
| $\alpha$ | Angle of attack | rad | Angle between wing chord and relative wind |
| $q$ | Pitch rate | rad/s | Angular velocity about lateral axis |
| $\theta$ | Pitch angle | rad | Orientation of longitudinal axis w.r.t. horizon |
| $\delta_e$ | Elevator deflection | rad | Control surface for pitching moment |
| $\delta_t$ | Throttle command | – | Normalized thrust input $\in [0,1]$ |
| $m$ | Aircraft mass | kg | Inertial property |
| $g$ | Gravitational acceleration | m/s² | $\approx 9.81$ |
| $\rho$ | Air density | kg/m³ | Depends on altitude |
| $S$ | Wing reference area | m² | Scales aerodynamic forces |
| $\bar{c}$ | Mean aerodynamic chord | m | Scales pitching moment |
| $I_y$ | Pitch moment of inertia | kg·m² | Resistance to angular acceleration |
| $L$ | Lift force | N | Perpendicular to relative wind |
| $D$ | Drag force | N | Parallel to relative wind |
| $M$ | Pitching moment | N·m | About center of gravity |
| $T$ | Thrust force | N | Along engine axis |
| $T_{\text{max}}$ | Maximum thrust | N | Engine limit at $\delta_t=1$ |
| $C_L, C_D, C_m$ | Aerodynamic coefficients | – | Dimensionless force/moment scalars |
| $C_{(\cdot)_\alpha}, C_{(\cdot)_{\delta_e}}, C_{m_q}$ | Stability/control derivatives | rad⁻¹ or – | Linearized sensitivity parameters |
| $AR$ | Wing aspect ratio | – | $b^2/S$, affects induced drag |
| $e$ | Oswald efficiency factor | – | $0.7\text{–}0.9$ for typical wings |

---
##  4. Our System Schematic & Physical Interpretation

```
                          ↑ L (Lift)
                          │
          ┌───────────────┼───────────────┐
          │               │               │
          │    Wing (S)   │   Fuselage    │
          │               │   (mass m)    │
          │               │               │
          └───────┬───────┴───────┬───────┘
                  │               │
        ← D       │               │       → T (Thrust)
                  │               │
            ┌─────┴─────┐   ┌─────┴─────┐
            │    CG     │   │  Engine   │
            │  (s=[V,α, │   │  δ_t → T  │
            │   q,θ])   │   └───────────┘
            └─────┬─────┘
                  │ q, θ
          ┌───────┴───────┐
          │ Horizontal    │
          │ Stabilizer    │
          │ δ_e → M       │
          └───────────────┘
```

**How the diagram maps to the model:**
- **Center of Gravity (CG):** Reference point for all forces and moments. The state $s$ describes motion relative to CG.
- **Wing & Stabilizer:** Generate $L$, $D$, and $M$. Their geometry defines $S$, $\bar{c}$, and the aerodynamic derivatives $C_{L_\alpha}, C_{m_\alpha}$, etc.
- **Elevator ($\delta_e$):** Deflects airflow on the tail, changing $C_m$ and producing pitching moment $M$. This is the primary control for $\alpha$ and $\theta$.
- **Engine ($\delta_t$):** Scales thrust $T$ linearly. Controls airspeed $V$ and indirectly affects $\alpha$ through force coupling.
- **State $s$:** $V$ and $\alpha$ describe translational aerodynamics; $q$ and $\theta$ describe rotational kinematics. Together they form the minimal longitudinal state.
- **Dynamics $P(s,a)$:** Encodes Newton's 2nd law (force → $\dot{V}, \dot{\alpha}$) and Euler's rotational equation (moment → $\dot{q}$), plus kinematic coupling ($\dot{\theta}=q$).
- **Disturbance $d(t)$:** Represents unmodeled wind, turbulence, or parameter drift entering additively into each state channel.

---
## 5. Cessna 172 Parameters Table (Representative Light Aircraft)

| Parameter | Symbol | Typical Value | Notes |
|-----------|--------|---------------|-------|
| Mass | $m$ | 1100 kg | Includes fuel & payload |
| Pitch inertia | $I_y$ | 1800 kg·m² | About lateral axis through CG |
| Wing area | $S$ | 16.2 m² | Reference for $L, D$ |
| Mean chord | $\bar{c}$ | 1.5 m | Reference for $M$ |
| Max thrust | $T_{\text{max}}$ | 2500 N | Sea-level static thrust |
| Lift slope | $C_{L_\alpha}$ | 5.0 rad⁻¹ | Linear range before stall |
| Pitch stiffness | $C_{m_\alpha}$ | −0.6 rad⁻¹ | Negative → statically stable |
| Pitch damping | $C_{m_q}$ | −8.0 rad⁻¹ | Stabilizes short-period mode |
| Elevator effectiveness | $C_{m_{\delta_e}}$ | −1.2 rad⁻¹ | Control authority |
| Zero-lift drag | $C_{D_0}$ | 0.03 | Parasitic drag baseline |
| Air density (SL) | $\rho$ | 1.225 kg/m³ | Decreases with altitude |

---
Also can be done than  **Jacobian linearization** $A = \frac{\partial P}{\partial s}$, $B = \frac{\partial P}{\partial a}$