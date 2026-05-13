# Nonlinear Flexible-Joint Drive with Smooth Friction and Backstepping-Oriented Control

*Theory, model derivation, Lyapunov analysis, and nominal nonlinear control synthesis for a two-inertia flexible-joint servo system.*

---

## Project Purpose

This document describes a nonlinear flexible-joint rotational drive consisting of a motor inertia connected to a load inertia through a compliant shaft. The main objective is to formulate a physically meaningful nonlinear state-space model and prepare it for Lyapunov-based nonlinear control synthesis, especially backstepping-oriented design.

The model includes three important effects:

1. nonlinear torsional stiffness of the flexible shaft;
2. structural damping in the coupling;
3. smooth Coulomb-viscous friction modeled with a hyperbolic tangent approximation.

The key controlled output is the load angular position $\theta_l$, while the control input is the motor electromagnetic torque $u=\tau_m$. This makes the system non-collocated: the input acts on the motor side, but the desired output is located on the load side.

> **Note:** GitHub renders LaTeX math in Markdown using `$...$` for inline and `$$...$$` for display math. All formulas in this document use this syntax and should render correctly on GitHub.

---

## 1. State-Space Model

### 1.1 State Vector

The state vector is chosen as

$$
x = \begin{bmatrix}\theta_l \\ \omega_l \\ \theta_m \\ \omega_m\end{bmatrix} = \begin{bmatrix} x_1 \\ x_2 \\ x_3 \\ x_4 \end{bmatrix} \in \mathbb{R}^4.
$$

| State | Meaning | Units |
|:---|:---|:---|
| $x_1=\theta_l$ | load angular position | rad |
| $x_2=\omega_l$ | load angular velocity | rad/s |
| $x_3=\theta_m$ | motor angular position | rad |
| $x_4=\omega_m$ | motor angular velocity | rad/s |

The control input is

$$u = \tau_m,$$

where $\tau_m$ is the motor electromagnetic torque.

---

### 1.2 Relative Variables

For compact notation, define the relative shaft twist and relative angular velocity:

$$\delta = \theta_m - \theta_l = x_3 - x_1,$$

$$\nu = \omega_m - \omega_l = x_4 - x_2.$$

The flexible coupling torque is

$$\tau_c = k\delta + k_3\delta^3 + b\nu.$$

| Symbol | Meaning | Units |
|:---|:---|:---|
| $k$ | linear torsional stiffness | N·m/rad |
| $k_3$ | cubic stiffness coefficient | N·m/rad$^3$ |
| $b$ | coupling damping coefficient | N·m·s/rad |
| $\delta$ | shaft twist | rad |
| $\nu$ | shaft relative velocity | rad/s |
| $\tau_c$ | transmitted coupling torque | N·m |

For the usual hardening case,

$$k > 0, \qquad k_3 \ge 0, \qquad b > 0.$$

If $k_3 > 0$, the shaft stiffness increases with deflection. If $k_3 < 0$, the model describes softening behavior, but then the elastic potential may lose global positive definiteness. For Lyapunov-based global or large-domain arguments, the hardening case $k_3 \ge 0$ is preferable.

---

### 1.3 Smooth Friction Model

The load-side and motor-side friction torques are modeled as

$$T_{f,l}(\omega_l) = F_{c,l}\tanh\left(\frac{\omega_l}{v_{s,l}}\right) + B_{v,l}\,\omega_l,$$

$$T_{f,m}(\omega_m) = F_{c,m}\tanh\left(\frac{\omega_m}{v_{s,m}}\right) + B_{v,m}\,\omega_m.$$

For a generic angular velocity $\omega$, the friction model is

$$T_f(\omega) = F_c\tanh\left(\frac{\omega}{v_s}\right) + B_v\,\omega.$$

| Symbol | Meaning | Units |
|:---|:---|:---|
| $F_c$ | Coulomb friction level | N·m |
| $v_s$ | smoothing velocity threshold | rad/s |
| $B_v$ | viscous friction coefficient | N·m·s/rad |

This is not a full static-friction/stick-slip model. It is a smooth approximation of Coulomb plus viscous friction. The main reason for using $\tanh(\cdot)$ instead of $\mathrm{sign}(\cdot)$ is that the hyperbolic tangent is differentiable everywhere, which makes Lyapunov and backstepping derivations much cleaner.

---

### 1.4 Nonlinear Dynamics

The Newton–Euler equations are

$$J_l\ddot{\theta}_l = \tau_c - T_{f,l}(\dot{\theta}_l) + \tau_{d,l}(t),$$

$$J_m\ddot{\theta}_m = u - \tau_c - T_{f,m}(\dot{\theta}_m) + \tau_{d,m}(t).$$

Here $\tau_{d,l}$ and $\tau_{d,m}$ are external disturbance torques. In acceleration form, define

$$d_l(t) = \frac{\tau_{d,l}(t)}{J_l}, \qquad d_m(t) = \frac{\tau_{d,m}(t)}{J_m}.$$

Then the state-space model is

$$\dot{x}_1 = x_2,$$

$$\dot{x}_2 = \frac{1}{J_l}\left[k(x_3 - x_1) + k_3(x_3 - x_1)^3 + b(x_4 - x_2) - T_{f,l}(x_2)\right] + d_l(t),$$

$$\dot{x}_3 = x_4,$$

$$\dot{x}_4 = \frac{1}{J_m}\left[u - k(x_3 - x_1) - k_3(x_3 - x_1)^3 - b(x_4 - x_2) - T_{f,m}(x_4)\right] + d_m(t).$$

Equivalently,

$$\dot{x} = f(x) + g\,u + d(t),$$

where

$$
g = \begin{bmatrix}0 \\ 0 \\ 0 \\ 1/J_m\end{bmatrix}.
$$

Therefore, the system is nonlinear in the state variables but affine in the input. The input appears linearly in the motor acceleration channel.

---

## 2. Physical Derivation of the Model

### 2.1 Load-Side Dynamics

The load inertia receives torque from the flexible coupling. Its rotational equation is

$$J_l\ddot{\theta}_l = \tau_c - T_{f,l}(\dot{\theta}_l) + \tau_{d,l}(t).$$

The load is not directly actuated. This is important because the controlled output $\theta_l$ cannot be shaped directly by the motor torque. Instead, the motor torque changes $\theta_m$, which changes the shaft twist $\delta = \theta_m - \theta_l$, which then changes the transmitted torque $\tau_c$.

---

### 2.2 Motor-Side Dynamics

The motor inertia is driven by the electromagnetic torque $u$, but it is opposed by the flexible coupling torque and motor-side friction:

$$J_m\ddot{\theta}_m = u - \tau_c - T_{f,m}(\dot{\theta}_m) + \tau_{d,m}(t).$$

Thus, the motor is the actuated subsystem, while the load is the controlled subsystem. This mismatch between actuation and output location creates the main control challenge.

---

### 2.3 Nonlinear Flexible Coupling

The coupling torque is modeled as

$$\tau_c = k\delta + k_3\delta^3 + b\dot{\delta}.$$

The first term $k\delta$ is the classical linear torsional spring torque. The cubic term $k_3\delta^3$ captures amplitude-dependent stiffness. For $k_3 > 0$, the shaft becomes effectively stiffer as the twist magnitude increases. The damping term $b\dot{\delta}$ dissipates relative motion between the motor and load.

The corresponding elastic potential energy is

$$U(\delta) = \frac{1}{2}k\delta^2 + \frac{1}{4}k_3\delta^4.$$

Indeed,

$$\frac{\partial U}{\partial \delta} = k\delta + k_3\delta^3.$$

Thus, the spring part of the coupling torque is the gradient of the potential energy.

---

## 3. Mathematical Properties of the Friction Model

### 3.1 Smooth Approximation of Coulomb Friction

Classical Coulomb-viscous friction is often written as

$$T_f(\omega) = F_c\mathrm{sign}(\omega) + B_v\,\omega.$$

The difficulty is that $\mathrm{sign}(\omega)$ is discontinuous at $\omega = 0$. This can make the closed-loop vector field discontinuous and may require nonsmooth analysis, for example Filippov solutions.

Instead, we use

$$T_f(\omega) = F_c\tanh\left(\frac{\omega}{v_s}\right) + B_v\,\omega.$$

Since

$$\tanh(z) \to 1 \quad \text{as} \quad z \to +\infty,$$

and

$$\tanh(z) \to -1 \quad \text{as} \quad z \to -\infty,$$

$\tanh(\omega/v_s)$ behaves like a smooth version of $\mathrm{sign}(\omega)$.

---

### 3.2 Proof of Smoothness

The function $\tanh(z)$ is analytic for all real $z$. Therefore, $\tanh(\omega/v_s)$ is smooth for every $v_s > 0$. Its derivative is

$$\frac{d}{d\omega}\tanh\left(\frac{\omega}{v_s}\right) = \frac{1}{v_s}\mathrm{sech}^2\left(\frac{\omega}{v_s}\right).$$

Therefore,

$$\frac{dT_f}{d\omega} = \frac{F_c}{v_s}\mathrm{sech}^2\left(\frac{\omega}{v_s}\right) + B_v.$$

Since $\mathrm{sech}^2(z)$ is continuous and bounded, $T_f(\omega)$ is continuously differentiable. In fact, because $\tanh(z)$ is analytic, $T_f \in C^\infty$.

This is exactly why the model is convenient for backstepping: all derivatives required in the recursive Lyapunov design exist and are continuous.

---

### 3.3 Dissipativity of Smooth Friction

Assume

$$F_c \ge 0, \qquad B_v \ge 0, \qquad v_s > 0.$$

Then for every $\omega$,

$$\omega\,T_f(\omega) = F_c\,\omega\tanh\left(\frac{\omega}{v_s}\right) + B_v\,\omega^2.$$

The function $\tanh(z)$ has the same sign as $z$. Therefore, $\omega$ and $\tanh(\omega/v_s)$ have the same sign. Hence,

$$\omega\tanh\left(\frac{\omega}{v_s}\right) \ge 0.$$

Also, $B_v\omega^2 \ge 0$. Thus,

$$\omega\,T_f(\omega) \ge 0.$$

This means that the friction torque always dissipates mechanical energy. It never injects energy into the system.

---

### 3.4 Near-Zero Velocity Behavior

For small $z$,

$$\tanh(z) = z + O(z^3).$$

Therefore, near $\omega = 0$,

$$T_f(\omega) \approx \left(\frac{F_c}{v_s} + B_v\right)\omega.$$

So near zero velocity, the friction behaves like an additional viscous damping term. The effective small-signal damping coefficient is

$$B_{\text{eff}} = \frac{F_c}{v_s} + B_v.$$

This is useful for smooth control design, but it also means that the model does not represent true static locking or discontinuous breakaway friction.

---

## 4. Energy and Passivity Analysis

### 4.1 Total Mechanical Energy

For the nominal system without external disturbances, define the total mechanical energy as

$$E(x) = \frac{1}{2}J_l\omega_l^2 + \frac{1}{2}J_m\omega_m^2 + \frac{1}{2}k\delta^2 + \frac{1}{4}k_3\delta^4.$$

In state variables:

$$E(x) = \frac{1}{2}J_l x_2^2 + \frac{1}{2}J_m x_4^2 + \frac{1}{2}k(x_3 - x_1)^2 + \frac{1}{4}k_3(x_3 - x_1)^4.$$

If $J_l > 0$, $J_m > 0$, $k > 0$, $k_3 \ge 0$, then $E(x)$ is positive definite with respect to $(\omega_l, \omega_m, \delta)$. It is radially unbounded in these variables.

---

### 4.2 Derivative of the Energy

Separate the spring torque and damping torque:

$$\tau_s = k\delta + k_3\delta^3, \qquad \tau_d = b\nu = b(\omega_m - \omega_l),$$

so that $\tau_c = \tau_s + \tau_d$. The derivative of the energy is

$$\dot{E} = J_l\omega_l\dot{\omega}_l + J_m\omega_m\dot{\omega}_m + \tau_s\dot{\delta}.$$

Using

$$J_l\dot{\omega}_l = \tau_c - T_{f,l}(\omega_l), \qquad J_m\dot{\omega}_m = u - \tau_c - T_{f,m}(\omega_m), \qquad \dot{\delta} = \omega_m - \omega_l,$$

and collecting terms (the spring terms cancel, the damping terms combine), we obtain

$$\dot{E} = \omega_m u - b(\omega_m - \omega_l)^2 - \omega_l T_{f,l}(\omega_l) - \omega_m T_{f,m}(\omega_m).$$

Using the dissipativity of friction, $\omega_l T_{f,l}(\omega_l) \ge 0$ and $\omega_m T_{f,m}(\omega_m) \ge 0$, hence

$$\dot{E} \le \omega_m u.$$

This shows that the nominal flexible-joint system is **passive** from the input torque $u$ to the motor angular velocity $\omega_m$. If $u = 0$, then

$$\dot{E} = -b(\omega_m - \omega_l)^2 - \omega_l T_{f,l}(\omega_l) - \omega_m T_{f,m}(\omega_m) \le 0.$$

Therefore, without actuation, the system cannot generate mechanical energy by itself.

---

## 5. Existence, Uniqueness, and Model Regularity

### 5.1 Local Lipschitz Property

The right-hand side of the state-space model consists of polynomial functions of the states and smooth friction terms based on $\tanh(\cdot)$. Therefore, the vector field $f(x) + gu$ is locally Lipschitz in $x$. As a result, for every initial condition $x(0) = x_0$ and every piecewise continuous bounded input $u(t)$, the system has a unique local solution.

A careful point is important here: the complete vector field is not globally Lipschitz when $k_3 \neq 0$, because the derivative of the cubic term grows without bound as $|\delta| \to \infty$. The $\tanh$-friction part is globally Lipschitz, but the cubic stiffness makes the full nonlinear vector field only locally Lipschitz.

---

### 5.2 Forward Completeness under Physical Assumptions

Even though the vector field is not globally Lipschitz, the energy function gives a physical argument against finite escape under normal assumptions.

If $k > 0$, $k_3 \ge 0$, $b > 0$, and the input torque $u(t)$ is bounded, then using Young's inequality on the bound $\dot{E} \le \omega_m u$, there exist constants $c_1 > 0$ and $c_2 > 0$ such that

$$\dot{E} \le c_1 E + c_2.$$

By the comparison lemma, $E(t)$ cannot blow up in finite time if $E(0)$ is finite and $u(t)$ is bounded. Since $E$ is radially unbounded in $(\omega_l, \omega_m, \delta)$, these variables remain finite over finite time intervals.

---

## 6. Linearization and Why Linear Control Is Only Local

### 6.1 Small-Signal Linearization

Around the equilibrium $\theta_l = \theta_m = \theta_0$, $\omega_l = \omega_m = 0$, $u = 0$, define

$$\beta_l = \frac{F_{c,l}}{v_{s,l}} + B_{v,l}, \qquad \beta_m = \frac{F_{c,m}}{v_{s,m}} + B_{v,m}.$$

The small-signal linearized model has the form $\dot{x} = Ax + Bu$, where

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
-\dfrac{k}{J_l} & -\dfrac{b+\beta_l}{J_l} & \dfrac{k}{J_l} & \dfrac{b}{J_l} \\
0 & 0 & 0 & 1 \\
\dfrac{k}{J_m} & \dfrac{b}{J_m} & -\dfrac{k}{J_m} & -\dfrac{b+\beta_m}{J_m}
\end{bmatrix},
\qquad
B = \begin{bmatrix}
0 \\
0 \\
0 \\
\dfrac{1}{J_m}
\end{bmatrix}.
$$

This linear model is valid only near the chosen equilibrium.

---

### 6.2 Why LQR, PID, and Pole Placement Are Limited

Linear controllers can be useful near a nominal operating point. However, the flexible-joint system has nonlinear phenomena that are not captured by a single linear model:

- the cubic stiffness term changes the effective stiffness as the shaft twist grows;
- the resonance frequency becomes amplitude-dependent;
- friction changes character between near-zero velocity and higher velocity;
- the load is not directly actuated;
- disturbances and payload variations enter through acceleration channels.

The purpose of nonlinear control is not to claim that linear control is always bad. Rather, the goal is to exploit the known nonlinear model instead of treating nonlinearities as small perturbations.

---

## 7. Backstepping-Oriented Control Problem

### 7.1 Control Objective

Let the desired load trajectory be $\theta_d(t)$. The tracking objective is

$$e(t) = \theta_l(t) - \theta_d(t) \to 0.$$

At the same time, the controller should suppress torsional oscillations in the flexible coupling and keep the motor-side states bounded.

---

### 7.2 Important Structural Point

The system is cascade-like and suitable for backstepping-inspired design, but it is not in the simplest textbook strict-feedback form in the original coordinates. The load acceleration depends on the coupling torque, which is a combination of motor position and motor velocity.

A clean way to handle this is to use the coupling torque $\tau_c$ as an intermediate virtual control. This transforms the design into a physically meaningful backstepping-oriented torque-shaping procedure.

---

## 8. Nominal Backstepping-Oriented Design Using Coupling Torque

This section gives a nominal derivation without external disturbances. The nominal equations are

$$\dot{x}_1 = x_2, \qquad \dot{x}_2 = \frac{1}{J_l}\left(\tau_c - T_{f,l}(x_2)\right),$$

$$\dot{x}_3 = x_4, \qquad \dot{x}_4 = \frac{1}{J_m}\left(u - \tau_c - T_{f,m}(x_4)\right).$$

---

### 8.1 Step 1: Load Position Error

Define the first tracking error $z_1 = x_1 - \theta_d$. Choose the desired load velocity as the first virtual control:

$$\alpha_1 = \dot{\theta}_d - c_1 z_1, \quad c_1 > 0.$$

Define $z_2 = x_2 - \alpha_1$. Then

$$\dot{z}_1 = -c_1 z_1 + z_2.$$

With $V_1 = \tfrac{1}{2}z_1^2$, we get $\dot{V}_1 = -c_1 z_1^2 + z_1 z_2$.

---

### 8.2 Step 2: Desired Coupling Torque

If $\tau_c$ could be directly assigned, the stabilizing desired coupling torque is

$$\tau_c^* = J_l\left(\dot{\alpha}_1 - c_2 z_2 - z_1\right) + T_{f,l}(x_2), \quad c_2 > 0.$$

Define the coupling torque error $z_3 = \tau_c - \tau_c^*$. Then

$$\dot{z}_2 = -c_2 z_2 - z_1 + \frac{1}{J_l}z_3.$$

With $V_2 = \tfrac{1}{2}z_1^2 + \tfrac{1}{2}z_2^2$:

$$\dot{V}_2 = -c_1 z_1^2 - c_2 z_2^2 + \frac{1}{J_l}z_2 z_3.$$

---

### 8.3 Step 3: Dynamics of the Coupling Torque

Differentiating $\tau_c = k\delta + k_3\delta^3 + b\nu$ and substituting the system dynamics gives

$$\dot{\tau}_c = \frac{b}{J_m}u + \Phi(x),$$

where

$$\Phi(x) = (k + 3k_3\delta^2)\nu - b\left(\frac{1}{J_m} + \frac{1}{J_l}\right)\tau_c - \frac{b}{J_m}T_{f,m}(x_4) + \frac{b}{J_l}T_{f,l}(x_2).$$

Since $b > 0$, the motor torque $u$ appears directly in $\dot{\tau}_c$.

---

### 8.4 Final Torque Control Law

Choose

$$u = \frac{J_m}{b}\left[-\Phi(x) + \dot{\tau}_c^* - c_3 z_3 - \frac{1}{J_l}z_2\right], \quad c_3 > 0.$$

Then $\dot{z}_3 = -c_3 z_3 - \tfrac{1}{J_l}z_2$.

---

### 8.5 Closed-Loop Error Dynamics

$$\dot{z}_1 = -c_1 z_1 + z_2,$$

$$\dot{z}_2 = -c_2 z_2 - z_1 + \frac{1}{J_l}z_3,$$

$$\dot{z}_3 = -c_3 z_3 - \frac{1}{J_l}z_2.$$

---

## 9. Lyapunov Proof of Nominal Stability

Define the Lyapunov function

$$V(z) = \frac{1}{2}z_1^2 + \frac{1}{2}z_2^2 + \frac{1}{2}z_3^2.$$

Differentiating and substituting the closed-loop error dynamics:

$$\dot{V} = -c_1 z_1^2 + z_1 z_2 - c_2 z_2^2 - z_1 z_2 + \frac{1}{J_l}z_2 z_3 - c_3 z_3^2 - \frac{1}{J_l}z_2 z_3.$$

The cross-terms cancel exactly, leaving

$$\dot{V} = -c_1 z_1^2 - c_2 z_2^2 - c_3 z_3^2 < 0 \quad \text{for all } z \neq 0.$$

Hence $z_1(t) \to 0$, $z_2(t) \to 0$, $z_3(t) \to 0$, and $\theta_l(t) \to \theta_d(t)$.

---

## 10. Internal Stability of the Flexible Coupling

The coupling relation can be written as a first-order ODE for $\delta$:

$$b\dot{\delta} + k\delta + k_3\delta^3 = \tau_c.$$

Using $W(\delta) = \tfrac{1}{2}\delta^2$ as a storage function and Young's inequality,

$$\dot{W} \le -\frac{k}{2b}\delta^2 - \frac{k_3}{b}\delta^4 + \frac{1}{2kb}\tau_c^2.$$

This shows that the shaft twist dynamics are **input-to-state stable** with respect to $\tau_c$. Bounded $\tau_c$ implies bounded $\delta$ and $\nu$, completing the physical stability picture.

---

## 11. Disturbance Effects and Robustness

With acceleration disturbances $d_l(t)$ and $d_m(t)$, the error dynamics become

$$\dot{z}_1 = -c_1 z_1 + z_2,$$

$$\dot{z}_2 = -c_2 z_2 - z_1 + \frac{1}{J_l}z_3 + d_l(t),$$

$$\dot{z}_3 = -c_3 z_3 - \frac{1}{J_l}z_2 + b\left(d_m(t) - d_l(t)\right).$$

If $d_l$ and $d_m$ are bounded, the disturbance terms can be bounded by Young's inequality, and the tracking error system is **input-to-state stable** with respect to the disturbances. Larger gains $c_1, c_2, c_3$ reduce the ultimate error bound.

For exact disturbance rejection, one may add adaptive estimation, robust damping terms, disturbance observers, or sliding-mode correction terms.

---

## 12. Role of Friction Compensation in the Control Law

The control law contains $\Phi(x)$, which includes motor-side and load-side friction. Since the controller uses $-\Phi(x)$, it compensates the known nonlinear dynamics including friction. The smooth $\tanh$-based friction model is especially convenient because its derivative exists everywhere.

> A hyperbolic tangent approximation is used instead of the discontinuous sign function to preserve differentiability of the closed-loop vector field and enable Lyapunov-based recursive nonlinear control synthesis.

---

## 13. Practical Actuator Realization Through Motor Current

### 13.1 Torque Generation by Motor Current

For a DC motor or a drive with a fast inner current-control loop, the electromagnetic torque is approximately proportional to the motor current:

$$\tau_m = K_t\,i,$$

| Symbol | Meaning | Units |
|:---|:---|:---|
| $i$ | Motor current | A |
| $K_t$ | Motor torque constant | N·m/A |
| $\tau_m$ | Electromagnetic motor torque | N·m |

---

### 13.2 Current-Controlled Flexible-Joint Model

With $u = i$ as the control input, the motor-side dynamics become

$$J_m\dot{\omega}_m = K_t\,i - \tau_c - T_{f,m}(\omega_m),$$

and the fourth state equation is

$$\dot{x}_4 = \frac{K_t}{J_m}u + f_4(x).$$

The system remains input-affine with a direct physical interpretation.

---

### 13.3 Relation to the Torque-Based Controller

The desired torque $\tau_m^*$ from the backstepping design is converted to a current command:

$$i^* = \frac{\tau_m^*}{K_t}.$$

If the motor drive tracks the commanded current sufficiently fast, then $K_t\,i \approx \tau_m^*$ and the current-controlled implementation reproduces the same mechanical torque assumed in the theoretical model.

---

### 13.4 Physical Control Chain

The physical control chain becomes

$$i^* \;\to\; \tau_m \;\to\; \omega_m \;\to\; \theta_m \;\to\; \tau_c \;\to\; \theta_l.$$

---

### 13.5 Lyapunov Consistency

With the choice $i = i^* = \tau_m^*/K_t$, we have $\tau_m = K_t({\tau_m^*}/{K_t}) = \tau_m^*$. Therefore, the mechanical closed-loop error dynamics remain unchanged and the same Lyapunov proof applies:

$$\dot{V} = -c_1 z_1^2 - c_2 z_2^2 - c_3 z_3^2 < 0 \quad \text{for } z \neq 0.$$

---

### 13.6 Stability Proof for the Current-Controlled Model

#### Assumptions

$$J_l > 0,\quad J_m > 0,\quad K_t > 0,\quad k > 0,\quad k_3 \ge 0,\quad b > 0.$$

#### Backstepping Steps

**Step 1.** Define $z_1 = x_1 - \theta_d$, $\alpha_1 = \dot{\theta}_d - c_1 z_1$, $z_2 = x_2 - \alpha_1$:

$$\dot{z}_1 = -c_1 z_1 + z_2.$$

**Step 2.** Define $\tau_c^* = J_l(\dot{\alpha}_1 - c_2 z_2 - z_1) + T_{f,l}(x_2)$, $z_3 = \tau_c - \tau_c^*$:

$$\dot{z}_2 = -c_2 z_2 - z_1 + \frac{1}{J_l}z_3.$$

**Step 3.** Differentiate $\tau_c$ using the current-controlled dynamics:

$$\dot{\tau}_c = \frac{bK_t}{J_m}i + \Phi(x),$$

where $\Phi(x)$ is defined as before. Choose the current control law:

$$i = \frac{J_m}{bK_t}\left[-\Phi(x) + \dot{\tau}_c^* - c_3 z_3 - \frac{1}{J_l}z_2\right].$$

Then $\dot{z}_3 = -c_3 z_3 - \tfrac{1}{J_l}z_2$.

#### Lyapunov Analysis

With $V(z) = \tfrac{1}{2}(z_1^2 + z_2^2 + z_3^2)$, all cross-terms cancel and

$$\dot{V} = -c_1 z_1^2 - c_2 z_2^2 - c_3 z_3^2.$$

Setting $c_{\min} = \min(c_1, c_2, c_3)$ gives $\dot{V} \le -2c_{\min}V$, so by the comparison lemma

$$\|z(t)\| \le \|z(0)\|\,e^{-c_{\min}t}.$$

The tracking error dynamics are **exponentially stable** in the transformed coordinates.

#### Stability Statement

Under the stated assumptions and for $c_1, c_2, c_3 > 0$, the current control law

$$\boxed{i = \frac{J_m}{bK_t}\left[-\Phi(x) + \dot{\tau}_c^* - c_3 z_3 - \frac{1}{J_l}z_2\right]}$$

yields exponentially stable tracking error dynamics, and the load position $\theta_l$ asymptotically tracks the desired reference $\theta_d(t)$ in the nominal model.

---

### 13.7 Practical Meaning

The current-controlled model is more realistic than direct torque control because current is a measurable and controllable electrical quantity. The main practical limitation is current saturation:

$$|i| \le i_{\max}, \qquad |\tau_m| \le K_t\,i_{\max}.$$

If the required current exceeds this limit, exact Lyapunov cancellation may no longer hold, and the stability result becomes local or semi-global.

---

### 13.8 Final Interpretation

The torque-input model is useful for deriving the controller ($u = \tau_m$). The current-input model is used for implementation ($u = i$, $\tau_m = K_t i$). The final current command is

$$i^* = \frac{\tau_m^*}{K_t}.$$

---

## 14. Implementation Notes

### 14.1 Required Signals

The nominal controller requires:

- Measured or estimated states: $\theta_l,\; \omega_l,\; \theta_m,\; \omega_m$.
- Reference trajectory and derivatives: $\theta_d,\; \dot{\theta}_d,\; \ddot{\theta}_d$ (and possibly higher-order derivatives for exact computation of $\dot{\tau}_c^*$).

In practice, exact differentiation is usually avoided. Alternatives include command filters, dynamic surface control, numerical differentiation with filtering, or observer-based derivative estimation.

### 14.2 Torque Saturation

Real motors have torque limits $|u| \le u_{\max}$. A practical saturated input is $u_{\text{sat}} = \mathrm{sat}(u,\, u_{\max})$. The Lyapunov proof then becomes local or semi-global rather than global.

### 14.3 Parameter Uncertainty

The nominal controller assumes known parameters: $J_l,\, J_m,\, k,\, k_3,\, b,\, F_c,\, v_s,\, B_v$. For a more complete design, adaptive laws can be introduced for uncertain parameters, especially stiffness $k$, nonlinear stiffness $k_3$, damping $b$, friction parameters $F_c$ and $B_v$, and load inertia $J_l$.

---

## 15. Final Theoretical Summary

The proposed flexible-joint model is a nonlinear, input-affine, non-collocated mechanical system. Its main nonlinearities are the cubic torsional stiffness and smooth Coulomb-viscous friction. The use of $\tanh(\cdot)$ instead of $\mathrm{sign}(\cdot)$ preserves differentiability and avoids discontinuous vector fields.

The system has a natural energy structure. The nominal plant is passive from motor torque to motor velocity.

The backstepping-oriented design follows a clean recursive structure:

1. Stabilize load position error using desired load velocity.
2. Stabilize load velocity error using desired coupling torque.
3. Force the real coupling torque to track the desired coupling torque using motor torque.

With the proposed nominal control law, the closed-loop tracking error dynamics satisfy

$$\dot{V} = -c_1 z_1^2 - c_2 z_2^2 - c_3 z_3^2,$$

which proves asymptotic (and exponential in the transformed coordinates) convergence of the tracking errors. Internal shaft dynamics are stable because $b\dot{\delta} + k\delta + k_3\delta^3 = \tau_c$ is input-to-state stable for $b > 0$, $k > 0$, $k_3 \ge 0$.

Therefore, the model and the proposed nonlinear control framework provide a physically consistent and mathematically justified basis for simulation, analysis, and further adaptive or robust controller development.