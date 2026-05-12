# Nonlinear Flexible-Joint Drive with Smooth Friction and Backstepping-Oriented Control

*Theory, model derivation, Lyapunov analysis, and nominal nonlinear control synthesis for a two-inertia flexible-joint servo system.*

---

## Project Purpose

This document describes a nonlinear flexible-joint rotational drive consisting of a motor inertia connected to a load inertia through a compliant shaft. The main objective is to formulate a physically meaningful nonlinear state-space model and prepare it for Lyapunov-based nonlinear control synthesis, especially backstepping-oriented design.

The model includes three important effects:

1. nonlinear torsional stiffness of the flexible shaft;
2. structural damping in the coupling;
3. smooth Coulomb-viscous friction modeled with a hyperbolic tangent approximation.

The key controlled output is the load angular position \(\theta_l\), while the control input is the motor electromagnetic torque \(u=\tau_m\). This makes the system non-collocated: the input acts on the motor side, but the desired output is located on the load side.

---

## 1. State-Space Model

### 1.1 State Vector

The state vector is chosen as

$$x =\begin{bmatrix}\theta_l \\\omega_l \\\theta_m \\\omega_m\end{bmatrix}=\begin{bmatrix}
x_1 \\
x_2 \\
x_3 \\
x_4
\end{bmatrix}
\in \mathbb{R}^4.
$$

| State | Meaning | Units |
|:---|:---|:---|
| \(x_1=\theta_l\) | load angular position | rad |
| \(x_2=\omega_l\) | load angular velocity | rad/s |
| \(x_3=\theta_m\) | motor angular position | rad |
| \(x_4=\omega_m\) | motor angular velocity | rad/s |

The control input is

$$
u = \tau_m,
$$

where \(\tau_m\) is the motor electromagnetic torque.

---

### 1.2 Relative Variables

For compact notation, define the relative shaft twist and relative angular velocity:

$$
\delta = \theta_m - \theta_l = x_3-x_1,
$$

$$
\nu = \omega_m - \omega_l = x_4-x_2.
$$

The flexible coupling torque is

$$
\tau_c = k\delta + k_3\delta^3 + b\nu.
$$

Here:

| Symbol | Meaning | Units |
|:---|:---|:---|
| \(k\) | linear torsional stiffness | N·m/rad |
| \(k_3\) | cubic stiffness coefficient | N·m/rad\(^3\) |
| \(b\) | coupling damping coefficient | N·m·s/rad |
| \(\delta\) | shaft twist | rad |
| \(\nu\) | shaft relative velocity | rad/s |
| \(\tau_c\) | transmitted coupling torque | N·m |

For the usual hardening case,

$$
k>0, \qquad k_3 \ge 0, \qquad b>0.
$$

If \(k_3>0\), the shaft stiffness increases with deflection. If \(k_3<0\), the model describes softening behavior, but then the elastic potential may lose global positive definiteness. For Lyapunov-based global or large-domain arguments, the hardening case \(k_3\ge 0\) is preferable.

---

### 1.3 Smooth Friction Model

The load-side and motor-side friction torques are modeled as

$$
T_{f,l}(\omega_l)=F_{c,l}\tanh\left(\frac{\omega_l}{v_{s,l}}\right)+B_{v,l}\omega_l,
$$

$$
T_{f,m}(\omega_m)=F_{c,m}\tanh\left(\frac{\omega_m}{v_{s,m}}\right)+B_{v,m}\omega_m.
$$

For a generic angular velocity \(\omega\), the friction model is

$$
T_f(\omega)=F_c\tanh\left(\frac{\omega}{v_s}\right)+B_v\omega.
$$

The parameters are:

| Symbol | Meaning | Units |
|:---|:---|:---|
| \(F_c\) | Coulomb friction level | N·m |
| \(v_s\) | smoothing velocity threshold | rad/s |
| \(B_v\) | viscous friction coefficient | N·m·s/rad |

This is not a full static-friction/stick-slip model. It is a smooth approximation of Coulomb plus viscous friction. The main reason for using \(\tanh(\cdot)\) instead of \(\operatorname{sign}(\cdot)\) is that the hyperbolic tangent is differentiable everywhere, which makes Lyapunov and backstepping derivations much cleaner.

---

### 1.4 Nonlinear Dynamics

The Newton-Euler equations are

$$
J_l\ddot{\theta}_l = \tau_c - T_{f,l}(\dot{\theta}_l) + \tau_{d,l}(t),
$$

$$
J_m\ddot{\theta}_m = u - \tau_c - T_{f,m}(\dot{\theta}_m) + \tau_{d,m}(t).
$$

Here \(\tau_{d,l}\) and \(\tau_{d,m}\) are external disturbance torques. In acceleration form, define

$$
d_l(t)=\frac{\tau_{d,l}(t)}{J_l},
\qquad
 d_m(t)=\frac{\tau_{d,m}(t)}{J_m}.
$$

Then the state-space model is

$$
\dot{x}_1=x_2,
$$

$$
\dot{x}_2=
\frac{1}{J_l}\left[k(x_3-x_1)+k_3(x_3-x_1)^3+b(x_4-x_2)-T_{f,l}(x_2)\right]+d_l(t),
$$

$$
\dot{x}_3=x_4,
$$

$$
\dot{x}_4=
\frac{1}{J_m}\left[u-k(x_3-x_1)-k_3(x_3-x_1)^3-b(x_4-x_2)-T_{f,m}(x_4)\right]+d_m(t).
$$

Equivalently,

$$
\dot{x}=f(x)+g u+d(t),
$$

where

$$
g=\begin{bmatrix}0\\0\\0\\1/J_m\end{bmatrix}.
$$

Therefore, the system is nonlinear in the state variables but affine in the input. The input appears linearly in the motor acceleration channel.

---

## 2. Physical Derivation of the Model

### 2.1 Load-Side Dynamics

The load inertia receives torque from the flexible coupling. Its rotational equation is

$$
J_l\ddot{\theta}_l = \tau_c - T_{f,l}(\dot{\theta}_l)+\tau_{d,l}(t).
$$

The load is not directly actuated. This is important because the controlled output \(\theta_l\) cannot be shaped directly by the motor torque. Instead, the motor torque changes \(\theta_m\), which changes the shaft twist \(\delta=\theta_m-\theta_l\), which then changes the transmitted torque \(\tau_c\).

---

### 2.2 Motor-Side Dynamics

The motor inertia is driven by the electromagnetic torque \(u\), but it is opposed by the flexible coupling torque and motor-side friction:

$$
J_m\ddot{\theta}_m = u-\tau_c-T_{f,m}(\dot{\theta}_m)+\tau_{d,m}(t).
$$

Thus, the motor is the actuated subsystem, while the load is the controlled subsystem. This mismatch between actuation and output location creates the main control challenge.

---

### 2.3 Nonlinear Flexible Coupling

The coupling torque is modeled as

$$
\tau_c = k\delta+k_3\delta^3+b\dot{\delta}.
$$

The first term \(k\delta\) is the classical linear torsional spring torque. The cubic term \(k_3\delta^3\) captures amplitude-dependent stiffness. For \(k_3>0\), the shaft becomes effectively stiffer as the twist magnitude increases. The damping term \(b\dot{\delta}\) dissipates relative motion between the motor and load.

The corresponding elastic potential energy is

$$
U(\delta)=\frac{1}{2}k\delta^2+\frac{1}{4}k_3\delta^4.
$$

Indeed,

$$
\frac{\partial U}{\partial \delta}=k\delta+k_3\delta^3.
$$

Thus, the spring part of the coupling torque is the gradient of the potential energy.

---

## 3. Mathematical Properties of the Friction Model

### 3.1 Smooth Approximation of Coulomb Friction

Classical Coulomb-viscous friction is often written as

$$
T_f(\omega)=F_c\operatorname{sign}(\omega)+B_v\omega.
$$

The difficulty is that \(\operatorname{sign}(\omega)\) is discontinuous at \(\omega=0\). This can make the closed-loop vector field discontinuous and may require nonsmooth analysis, for example Filippov solutions.

Instead, we use

$$
T_f(\omega)=F_c\tanh\left(\frac{\omega}{v_s}\right)+B_v\omega.
$$

Since

$$
\tanh(z)\to 1 \quad \text{as} \quad z\to +\infty,
$$

and

$$
\tanh(z)\to -1 \quad \text{as} \quad z\to -\infty,
$$

\(\tanh(\omega/v_s)\) behaves like a smooth version of \(\operatorname{sign}(\omega)\).

---

### 3.2 Proof of Smoothness

The function \(\tanh(z)\) is analytic for all real \(z\). Therefore,

$$
\tanh\left(\frac{\omega}{v_s}\right)
$$

is smooth for every \(v_s>0\). Its derivative is

$$\frac{d}{d\omega}\tanh\left(\frac{\omega}{v_s}\right)=\frac{1}{v_s}\operatorname{sech}^2\left(\frac{\omega}{v_s}\right).
$$

Therefore,

$$\frac{dT_f}{d\omega}=\frac{F_c}{v_s}\operatorname{sech}^2\left(\frac{\omega}{v_s}\right)+B_v.
$$

Since \(\operatorname{sech}^2(z)\) is continuous and bounded, \(T_f(\omega)\) is continuously differentiable. In fact, because \(\tanh(z)\) is analytic, \(T_f\in C^\infty\).

This is exactly why the model is convenient for backstepping: all derivatives required in the recursive Lyapunov design exist and are continuous.

---

### 3.3 Dissipativity of Smooth Friction

Assume

$$
F_c\ge 0, \qquad B_v\ge 0, \qquad v_s>0.
$$

Then for every \(\omega\),

$$\omega T_f(\omega)=F_c\omega\tanh\left(\frac{\omega}{v_s}\right)+B_v\omega^2.
$$

The function \(\tanh(z)\) has the same sign as \(z\). Therefore, \(\omega\) and \(\tanh(\omega/v_s)\) have the same sign. Hence,

$$
\omega\tanh\left(\frac{\omega}{v_s}\right)\ge 0.
$$

Also,

$$
B_v\omega^2\ge 0.
$$

Thus,

$$
\omega T_f(\omega)\ge 0.
$$

This means that the friction torque always dissipates mechanical energy. It never injects energy into the system.

---

### 3.4 Near-Zero Velocity Behavior

For small \(z\),

$$
\tanh(z)=z+O(z^3).
$$

Therefore, near \(\omega=0\),

$$
\tanh\left(\frac{\omega}{v_s}\right)\approx \frac{\omega}{v_s}.
$$

Thus,

$$
T_f(\omega)
\approx
\left(\frac{F_c}{v_s}+B_v\right)\omega.
$$

So near zero velocity, the friction behaves like an additional viscous damping term. The effective small-signal damping coefficient is

$$
B_{\text{eff}}=\frac{F_c}{v_s}+B_v.
$$

This is useful for smooth control design, but it also means that the model does not represent true static locking or discontinuous breakaway friction.

---

## 4. Energy and Passivity Analysis

### 4.1 Total Mechanical Energy

For the nominal system without external disturbances, define the total mechanical energy as

$$
E(x)=
\frac{1}{2}J_l\omega_l^2
+
\frac{1}{2}J_m\omega_m^2
+
\frac{1}{2}k\delta^2
+
\frac{1}{4}k_3\delta^4.
$$

In state variables:

$$
E(x)=
\frac{1}{2}J_lx_2^2
+
\frac{1}{2}J_mx_4^2
+
\frac{1}{2}k(x_3-x_1)^2
+
\frac{1}{4}k_3(x_3-x_1)^4.
$$

If

$$
J_l>0,\quad J_m>0,\quad k>0,\quad k_3\ge 0,
$$

then \(E(x)\) is positive definite with respect to \((\omega_l,\omega_m,\delta)\). It is radially unbounded in these variables.

---

### 4.2 Derivative of the Energy

Separate the spring torque and damping torque:

$$
\tau_s=k\delta+k_3\delta^3,
$$

$$
\tau_d=b\nu=b(\omega_m-\omega_l),
$$

so that

$$
\tau_c=\tau_s+\tau_d.
$$

The derivative of the energy is

$$\dot{E}=
J_l\omega_l\dot{\omega}_l
+J_m\omega_m\dot{\omega}_m
+\tau_s\dot{\delta}.
$$

Using

$$
J_l\dot{\omega}_l=\tau_c-T_{f,l}(\omega_l),
$$

$$
J_m\dot{\omega}_m=u-\tau_c-T_{f,m}(\omega_m),
$$

and

$$
\dot{\delta}=\omega_m-\omega_l,
$$

we get

$$\dot{E}=\omega_l(\tau_s+\tau_d-T_{f,l})+\omega_m(u-\tau_s-\tau_d-T_{f,m})+\tau_s(\omega_m-\omega_l).$$

Now collect terms. The spring terms cancel:

$$
\omega_l\tau_s-\omega_m\tau_s+\tau_s(\omega_m-\omega_l)=0.
$$

The damping terms give

$$\omega_l\tau_d-\omega_m\tau_d=\tau_d(\omega_l-\omega_m)=b(\omega_m-\omega_l)(\omega_l-\omega_m)=-b(\omega_m-\omega_l)^2.$$

Therefore,

$$\dot{E}=\omega_m u-b(\omega_m-\omega_l)^2-\omega_lT_{f,l}(\omega_l)-\omega_mT_{f,m}(\omega_m).$$

Using the dissipativity of friction,

$$
\omega_lT_{f,l}(\omega_l)\ge 0,
$$

$$
\omega_mT_{f,m}(\omega_m)\ge 0.
$$

Hence,

$$
\dot{E}\le \omega_m u.
$$

This shows that the nominal flexible-joint system is passive from the input torque \(u\) to the motor angular velocity \(\omega_m\). If \(u=0\), then

$$
\dot{E}
= -b(\omega_m-\omega_l)^2
-\omega_lT_{f,l}(\omega_l)
-\omega_mT_{f,m}(\omega_m)
\le 0.
$$

Therefore, without actuation, the system cannot generate mechanical energy by itself.

---

## 5. Existence, Uniqueness, and Model Regularity

### 5.1 Local Lipschitz Property

The right-hand side of the state-space model consists of polynomial functions of the states and smooth friction terms based on \(\tanh(\cdot)\). Polynomial functions are smooth. The hyperbolic tangent is also smooth.

Therefore, the vector field \(f(x)+gu\) is locally Lipschitz in \(x\). As a result, for every initial condition \(x(0)=x_0\) and every piecewise continuous bounded input \(u(t)\), the system has a unique local solution.

A careful point is important here: the complete vector field is not globally Lipschitz when \(k_3\neq 0\), because the derivative of the cubic term grows without bound as \(|\delta|\to\infty\). The \(\tanh\)-friction part is globally Lipschitz, but the cubic stiffness makes the full nonlinear vector field only locally Lipschitz.

---

### 5.2 Forward Completeness under Physical Assumptions

Even though the vector field is not globally Lipschitz, the energy function gives a physical argument against finite escape under normal assumptions.

If \(k>0\), \(k_3\ge 0\), \(b>0\), and the input torque \(u(t)\) is bounded, then the energy satisfies an inequality of the form

$$
\dot{E}\le \omega_m u.
$$

Using Young's inequality, for any \(\varepsilon>0\),

$$
\omega_m u
\le
\frac{\varepsilon}{2}\omega_m^2
+
\frac{1}{2\varepsilon}u^2.
$$

Since \(\omega_m^2\le \frac{2}{J_m}E\), there exist constants \(c_1>0\) and \(c_2>0\) such that

$$
\dot{E}\le c_1E+c_2.
$$

By the comparison lemma, \(E(t)\) cannot blow up in finite time if \(E(0)\) is finite and \(u(t)\) is bounded. Since \(E\) is radially unbounded in \((\omega_l,\omega_m,\delta)\), these variables remain finite over finite time intervals.

This provides a forward-completeness argument for the physically relevant variables. Absolute angular positions may drift if the system is not position regulated, but this is expected for a rotational system.

---

## 6. Linearization and Why Linear Control Is Only Local

### 6.1 Small-Signal Linearization

Around the equilibrium

$$
\theta_l=\theta_m=\theta_0,
\qquad
\omega_l=\omega_m=0,
\qquad
u=0,
$$

the cubic term has zero first-order contribution if \(\delta=0\). The friction term behaves near zero as

$$
T_f(\omega)\approx \left(\frac{F_c}{v_s}+B_v\right)\omega.
$$

Define

$$
\beta_l=\frac{F_{c,l}}{v_{s,l}}+B_{v,l},
$$

$$
\beta_m=\frac{F_{c,m}}{v_{s,m}}+B_{v,m}.
$$

Then the small-signal linearized model has the form

$$
\dot{x}=Ax+Bu,
$$

where

$$
A=
\begin{bmatrix}
0 & 1 & 0 & 0 \\
-\frac{k}{J_l} & -\frac{b+\beta_l}{J_l} & \frac{k}{J_l} & \frac{b}{J_l} \\
0 & 0 & 0 & 1 \\
\frac{k}{J_m} & \frac{b}{J_m} & -\frac{k}{J_m} & -\frac{b+\beta_m}{J_m}
\end{bmatrix},
$$

and

$$
B=
\begin{bmatrix}
0\\0\\0\\\frac{1}{J_m}
\end{bmatrix}.
$$

This linear model is valid only near the chosen equilibrium. It does not represent large-amplitude shaft twist, amplitude-dependent stiffness, or nonlinear friction saturation.

---

### 6.2 Why LQR, PID, and Pole Placement Are Limited

Linear controllers can be useful near a nominal operating point. However, the flexible-joint system has nonlinear phenomena that are not captured by a single linear model:

- the cubic stiffness term changes the effective stiffness as the shaft twist grows;
- the resonance frequency becomes amplitude-dependent;
- friction changes character between near-zero velocity and higher velocity;
- the load is not directly actuated;
- disturbances and payload variations enter through acceleration channels.

Therefore, a linear controller designed around one operating point may work locally but lose performance during large transients, fast reversals, strong coupling deflections, or parameter drift.

The purpose of nonlinear control is not to claim that linear control is always bad. Rather, the goal is to exploit the known nonlinear model instead of treating nonlinearities as small perturbations.

---

## 7. Backstepping-Oriented Control Problem

### 7.1 Control Objective

Let the desired load trajectory be

$$
\theta_d(t).
$$

The tracking objective is

$$
\theta_l(t)\to \theta_d(t),
$$

or equivalently

$$
e(t)=\theta_l(t)-\theta_d(t)\to 0.
$$

At the same time, the controller should suppress torsional oscillations in the flexible coupling and keep the motor-side states bounded.

---

### 7.2 Important Structural Point

The system is cascade-like and suitable for backstepping-inspired design, but it is not in the simplest textbook strict-feedback form in the original coordinates.

The reason is that

$$
\dot{x}_2
$$

depends on both \(x_3\) and \(x_4\) through

$$
\tau_c=k(x_3-x_1)+k_3(x_3-x_1)^3+b(x_4-x_2).
$$

In a canonical strict-feedback system, one usually expects the next state to appear as a virtual control in a more direct triangular way. Here, the load acceleration depends on the coupling torque, which is a combination of motor position and motor velocity.

A clean way to handle this is to use the coupling torque \(\tau_c\) as an intermediate virtual control. This transforms the design into a physically meaningful backstepping-oriented torque-shaping procedure.

---

## 8. Nominal Backstepping-Oriented Design Using Coupling Torque

This section gives a nominal derivation without external disturbances. Disturbances are discussed later.

The nominal equations are

$$
\dot{x}_1=x_2,
$$

$$
\dot{x}_2=\frac{1}{J_l}\left(\tau_c-T_{f,l}(x_2)\right),
$$

$$
\dot{x}_3=x_4,
$$

$$
\dot{x}_4=\frac{1}{J_m}\left(u-\tau_c-T_{f,m}(x_4)\right).
$$

---

### 8.1 Step 1: Load Position Error

Define the first tracking error:

$$
z_1=x_1-\theta_d.
$$

Then

$$
\dot{z}_1=x_2-\dot{\theta}_d.
$$

Choose the desired load velocity as the first virtual control:

$$
\alpha_1=\dot{\theta}_d-c_1z_1,
$$

where \(c_1>0\). Define the second error:

$$
z_2=x_2-\alpha_1.
$$

Then

$$
\dot{z}_1=-c_1z_1+z_2.
$$

Take the Lyapunov function

$$
V_1=\frac{1}{2}z_1^2.
$$

Its derivative is

$$
\dot{V}_1=z_1\dot{z}_1
=-c_1z_1^2+z_1z_2.
$$

The cross-term \(z_1z_2\) will be cancelled in the next step.

---

### 8.2 Step 2: Desired Coupling Torque

The load velocity error satisfies

$$
\dot{z}_2=\dot{x}_2-\dot{\alpha}_1.
$$

Using the load equation,

$$
\dot{z}_2=
\frac{1}{J_l}\left(\tau_c-T_{f,l}(x_2)\right)-\dot{\alpha}_1.
$$

If \(\tau_c\) could be directly assigned, a stabilizing desired coupling torque would be

$$\tau_c^*=J_l\left(\dot{\alpha}_1-c_2z_2-z_1\right)+T_{f,l}(x_2),$$

where \(c_2>0\).

Define the coupling torque error:

$$
z_3=\tau_c-\tau_c^*.
$$

Substituting

$$
\tau_c=\tau_c^*+z_3,
$$

we obtain

$$\dot{z}_2=-c_2z_2-z_1+\frac{1}{J_l}z_3.$$

Now choose

$$
V_2=\frac{1}{2}z_1^2+\frac{1}{2}z_2^2.
$$

Then

$$
\dot{V}_2
=z_1\dot{z}_1+z_2\dot{z}_2.
$$

Using

$$
\dot{z}_1=-c_1z_1+z_2,
$$

and

$$
\dot{z}_2=-c_2z_2-z_1+\frac{1}{J_l}z_3,
$$

we get

$$
\dot{V}_2
=-c_1z_1^2+z_1z_2
-c_2z_2^2-z_1z_2+\frac{1}{J_l}z_2z_3.
$$

The terms \(z_1z_2\) cancel:

$$
\dot{V}_2
=-c_1z_1^2-c_2z_2^2+\frac{1}{J_l}z_2z_3.
$$

The remaining cross-term \(z_2z_3/J_l\) will be cancelled by the motor torque control law.

---

### 8.3 Step 3: Dynamics of the Coupling Torque

Recall that

$$
\tau_c=k\delta+k_3\delta^3+b\nu,
$$

where

$$
\delta=x_3-x_1,
$$

$$
\nu=x_4-x_2.
$$

Then

$$
\dot{\delta}=\nu.
$$

The derivative of \(\tau_c\) is

$$
\dot{\tau}_c
=(k+3k_3\delta^2)\nu+b\dot{\nu}.
$$

Since

$$
\dot{\nu}=\dot{x}_4-\dot{x}_2,
$$

we use

$$
\dot{x}_4=\frac{1}{J_m}\left(u-\tau_c-T_{f,m}(x_4)\right),
$$

and

$$
\dot{x}_2=\frac{1}{J_l}\left(\tau_c-T_{f,l}(x_2)\right).
$$

Therefore,

$$\dot{\nu}=\frac{1}{J_m}u-\left(\frac{1}{J_m}+\frac{1}{J_l}\right)\tau_c-\frac{1}{J_m}T{f,m}(x_4)+\frac{1}{J_l}T_{f,l}(x_2).$$

Thus,

$$\dot{\tau}_c=\frac{b}{J_m}u+\Phi(x),$$

where

$$
\Phi(x)=
(k+3k_3\delta^2)\nu
-b\left(\frac{1}{J_m}+\frac{1}{J_l}\right)\tau_c
-\frac{b}{J_m}T_{f,m}(x_4)
+\frac{b}{J_l}T_{f,l}(x_2).
$$

This equation is crucial. It shows that, when \(b>0\), the motor torque \(u\) appears directly in the first derivative of the coupling torque. Therefore, \(u\) can be designed to make \(\tau_c\) track \(\tau_c^*\).

---

### 8.4 Final Torque Control Law

The coupling torque error is

$$
z_3=\tau_c-\tau_c^*.
$$

Its derivative is

$$
\dot{z}_3=\dot{\tau}_c-\dot{\tau}_c^*.
$$

Using

$$
\dot{\tau}_c=\frac{b}{J_m}u+\Phi(x),
$$

we obtain

$$
\dot{z}_3=\frac{b}{J_m}u+\Phi(x)-\dot{\tau}_c^*.
$$

Choose the control law

$$u=\frac{J_m}{b}\left[-\Phi(x)+\dot{\tau}_c^*-c_3z_3-\frac{1}{J_l}z_2\right],$$

where \(c_3>0\).

Then

$$
\dot{z}_3=-c_3z_3-\frac{1}{J_l}z_2.
$$

---

### 8.5 Closed-Loop Error Dynamics

The closed-loop error system becomes

$$
\dot{z}_1=-c_1z_1+z_2,
$$

$$
\dot{z}_2=-c_2z_2-z_1+\frac{1}{J_l}z_3,
$$

$$
\dot{z}_3=-c_3z_3-\frac{1}{J_l}z_2.
$$

This is a stable cascade with exact cross-term cancellation.

---

## 9. Lyapunov Proof of Nominal Stability

Define the Lyapunov function

$$
V(z)=\frac{1}{2}z_1^2+\frac{1}{2}z_2^2+\frac{1}{2}z_3^2.
$$

Its derivative is

$$
\dot{V}=z_1\dot{z}_1+z_2\dot{z}_2+z_3\dot{z}_3.
$$

Substitute the closed-loop error dynamics:

$$
\dot{V}
=z_1(-c_1z_1+z_2)
+z_2\left(-c_2z_2-z_1+\frac{1}{J_l}z_3\right)
+z_3\left(-c_3z_3-\frac{1}{J_l}z_2\right).
$$

Expanding terms gives

$$
\dot{V}
=-c_1z_1^2+z_1z_2
-c_2z_2^2-z_1z_2+\frac{1}{J_l}z_2z_3
-c_3z_3^2-\frac{1}{J_l}z_2z_3.
$$

The cross-terms cancel exactly:

$$
z_1z_2-z_1z_2=0,
$$

$$
\frac{1}{J_l}z_2z_3-\frac{1}{J_l}z_2z_3=0.
$$

Therefore,

$$
\dot{V}
=-c_1z_1^2-c_2z_2^2-c_3z_3^2.
$$

Since \(c_1,c_2,c_3>0\), we have

$$
\dot{V}\le 0.
$$

Moreover, \(\dot{V}\) is negative definite in \(z\). Hence,

$$
z_1(t)\to 0,
\qquad
z_2(t)\to 0,
\qquad
z_3(t)\to 0.
$$

Thus,

$$
\theta_l(t)\to \theta_d(t),
$$

and the coupling torque tracks the desired coupling torque:

$$
\tau_c(t)\to \tau_c^*(t).
$$

This proves nominal asymptotic stability of the tracking error system. Since the error dynamics are linear and exponentially stable in \(z\), the convergence is exponential for the nominal transformed error system.

---

## 10. Internal Stability of the Flexible Coupling

The previous proof shows convergence of \(z_1\), \(z_2\), and \(z_3\). However, the physical system has four states, while the error vector \(z\) has three components. Therefore, we must also discuss the internal flexible dynamics.

The coupling relation is

$$
\tau_c=k\delta+k_3\delta^3+b\dot{\delta}.
$$

This can be written as a first-order nonlinear differential equation for \(\delta\):

$$
b\dot{\delta}+k\delta+k_3\delta^3=\tau_c.
$$

Assume

$$
b>0,\qquad k>0,\qquad k_3\ge 0.
$$

Consider the storage function

$$
W(\delta)=\frac{1}{2}\delta^2.
$$

Then

$$
\dot{W}=\delta\dot{\delta}.
$$

From the coupling equation,

$$
\dot{\delta}=\frac{1}{b}\left(\tau_c-k\delta-k_3\delta^3\right).
$$

Therefore,

$$\dot{W}=\frac{1}{b}\delta\tau_c-\frac{k}{b}\delta^2-\frac{k_3}{b}\delta^4.$$

Using Young's inequality,

$$
\frac{1}{b}\delta\tau_c
\le
\frac{k}{2b}\delta^2
+
\frac{1}{2kb}\tau_c^2.
$$

Thus,

$$\dot{W}\le-\frac{k}{2b}\delta^2-\frac{k_3}{b}\delta^4+\frac{1}{2kb}\tau_c^2.$$

This shows that the internal shaft twist dynamics are input-to-state stable with respect to the input \(\tau_c\). Therefore, if \(\tau_c\) is bounded, then \(\delta\) remains bounded. Since

$$
\dot{\delta}=\frac{1}{b}\left(\tau_c-k\delta-k_3\delta^3\right),
$$

bounded \(\tau_c\) and bounded \(\delta\) imply bounded \(\dot{\delta}=\nu\).

Hence, the internal flexible dynamics remain stable under bounded desired coupling torque. This completes the physical stability interpretation of the torque-based backstepping design.

---

## 11. Disturbance Effects and Robustness

Now include acceleration disturbances:

$$
\dot{x}_2=\frac{1}{J_l}\left(\tau_c-T_{f,l}(x_2)\right)+d_l(t),
$$

$$
\dot{x}_4=\frac{1}{J_m}\left(u-\tau_c-T_{f,m}(x_4)\right)+d_m(t).
$$

If the same nominal control law is used without exact disturbance compensation, the error dynamics become approximately

$$
\dot{z}_1=-c_1z_1+z_2,
$$

$$
\dot{z}_2=-c_2z_2-z_1+\frac{1}{J_l}z_3+d_l(t),
$$

$$
\dot{z}_3=-c_3z_3-\frac{1}{J_l}z_2+b\left(d_m(t)-d_l(t)\right).
$$

Then

$$\dot{V}=-c_1z_1^2-c_2z_2^2-c_3z_3^2+z_2d_l+bz_3(d_m-d_l).$$

If \(d_l\) and \(d_m\) are bounded, then the last two terms can be bounded by Young's inequality. Therefore, the tracking error system is input-to-state stable with respect to the disturbances. In practical terms, this means that bounded disturbances lead to bounded tracking errors, and larger controller gains \(c_1,c_2,c_3\) reduce the ultimate error bound.

If exact disturbance rejection is required, one can add:

1. adaptive estimation of uncertain parameters;
2. robust damping terms;
3. disturbance observers;
4. sliding-mode or high-gain correction terms.

However, such additions should be designed carefully to avoid excessive torque, chattering, or noise amplification.

---

## 12. Role of Friction Compensation in the Control Law

The final control law contains the term \(\Phi(x)\), which includes motor-side and load-side friction:

$$
\Phi(x)=
(k+3k_3\delta^2)\nu
-b\left(\frac{1}{J_m}+\frac{1}{J_l}\right)\tau_c
-\frac{b}{J_m}T_{f,m}(x_4)
+\frac{b}{J_l}T_{f,l}(x_2).
$$

Since the controller uses \(-\Phi(x)\), it compensates the known nonlinear dynamics, including friction. The smooth \(\tanh\)-based friction model is especially convenient because its derivative exists everywhere. This matters because \(\dot{\tau}_c^*\) may contain derivatives of friction-related terms if implemented exactly.

A useful explanation is:

> A hyperbolic tangent approximation is used instead of the discontinuous sign function to preserve differentiability of the closed-loop vector field and enable Lyapunov-based recursive nonlinear control synthesis.

This is mathematically stronger and cleaner than using discontinuous Coulomb friction directly.

---

## 13. Implementation Notes

### 13.1 Required Signals

The nominal controller requires the following measured or estimated quantities:

$$
x_1=\theta_l,
\quad
x_2=\omega_l,
\quad
x_3=\theta_m,
\quad
x_4=\omega_m.
$$

It also requires the reference trajectory and its derivatives:

$$
\theta_d,\quad \dot{\theta}_d,\quad \ddot{\theta}_d,
$$

and, for exact computation of \(\dot{\tau}_c^*\), possibly higher-order derivatives depending on implementation.

In practice, exact differentiation is usually avoided. Instead, one may use:

1. command filters;
2. dynamic surface control;
3. numerical differentiation with filtering;
4. observer-based derivative estimation.

---

### 13.2 Torque Saturation

Real motors have torque limits:

$$
|u|\le u_{\max}.
$$

If the theoretical control law demands torque beyond this limit, the actual input becomes saturated. Saturation can break the exact Lyapunov cancellation. Therefore, simulations should include motor torque saturation.

A practical saturated input is

$$
u_{sat}=\operatorname{sat}(u,u_{\max}).
$$

The Lyapunov proof then becomes local or semi-global rather than global, depending on the available torque authority.

---

### 13.3 Parameter Uncertainty

The nominal controller assumes known parameters:

$$
J_l,J_m,k,k_3,b,F_c,v_s,B_v.
$$

If these parameters are uncertain, exact cancellation is no longer exact. The system may still remain stable if the uncertainty is small and the feedback gains are sufficiently large, but tracking accuracy can degrade.

For a more complete design, adaptive laws can be introduced for unknown parameters, especially for:

- stiffness \(k\);
- nonlinear stiffness \(k_3\);
- damping \(b\);
- friction parameters \(F_c\) and \(B_v\);
- load inertia \(J_l\).

---

## 14. Final Theoretical Summary

The proposed flexible-joint model is a nonlinear, input-affine, non-collocated mechanical system. Its main nonlinearities are the cubic torsional stiffness and smooth Coulomb-viscous friction. The use of \(\tanh(\cdot)\) instead of \(\operatorname{sign}(\cdot)\) preserves differentiability and avoids discontinuous vector fields.

The system has a natural energy structure. The elastic energy is stored in the flexible shaft, kinetic energy is stored in the motor and load inertias, while coupling damping and friction dissipate energy. The nominal plant is passive from motor torque to motor velocity.

Although the original state-space model is not a simple textbook strict-feedback system in the original coordinates, it is well suited for backstepping-oriented design if the coupling torque is treated as an intermediate virtual control. This leads to a clean recursive construction:

1. stabilize load position error using desired load velocity;
2. stabilize load velocity error using desired coupling torque;
3. force the real coupling torque to track the desired coupling torque using motor torque.

With the proposed nominal control law, the closed-loop tracking error dynamics satisfy

$$
\dot{V}
=-c_1z_1^2-c_2z_2^2-c_3z_3^2,
$$

which proves asymptotic, and in the transformed error coordinates exponential, convergence of the tracking errors. Internal shaft dynamics are stable because the relation

$$
b\dot{\delta}+k\delta+k_3\delta^3=\tau_c
$$

is input-to-state stable for \(b>0\), \(k>0\), and \(k_3\ge 0\).

Therefore, the model and the proposed nonlinear control framework provide a physically consistent and mathematically justified basis for simulation, analysis, and further adaptive or robust controller development.
