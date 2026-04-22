# 🛩️ Mathematical Model of the F-16 Aircraft (Pitch-Rate Plant)

> **Scope**: This document defines the **plant dynamics** of the F-16 pitch-rate channel. All notation, operator formalism, and stability certificates strictly follow the lecture framework (`S = P[S, α]`, `b` for uncertainty, `L(S)` for Lyapunov functions). Controller synthesis is intentionally excluded.

---

## 1. 📖 Notation Alignment (Lecture ↔ Aircraft)

| Lecture Symbol | Aircraft Mapping | Description |
|:---:|:---:|:---|
| $S$ | $q$ | **State**: Pitch rate [rad/s] |
| $\alpha$ | $\{M_q, M_{\delta_e}\}$ | **Parameters**: Aerodynamic derivatives (time-varying) |
| $b(t)$ | $d(t)$ | **Disturbance/Uncertainty**: Turbulence, unmodeled dynamics |
| $t$ | $t \in \mathbb{R}_{\geq 0}$ | **Time**: Continuous evolution |
| $u$ | $\delta_e$ | **Control input**: Elevator deflection [rad] |
| $y$ | $h(S)$ | **Observation**: Measured pitch rate |
| $P[\cdot]$ | $\dot{S} = P(S, \alpha)$ | **Plant operator**: Governs state evolution |

---

## 2. 📐 Continuous-Time Plant Dynamics

Following the lecture's continuous plant formulation:
$$
\dot{S}(t) = P[S(t), \alpha(t)] + b(t)
$$

Expanding $P(\cdot)$ for the linearized short-period pitch-rate channel:
$$
\boxed{\dot{S}(t) = M_q(\alpha) \cdot S(t) + M_{\delta_e}(\alpha) \cdot u(t) + b(t)}
$$

### 🔍 Component Breakdown:
- $\dot{S}(t)$: State derivative (angular acceleration)
- $M_q(\alpha) \cdot S(t)$: Aerodynamic damping (self-stabilizing, $M_q < 0$)
- $M_{\delta_e}(\alpha) \cdot u(t)$: Control effectiveness (actuator-driven, $M_{\delta_e} > 0$)
- $b(t)$: Additive uncertainty/disturbance term

---

## 3. 🖼️ System Architecture Diagram

```text

```
