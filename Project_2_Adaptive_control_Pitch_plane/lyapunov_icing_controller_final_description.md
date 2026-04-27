# Lyapunov-Based Switching Adaptive Controller for Aircraft Icing

## 1. System dynamics

We consider nonlinear longitudinal aircraft dynamics with the state vector

$$
x =
\begin{bmatrix}
V & \alpha & q & \theta
\end{bmatrix}^T
$$

and the control vector

$$
u =
\begin{bmatrix}
\delta_e & \delta_t
\end{bmatrix}^T .
$$

Here $V$ is airspeed, $\alpha$ is angle of attack, $q$ is pitch rate, $\theta$ is pitch angle, $\delta_e$ is elevator deflection, and $\delta_t$ is throttle command. The flight-path angle is

$$
\gamma=\theta-\alpha .
$$

The thrust is

$$
T=T_{\max}\delta_t .
$$

The equations of motion are

$$
\dot{V} =
\frac{1}{m}
\left[T\cos\alpha-D-
mg\sin\gamma\right],
$$

$$
\dot{\alpha} =
q-\frac{1}{mV}
\left[T\sin\alpha+L-mg\cos\gamma\right],
$$

$$
\dot{q}=
\frac{1}{I_y}\left[\frac{1}{2}\rho V^2S\bar{c}C_m\right],
$$

$$
\dot{\theta}=q .
$$

The aerodynamic lift and drag are

$$
L=\frac{1}{2}\rho V^2S C_L,
$$

$$
D=\frac{1}{2}\rho V^2S C_D .
$$

The aerodynamic coefficients are

$$
C_L=C_{L0}+C_{L_\alpha}\alpha+C_{L_{\delta_e}}\delta_e,
$$

$$
C_D=C_{D0}+\frac{\left(C_{L_\alpha}\alpha+C_{L_{\delta_e}}\delta_e\right)^2}{\pi e AR},
$$

$$
C_m=C_{m0}+C_{m\alpha}\alpha+C_{mq}\frac{q\bar{c}}{2V}+C_{m\delta_e}\delta_e .
$$

The coefficients have the following meaning.

| Symbol | Meaning |
|---|---|
| $m$ | aircraft mass |
| $I_y$ | pitch moment of inertia |
| $\rho$ | air density |
| $S$ | wing reference area |
| $\bar{c}$ | mean aerodynamic chord |
| $AR$ | aspect ratio |
| $e$ | Oswald efficiency factor |
| $g$ | gravitational acceleration |
| $C_{L0}$ | zero-angle lift coefficient |
| $C_{L_\alpha}$ | lift-curve slope |
| $C_{L_{\delta_e}}$ | lift derivative with respect to elevator |
| $C_{D0}$ | zero-lift drag coefficient |
| $C_{m0}$ | zero-angle pitching moment coefficient |
| $C_{m\alpha}$ | pitching moment derivative with respect to angle of attack |
| $C_{mq}$ | pitch damping derivative |
| $C_{m\delta_e}$ | pitching moment derivative with respect to elevator |

---

## 2. Physical problem: icing-induced degradation

The controller is designed for the case where aircraft icing degrades the wing aerodynamic performance. Ice accretion changes the wing surface roughness and effective shape. As a result, the wing produces less lift for the same angle of attack.

This is modeled as a reduction of the lift-curve slope:

$$
C_{L_\alpha}^{ice}
<
C_{L_\alpha}^{clean}.
$$

Define

$$
\Delta C_{L_\alpha}=C_{L_\alpha}^{ice}-C_{L_\alpha}^{clean}.
$$

Since icing reduces lift effectiveness,

$$
\Delta C_{L\_\alpha}<0.
$$

The piecewise icing model is

$$
\Delta C_{L_{\alpha}}(t)
=\begin{cases}
0, & t < t_{ice},\\\\
\Delta C_{L_{\alpha}}^{ice}, & t \geq t_{ice}.
\end{cases}
$$

Thus,

$$
C_{L_\alpha}(t)=C_{L_\alpha}^{clean}+\Delta C_{L_\alpha}(t).
$$

The lift variation caused by icing is

$$
\Delta C_L=\alpha\Delta C_{L_\alpha},
$$

and therefore

$$
\Delta L=\frac{1}{2}\rho V^2S\alpha\Delta C_{L_\alpha}.
$$

Because lift enters the angle-of-attack equation as

$$
\dot{\alpha}=q-\frac{T\sin\alpha+L-mg\cos\gamma}{mV},
$$

the icing-induced contribution to the angle-of-attack dynamics is

$$
\Delta\dot{\alpha}_{ice}=-\frac{\Delta L}{mV}.
$$

Substituting $\Delta L$,

$$
\Delta\dot{\alpha}_{ice}=-\frac{1}{mV}\left(\frac{1}{2}\rho V^2S\alpha\Delta C_{L_\alpha}\right),
$$

hence

$$
\boxed{
\Delta\dot{\alpha}_{ice}=-\frac{1}{2}\frac{\rho VS}{m}\alpha\Delta C_{L_\alpha}}.
$$

Therefore, in this system, icing does not directly enter the pitch-rate equation $\dot{q}$. It affects the lift force and therefore the angle-of-attack dynamics. This is the reason why the controller is derived using the filtered error $r$, where $\dot{\alpha}$ contributes to the reduced error dynamics.

---

## 3. Control objective

Define the tracking errors

$$
e_\alpha=\alpha-\alpha_{ref},
$$

$$
e_q=q-q_{ref}.
$$

The control objective is

$$
\boxed{
e_\alpha(t)\rightarrow0
}
$$

and

$$
\boxed{
e_q(t)\rightarrow0
}
$$

despite the unknown degradation $\Delta C_{L_\alpha}<0$.

The control action is

$$
a=\delta_e.
$$

The final controller is written as

$$
\boxed{
a=g(s_a)
},
$$

where the augmented controller state is

$$
s_a=
\begin{bmatrix}
V\\
\alpha\\
q\\
\theta\\
\widehat{\Delta C}_{L_\alpha}
\end{bmatrix}.
$$

---

## 4. Filtered error and reduced dynamics

Define the filtered error

$$
\boxed{
r=e_q+\lambda_\alpha e_\alpha
}
$$

with

$$
\lambda_\alpha>0.
$$

Since

$$
\dot{e}_\alpha=\dot{\alpha}-\dot{\alpha}_{ref}
$$

and

$$
\dot{e}_q=\dot{q}-\dot{q}_{ref},
$$

we have

$$
\dot{r}=\dot{e}_q+\lambda_\alpha\dot{e}_\alpha.
$$

Using the plant dynamics, this can be written as

$$
\boxed{\dot{r}=F(s)+B(s)\delta_e+Y(s)\Delta C_{L_\alpha}}.
$$

The following functions are used.

The pitch-rate dynamics is decomposed as

$$
\dot{q}=f_q(s)+b_q(s)\delta_e,
$$

where

$$
\boxed{f_q(s)=\frac{1}{2I_y}\rho V^2S\bar{c}\left[C_{m0}+C_{m\alpha}\alpha+C_{mq}\frac{q\bar{c}}{2V}\right]}
$$

and

$$
\boxed{b_q(s)=\frac{1}{2I_y}\rho V^2S\bar{c}C_{m\delta_e}}.
$$

The angle-of-attack dynamics is decomposed as

$$\dot{\alpha}=f_\alpha(s)+b_\alpha(s)\delta_e+Y_\alpha(s)\Delta C_{L_\alpha}.
$$

The nominal part is

$$
\boxed{f_\alpha(s)=q-\frac{T\sin\alpha+L_{clean,no\_de}-mg\cos(\theta-\alpha)}{mV}},
$$

where

$$
L_{clean,no\_de}=\frac{1}{2}\rho V^2S\left(C_{L0}+C_{L_\alpha}^{clean}\alpha\right).
$$

The elevator effectiveness in $\alpha$-dynamics is

$$
\boxed{b_\alpha(s)=-\frac{1}{2}\frac{\rho VS}{m}C_{L_{\delta_e}}}.
$$

The icing regressor in $\alpha$-dynamics is

$$
\boxed{Y_\alpha(s)=-\frac{1}{2}\frac{\rho VS}{m}\alpha}.
$$

Therefore, for the reduced $r$-dynamics,

$$
\boxed{F(s)=f_q(s)+\lambda_\alpha f_\alpha(s)-\dot{q}_{ref}-\lambda_\alpha\dot{\alpha}_{ref}},
$$

$$
\boxed{B(s)=b_q(s)+\lambda_\alpha b_\alpha(s)},
$$

$$
\boxed{Y(s)=\lambda_\alpha Y_\alpha(s)}.
$$

For constant references,

$$
\dot{\alpha}_{ref}=0,
$$

$$
\dot{q}_{ref}=0,
$$

so

$$
F(s)=f_q(s)+\lambda_\alpha f_\alpha(s).
$$

---

## 5. Nominal Lyapunov controller

Before icing,

$$
\Delta C_{L_\alpha}=0.
$$

Then

$$
\dot{r}=F(s)+B(s)\delta_e.
$$

Choose the desired stable dynamics

$$
\dot{r}=-k_r r,
$$

where

$$
k_r>0.
$$

Imposing this gives

$$
F(s)+B(s)\delta_e=-k_r r.
$$

Thus, the nominal clean-wing controller is

$$
\boxed{\delta_{e,nom}=\frac{-F(s)-k_r r}{B(s)}}.
$$

---

## 6. Nominal Lyapunov proof

Choose

$$
\boxed{
\mathcal{V}_0=\frac{1}{2}r^2
}.
$$

Then

$$
\dot{\mathcal{V}}_0=r\dot{r}.
$$

Since the controller imposes

$$
\dot{r}=-k_r r,
$$

we get

$$
\dot{\mathcal{V}}_0=r(-k_r r),
$$

therefore

$$
\boxed{\dot{\mathcal{V}}_0=-k_r r^2\leq0}.
$$

Hence $r$ is stable. Since

$$
r=e_q+\lambda_\alpha e_\alpha,
$$

and for $q_{ref}=0$

$$
e_q=\dot{e}_\alpha,
$$

we have

$$
\dot{e}_\alpha+\lambda_\alpha e_\alpha=r(t).
$$

If $r(t)\rightarrow0$, this stable first-order system gives

$$
e_\alpha(t)\rightarrow0,
$$

and

$$
e_q(t)=r(t)-\lambda_\alpha e_\alpha(t)\rightarrow0.
$$

---

## 7. Adaptive controller for icing

After icing,

$$
\Delta C_{L_\alpha}\neq0
$$

and it is unknown.

Introduce the estimate

$$
\widehat{\Delta C}_{L_\alpha}
$$

and the estimation error

$$
\boxed{\widetilde{\Delta C}_{L_\alpha}=\widehat{\Delta C}_{L_\alpha}-\Delta C_{L_\alpha}}.
$$

The adaptive controller is

$$
\boxed{\delta_e=\frac{-F(s)-k_r r-Y(s)\widehat{\Delta C}_{L_\alpha}}{B(s)}}.
$$

Equivalently,

$$
\delta_e=\delta_{e,nom}+\delta_{e,adapt},
$$

where

$$
\boxed{\delta_{e,nom}=\frac{-F(s)-k_r r}{B(s)}}
$$

and

$$
\boxed{\delta_{e,adapt}=-\frac{Y(s)\widehat{\Delta C}_{L_\alpha}}{B(s)}}.
$$

Substituting the adaptive controller into

$$
\dot{r}=F(s)+B(s)\delta_e+Y(s)\Delta C_{L_\alpha}
$$

gives

$$
\dot{r}=-k_r r-Y(s)\widehat{\Delta C}_{L_\alpha}+Y(s)\Delta C_{L_\alpha}.
$$

Therefore,

$$
\boxed{\dot{r}=-k_r r-Y(s)\widetilde{\Delta C}_{L_\alpha}}.
$$

---

## 8. Lyapunov proof for the adaptive controller

Choose

$$
\boxed{\mathcal{V}=\frac{1}{2}r^2+\frac{1}{2\gamma_C}\widetilde{\Delta C}_{L_\alpha}^2}
$$

with

$$
\gamma_C>0.
$$

The derivative is

$$
\dot{\mathcal{V}}=r\dot{r}+\frac{1}{\gamma_C}\widetilde{\Delta C}_{L_\alpha}\dot{\widetilde{\Delta C}}_{L_\alpha}.
$$

After icing is established, assume the degradation is piecewise constant:

$$
\Delta C_{L_\alpha}=const.
$$

Therefore,

$$
\dot{\widetilde{\Delta C}}_{L_\alpha}=\dot{\widehat{\Delta C}}_{L_\alpha}.
$$

Using

$$
\dot{r}=-k_r r-Y(s)\widetilde{\Delta C}_{L_\alpha},
$$

we obtain

$$
\dot{\mathcal{V}}=-k_r r^2-rY(s)\widetilde{\Delta C}_{L_\alpha}+\frac{1}{\gamma_C}\widetilde{\Delta C}_{L_\alpha}\dot{\widehat{\Delta C}}_{L_\alpha}.
$$

Group the mixed term:

$$
\dot{\mathcal{V}}=-k_r r^2+\widetilde{\Delta C}_{L_\alpha}\left[-rY(s)+\frac{1}{\gamma_C}\dot{\widehat{\Delta C}}_{L_\alpha}\right].
$$

Choose the adaptive law

$$
\boxed{\dot{\widehat{\Delta C}}_{L_\alpha}=\gamma_CY(s)r}.
$$

Then

$$
-rY(s)+\frac{1}{\gamma_C}\gamma_CY(s)r=0.
$$

Thus,

$$
\boxed{\dot{\mathcal{V}}=-k_r r^2\leq0}.
$$

This proves Lyapunov stability of the adaptive closed-loop system.

Moreover, since

$$
\mathcal{V}(t)\leq\mathcal{V}(t_s),
$$

both $r(t)$ and $\widetilde{\Delta C}_{L_\alpha}(t)$ are bounded. Since

$$
\dot{\mathcal{V}}=-k_r r^2,
$$

we have

$$
r\in L_2.
$$

If $Y(s)$ is bounded, then $\dot r$ is bounded. By Barbalat's lemma,

$$
\boxed{r(t)\rightarrow0}.
$$

Using

$$
\dot{e}_\alpha+\lambda_\alpha e_\alpha=r(t),
$$

with $\lambda_\alpha>0$, we obtain

$$
\boxed{e_\alpha(t)\rightarrow0}
$$

and

$$
\boxed{e_q(t)\rightarrow0}.
$$

Thus, the adaptive controller stabilizes the longitudinal inner-loop dynamics despite the unknown icing-induced reduction of $C_{L_\alpha}$.

---

## 9. Explicit switching control law

The control action is

$$
a=\delta_e.
$$

The switching controller is

$$
\boxed{a=g(s_a)=\begin{cases}g_{clean}(s), & adaptive\_mode=False,\\g_{ice}(s_a), & adaptive\_mode=True.\end{cases}}
$$

where

$$
\boxed{g_{clean}(s)=\frac{-F(s)-k_r r}{B(s)}}
$$

and

$$
\boxed{g_{ice}(s_a)=\frac{-F(s)-k_r r-Y(s)\widehat{\Delta C}_{L_\alpha}}{B(s)}}.
$$

The adaptive estimate evolves according to

$$
\boxed{\dot{\widehat{\Delta C}}_{L_\alpha}=\gamma_CY(s)r}.
$$

The implementation uses projection:

$$
\boxed{\widehat{\Delta C}_{L_\alpha}=clip\left(\widehat{\Delta C}_{L_\alpha},\Delta C_{min},0\right)
}
$$

because icing can only reduce $C_{L_\alpha}$.

The elevator command is saturated:

$$
\boxed{\delta_e=clip\left(\delta_e,-\delta_{e,max},\delta_{e,max}\right)}.
$$

---

## 10. Assumptions for the proof

The Lyapunov proof requires the following assumptions:

1. $B(s)\neq0$, so the elevator has authority over the reduced dynamics.
2. The elevator is not saturated during the strict Lyapunov proof.
3. After icing occurs, $\Delta C_{L_\alpha}$ is piecewise constant.
4. The aircraft remains in the valid flight envelope, especially $V>0$.
5. $Y(s)$ is bounded.
6. Reference signals and their derivatives are bounded.
7. The controller model matches the reduced plant dynamics sufficiently well.

If saturation is active or icing continues to grow slowly, the result should be interpreted as practical stability rather than exact asymptotic convergence.

---

## 11. Parameters used

### Aircraft parameters

| Parameter | Value | Meaning |
|---|---:|---|
| $m$ | $1200.0$ kg | aircraft mass |
| $S$ | $16.2\ \mathrm{m^2}$ | wing area |
| $\bar{c}$ | $1.5\ \mathrm{m}$ | mean aerodynamic chord |
| $AR$ | $7.32$ | aspect ratio |
| $e$ | $0.81$ | Oswald efficiency factor |
| $I_y$ | $1800.0\ \mathrm{kg\,m^2}$ | pitch moment of inertia |
| $T_{\max}$ | $3000.0\ \mathrm{N}$ | maximum thrust |
| $\rho$ | $1.225\ \mathrm{kg/m^3}$ | air density |
| $g$ | $9.81\ \mathrm{m/s^2}$ | gravitational acceleration |

### Aerodynamic parameters

| Parameter | Value | Meaning |
|---|---:|---|
| $C_{L0}$ | $0.0901$ | zero-angle lift coefficient |
| $C_{L_\alpha}$ | $3.50$ | clean lift-curve slope |
| $C_{L_{\delta_e}}$ | $0.35$ | lift derivative with respect to elevator |
| $C_{D0}$ | $0.027$ | zero-lift drag coefficient |
| $C_{m0}$ | $0.02$ | zero-angle pitching moment coefficient |
| $C_{m\alpha}$ | $-0.60$ | pitching moment derivative with respect to angle of attack |
| $C_{mq}$ | $-8.00$ | pitch damping derivative |
| $C_{m\delta_e}$ | $-1.10$ | pitching moment derivative with respect to elevator |

### Controller parameters

| Parameter | Value | Meaning |
|---|---:|---|
| $\lambda_\alpha$ | $1.5$ | filtered error coefficient |
| $k_r$ | $2.0$ | Lyapunov convergence gain |
| $\gamma_C$ | $300.0$ | adaptation gain |
| $\Delta C_{min}$ | $-3.0$ | lower projection bound |
| $\delta_{e,max}$ | $25^\circ$ | elevator saturation |
| $e_{\alpha,thr}$ | $3^\circ$ | detection threshold for angle-of-attack error |
| $r_{thr}$ | $2^\circ$ | detection threshold for filtered error |
| $\delta_{e,thr}$ | $10^\circ$ | detection threshold for elevator effort |
| $detect\_steps$ | $20$ | consecutive detection steps |
| $V_{min}$ | $1.0\ \mathrm{m/s}$ | numerical lower bound for airspeed |
| $B_{min}$ | $10^{-6}$ | numerical lower bound for $B(s)$ |

---

## 12. Final summary

The icing problem is modeled as

$$
C_{L_\alpha}^{ice}<C_{L_\alpha}^{clean}
$$

or equivalently

$$
\Delta C_{L_\alpha}<0.
$$

The reduced dynamics is

$$
\dot{r}=F(s)+B(s)\delta_e+Y(s)\Delta C_{L_\alpha}.
$$

The adaptive controller is

$$
\delta_e=\frac{-F(s)-k_r r-Y(s)\widehat{\Delta C}_{L_\alpha}}{B(s)}.
$$

The adaptation law is

$$
\dot{\widehat{\Delta C}}_{L_\alpha}=\gamma_CY(s)r.
$$

The Lyapunov function is

$$
\mathcal{V}=\frac{1}{2}r^2+\frac{1}{2\gamma_C}\widetilde{\Delta C}_{L_\alpha}^2.
$$

Its derivative is

$$
\dot{\mathcal{V}}=-k_r r^2\leq0.
$$

Therefore, under the stated assumptions,

$$
r(t)\rightarrow0,
$$

$$
e_\alpha(t)\rightarrow0,
$$

$$
e_q(t)\rightarrow0.
$$

Thus, the proposed controller stabilizes the longitudinal inner-loop dynamics and compensates for the aerodynamic degradation caused by icing.
