# 🛩️ Aircraft Pitch-Rate Adaptive Control

*A simulation of a nonlinear longitudinal aircraft model under in-flight icing, using a Lyapunov-based adaptive controller that detects lift degradation and compensates for it in real time.*

The project includes a 4-DOF aircraft model, baseline and adaptive controllers, static plots with time-series responses and phase portraits, Lyapunov-based stability analysis, and an interactive `pygame` visualization with headless GIF export.

---

## 📋 Brief Description

Airframe icing reduces the lift-curve slope $C_{L_\alpha}$. This decreases lift generation, changes the longitudinal equilibrium, and forces the elevator to work harder to maintain the reference angle of attack. The adaptive controller detects this degradation through tracking-error growth and activates an online estimator based on a Lyapunov switching law.

![alt text](animations/aircraft_adaptive.gif)

| Component | Description |
|---|---|
| **Baseline controller** | Fixed nominal Lyapunov elevator law without real-time parameter estimation. |
| **Adaptive controller** | Nominal law augmented with an online estimator for $\Delta C_{L_\alpha}$, activated after anomalous tracking errors are detected. |
| **Aircraft dynamics** | Nonlinear longitudinal equations of motion with lift, drag, pitch moment, thrust, gravity, and icing degradation. |

> **Mathematical reference:** The complete derivation of the longitudinal aircraft model, aerodynamic scaling, and control law is provided in `model_draft.md` and `lyapunov_icing_controller_final_description.md`.

---

## ▶️ Run Project

```bash
cd Project_2_Adaptive_control_Pitch_plane/src
python main.py
```

---

## 🔧 Project Architecture

```text
Project_2_Adaptive_control_Pitch_plane/
├── src/
│   ├── system.py          # Aircraft longitudinal dynamics model
│   ├── controller.py      # Baseline and adaptive controllers
│   ├── simulation.py      # Simulation and data collection
│   ├── visualization.py   # Visualization and animation tools
│   └── main.py            # Main entry point
├── configs/               # Configuration files
├── figures/               # Result figures
├── animations/            # GIF animations
└── README.md
```

---

## 1. System Description and Symbol Dictionary

*Full mathematical derivation: `model_draft.md` §1–2 and `lyapunov_icing_controller_final_description.md` §1.*

![alt text](figures/plane.jpg)

*The scheme is used only as a simplified visual reference. The main symbols are described below.*

### State and Control Vectors

$$
s =
\begin{bmatrix}
V \\
\alpha \\
q \\
\theta
\end{bmatrix}
\in \mathbb{R}^4,
\qquad
a =
\begin{bmatrix}
\delta_e \\
\delta_t
\end{bmatrix}
\in \mathbb{R}^2
$$

| Symbol | Meaning | Units |
|---|---|---|
| $s$ | State vector | – |
| $V$ | True airspeed | m/s |
| $\alpha$ | Angle of attack | rad |
| $q$ | Pitch rate about the lateral axis | rad/s |
| $\theta$ | Pitch angle relative to the horizon | rad |
| $a$ | Control vector | – |
| $\delta_e$ | Elevator deflection | rad |
| $\delta_t$ | Normalized throttle command | – |

### Nonlinear Dynamics

$$
\boxed{\dot{s} = P(s,a) + d(t)}
$$

The nominal dynamics vector field is written as

$$
P(s,a) =
\begin{bmatrix}
\displaystyle \frac{1}{m}
\left[
T(\delta_t)\cos\alpha
-
D(V,\alpha,\delta_e)
-
mg\sin\gamma
\right]
\\[8pt]
\displaystyle
q
-
\frac{1}{mV}
\left[
T(\delta_t)\sin\alpha
+
L(V,\alpha,\delta_e)
-
mg\cos\gamma
\right]
\\[8pt]
\displaystyle
\frac{1}{I_y}M(V,\alpha,q,\delta_e)
\\[8pt]
q
\end{bmatrix},
\qquad
\gamma = \theta - \alpha .
$$

| Symbol | Meaning | Units |
|---|---|---|
| $\dot{s}$ | State derivative | varies |
| $P(s,a)$ | Nominal dynamics vector field | varies |
| $d(t) \in \mathbb{R}^4$ | Additive disturbance caused by gusts or noise | varies |
| $m$ | Aircraft mass | kg |
| $g$ | Gravitational acceleration | m/s² |
| $\gamma$ | Flight-path angle | rad |
| $I_y$ | Pitch moment of inertia | kg·m² |
| $T$ | Thrust force | N |
| $L$ | Lift force | N |
| $D$ | Drag force | N |
| $M$ | Pitching moment | N·m |

---

## 2. Aerodynamics and Icing Model

*Full derivation: `model_draft.md` §2 and `lyapunov_icing_controller_final_description.md` §2.*

### Dynamic Pressure and Force Scaling

From Bernoulli's equation and dimensional analysis,

$$
\bar{q} \triangleq \frac{1}{2}\rho V^2,
\qquad
F = \bar{q}SC_F
=
\frac{1}{2}\rho V^2SC_F .
$$

| Symbol | Meaning | Units |
|---|---|---|
| $\bar{q}$ | Dynamic pressure | Pa |
| $\rho$ | Air density | kg/m³ |
| $S$ | Wing reference area | m² |
| $C_F$ | Dimensionless aerodynamic coefficient | – |

### Lift, Drag, and Moment Coefficients

$$
\begin{aligned}
C_L &=
C_{L0}
+
C_{L_\alpha}\alpha
+
C_{L_{\delta_e}}\delta_e ,
\\
C_D &=
C_{D0}
+
\frac{
\left(
C_{L_\alpha}\alpha
+
C_{L_{\delta_e}}\delta_e
\right)^2
}{
\pi e \,\mathrm{AR}
},
\\
C_m &=
C_{m0}
+
C_{m_\alpha}\alpha
+
C_{mq}
\left(
\frac{q\bar{c}}{2V}
\right)
+
C_{m_{\delta_e}}\delta_e .
\end{aligned}
$$

| Symbol | Meaning | Units |
|---|---|---|
| $C_L, C_D, C_m$ | Lift, drag, and pitching-moment coefficients | – |
| $C_{L0}, C_{D0}, C_{m0}$ | Zero-angle coefficients | – |
| $C_{L_\alpha}$ | Lift-curve slope | rad⁻¹ |
| $C_{L_{\delta_e}}$ | Lift derivative with respect to elevator deflection | rad⁻¹ |
| $C_{m_\alpha}$ | Pitch stiffness derivative | rad⁻¹ |
| $C_{mq}$ | Pitch damping derivative | – |
| $C_{m_{\delta_e}}$ | Elevator effectiveness derivative | rad⁻¹ |
| $\bar{c}$ | Mean aerodynamic chord | m |
| $\frac{q\bar{c}}{2V}$ | Dimensionless pitch rate | – |
| $\mathrm{AR} = b^2/S$ | Wing aspect ratio | – |
| $e$ | Oswald efficiency factor | – |

### Icing Physical Model

Icing is modeled as a reduction of the lift-curve slope at time $t_{\mathrm{ice}}$:

$$
C_{L_\alpha}(t)
=
C_{L_\alpha}^{\mathrm{clean}}
+
\Delta C_{L_\alpha}(t),
$$

where

$$
\Delta C_{L_\alpha}(t)
=
\begin{cases}
0, & t < t_{\mathrm{ice}}, \\
\Delta C_{L_\alpha}^{\mathrm{ice}} < 0, & t \geq t_{\mathrm{ice}} .
\end{cases}
$$

The icing-induced perturbation in the angle-of-attack dynamics is

$$
\boxed{
\Delta\dot{\alpha}_{\mathrm{ice}}
=
\frac{1}{2}
\frac{\rho VS}{m}
\alpha
\Delta C_{L_\alpha}
}
$$

| Symbol | Meaning | Units |
|---|---|---|
| $t_{\mathrm{ice}}$ | Icing onset time | s |
| $\Delta C_{L_\alpha}$ | Lift-curve slope degradation | rad⁻¹ |
| $\Delta\dot{\alpha}_{\mathrm{ice}}$ | Icing perturbation in angle-of-attack rate | rad/s |

---

## 3. Control Strategy and Lyapunov Proofs

*Full derivation: `lyapunov_icing_controller_final_description.md` §4–9.*

### 3.1 Filtered Error and Reduced Dynamics

The tracking errors and filtered error are defined as

$$
e_\alpha = \alpha - \alpha_{\mathrm{ref}},
\qquad
e_q = q - q_{\mathrm{ref}},
\qquad
\boxed{r = e_q + \lambda_\alpha e_\alpha}.
$$

The reduced scalar dynamics for $r$ are

$$
\boxed{
\dot{r}
=
F(s)
+
B(s)\delta_e
+
Y(s)\Delta C_{L_\alpha}
}
$$

| Symbol | Meaning | Units |
|---|---|---|
| $e_\alpha$ | Angle-of-attack tracking error | rad |
| $e_q$ | Pitch-rate tracking error | rad/s |
| $r$ | Filtered tracking error | rad/s |
| $\lambda_\alpha > 0$ | Filter blending weight | – |
| $F(s)$ | Nominal pitch/AoA dynamics term | rad/s² |
| $B(s)$ | Elevator authority term | rad/s²/rad |
| $Y(s)$ | Icing regressor term | rad/s²/rad⁻¹ |

### 3.2 Nominal Lyapunov Controller for the Clean Wing

Before icing, $\Delta C_{L_\alpha}=0$. The nominal controller imposes

$$
\dot{r} = -k_r r .
$$

Therefore,

$$
\boxed{
\delta_{e,\mathrm{nom}}
=
\frac{-F(s)-k_rr}{B(s)}
}
$$

Using the Lyapunov candidate

$$
\mathcal{V}_0 = \frac{1}{2}r^2 ,
$$

its derivative becomes

$$
\dot{\mathcal{V}}_0
=
-k_rr^2
\leq 0 .
$$

| Symbol | Meaning | Units |
|---|---|---|
| $k_r > 0$ | Lyapunov convergence gain | s⁻¹ |
| $\delta_{e,\mathrm{nom}}$ | Nominal elevator command | rad |
| $\mathcal{V}_0$ | Nominal Lyapunov candidate | – |

### 3.3 Adaptive Switching Controller for the Iced Wing

After icing, the controller introduces the estimate $\widehat{\Delta C}_{L_\alpha}$ and the estimation error

$$
\widetilde{\Delta C}_{L_\alpha}
=
\widehat{\Delta C}_{L_\alpha}
-
\Delta C_{L_\alpha}.
$$

The adaptive elevator command is

$$
\boxed{
\delta_e
=
\frac{
-F(s)
-
k_rr
-
Y(s)\widehat{\Delta C}_{L_\alpha}
}{
B(s)
}
}
$$

The adaptation law is

$$
\boxed{
\dot{\widehat{\Delta C}}_{L_\alpha}
=
\gamma_C Y(s)r
}
$$

The composite Lyapunov candidate is

$$
\boxed{
\mathcal{V}
=
\frac{1}{2}r^2
+
\frac{1}{2\gamma_C}
\widetilde{\Delta C}_{L_\alpha}^2
}
$$

With the selected adaptation law, its derivative is

$$
\dot{\mathcal{V}}
=
-k_rr^2
\leq 0 .
$$

| Symbol | Meaning | Units |
|---|---|---|
| $\widehat{\Delta C}_{L_\alpha}$ | Online estimate of lift degradation | rad⁻¹ |
| $\widetilde{\Delta C}_{L_\alpha}$ | Estimation error | rad⁻¹ |
| $\gamma_C > 0$ | Adaptation gain | s⁻¹·rad |
| $\mathcal{V}$ | Composite Lyapunov function | – |
| $\delta_e$ | Adaptive elevator command | rad |

### 3.4 Switching Logic and Saturation

Adaptive mode is activated when the angle-of-attack error and the filtered error exceed predefined thresholds for `detect_steps` consecutive samples:

$$
\mathrm{adaptive\_mode}
=
\begin{cases}
\mathrm{True},
&
|e_\alpha| > e_{\alpha,\mathrm{thr}}
\ \land\
|r| > r_{\mathrm{thr}}
\ \text{for}\
N \geq \mathrm{detect\_steps},
\\
\mathrm{False},
&
\text{otherwise}.
\end{cases}
$$

Projection and actuator saturation keep the estimate and elevator command within physical limits:

$$
\widehat{\Delta C}_{L_\alpha}
\leftarrow
\mathrm{clip}
\left(
\widehat{\Delta C}_{L_\alpha},
\Delta C_{\min},
0
\right),
\qquad
\delta_e
\leftarrow
\mathrm{clip}
\left(
\delta_e,
-\delta_{e,\max},
+\delta_{e,\max}
\right).
$$

| Symbol | Meaning | Units |
|---|---|---|
| $e_{\alpha,\mathrm{thr}}$ | Angle-of-attack error detection threshold | rad |
| $r_{\mathrm{thr}}$ | Filtered-error detection threshold | rad/s |
| $\mathrm{detect\_steps}$ | Number of consecutive detections required for activation | – |
| $\Delta C_{\min}$ | Lower projection bound for the estimate | rad⁻¹ |
| $\delta_{e,\max}$ | Elevator saturation limit | rad |

---

## 4. Parameters Reference

*Sources: `project_description.md` and `lyapunov_icing_controller_final_description.md` §11.*

### Aircraft and Aerodynamic Parameters

| Symbol | Value | Units | Meaning |
|---|---:|---|---|
| $m$ | 1200.0 | kg | Aircraft mass |
| $I_y$ | 1800.0 | kg·m² | Pitch moment of inertia |
| $S$ | 16.2 | m² | Wing reference area |
| $\bar{c}$ | 1.5 | m | Mean aerodynamic chord |
| $T_{\max}$ | 3000.0 | N | Maximum thrust at $\delta_t=1$ |
| $\rho$ | 1.225 | kg/m³ | Sea-level air density |
| $C_{L_\alpha}$ | 3.50 | rad⁻¹ | Clean lift-curve slope |
| $C_{m_\alpha}$ | −0.60 | rad⁻¹ | Pitch stiffness derivative |
| $C_{mq}$ | −8.00 | – | Pitch damping derivative |
| $C_{m_{\delta_e}}$ | −1.10 | rad⁻¹ | Elevator effectiveness derivative |
| $C_{D0}$ | 0.027 | – | Zero-lift parasitic drag |
| $\mathrm{AR}$ | 7.32 | – | Wing aspect ratio |
| $e$ | 0.81 | – | Oswald efficiency factor |

### Controller Parameters

| Symbol | Value | Units | Meaning |
|---|---:|---|---|
| $\lambda_\alpha$ | 1.5 | – | Filtered-error blending weight |
| $k_r$ | 2.0 | s⁻¹ | Lyapunov damping coefficient |
| $\gamma_C$ | 300.0 | s⁻¹·rad | Adaptation gain |
| $\Delta C_{\min}$ | −3.0 | rad⁻¹ | Lower bound for $\widehat{\Delta C}_{L_\alpha}$ |
| $\delta_{e,\max}$ | 25° | rad | Elevator actuator limit |
| $e_{\alpha,\mathrm{thr}}$ | 3° | rad | Angle-of-attack error trigger threshold |
| $r_{\mathrm{thr}}$ | 2°/s | rad/s | Filtered-error trigger threshold |
| $\mathrm{detect\_steps}$ | 20 | – | Number of consecutive detections required for activation |

---

## 5. Interactive Control

| Key | Action | Key | Action |
|---|---|---|---|
| `SPACE` / `→` | Step forward | `←` | Step backward |
| `R` | Restart | `A` | Toggle autoplay |
| `+` / `-` | Change playback speed | `S` / `G` | Save PNG / Export GIF |
| `Q` / `Esc` | Quit | Click scrubber | Seek to a selected time |

---

## 6. Results

Icing was introduced at $t = 10\,\mathrm{s}$, and adaptive control was activated at approximately $t = 12.01\,\mathrm{s}$. After icing, the baseline controller remains stable but develops a steady tracking error: the angle of attack increases from about $4^\circ$ to $5.4^\circ$. This occurs because the reduced lift-curve slope $C_{L_\alpha}$ lowers lift generation and shifts the longitudinal equilibrium.

![alt text](figures/comparison.png)

The adaptive controller shows a short transient after icing, but then returns $\alpha$ close to the reference value and damps the pitch rate $q$ toward zero. The estimate $\widehat{\Delta C}_{L_\alpha}$ converges to the imposed degradation, confirming that the controller compensates for the loss of lift effectiveness.

The phase portraits support this result. In the Lyapunov error space, the baseline trajectory settles in a nonzero error region, while the adaptive trajectory returns toward the desired equilibrium after activation.

![alt text](figures/phase_portraits/phase_error_r_baseline.png)

![alt text](figures/phase_portraits/phase_error_r_adaptive.png)

The $\alpha$-$\theta$ portraits show that the baseline system shifts to a new post-icing operating point with a higher angle of attack and pitch angle. The adaptive controller produces a larger transient because it actively changes the elevator command, but then restores the controlled inner-loop behavior.

![alt text](figures/phase_portraits/phase_alpha_theta_baseline.png)

![alt text](figures/phase_portraits/phase_alpha_theta_adaptive.png)

The $\alpha$-$q$ portraits confirm the same conclusion: the baseline controller keeps the system bounded but away from the original reference, whereas the adaptive controller returns the motion toward $\alpha \approx 4^\circ$ and $q \approx 0$.

![alt text](figures/phase_portraits/phase_alpha_q_baseline.png)

![alt text](figures/phase_portraits/phase_alpha_q_adaptive.png)

---

## 7. Discussion and Limitations

The baseline nominal controller successfully tracks $\alpha_{\mathrm{ref}}$ under clean-wing conditions. However, when icing reduces $C_{L_\alpha}$, the baseline controller develops a steady-state error and requires increased elevator effort. This happens because the aerodynamic model used by the nominal controller no longer matches the true degraded aircraft dynamics.

The adaptive controller detects the anomaly through the filtered error $r$, activates the parameter estimator, and drives $\widehat{\Delta C}_{L_\alpha}$ toward the imposed degradation value. The composite Lyapunov function satisfies $\dot{\mathcal{V}} \leq 0$, which guarantees bounded closed-loop signals and supports tracking recovery under the assumptions listed in `lyapunov_icing_controller_final_description.md` §10.

The main limitation is that the model considers only one dominant icing effect: the degradation of the lift-curve slope. Real icing can also change drag, pitching moment, elevator effectiveness, stall behavior, and actuator limits. Therefore, the presented controller should be interpreted as a simplified proof-of-concept rather than a complete flight-certified icing protection system.

---

## AI Guidance

AI tools were used to support theoretical research, result visualization, controller tuning, and README text editing.
