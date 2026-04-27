# Lyapunov Adaptive Icing Controller — Simulation & Visualisation

A Python simulation of a **nonlinear longitudinal aircraft model** subject to in-flight icing, with a **Lyapunov-based switching adaptive controller** that detects lift degradation online and compensates for it in real time.

The project includes the full physics model, both adaptive and baseline controllers, static matplotlib plots (time-series, phase portraits, Lyapunov function), and an interactive pygame visualisation with GIF export.

---

## Table of Contents

1. [Background](#background)
2. [Repository structure](#repository-structure)
3. [Installation](#installation)
4. [Quick start](#quick-start)
5. [Running the simulation — `main.py` arguments](#running-the-simulation--mainpy-arguments)
6. [Output files](#output-files)
7. [Module reference](#module-reference)
8. [Physics reference](#physics-reference)

---

## Background

Airframe icing reduces the lift-curve slope `C_Lα`, destabilising the pitch axis and forcing the elevator to work harder to hold a reference angle of attack.  The adaptive controller detects this degradation automatically using a Lyapunov-based switching law — no explicit icing sensor is required.

The **baseline controller** applies a fixed Lyapunov nominal elevator law with no estimation.  The **adaptive controller** augments this with an online parameter estimator for `ΔC_Lα` that activates once anomalous tracking errors are detected.

The **Lyapunov function** used in the stability proof is:

```
V(t) = ½ r(t)²  +  (1 / 2γ_C) · ΔC̃_Lα(t)²
```

where `r = e_q + λ_α · e_α` is the filtered tracking error and `ΔC̃_Lα = ΔĈ_Lα − ΔC_Lα*` is the parameter estimation error.

---

## Repository structure

```
project/
│
├── src/                          # All Python source files
│   ├── main.py                   # Entry point — run this
│   │
│   ├── system.py                 # Aircraft physics model (EOM + RK4 integrator)
│   ├── controller.py             # Lyapunov adaptive elevator controller
│   ├── simulation.py             # Scenario setup and simulation loop
│   │
│   ├── visualization.py          # Time-series matplotlib plots
│   ├── visualization_phase.py    # Phase portraits with time heatmap
│   ├── visualization_lyapunov.py # Lyapunov function V(t) static plots
│   └── visualization_aircraft.py # Interactive pygame animation + GIF export
│
└── results/                      # Output directory (created on first run)
    ├── comparison.png / .pdf     # Time-series plots (--plots)
    ├── lyapunov.png / .pdf       # Lyapunov plots (--lyapunov)
    ├── phase_portraits/          # Phase portrait subdirectory (--phase)
    │   ├── phase_alpha_q_adaptive.png / .pdf
    │   ├── phase_alpha_q_baseline.png / .pdf
    │   ├── phase_error_r_adaptive.png / .pdf
    │   ├── phase_error_r_baseline.png / .pdf
    │   ├── phase_alpha_theta_adaptive.png / .pdf
    │   └── phase_alpha_theta_baseline.png / .pdf
    ├── aircraft_adaptive.gif     # Animation GIF (--gif)
    └── aircraft_baseline.gif     # Baseline animation GIF (--gif --animate-controller both)
```

All source files must be run from within the `src/` directory (or with `src/` on the Python path) because they import each other by name.

---

## Installation

**Python 3.10 or later** is required.

```bash
pip install numpy matplotlib pygame pillow
```

| Package    | Purpose                                      |
|------------|----------------------------------------------|
| `numpy`    | Array maths, RK4 integration                 |
| `matplotlib` | Static plots (time-series, phase, Lyapunov)|
| `pygame`   | Interactive animation window                 |
| `pillow`   | GIF export from pygame surfaces              |

---

## Quick start

```bash
cd src

# Run both controllers, print summaries only (no plots)
python main.py

# Run and generate all static plots
python main.py --plots --phase --lyapunov

# Run and open the interactive aircraft animation (adaptive controller)
python main.py --animation

# Export GIFs for both controllers
python main.py --gif --animate-controller both

# Full run — everything
python main.py --plots --phase --lyapunov --gif --animation
```

---

## Running the simulation — `main.py` arguments

Both the adaptive and baseline controllers are always simulated on every run.  All visualisation outputs are **opt-in** — pass the relevant flags to enable them.

### Simulation parameters

| Argument | Type | Default | Description |
|---|---|---|---|
| `--t-end` | float | `200.0` | Total simulation duration in seconds |
| `--t-ice` | float | `10.0` | Time of icing onset in seconds |
| `--severity` | float | `0.30` | Fraction of `C_Lα` lost to icing (e.g. `0.30` = 30 % drop) |
| `--V-trim` | float | `60.0` | Trim airspeed in m/s |
| `--alpha-ref` | float | `None` | Reference angle of attack in degrees; defaults to trim alpha (4°) |
| `--dt` | float | `0.01` | Integration timestep in seconds |
| `--out-dir` | str | `results` | Root directory for all output files |

### Plot flags (opt-in, all off by default)

| Argument | Description |
|---|---|
| `--plots` | Save time-series comparison plots (α, q, δe, ΔĈ_Lα, V) for both controllers as `<out-dir>/comparison.png/.pdf` |
| `--phase` | Save 6 phase portraits (3 types × 2 controllers) to `<out-dir>/phase_portraits/` |
| `--lyapunov` | Save the Lyapunov function decomposition plot to `<out-dir>/lyapunov.png/.pdf` |

### Animation flags

| Argument | Type | Default | Description |
|---|---|---|---|
| `--animation` | flag | off | Open the interactive pygame aircraft window |
| `--animate-controller` | choice | `adaptive` | Which controller to animate: `adaptive`, `baseline`, or `both`. When `both` is chosen, windows open sequentially |
| `--gif` | flag | off | Export an animated GIF (headless, no display required) |
| `--gif-step` | int | `15` | Sample every N-th simulation frame when building the GIF — higher value = smaller file, lower frame rate |
| `--gif-fps` | int | `25` | Output frame rate for the exported GIF |

### Lyapunov scale

| Argument | Type | Default | Description |
|---|---|---|---|
| `--lya-scale` | choice | `log` | Y-axis scale for Lyapunov plots and the animation strip: `log` (log₁₀, floor at 10⁻¹⁴) or `linear` |

### Interactive animation controls

Once the pygame window is open:

| Key | Action |
|---|---|
| `SPACE` / `→` | Step forward one frame |
| `←` | Step backward one frame |
| `R` | Restart from t = 0 |
| `A` | Toggle auto-play |
| `+` / `-` | Increase / decrease playback speed |
| `S` | Save a PNG screenshot of the current frame |
| `G` | Export a GIF of the full simulation from the current window |
| `Q` / `Esc` | Quit |

Click anywhere on the scrubber bar at the bottom to seek to that time.

### Examples

```bash
# Severe icing (50 %), short run to check stability
python main.py --severity 0.5 --t-end 60 --plots

# Icing starts very early, adaptive controller only, log Lyapunov plot
python main.py --t-ice 5 --lyapunov --lya-scale log

# Linear Lyapunov scale in animation (useful when changes are small)
python main.py --animation --lya-scale linear

# Export GIFs for both controllers with a finer frame sample
python main.py --gif --animate-controller both --gif-step 5 --gif-fps 30

# Full run with all outputs, results saved to custom directory
python main.py --plots --phase --lyapunov --gif --out-dir my_results
```

---

## Output files

| File | Generated by | Description |
|---|---|---|
| `results/comparison.png` | `--plots` | 5-panel time-series: α, q, δe, ΔĈ_Lα, V — adaptive vs baseline overlaid |
| `results/lyapunov.png` | `--lyapunov` | 3-panel Lyapunov decomposition: V total, ½r², estimation term |
| `results/phase_portraits/phase_alpha_q_adaptive.png` | `--phase` | Phase portrait α vs q, adaptive run |
| `results/phase_portraits/phase_alpha_q_baseline.png` | `--phase` | Phase portrait α vs q, baseline run |
| `results/phase_portraits/phase_error_r_adaptive.png` | `--phase` | Error-space portrait eα vs r, adaptive |
| `results/phase_portraits/phase_error_r_baseline.png` | `--phase` | Error-space portrait eα vs r, baseline |
| `results/phase_portraits/phase_alpha_theta_adaptive.png` | `--phase` | Portrait α vs θ, adaptive |
| `results/phase_portraits/phase_alpha_theta_baseline.png` | `--phase` | Portrait α vs θ, baseline |
| `results/aircraft_adaptive.gif` | `--gif` | Aircraft animation, adaptive controller |
| `results/aircraft_baseline.gif` | `--gif --animate-controller both` | Aircraft animation, baseline controller |

All static plots are saved in both `.png` and `.pdf` format.

---

## Module reference

### `system.py` — `AircraftSystem`

Implements the **nonlinear longitudinal aircraft equations of motion** with RK4 integration.

```python
aircraft = AircraftSystem(
    params=None,               # dict of parameter overrides (see DEFAULT_AIRCRAFT_PARAMS)
    t_ice=10.0,                # icing onset time [s]
    delta_CL_alpha_ice=-1.05,  # ΔC_Lα at icing onset (negative)
)
x_next = aircraft.step(t, x, delta_e, delta_t, dt)
de_trim, dt_trim, theta_trim = aircraft.trim(V_trim, alpha_trim)
```

Default aircraft parameters: m = 1200 kg, S = 16.2 m², c̄ = 1.5 m, I_y = 1800 kg·m², ρ = 1.225 kg/m³, C_Lα = 3.5 /rad.

---

### `controller.py` — `LyapunovIcingAdaptiveController`

Implements the **Lyapunov switching adaptive elevator law**.

```python
ctrl = LyapunovIcingAdaptiveController(
    params=None,          # dict of parameter overrides
    use_adaptation=True,  # False = baseline (no estimation)
)
delta_e, info = ctrl.compute_control(state, reference, dt)
```

Key controller parameters (in `DEFAULT_CONTROLLER_PARAMS`):

| Parameter | Default | Description |
|---|---|---|
| `lambda_alpha` | 1.5 | Filtered-error blending weight |
| `k_r` | 2.0 | Lyapunov damping gain |
| `gamma_C` | 300.0 | Adaptation gain |
| `delta_C_min` | −3.0 | Projection lower bound for ΔĈ_Lα |
| `e_alpha_thr` | 3° | Detection threshold on α error |
| `r_thr` | 2°/s | Detection threshold on filtered error r |
| `detect_steps` | 20 | Steps above threshold before adaptive mode activates |

---

### `simulation.py` — `SimConfig`, `run_simulation`

Sets up the scenario and drives the simulation loop.

```python
from simulation import SimConfig, run_simulation

cfg = SimConfig(t_end=200.0, t_ice=10.0, delta_CL_alpha_ice=-1.05)
results = run_simulation(config=cfg, use_adaptation=True)
```

`run_simulation` returns a dict of logged arrays:

| Key | Shape | Description |
|---|---|---|
| `t` | (N,) | Time vector [s] |
| `alpha`, `q`, `theta`, `V` | (N,) | State variables [rad, rad/s, rad, m/s] |
| `delta_e` | (N,) | Total elevator deflection [rad] |
| `delta_e_nom`, `delta_e_adapt` | (N,) | Nominal and adaptive components [rad] |
| `e_alpha`, `e_q`, `r` | (N,) | Tracking errors [rad, rad/s, rad/s] |
| `delta_CL_alpha_hat` | (N,) | Online estimate ΔĈ_Lα |
| `adaptive_mode` | (N,) bool | Whether adaptive mode is active |
| `F`, `B`, `Y` | (N,) | r-dynamics terms |
| `C_La_clean`, `C_La_iced` | scalar | Clean and iced lift-curve slopes |
| `t_ice` | scalar | Icing onset time [s] |

---

### `visualization.py` — `plot_matplotlib`

Saves a 5-panel time-series figure comparing the two controllers.

```python
from visualization import plot_matplotlib
plot_matplotlib(res_adp, res_base=res_base, save_path="results/comparison")
```

---

### `visualization_phase.py` — `plot_phase_portraits`

Saves 6 phase-portrait figures (3 types × 2 controllers) with time-coloured trajectories.

```python
from visualization_phase import plot_phase_portraits
plot_phase_portraits(res_adp, res_base, save_dir="results/phase_portraits")
```

Portrait types: α vs q, eα vs r (Lyapunov error space), α vs θ.  Trajectory colour encodes time using the `viridis` colormap (dark = early, bright = late).  Events marked: ◆ icing onset, ▲ adaptive mode trigger, ★ equilibrium.

Can also be run standalone:

```bash
python visualization_phase.py --t-end 200 --out-dir results
```

---

### `visualization_lyapunov.py` — `plot_lyapunov`

Saves a 3-panel figure of the Lyapunov function decomposition over time.

```python
from visualization_lyapunov import plot_lyapunov
plot_lyapunov(res_adp, res_base=res_base, save_dir="results", lya_scale="log")
```

Panels: total V(t), tracking term ½r², estimation term (1/2γ)ΔC̃².  For the baseline controller the estimation term is identically zero.

Can also be run standalone:

```bash
python visualization_lyapunov.py --t-end 200 --lya-scale log --out-dir results
```

---

### `visualization_aircraft.py` — `run_aircraft_view`, `export_gif`

Interactive pygame animation window and headless GIF renderer.

```python
from visualization_aircraft import run_aircraft_view, export_gif

# Interactive window
run_aircraft_view(results, label="Adaptive", gif_path="results/adaptive.gif",
                  lya_scale="log")

# Headless GIF only (no display required)
export_gif(results, "results/adaptive.gif",
           step=15, fps=25, label="Adaptive", lya_scale="log")
```

The window layout is:

```
┌──────────────────────────────────────────┐
│  HUD panel  │  Sky / aircraft scene      │  560 px
├──────────────────────────────────────────┤
│  Lyapunov V(t) strip  (log or linear)    │  140 px
├──────────────────────────────────────────┤
│  Time scrubber + key hints               │   40 px
└──────────────────────────────────────────┘
```

Can also be run standalone:

```bash
python visualization_aircraft.py --controller adaptive --lya-scale log
python visualization_aircraft.py --controller baseline --gif-only --lya-scale linear
```

---

## Physics reference

### Equations of motion

```
V_dot     = (1/m)   [ T cos α − D − mg sin γ ]
α_dot     = q − (1/mV) [ T sin α + L − mg cos γ ]
q_dot     = M / I_y
θ_dot     = q

γ = θ − α   (flight-path angle)
```

### Aerodynamic coefficients

```
C_L = C_L0 + C_Lα · α + C_Lδe · δe
C_D = C_D0 + (C_Lα · α + C_Lδe · δe)² / (π · e · AR)
C_m = C_m0 + C_mα · α + C_mq · (q c̄)/(2V) + C_mδe · δe
```

### Icing model

At `t ≥ t_ice`:  `C_Lα → C_Lα + ΔC_Lα`  where `ΔC_Lα < 0`

The default severity of 30 % gives `ΔC_Lα = −1.05` (from `C_Lα = 3.5`).

### Control law

```
r     = e_q + λ_α · e_α                     (filtered error)
δe    = (−F(s) − k_r · r − Y(s) · ΔĈ_Lα) / B(s)
ΔĈ_Lα_dot = γ_C · Y(s) · r               (adaptation law)
```

The functions `F(s)`, `B(s)`, `Y(s)` depend on the current state and capture the nominal pitch dynamics, elevator authority, and icing regression term respectively.

### Lyapunov stability

The candidate function `V = ½r² + (1/2γ_C) ΔC̃_Lα²` satisfies `V̇ ≤ −k_r r²` under the given control law (in the absence of elevator saturation).  This guarantees that `r → 0` and `ΔĈ_Lα → ΔC_Lα*` asymptotically after the adaptive mode is triggered.