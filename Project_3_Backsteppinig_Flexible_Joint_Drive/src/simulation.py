"""
simulation.py
-------------
Functions to run closed-loop and open-loop simulations of the
flexible-joint drive.

All simulation results are returned as SimResult dataclasses so that
visualization.py can consume them without any knowledge of the ODE solver.
"""

from __future__ import annotations
import numpy as np
from dataclasses import dataclass, field
from scipy.integrate import solve_ivp

from system     import FlexibleJointSystem, SystemParams
from controller import (BacksteppingController, ControllerGains,
                        PDController, PDGains,
                        PIDController, PIDGains)


# ──────────────────────────────────────────────────────────────────────
# Reference trajectory generators
# ──────────────────────────────────────────────────────────────────────

def step_reference(t: float,
                   amplitude: float = 1.0,
                   t_step: float = 0.5) -> tuple[float, float, float, float]:
    """Heaviside step; derivatives are zero (discontinuous – use for testing)."""
    theta = amplitude if t >= t_step else 0.0
    return theta, 0.0, 0.0, 0.0


def smooth_step_reference(t: float,
                           amplitude: float = 1.0,
                           t_rise: float    = 1.0,
                           t_start: float   = 0.5) -> tuple[float, float, float, float]:
    """
    Smooth sigmoid transition:  θ_d(t) = A·σ((t−t_start)·8/t_rise)
    where σ is the logistic function.  Provides C^∞ reference with
    bounded third derivative.
    """
    s      = (t - t_start) * 8.0 / t_rise
    sigma  = 1.0 / (1.0 + np.exp(-s))
    q      = sigma * (1.0 - sigma)          # σ' / ds_dt

    ds     = 8.0 / t_rise
    theta   = amplitude * sigma
    dtheta  = amplitude * q * ds
    ddtheta = amplitude * q * (1.0 - 2.0 * sigma) * ds**2
    # third derivative: d/dt[ddtheta] = A·ds³·q·(1 - 6σ(1-σ))·... 
    # = A·ds³·(q - 6q²)(1-2σ) ... derive cleanly:
    # Let p = σ(1-σ), then dp/dt = p(1-2σ)ds
    # ddtheta = A·ds²·p·(1-2σ)
    # dddtheta = A·ds³·[p(1-2σ)² + p·(-2)·(1-2σ)²... ]
    # Full result: A·ds³·p·(1 - 6p)
    dddtheta = amplitude * q * (1.0 - 6.0 * q) * ds**3
    return theta, dtheta, ddtheta, dddtheta


def equilibrium_reference(t: float,
                          setpoint: float = 1.0) -> tuple[float, float, float, float]:
    """Constant setpoint reference — all derivatives zero."""
    return setpoint, 0.0, 0.0, 0.0


def sinusoidal_reference(t: float,
                          amplitude: float = 1.0,
                          frequency: float = 0.5) -> tuple[float, float, float, float]:
    """Sinusoidal tracking reference including third derivative."""
    omega   = 2.0 * np.pi * frequency
    theta   =  amplitude * np.sin(omega * t)
    dtheta  =  amplitude * omega       * np.cos(omega * t)
    ddtheta = -amplitude * omega**2    * np.sin(omega * t)
    dddtheta= -amplitude * omega**3    * np.cos(omega * t)
    return theta, dtheta, ddtheta, dddtheta


# ──────────────────────────────────────────────────────────────────────
# Result container
# ──────────────────────────────────────────────────────────────────────

@dataclass
class SimResult:
    """Container for a single simulation run."""
    t:         np.ndarray           # time vector             [N]
    x:         np.ndarray           # states                  [N×4]
    u:         np.ndarray           # control input           [N]
    theta_d:   np.ndarray           # reference position      [N]
    dtheta_d:  np.ndarray           # reference velocity      [N]
    lyapunov:  np.ndarray           # Lyapunov value V(t)     [N]
    e1:        np.ndarray           # position error          [N]
    e2:        np.ndarray           # velocity error          [N]
    e3:        np.ndarray           # torque error            [N]
    tau_c:     np.ndarray           # coupling torque         [N]
    energy:    np.ndarray           # total mechanical energy [N]
    label:     str = "simulation"


# ──────────────────────────────────────────────────────────────────────
# Core simulation runner
# ──────────────────────────────────────────────────────────────────────

def run_simulation(
        sys_params:      SystemParams      | None = None,
        ctrl_gains:      ControllerGains   | None = None,
        pd_gains:        PDGains           | None = None,
        pid_gains:       PIDGains          | None = None,
        controller_type: str                      = "backstepping",
        x0:              np.ndarray        | None = None,
        t_span:          tuple[float,float]       = (0.0, 10.0),
        dt:              float                    = 1e-3,
        reference:       str                      = "smooth_step",
        ref_kwargs:      dict                     | None = None,
        u_clip:          float                    = 100.0,
        label:           str                      = "simulation",
) -> SimResult:
    """
    Run a closed-loop simulation.

    Parameters
    ----------
    sys_params      : system physical parameters (default if None)
    ctrl_gains      : backstepping gains (used when controller_type='backstepping')
    pd_gains        : PD gains           (used when controller_type='pd')
    pid_gains       : PID gains          (used when controller_type='pid')
    controller_type : 'backstepping' | 'pd' | 'pid'
    x0              : initial state [θ_l, ω_l, θ_m, ω_m] (zeros if None)
    t_span          : (t_start, t_end) [s]
    dt              : output sample interval [s]
    reference       : one of {'smooth_step', 'step', 'sinusoidal'}
    ref_kwargs      : keyword arguments forwarded to reference generator
    u_clip          : torque saturation limit [N·m]
    label           : human-readable tag for plots

    Returns
    -------
    SimResult
    """
    system = FlexibleJointSystem(sys_params)

    ctype = controller_type.lower()
    if ctype == "backstepping":
        ctrl = BacksteppingController(system, ctrl_gains)
    elif ctype == "pd":
        ctrl = PDController(system, pd_gains)
    elif ctype == "pid":
        ctrl = PIDController(system, pid_gains)
    else:
        raise ValueError(f"Unknown controller_type '{controller_type}'. "
                         f"Choose from 'backstepping', 'pd', 'pid'.")

    ref_kw = ref_kwargs or {}

    if x0 is None:
        x0 = np.zeros(4)

    # Choose reference generator
    _ref_map = {
        "smooth_step":  smooth_step_reference,
        "step":         step_reference,
        "sinusoidal":   sinusoidal_reference,
        "equilibrium":  equilibrium_reference,
    }
    if reference not in _ref_map:
        raise ValueError(f"Unknown reference '{reference}'. "
                         f"Choose from {list(_ref_map)}")
    ref_fn = _ref_map[reference]

    # Build dense output time vector
    t_eval = np.arange(t_span[0], t_span[1] + dt, dt)
    N = len(t_eval)

    # Allocate history arrays
    x_hist    = np.zeros((N, 4))
    u_hist    = np.zeros(N)
    td_hist   = np.zeros(N)
    dtd_hist  = np.zeros(N)
    lya_hist  = np.zeros(N)
    e1_hist   = np.zeros(N)
    e2_hist   = np.zeros(N)
    e3_hist   = np.zeros(N)
    tauc_hist = np.zeros(N)
    ener_hist = np.zeros(N)

    # ── integrate with fixed-step RK45 (scipy dense output) ──────────
    x_cur = x0.copy()
    x_hist[0] = x_cur

    for i, t in enumerate(t_eval):
        theta_d, dtheta_d, ddtheta_d, dddtheta_d = ref_fn(t, **ref_kw)

        u_raw = ctrl.compute(t, x_cur, theta_d, dtheta_d, ddtheta_d, dddtheta_d)
        u     = float(np.clip(u_raw, -u_clip, u_clip))

        # Log
        e1, e2, e3 = ctrl.errors
        u_hist[i]    = u
        td_hist[i]   = theta_d
        dtd_hist[i]  = dtheta_d
        lya_hist[i]  = ctrl.lyapunov_value(x_cur, theta_d, dtheta_d, ddtheta_d)
        e1_hist[i]   = e1
        e2_hist[i]   = e2
        e3_hist[i]   = e3
        tauc_hist[i] = system.coupling_torque(x_cur)
        ener_hist[i] = system.total_energy(x_cur)

        if i < N - 1:
            # One RK4 step
            dt_int = t_eval[i + 1] - t
            x_cur = _rk4_step(system, t, x_cur, u, dt_int)
            x_hist[i + 1] = x_cur

    return SimResult(
        t        = t_eval,
        x        = x_hist,
        u        = u_hist,
        theta_d  = td_hist,
        dtheta_d = dtd_hist,
        lyapunov = lya_hist,
        e1       = e1_hist,
        e2       = e2_hist,
        e3       = e3_hist,
        tau_c    = tauc_hist,
        energy   = ener_hist,
        label    = label,
    )


def _rk4_step(system: FlexibleJointSystem,
              t: float,
              x: np.ndarray,
              u: float,
              h: float) -> np.ndarray:
    """Classic 4th-order Runge-Kutta with fixed step h."""
    k1 = system.dynamics(t,          x,               u)
    k2 = system.dynamics(t + h/2,    x + h/2 * k1,   u)
    k3 = system.dynamics(t + h/2,    x + h/2 * k2,   u)
    k4 = system.dynamics(t + h,      x + h   * k3,   u)
    return x + (h / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


# ──────────────────────────────────────────────────────────────────────
# Convenience wrappers
# ──────────────────────────────────────────────────────────────────────

def run_step_response(**kwargs) -> SimResult:
    """Step-reference closed-loop simulation (smooth transition)."""
    kwargs.setdefault("reference", "smooth_step")
    kwargs.setdefault("label", "Step response")
    return run_simulation(**kwargs)


def run_sinusoidal_tracking(**kwargs) -> SimResult:
    """Sinusoidal tracking simulation."""
    kwargs.setdefault("reference", "sinusoidal")
    kwargs.setdefault("label", "Sinusoidal tracking")
    return run_simulation(**kwargs)


def run_gain_study(gain_sets: list[ControllerGains],
                   **sim_kwargs) -> list[SimResult]:
    """
    Run the same scenario with multiple gain sets and return all results.
    Useful for tuning visualisations.
    """
    results = []
    for i, gains in enumerate(gain_sets):
        label = sim_kwargs.pop("label", f"Gains set {i+1}")
        r = run_simulation(ctrl_gains=gains, label=label, **sim_kwargs)
        results.append(r)
        sim_kwargs["label"] = f"Gains set {i+2}"   # won't be used after first pop
    return results

def run_equilibrium_stabilisation(
        setpoint:    float = 1.0,
        x0:          np.ndarray | None = None,
        **sim_kwargs,
) -> SimResult:
    """
    Stabilise the system to a constant equilibrium θ* = setpoint.

    The initial condition defaults to a perturbed state
    (θ_l=0, ω_l=0.5, θ_m=0.3, ω_m=0) so the transient is visible.
    This is the scenario the Lyapunov proof covers: because θ̇_d = θ̈_d = 0,
    V̇ ≤ 0 is a strict global result, not just along time-varying trajectories.
    """
    if x0 is None:
        # Perturbed away from equilibrium: load at 0, motor at 0.3 rad,
        # small initial load velocity to excite the shaft dynamics
        x0 = np.array([0.0, 0.5, 0.3, 0.0])

    sim_kwargs.setdefault("reference",  "equilibrium")
    sim_kwargs.setdefault("ref_kwargs", {"setpoint": setpoint})
    sim_kwargs.setdefault("label",      f"Backstepping equilibrium θ*={setpoint:.2f}")
    sim_kwargs.setdefault("t_span",     (0.0, sim_kwargs.pop("t_end", 10.0)))
    sim_kwargs.setdefault("dt",         5e-4)
    sim_kwargs.setdefault("u_clip",     200.0)
    return run_simulation(x0=x0, **sim_kwargs)


def run_controller_comparison(
        base_kwargs: dict | None = None,
        t_end: float = 10.0,
        use_equilibrium: bool = False,
        use_sinusoidal: bool = False,
) -> list[SimResult]:
    """
    Run backstepping, PD, and PID on the same scenario.

    Parameters
    ----------
    base_kwargs      : shared kwargs (sys_params, reference, ref_kwargs, u_clip, x0, dt)
    t_end            : simulation end time [s]
    use_equilibrium  : constant-setpoint reference (Lyapunov scenario)
    use_sinusoidal   : sinusoidal tracking reference
    If neither flag is set the default is a smooth step trajectory.
    """
    kw = dict(base_kwargs or {})
    kw.setdefault("t_span",  (0.0, t_end))
    kw.setdefault("dt",      5e-4)
    kw.setdefault("u_clip",  200.0)

    if use_equilibrium:
        kw.setdefault("reference",  "equilibrium")
        kw.setdefault("ref_kwargs", {"setpoint": 1.0})
        kw.setdefault("x0",         np.array([0.0, 0.5, 0.3, 0.0]))
    elif use_sinusoidal:
        kw.setdefault("reference",  "sinusoidal")
        kw.setdefault("ref_kwargs", {"amplitude": 0.8, "frequency": 0.4})
    else:
        kw.setdefault("reference",  "smooth_step")

    results = []
    for ctype, lbl in [("pd",          "PD"),
                        ("pid",         "PID"),
                        ("backstepping","Backstepping")]:
        r = run_simulation(controller_type=ctype, label=lbl, **kw)
        results.append(r)
    return results