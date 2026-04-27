"""
simulation.py
-------------
Scenario definition and main simulation loop.

Scenario
--------
  t ∈ [0, 30] s,  dt = 0.01 s
  Constant alpha_ref = alpha_trim (4 deg).
  At t = 10 s icing begins: C_La drops by 30 % (ΔC_La = -1.05).
  Throttle is fixed at trim value throughout.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from typing import Optional

from system     import AircraftSystem
from controller import LyapunovIcingAdaptiveController


@dataclass
class SimConfig:
    t_start:             float = 0.0
    t_end:               float = 200.0
    dt:                  float = 0.01

    V_trim:              float = 80.0
    alpha_trim:          float = np.deg2rad(4.0)

    t_ice:               float = 10.0
    delta_CL_alpha_ice:  float = -1.05    # 30 % of C_La=3.5

    delta_t:             Optional[float] = None   # set from trim if None
    alpha_ref:           Optional[float] = None   # set to alpha_trim if None


def run_simulation(
    config:            SimConfig | None = None,
    aircraft_params:   dict | None = None,
    controller_params: dict | None = None,
) -> dict:
    """
    Run the full closed-loop simulation.

    Returns a flat dict of time-series arrays.
    """
    cfg = config or SimConfig()

    aircraft   = AircraftSystem(
        params             = aircraft_params,
        t_ice              = cfg.t_ice,
        delta_CL_alpha_ice = cfg.delta_CL_alpha_ice,
    )
    controller = LyapunovIcingAdaptiveController(params=controller_params)

    # ---- Trim ----------------------------------------------------------
    de_trim, dt_trim, theta_trim = aircraft.trim(cfg.V_trim, cfg.alpha_trim)

    alpha_ref = cfg.alpha_ref if cfg.alpha_ref is not None else cfg.alpha_trim
    delta_t   = cfg.delta_t   if cfg.delta_t   is not None else dt_trim

    # Verify trim quality
    xdot = aircraft.trim_check(cfg.V_trim, cfg.alpha_trim, de_trim, delta_t, theta_trim)
    print(f"[Trim]  V={cfg.V_trim:.1f} m/s, alpha={np.rad2deg(cfg.alpha_trim):.2f} deg")
    print(f"        delta_e={np.rad2deg(de_trim):.3f} deg, delta_t={delta_t:.4f}")
    print(f"        EOM residual: V_dot={xdot[0]:.5f}, alpha_dot={np.rad2deg(xdot[1]):.5f} deg/s, "
          f"q_dot={np.rad2deg(xdot[2]):.5f} deg/s²")
    print(f"[Icing] onset t={cfg.t_ice}s, "
          f"C_La: {aircraft.C_La_clean:.3f} → {aircraft.C_La_iced:.3f} "
          f"(ΔC_La={cfg.delta_CL_alpha_ice:.3f})")
    print(f"[Ref]   alpha_ref = {np.rad2deg(alpha_ref):.2f} deg (constant)")
    print()

    # ---- Initial state -------------------------------------------------
    x = np.array([cfg.V_trim, cfg.alpha_trim, 0.0, theta_trim])

    # ---- Time grid -----------------------------------------------------
    t_arr = np.arange(cfg.t_start, cfg.t_end + cfg.dt / 2, cfg.dt)
    N     = len(t_arr)
    
    # ---- Allocate logs -------------------------------------------------
# ---- Allocate logs -------------------------------------------------
    keys = [
        "V", "alpha", "q", "theta",
        "delta_e", "delta_e_nom", "delta_e_adapt",
        "e_alpha", "e_q", "r",
        "delta_CL_alpha_hat",

        # Reduced r-dynamics terms
        "F", "B", "Y",

        # Components
        "f_q", "b_q",
        "f_alpha", "b_alpha", "Y_alpha",
    ]

    logs = {k: np.zeros(N) for k in keys}
    logs["adaptive_mode"]  = np.zeros(N, dtype=bool)
    logs["detect_counter"] = np.zeros(N, dtype=int)
    logs["t"]              = t_arr

    # ---- Loop ----------------------------------------------------------
    ref = {"alpha_ref": alpha_ref, "q_ref": 0.0}

    for k, t in enumerate(t_arr):
        V, alpha, q, theta = x

        state = {
            "V": V,
            "alpha": alpha,
            "q": q,
            "theta": theta,
            "delta_t": delta_t,
        }

        delta_e, info = controller.compute_control(state, ref, cfg.dt)

        logs["V"][k]                  = V
        logs["alpha"][k]              = alpha
        logs["q"][k]                  = q
        logs["theta"][k]              = theta

        logs["delta_e"][k]            = delta_e
        logs["delta_e_nom"][k]        = info["delta_e_nom"]
        logs["delta_e_adapt"][k]      = info["delta_e_adapt"]

        logs["e_alpha"][k]            = info["e_alpha"]
        logs["e_q"][k]                = info["e_q"]
        logs["r"][k]                  = info["r"]

        logs["delta_CL_alpha_hat"][k] = info["delta_CL_alpha_hat"]
        logs["adaptive_mode"][k]      = info["adaptive_mode"]
        logs["detect_counter"][k]     = info["detect_counter"]

        logs["F"][k]                  = info["F"]
        logs["B"][k]                  = info["B"]
        logs["Y"][k]                  = info["Y"]

        logs["f_q"][k]                = info["f_q"]
        logs["b_q"][k]                = info["b_q"]

        logs["f_alpha"][k]            = info["f_alpha"]
        logs["b_alpha"][k]            = info["b_alpha"]
        logs["Y_alpha"][k]            = info["Y_alpha"]

        x = aircraft.step(t, x, delta_e, delta_t, cfg.dt)

    # ---- Metadata ------------------------------------------------------
    logs["alpha_ref"]   = np.full(N, alpha_ref)
    logs["t_ice"]       = cfg.t_ice
    logs["C_La_clean"]  = aircraft.C_La_clean
    logs["C_La_iced"]   = aircraft.C_La_iced

    return logs