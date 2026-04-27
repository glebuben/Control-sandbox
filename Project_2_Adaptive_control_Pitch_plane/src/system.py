"""
system.py
---------
Nonlinear longitudinal aircraft dynamics.

State vector:  x = [V, alpha, q, theta]
Control input: u = [delta_e, delta_t]

Equations of motion (body-axis, flat Earth, gamma = theta - alpha):

    V_dot     = (1/m)  [ T*cos(alpha) - D - m*g*sin(gamma) ]
    alpha_dot = q - (1/(m*V)) [ T*sin(alpha) + L - m*g*cos(gamma) ]
    q_dot     = (1/I_y) [ 0.5*rho*V^2*S*c_bar * C_m ]
    theta_dot = q

Aerodynamic coefficients:
    C_L = C_L0 + C_La*alpha + C_Lde*delta_e
    C_D = C_D0 + (C_La*alpha + C_Lde*delta_e)^2 / (pi*e*AR)
    C_m = C_m0 + C_ma*alpha + C_mq*(q*c_bar)/(2*V) + C_mde*delta_e

Icing model:
    At t >= t_ice, C_La is reduced by |delta_CL_alpha_ice|.

Parameters match the notebook (Section 15):
    I_y0=1800, C_m0=0.02, C_ma=-0.6, C_mq=-8.0, C_mde=-1.1, rho=1.225, S=16.2, c_bar=1.5
"""

import numpy as np


DEFAULT_AIRCRAFT_PARAMS = {
    # Geometry
    "m":       1200.0,
    "S":       16.2,        # notebook
    "c_bar":   1.5,         # notebook
    "AR":      7.32,
    "e":       0.81,
    "I_y":     1800.0,      # notebook I_y0

    # Engine
    "T_max":   3000.0,

    # Atmosphere
    "rho":     1.225,       # notebook
    "g":       9.81,

    # Aerodynamics (clean)
    # C_L_trim at V=60, alpha=4deg: m*g/(0.5*rho*V^2*S) = 1200*9.81/(0.5*1.225*3600*16.2) = 0.328
    # C_L0 = 0.328 - 3.5*sin(4deg) ~ 0.084
    "C_L0":    0.0901,  # analytically derived for exact alpha_dot=0 trim
    "C_La":    3.50,
    "C_Lde":   0.35,
    "C_D0":    0.027,

    # Pitching moment — exact notebook values
    "C_m0":    0.02,
    "C_ma":   -0.60,
    "C_mq":   -8.00,
    "C_mde":  -1.10,
}


class AircraftSystem:
    """
    Nonlinear longitudinal aircraft dynamics with RK4 integration.

    Parameters
    ----------
    params              : dict  — overrides to DEFAULT_AIRCRAFT_PARAMS
    t_ice               : float | None  — icing onset time [s]
    delta_CL_alpha_ice  : float  — ΔC_La (negative), e.g. -1.05 for 30% drop
    """

    def __init__(self, params=None, t_ice=None, delta_CL_alpha_ice=-1.05):
        self.p = {**DEFAULT_AIRCRAFT_PARAMS, **(params or {})}
        self.t_ice = t_ice
        self.delta_CL_alpha_ice = delta_CL_alpha_ice
        self._C_La_iced = self.p["C_La"] + delta_CL_alpha_ice

    def _C_La(self, t):
        if self.t_ice is not None and t >= self.t_ice:
            return self._C_La_iced
        return self.p["C_La"]

    def _aero(self, alpha, q, delta_e, V, t):
        p = self.p
        V_s = max(V, 1.0)
        C_La = self._C_La(t)

        C_L = p["C_L0"] + C_La * alpha + p["C_Lde"] * delta_e
        C_D = p["C_D0"] + (C_La * alpha + p["C_Lde"] * delta_e) ** 2 / (
            np.pi * p["e"] * p["AR"])
        C_m = (p["C_m0"]
               + p["C_ma"] * alpha
               + p["C_mq"] * (q * p["c_bar"]) / (2.0 * V_s)
               + p["C_mde"] * delta_e)
        return C_L, C_D, C_m

    def _eom(self, t, x, delta_e, delta_t):
        p = self.p
        V, alpha, q, theta = x
        V = max(V, 1.0)

        gamma = theta - alpha
        T     = p["T_max"] * delta_t
        q_dyn = 0.5 * p["rho"] * V**2 * p["S"]

        C_L, C_D, C_m = self._aero(alpha, q, delta_e, V, t)
        L = q_dyn * C_L
        D = q_dyn * C_D
        M = q_dyn * p["c_bar"] * C_m

        V_dot     = (T * np.cos(alpha) - D - p["m"] * p["g"] * np.sin(gamma)) / p["m"]
        alpha_dot = q - (T * np.sin(alpha) + L - p["m"] * p["g"] * np.cos(gamma)) / (p["m"] * V)
        q_dot     = M / p["I_y"]
        theta_dot = q

        return np.array([V_dot, alpha_dot, q_dot, theta_dot])

    def step(self, t, x, delta_e, delta_t, dt):
        k1 = self._eom(t,        x,             delta_e, delta_t)
        k2 = self._eom(t+dt/2,   x+dt/2*k1,    delta_e, delta_t)
        k3 = self._eom(t+dt/2,   x+dt/2*k2,    delta_e, delta_t)
        k4 = self._eom(t+dt,     x+dt*k3,       delta_e, delta_t)
        return x + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)

    def trim(self, V_trim, alpha_trim):
        """
        Find (delta_e_trim, delta_t_trim, theta_trim) for level flight.
        Uses C_m=0 to get delta_e, then V_dot=0 to get thrust.
        """
        p = self.p
        theta_trim = alpha_trim   # gamma=0 → theta=alpha

        # C_m = 0, q=0:  C_m0 + C_ma*alpha + C_mde*de = 0
        de = -(p["C_m0"] + p["C_ma"] * alpha_trim) / p["C_mde"]

        q_dyn = 0.5 * p["rho"] * V_trim**2 * p["S"]
        C_L = p["C_L0"] + p["C_La"] * alpha_trim + p["C_Lde"] * de
        C_D = p["C_D0"] + (p["C_La"] * alpha_trim + p["C_Lde"] * de)**2 / (
            np.pi * p["e"] * p["AR"])
        D = q_dyn * C_D

        T  = D / max(np.cos(alpha_trim), 0.05)
        dt = float(np.clip(T / p["T_max"], 0.0, 1.0))

        return de, dt, theta_trim

    def trim_check(self, V_trim, alpha_trim, de, dt_val, theta):
        x = np.array([V_trim, alpha_trim, 0.0, theta])
        return self._eom(0.0, x, de, dt_val)

    @property
    def C_La_clean(self):
        return self.p["C_La"]

    @property
    def C_La_iced(self):
        return self._C_La_iced