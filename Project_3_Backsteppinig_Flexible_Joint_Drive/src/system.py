"""
system.py
---------
Nonlinear Flexible-Joint Drive system model.

State vector:  x = [θ_l, ω_l, θ_m, ω_m]^T
  x[0] = θ_l  – load angular position  [rad]
  x[1] = ω_l  – load angular velocity  [rad/s]
  x[2] = θ_m  – motor angular position [rad]
  x[3] = ω_m  – motor angular velocity [rad/s]

Control input: u = τ_m  [N·m]  (motor electromagnetic torque)
"""

from __future__ import annotations
import numpy as np
from dataclasses import dataclass, field


@dataclass
class SystemParams:
    """Physical parameters of the flexible-joint drive."""

    # Inertias [kg·m²]
    J_l: float = 0.5    # load inertia
    J_m: float = 0.1    # motor inertia

    # Coupling parameters
    k:   float = 50.0   # linear torsional stiffness  [N·m/rad]
    k3:  float = 20.0   # cubic hardening coefficient  [N·m/rad³]
    b:   float = 1.0    # structural damping           [N·m·s/rad]

    # Friction parameters (shared model shape; separate amplitudes)
    Fc_l: float = 0.3   # Coulomb level – load side  [N·m]
    Fc_m: float = 0.2   # Coulomb level – motor side [N·m]
    v_s:  float = 0.1   # Stribeck smoothing velocity [rad/s]
    Bv_l: float = 0.05  # viscous coefficient – load  [N·m·s/rad]
    Bv_m: float = 0.03  # viscous coefficient – motor [N·m·s/rad]

    # Disturbance bounds (used for robustness analysis)
    d_l_max: float = 0.0   # max normalised load disturbance  [rad/s²]
    d_m_max: float = 0.0   # max normalised motor disturbance [rad/s²]


class FlexibleJointSystem:
    """
    Nonlinear flexible-joint drive.

    Equations of motion (disturbance-free):
        ẋ₁ = x₂
        ẋ₂ = (1/J_l)[τ_c(δ,ν) - T_{f,l}(x₂)]
        ẋ₃ = x₄
        ẋ₄ = (1/J_m)[u - τ_c(δ,ν) - T_{f,m}(x₄)]

    where
        δ      = x₃ − x₁   (shaft twist)
        ν      = x₄ − x₂   (relative angular velocity)
        τ_c    = k·δ + k₃·δ³ + b·ν
        T_f(ω) = Fc·tanh(ω/v_s) + Bv·ω
    """

    def __init__(self, params: SystemParams | None = None):
        self.p = params or SystemParams()

    # ------------------------------------------------------------------
    # Elementary physical quantities
    # ------------------------------------------------------------------

    def shaft_twist(self, x: np.ndarray) -> float:
        """δ = θ_m − θ_l."""
        return float(x[2] - x[0])

    def relative_velocity(self, x: np.ndarray) -> float:
        """ν = ω_m − ω_l."""
        return float(x[3] - x[1])

    def coupling_torque(self, x: np.ndarray) -> float:
        """τ_c(δ,ν) = k·δ + k₃·δ³ + b·ν  [N·m]."""
        p = self.p
        delta = self.shaft_twist(x)
        nu    = self.relative_velocity(x)
        return p.k * delta + p.k3 * delta**3 + p.b * nu

    def friction_torque_load(self, omega_l: float) -> float:
        """T_{f,l}(ω_l) = Fc_l·tanh(ω_l/v_s) + Bv_l·ω_l  [N·m]."""
        p = self.p
        return p.Fc_l * np.tanh(omega_l / p.v_s) + p.Bv_l * omega_l

    def friction_torque_motor(self, omega_m: float) -> float:
        """T_{f,m}(ω_m) = Fc_m·tanh(ω_m/v_s) + Bv_m·ω_m  [N·m]."""
        p = self.p
        return p.Fc_m * np.tanh(omega_m / p.v_s) + p.Bv_m * omega_m

    def elastic_potential(self, x: np.ndarray) -> float:
        """U(δ) = ½·k·δ² + ¼·k₃·δ⁴  [J]."""
        p = self.p
        delta = self.shaft_twist(x)
        return 0.5 * p.k * delta**2 + 0.25 * p.k3 * delta**4

    def total_energy(self, x: np.ndarray) -> float:
        """E(x) = ½J_l·ω_l² + ½J_m·ω_m² + U(δ)  [J]."""
        p = self.p
        return (0.5 * p.J_l * x[1]**2
                + 0.5 * p.J_m * x[3]**2
                + self.elastic_potential(x))

    # ------------------------------------------------------------------
    # State-space dynamics
    # ------------------------------------------------------------------

    def dynamics(self,
                 t: float,
                 x: np.ndarray,
                 u: float,
                 d_l: float = 0.0,
                 d_m: float = 0.0) -> np.ndarray:
        """
        Compute ẋ = f(x) + g·u + d(t).

        Parameters
        ----------
        t   : current time (unused in nominal model, kept for ODE interface)
        x   : state vector [θ_l, ω_l, θ_m, ω_m]
        u   : motor torque [N·m]
        d_l : normalised load disturbance [rad/s²]
        d_m : normalised motor disturbance [rad/s²]

        Returns
        -------
        dx : time derivative of state [rad/s, rad/s², rad/s, rad/s²]
        """
        p   = self.p
        tau_c = self.coupling_torque(x)
        Tfl   = self.friction_torque_load(x[1])
        Tfm   = self.friction_torque_motor(x[3])

        dx = np.empty(4)
        dx[0] = x[1]
        dx[1] = (tau_c - Tfl) / p.J_l + d_l
        dx[2] = x[3]
        dx[3] = (u - tau_c - Tfm) / p.J_m + d_m
        return dx

    # ------------------------------------------------------------------
    # Linearisation (for analysis / comparison with linear methods)
    # ------------------------------------------------------------------

    def linearise(self, x_eq: np.ndarray, u_eq: float = 0.0):
        """
        Return (A, B) Jacobians at equilibrium (x_eq, u_eq).

        Uses central finite differences so no symbolic toolbox is required.
        """
        n  = 4
        eps = 1e-6
        A  = np.zeros((n, n))
        for i in range(n):
            xp, xm = x_eq.copy(), x_eq.copy()
            xp[i] += eps; xm[i] -= eps
            fp = self.dynamics(0.0, xp, u_eq)
            fm = self.dynamics(0.0, xm, u_eq)
            A[:, i] = (fp - fm) / (2 * eps)

        B = np.zeros(n)
        fp = self.dynamics(0.0, x_eq, u_eq + eps)
        fm = self.dynamics(0.0, x_eq, u_eq - eps)
        B  = (fp - fm) / (2 * eps)

        return A, B.reshape(-1, 1)