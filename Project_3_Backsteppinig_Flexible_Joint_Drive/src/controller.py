"""
controller.py
-------------
Torque-shaped backstepping controller for the nonlinear flexible-joint drive.

Design outline (four-step recursive Lyapunov construction)
----------------------------------------------------------
Step 1 – Load position error
    e₁ = θ_l − θ_d
    Virtual control: ω_l desired → α₁(x, θ_d, θ̇_d)

Step 2 – Load velocity error
    e₂ = ω_l − α₁
    Virtual control: τ_c desired → α₂(x, ref)

Step 3 – Coupling torque error
    e₃ = τ_c − α₂
    Virtual control: ω_m desired (implicitly through δ̇) → derived from
    the τ̇_c dynamics.

Step 4 – Motor velocity equation
    Synthesise u so that τ̇_c tracks α̇₂, closing the loop.

The controller also exposes a *tuning dataclass* so gains can be changed
from main.py without touching the algorithm.
"""

from __future__ import annotations
import numpy as np
from dataclasses import dataclass
from system import FlexibleJointSystem, SystemParams


@dataclass
class ControllerGains:
    """Backstepping gain set."""
    k1: float = 5.0    # c₁ — position-error damping
    k2: float = 8.0    # c₂ — load-velocity error damping
    k3: float = 25.0   # c₃ — coupling-torque error damping


class BacksteppingController:
    """
    Four-step backstepping controller for

        ẋ₁ = x₂
        ẋ₂ = (1/J_l)[τ_c − T_{f,l}]
        ẋ₃ = x₄
        ẋ₄ = (1/J_m)[u − τ_c − T_{f,m}]

    The design treats τ_c as a *virtual* control input, following the
    torque-shaped backstepping cascade described in the model document.

    Reference trajectory
    --------------------
    The controller tracks a smooth scalar reference θ_d(t) together with
    its first two time-derivatives (θ̇_d, θ̈_d).  If the caller omits
    higher-order derivatives they default to zero.
    """

    def __init__(self,
                 system: FlexibleJointSystem,
                 gains: ControllerGains | None = None):
        self.sys   = system
        self.gains = gains or ControllerGains()

        # Cached intermediate quantities (updated by compute())
        self._alpha1: float = 0.0   # desired ω_l
        self._alpha2: float = 0.0   # desired τ_c
        self._e1: float = 0.0
        self._e2: float = 0.0
        self._e3: float = 0.0
        self._lyapunov: float = 0.0

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def compute(self,
                t:          float,
                x:          np.ndarray,
                theta_d:    float,
                dtheta_d:   float = 0.0,
                ddtheta_d:  float = 0.0,
                dddtheta_d: float = 0.0) -> float:
        """
        Compute motor torque command u.

        Parameters
        ----------
        t            : current time   [s]
        x            : state [θ_l, ω_l, θ_m, ω_m]
        theta_d      : desired load angle           [rad]
        dtheta_d     : desired load velocity        [rad/s]
        ddtheta_d    : desired load acceleration    [rad/s²]
        dddtheta_d   : desired load jerk            [rad/s³]
                       Required for exact feedforward in dalpha2_dt.
                       Defaults to 0 (exact only for equilibrium reference).

        Returns
        -------
        u : motor torque command [N·m]
        """
        p  = self.sys.p
        g  = self.gains

        # ── convenience aliases ──────────────────────────────────────
        theta_l, omega_l, theta_m, omega_m = x
        delta = float(x[2] - x[0])
        nu    = float(x[3] - x[1])

        tau_c = self.sys.coupling_torque(x)
        Tfl   = self.sys.friction_torque_load(omega_l)
        Tfm   = self.sys.friction_torque_motor(omega_m)

        # Friction Jacobians (∂T_f/∂ω)
        dTfl_domega_l = (p.Fc_l / p.v_s * (1.0 / np.cosh(omega_l / p.v_s))**2
                         + p.Bv_l)
        dTfm_domega_m = (p.Fc_m / p.v_s * (1.0 / np.cosh(omega_m / p.v_s))**2
                         + p.Bv_m)

        # ── STEP 1 – position error ───────────────────────────────────
        e1 = theta_l - theta_d
        alpha1     = dtheta_d - g.k1 * e1
        # α̇₁ = θ̈_d - c₁(ω_l - θ̇_d)
        dalpha1_dt = ddtheta_d - g.k1 * (omega_l - dtheta_d)
        # α̈₁ = θ⃛_d - c₁(ω̇_l - θ̈_d)   — requires dddtheta_d and ω̇_l
        omega_l_dot  = (tau_c - Tfl) / p.J_l          # ẋ₂
        ddalpha1_dt  = dddtheta_d - g.k1 * (omega_l_dot - ddtheta_d)

        # ── STEP 2 – load-velocity error ─────────────────────────────
        e2 = omega_l - alpha1
        alpha2 = (p.J_l * (dalpha1_dt - g.k2 * e2 - e1) + Tfl)

        # α̇₂ = J_l·(α̈₁ - c₂·ė₂ - ė₁) + (∂T_{f,l}/∂ω_l)·ω̇_l
        # ė₁ = ω_l - θ̇_d
        # ė₂ = ω̇_l - α̇₁
        de1_dt     = omega_l - dtheta_d
        de2_dt     = omega_l_dot - dalpha1_dt
        dalpha2_dt = (p.J_l * (ddalpha1_dt - g.k2 * de2_dt - de1_dt)
                      + dTfl_domega_l * omega_l_dot)

        # ── STEP 3 – coupling-torque error ───────────────────────────
        e3 = tau_c - alpha2

        kappa = p.k + 3.0 * p.k3 * delta**2
        Phi = (kappa * nu
               + p.b * ((-tau_c - Tfm) / p.J_m
                         - (tau_c - Tfl) / p.J_l))

        u = (p.J_m / p.b) * (dalpha2_dt - Phi - g.k3 * e3 - e2 / p.J_l)

        # ── Store for introspection ───────────────────────────────────
        self._e1 = e1
        self._e2 = e2
        self._e3 = e3
        self._alpha1 = alpha1
        self._alpha2 = alpha2
        self._lyapunov = self.lyapunov_value(x, theta_d, dtheta_d, ddtheta_d)

        return float(u)

    # ------------------------------------------------------------------
    # Lyapunov function V(e₁, e₂, e₃)
    # ------------------------------------------------------------------

    def lyapunov_value(self,
                       x: np.ndarray,
                       theta_d: float,
                       dtheta_d: float = 0.0,
                       ddtheta_d: float = 0.0) -> float:
        """
        Composite Lyapunov function V = ½z₁² + ½z₂² + ½z₃²

        Uses the same alpha1, alpha2 expressions as compute() so that
        the reported V is exactly the Lyapunov function being decreased
        by the control law.  Requires ddtheta_d to evaluate dalpha1_dt
        correctly for time-varying references.
        """
        g  = self.gains
        p  = self.sys.p

        e1     = x[0] - theta_d
        alpha1 = dtheta_d - g.k1 * e1
        e2     = x[1] - alpha1

        tau_c      = self.sys.coupling_torque(x)
        Tfl        = self.sys.friction_torque_load(x[1])
        dalpha1_dt = ddtheta_d - g.k1 * (x[1] - dtheta_d)   # correct: includes θ̈_d
        alpha2     = p.J_l * (dalpha1_dt - g.k2 * e2 - e1) + Tfl
        e3         = tau_c - alpha2

        return 0.5 * e1**2 + 0.5 * e2**2 + 0.5 * e3**2

    # ------------------------------------------------------------------
    # Read-only properties for logging
    # ------------------------------------------------------------------

    @property
    def errors(self):
        return self._e1, self._e2, self._e3

    @property
    def virtual_controls(self):
        return self._alpha1, self._alpha2


# ══════════════════════════════════════════════════════════════════════
# Linear controllers  (for comparison with backstepping)
# ══════════════════════════════════════════════════════════════════════
#
# Both controllers act on the LOAD POSITION error  e = θ_l − θ_d.
# They see the full state x = [θ_l, ω_l, θ_m, ω_m] but can only use
# the load-side measurements (non-collocated: they cannot directly
# observe the motor angle through a simple PD/PID loop on the load).
#
# Design notes
# ------------
# • PD:  u = −Kp·e − Kd·ė    where ė = ω_l − ω_d
#   No integral; zero steady-state only when ω_d = const and friction = 0.
#   Fails for this plant: the cubic stiffness shifts the effective plant
#   gain with amplitude, so any fixed Kp/Kd becomes detuned.
#
# • PID: u = −Kp·e − Ki·∫e dt − Kd·ė
#   Integral term helps reject constant disturbances but causes
#   integrator windup during large transients and cannot compensate
#   the resonance introduced by the flexible shaft.
#
# Both controllers ignore the shaft dynamics entirely, which is why they
# produce oscillation or instability when the flexible coupling is present.
# ══════════════════════════════════════════════════════════════════════


@dataclass
class PDGains:
    Kp: float = 10.0    # proportional gain on θ_l error
    Kd: float = 3.0     # derivative gain  on ω_l error


@dataclass
class PIDGains:
    Kp: float = 10.0    # proportional gain
    Ki: float = 1.0     # integral gain
    Kd: float = 3.0     # derivative gain
    windup_limit: float = 50.0   # anti-windup clamp on integral state


class PDController:
    """
    Proportional-Derivative controller on load position.

        u = −Kp·(θ_l − θ_d) − Kd·(ω_l − ω̇_d)

    Ignores motor-side dynamics and shaft flexibility entirely.
    Provided as a baseline to show why linear control is inadequate
    for this non-collocated flexible-joint system.
    """

    def __init__(self,
                 system: "FlexibleJointSystem",
                 gains: PDGains | None = None) -> None:
        self.sys   = system
        self.gains = gains or PDGains()
        self._e1: float = 0.0
        self._e2: float = 0.0
        self._e3: float = 0.0

    def compute(self,
                t:          float,
                x:          np.ndarray,
                theta_d:    float,
                dtheta_d:   float = 0.0,
                ddtheta_d:  float = 0.0,
                dddtheta_d: float = 0.0) -> float:
        g = self.gains
        e  = x[0] - theta_d          # position error
        de = x[1] - dtheta_d         # velocity error
        u  = -g.Kp * e - g.Kd * de

        # Store compatible error fields (e3 = 0 for PD)
        self._e1 = e
        self._e2 = de
        self._e3 = 0.0
        return float(u)

    def lyapunov_value(self, x, theta_d, dtheta_d=0.0, ddtheta_d=0.0) -> float:
        """Use ½e₁² as a stand-in Lyapunov proxy for comparison plots."""
        return 0.5 * (x[0] - theta_d) ** 2

    @property
    def errors(self):
        return self._e1, self._e2, self._e3

    @property
    def virtual_controls(self):
        return 0.0, 0.0


class PIDController:
    """
    Proportional-Integral-Derivative controller on load position.

        u = −Kp·(θ_l − θ_d) − Ki·∫(θ_l−θ_d)dt − Kd·(ω_l − ω̇_d)

    Includes anti-windup clamping on the integral state.
    Ignores motor-side dynamics and shaft flexibility entirely.
    Provided as a baseline to show why linear control is inadequate
    for this non-collocated flexible-joint system.
    """

    def __init__(self,
                 system: "FlexibleJointSystem",
                 gains: PIDGains | None = None) -> None:
        self.sys    = system
        self.gains  = gains or PIDGains()
        self._i_err: float = 0.0      # integral accumulator
        self._prev_t: float | None = None
        self._e1: float = 0.0
        self._e2: float = 0.0
        self._e3: float = 0.0

    def reset(self) -> None:
        self._i_err  = 0.0
        self._prev_t = None

    def compute(self,
                t:          float,
                x:          np.ndarray,
                theta_d:    float,
                dtheta_d:   float = 0.0,
                ddtheta_d:  float = 0.0,
                dddtheta_d: float = 0.0) -> float:
        g  = self.gains
        e  = x[0] - theta_d
        de = x[1] - dtheta_d

        # Integrate with trapezoidal rule
        if self._prev_t is not None:
            dt = max(t - self._prev_t, 0.0)
            self._i_err += e * dt
            # Anti-windup clamp
            self._i_err = float(np.clip(self._i_err,
                                        -g.windup_limit, g.windup_limit))
        self._prev_t = t

        u = -g.Kp * e - g.Ki * self._i_err - g.Kd * de

        self._e1 = e
        self._e2 = de
        self._e3 = self._i_err
        return float(u)

    def lyapunov_value(self, x, theta_d, dtheta_d=0.0, ddtheta_d=0.0) -> float:
        """Use ½e₁² as a stand-in Lyapunov proxy for comparison plots."""
        return 0.5 * (x[0] - theta_d) ** 2

    @property
    def errors(self):
        return self._e1, self._e2, self._e3

    @property
    def virtual_controls(self):
        return 0.0, 0.0