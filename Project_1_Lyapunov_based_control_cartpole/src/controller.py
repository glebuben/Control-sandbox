import numpy as np
from dataclasses import dataclass
from scipy.linalg import solve_continuous_are

from system import CartPoleSystem


def _normalize(theta: float) -> float:
    """Map theta to [-pi, pi]. theta=0 is upright."""
    return (theta + np.pi) % (2 * np.pi) - np.pi


@dataclass
class ControllerParams:
    k_energy: float = 100.0
    k_x_swing: float = 10.0
    k_damp: float = 5.0
    k_x: float = 30.0
    k_x_dot: float = 10.0
    k_theta: float = 50.0
    k_theta_dot: float = 15.0
    theta_enter: float = 0.25
    theta_exit: float = 0.50
    omega_enter: float = 2.0
    omega_exit: float = 4.0


class LyapunovController:
    """Original controller — kept for comparison."""

    def __init__(self, system: CartPoleSystem, params: ControllerParams = None):
        self.system = system
        self.params = params or ControllerParams()
        self.mode = "swing_up"

    def compute_lyapunov(self, state):
        x, x_dot, theta, theta_dot = state
        if self.mode == "swing_up":
            E = self.system.get_energy(state)
            E_des = 2 * self.system.params.m_p * self.system.params.g * self.system.params.l
            return 0.5 * (E - E_des) ** 2
        else:
            theta_err = _normalize(theta)
            return (0.5 * self.params.k_x * x ** 2 +
                    0.5 * x_dot ** 2 +
                    0.5 * self.params.k_theta * theta_err ** 2 +
                    0.5 * theta_dot ** 2)

    def compute_control(self, state):
        x, x_dot, theta, theta_dot = state
        theta_norm = _normalize(theta)

        if self.mode == "swing_up":
            if (abs(theta_norm) < self.params.theta_enter
                    and abs(theta_dot) < self.params.omega_enter):
                self.mode = "stabilization"
        elif self.mode == "stabilization":
            if (abs(theta_norm) > self.params.theta_exit
                    or abs(theta_dot) > self.params.omega_exit):
                self.mode = "swing_up"

        if abs(x) > 3.0:
            self.mode = "recover_cart"
            u = np.clip(-30.0 * x - 15.0 * x_dot,
                        -self.system.params.max_force,
                        self.system.params.max_force)
            return u, self.mode

        if self.mode == "stabilization":
            th = _normalize(theta)
            u  = -2.0 * x - 1.0 * x_dot + 10.0 * th + 2.0 * theta_dot
            return np.clip(u, -self.system.params.max_force,
                           self.system.params.max_force), self.mode
        else:
            E     = self.system.get_energy(state)
            E_des = (2 * self.system.params.m_p
                     * self.system.params.g
                     * self.system.params.l)
            u_e = self.params.k_energy * (E - E_des * 1.05) * theta_dot * np.cos(theta)
            u_c = -self.params.k_x_swing * x - self.params.k_damp * x_dot
            return np.clip(u_e + u_c,
                           -self.system.params.max_force,
                           self.system.params.max_force), self.mode


class LQRController:
    """
    Two-phase cart-pole controller.

    ── Swing-up: exact energy-based (Åström–Furuta with full nonlinear dE/dt) ──

    Energy of the pendulum (as defined in system.py):
        E = 0.5 * m_p * (l * theta_dot)^2 + m_p * g * l * (1 + cos(theta))

    Exact time derivative (derived from your EOM):
        dE/dt = (m_p * l * theta_dot * cos(theta) / delta)
                * (-u - m_p*l*theta_dot^2*sin(theta) + m_p*g*sin(theta)*cos(theta))
              = (m_p * l * theta_dot * cos(theta) / delta) * (-u + f)

    where:
        delta = (m_c + m_p) - m_p * cos^2(theta)   [from your dynamics()]
        f     = -m_p * l * theta_dot^2 * sin(theta)
                + m_p * g * sin(theta) * cos(theta)

    Lyapunov candidate:
        V = 0.5 * (E - E_up)^2

    Time derivative:
        dV/dt = (E - E_up) * dE/dt
              = (E - E_up) * (m_p*l*theta_dot*cos(theta)/delta) * (-u + f)

    To make dV/dt <= 0, choose:
        u = f + k_E * (E - E_up) * theta_dot * cos(theta)

    Then:
        dV/dt = -(m_p*l*k_E/delta) * (E-E_up)^2 * theta_dot^2 * cos^2(theta) <= 0  ✓

    ── Stabilization: LQR ──

    Linearise around theta=0, solve CARE, u = -K @ s.
    Lyapunov function: V = s' P s  (certified dV/dt <= 0).
    """

    def __init__(self,
                 system: CartPoleSystem,
                 # LQR weights
                 Q: np.ndarray = None,
                 R: float = 1.0,
                 # Swing-up
                 k_energy: float = 50.0,
                 k_center: float = 0.5,
                 k_center_d: float = 0.2,
                 # Hysteresis
                 theta_enter: float = 0.3,
                 omega_enter: float = 2.0,
                 theta_exit: float = 0.8,
                 omega_exit: float = 6.0):

        self.sys   = system
        p          = system.params
        self.m_c   = p.m_c
        self.m_p   = p.m_p
        self.l     = p.l
        self.g     = p.g
        self.u_max = p.max_force

        # Desired energy at upright (theta=0, theta_dot=0):
        #   E = 0.5*m_p*(l*0)^2 + m_p*g*l*(1+cos(0)) = 2*m_p*g*l
        self.E_up = 2.0 * self.m_p * self.g * self.l

        # Swing-up
        self.k_energy   = k_energy
        self.k_center   = k_center
        self.k_center_d = k_center_d

        # Hysteresis
        self.theta_enter = theta_enter
        self.omega_enter = omega_enter
        self.theta_exit  = theta_exit
        self.omega_exit  = omega_exit

        self.mode = "swing_up"

        # ── Linearised system around theta=0 ──────────────────────────
        # delta(theta=0) = M - m_p*cos^2(0) = m_c
        #
        #   x_ddot  = (u - m_p*g*theta) / m_c
        #   th_ddot = (-u + (m_c+m_p)*g*theta) / (m_c*l)
        M   = self.m_c + self.m_p
        m_c = self.m_c
        m_p = self.m_p
        g   = self.g
        l   = self.l

        self.A = np.array([
            [0.0, 1.0,             0.0, 0.0],
            [0.0, 0.0,   -m_p*g / m_c, 0.0],
            [0.0, 0.0,             0.0, 1.0],
            [0.0, 0.0, M*g / (m_c*l),  0.0],
        ])
        self.B = np.array([
            [0.0         ],
            [1.0 / m_c   ],
            [0.0         ],
            [-1.0/(m_c*l)],
        ])

        # ── LQR design ────────────────────────────────────────────────
        if Q is None:
            Q = np.diag([1.0, 1.0, 100.0, 10.0])

        R_mat     = np.array([[float(R)]])
        P         = solve_continuous_are(self.A, self.B, Q, R_mat)
        self.K    = (np.linalg.inv(R_mat) @ self.B.T @ P).flatten()
        self.P    = P

        eigs = np.linalg.eigvals(self.A - self.B @ self.K.reshape(1, -1))
        print(f"✅ LQR gain  K = {np.round(self.K, 4)}")
        print(f"   Closed-loop eigenvalues = {np.round(eigs, 3)}")
        print(f"   All stable: {bool(np.all(eigs.real < 0))}")
        print(f"   E_up = {self.E_up:.4f} J")

    # ------------------------------------------------------------------ #
    #  Public interface                                                    #
    # ------------------------------------------------------------------ #

    def compute_control(self, state: np.ndarray):
        x, x_dot, theta, theta_dot = state
        th = _normalize(theta)

        # ── Mode switching ─────────────────────────────────────────────
        if self.mode == "swing_up":
            if abs(th) < self.theta_enter and abs(theta_dot) < self.omega_enter:
                self.mode = "stabilization"
        else:
            if abs(th) > self.theta_exit or abs(theta_dot) > self.omega_exit:
                self.mode = "swing_up"

        # ── Control ────────────────────────────────────────────────────
        if self.mode == "stabilization":
            u = self._lqr(x, x_dot, th, theta_dot)
        else:
            u = self._swing_up(x, x_dot, theta, theta_dot)

        return float(np.clip(u, -self.u_max, self.u_max)), self.mode

    def compute_lyapunov(self, state: np.ndarray) -> float:
        x, x_dot, theta, theta_dot = state
        th = _normalize(theta)
        if self.mode == "stabilization":
            s = np.array([x, x_dot, th, theta_dot])
            return float(s @ self.P @ s)
        else:
            E = self.sys.get_energy(state)
            return 0.5 * (E - self.E_up) ** 2

    # ------------------------------------------------------------------ #
    #  Private: swing-up                                                   #
    # ------------------------------------------------------------------ #

    def _swing_up(self, x, x_dot, theta, theta_dot):
        """
        Exact energy-based swing-up.

        Step 1 — compute exact dE/dt terms:

            delta = (m_c + m_p) - m_p * cos^2(theta)

            f = -m_p * l * theta_dot^2 * sin(theta)
                + m_p * g * sin(theta) * cos(theta)

            dE/dt = (m_p * l * theta_dot * cos(theta) / delta) * (-u + f)

        Step 2 — choose u so that dV/dt = (E-E_up)*dE/dt <= 0:

            u = f + k_E * (E - E_up) * theta_dot * cos(theta)

            =>  dV/dt = -(m_p*l*k_E/delta)*(E-E_up)^2*theta_dot^2*cos^2(theta) <= 0

        Step 3 — add a small centering term to keep cart near origin.
                  Keep gains small so they do not fight the energy injection.
        """
        m_p = self.m_p
        m_c = self.m_c
        M   = m_c + m_p
        l   = self.l
        g   = self.g

        E     = self.sys.get_energy(np.array([x, x_dot, theta, theta_dot]))
        E_err = E - self.E_up

        sin_t = np.sin(theta)
        cos_t = np.cos(theta)
        delta = M - m_p * cos_t ** 2          # same as in system.dynamics()

        # Nonlinear terms in dE/dt that do not involve u
        f = (-m_p * l * theta_dot ** 2 * sin_t
             + m_p * g * sin_t * cos_t)

        # Energy injection: makes dV/dt = -(m_p*l*k_E/delta)*E_err^2*td^2*cos^2 <= 0
        u_energy = f + self.k_energy * E_err * theta_dot * cos_t

        # Small centering PD (do NOT use large gains here)
        u_center = -self.k_center * x - self.k_center_d * x_dot

        return u_energy + u_center

    # ------------------------------------------------------------------ #
    #  Private: LQR                                                        #
    # ------------------------------------------------------------------ #

    def _lqr(self, x, x_dot, th, th_dot):
        """u = -K @ [x, x_dot, theta, theta_dot]"""
        s = np.array([x, x_dot, th, th_dot])
        return -float(self.K @ s)


class PureLyapunovController:
    """Kept for reference. Use LQRController instead."""

    def __init__(self, system: CartPoleSystem,
                 q1=10.0, q2=5.0, q3=100.0, q4=20.0,
                 k_d=50.0, eps=1e-3,
                 k_energy=50.0, k_center=5.0, k_center_d=3.0,
                 theta_enter=0.25, omega_enter=2.0,
                 theta_exit=0.50, omega_exit=4.0):

        self.sys   = system
        p          = system.params
        self.m_c   = p.m_c
        self.m_p   = p.m_p
        self.l     = p.l
        self.g     = p.g
        self.u_max = p.max_force

        self.q1, self.q2, self.q3, self.q4 = q1, q2, q3, q4
        self.k_d  = k_d
        self.eps  = eps
        self.D    = self.m_c + self.m_p
        self.L    = self.m_c * self.l

        self.k_energy   = k_energy
        self.k_center   = k_center
        self.k_center_d = k_center_d
        self.E_up       = 2.0 * self.m_p * self.g * self.l

        self.theta_enter = theta_enter
        self.omega_enter = omega_enter
        self.theta_exit  = theta_exit
        self.omega_exit  = omega_exit
        self.mode        = "swing_up"

    def compute_control(self, state):
        x, x_dot, theta, theta_dot = state
        th = _normalize(theta)

        if self.mode == "swing_up":
            if abs(th) < self.theta_enter and abs(theta_dot) < self.omega_enter:
                self.mode = "stabilization"
        else:
            if abs(th) > self.theta_exit or abs(theta_dot) > self.omega_exit:
                self.mode = "swing_up"

        if self.mode == "stabilization":
            u = self._stabilize(x, x_dot, th, theta_dot)
        else:
            u = self._swing_up(state)

        return float(np.clip(u, -self.u_max, self.u_max)), self.mode

    def compute_lyapunov(self, state):
        x, x_dot, theta, theta_dot = state
        if self.mode == "stabilization":
            th = _normalize(theta)
            return 0.5 * (self.q1 * x ** 2 + self.q2 * x_dot ** 2
                          + self.q3 * th ** 2  + self.q4 * theta_dot ** 2)
        else:
            E = self.sys.get_energy(state)
            return 0.5 * ((E - self.E_up) / self.E_up) ** 2

    def _stabilize(self, x, x_dot, th, th_dot):
        sigma = self.q2 * x_dot / self.D - self.q4 * th_dot / self.L
        phi   = (self.q1 * x * x_dot
                 + self.q3 * th * th_dot
                 - (self.q2 * self.m_p * self.g / self.D) * x_dot * th
                 + (self.q4 * self.D * self.g / self.L) * th_dot * th)
        if abs(sigma) > self.eps:
            return (-phi - self.k_d * sigma) / sigma
        return -5.0 * x - 3.0 * x_dot + 30.0 * th + 5.0 * th_dot

    def _swing_up(self, state):
        x, x_dot, theta, theta_dot = state
        m_p = self.m_p
        M   = self.D
        m_c = self.m_c
        l   = self.l
        g   = self.g

        E     = self.sys.get_energy(state)
        E_err = E - self.E_up

        sin_t = np.sin(theta)
        cos_t = np.cos(theta)
        delta = M - m_p * cos_t ** 2

        f = -m_p * l * theta_dot ** 2 * sin_t + m_p * g * sin_t * cos_t
        u_energy = f + self.k_energy * E_err * theta_dot * cos_t
        u_center = -self.k_center * x - self.k_center_d * x_dot
        return u_energy + u_center