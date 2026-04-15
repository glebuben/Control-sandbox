# controller.py
import numpy as np
from dataclasses import dataclass
from scipy.linalg import solve_continuous_are

from system import CartPoleSystem


def _normalize(theta: float) -> float:
    """Map theta to [-pi, pi]. theta=0 is upright."""
    return (theta + np.pi) % (2 * np.pi) - np.pi


class LQRController:
    """
    Two-phase cart-pole controller using PENDULUM energy for swing-up.

    ── Swing-up: Pendulum energy (Åström–Furuta) ──

    Energy:
        E = 0.5 * m_p * (l * θ̇)² + m_p * g * l * (1 + cos θ)

    Exact dE/dt from the EOM:
        dE/dt = (m_p·l·θ̇·cos θ / δ) · (−u + f)

    where:
        δ = M − m_p·cos²θ
        f = −m_p·l·θ̇²·sin θ + m_p·g·sin θ·cos θ

    Control law (makes V = 0.5·(E−E_up)² decrease):
        u = f + k_E · (E − E_up) · θ̇ · cos θ

    ── Stabilization: LQR ──
    """

    def __init__(
        self,
        system: CartPoleSystem,
        Q: np.ndarray = None,
        R: float = 1.0,
        k_energy: float = 50.0,
        k_center: float = 0.5,
        k_center_d: float = 0.2,
        theta_enter: float = 0.3,
        omega_enter: float = 2.0,
        theta_exit: float = 0.8,
        omega_exit: float = 6.0,
    ):
        self.sys = system
        p = system.params
        self.m_c = p.m_c
        self.m_p = p.m_p
        self.l = p.l
        self.g = p.g
        self.u_max = p.max_force

        self.E_up = 2.0 * self.m_p * self.g * self.l

        self.k_energy = k_energy
        self.k_center = k_center
        self.k_center_d = k_center_d

        self.theta_enter = theta_enter
        self.omega_enter = omega_enter
        self.theta_exit = theta_exit
        self.omega_exit = omega_exit

        self.mode = "swing_up"
        self.controller_name = "Pendulum Energy"

        # ── LQR ──
        self.A, self.B = self._linearize()
        self.K, self.P = self._design_lqr(Q, R)

        self._print_info()

    def _linearize(self):
        m_c, m_p, l, g = self.m_c, self.m_p, self.l, self.g
        M = m_c + m_p

        A = np.array([
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -m_p * g / m_c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, M * g / (m_c * l), 0.0],
        ])
        B = np.array([
            [0.0],
            [1.0 / m_c],
            [0.0],
            [-1.0 / (m_c * l)],
        ])
        return A, B

    def _design_lqr(self, Q, R):
        if Q is None:
            Q = np.diag([1.0, 1.0, 100.0, 10.0])
        R_mat = np.array([[float(R)]])
        P = solve_continuous_are(self.A, self.B, Q, R_mat)
        K = (np.linalg.inv(R_mat) @ self.B.T @ P).flatten()
        return K, P

    def _print_info(self):
        eigs = np.linalg.eigvals(
            self.A - self.B @ self.K.reshape(1, -1)
        )
        print(f"\n{'='*60}")
        print(f"Controller: {self.controller_name}")
        print(f"{'='*60}")
        print(f"  LQR gain  K = {np.round(self.K, 4)}")
        print(f"  Closed-loop eigenvalues = {np.round(eigs, 3)}")
        print(f"  All stable: {bool(np.all(eigs.real < 0))}")
        print(f"  E_up = {self.E_up:.4f} J")
        print(f"  k_energy = {self.k_energy}")

    # ── Public interface ──

    def compute_control(self, state: np.ndarray):
        x, x_dot, theta, theta_dot = state
        th = _normalize(theta)

        if self.mode == "swing_up":
            if (
                abs(th) < self.theta_enter
                and abs(theta_dot) < self.omega_enter
            ):
                self.mode = "stabilization"
        else:
            if (
                abs(th) > self.theta_exit
                or abs(theta_dot) > self.omega_exit
            ):
                self.mode = "swing_up"

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
            E = self.sys.get_pendulum_energy(state)
            return 0.5 * (E - self.E_up) ** 2

    # ── Swing-up (pendulum energy) ──

    def _swing_up(self, x, x_dot, theta, theta_dot):
        """
        Pendulum-energy swing-up (Åström–Furuta style).

        u = f + k_E·(E − E_up)·θ̇·cosθ  +  centering terms

        Guarantees dV/dt ≤ 0 for V = 0.5·(E−E_up)²
        """
        m_p, m_c, l, g = self.m_p, self.m_c, self.l, self.g
        M = m_c + m_p

        state = np.array([x, x_dot, theta, theta_dot])
        E = self.sys.get_pendulum_energy(state)
        E_err = E - self.E_up

        sin_t = np.sin(theta)
        cos_t = np.cos(theta)

        # f term from dE/dt derivation
        f = -m_p * l * theta_dot**2 * sin_t + m_p * g * sin_t * cos_t

        # Energy injection
        u_energy = f + self.k_energy * E_err * theta_dot * cos_t

        # Centering (keep cart near origin during swing)
        u_center = -self.k_center * x - self.k_center_d * x_dot

        return u_energy + u_center

    # ── LQR ──

    def _lqr(self, x, x_dot, th, th_dot):
        s = np.array([x, x_dot, th, th_dot])
        return -float(self.K @ s)


class FullEnergyLQRController(LQRController):
    """
    Two-phase controller using FULL SYSTEM energy for swing-up.

    ── Swing-up: Total energy control ──

    Full energy:
        E_full = 0.5·m_c·ẋ² + 0.5·m_p·(ẋ² + 2lẋθ̇cosθ + l²θ̇²)
                 + m_p·g·l·(1 + cosθ)

    Key simplification — the power balance gives:
        dE_full/dt = u · ẋ

    This is exact and model-independent (just Newton's third law:
    the only external force doing work is u on the cart).

    Lyapunov candidate:
        V = 0.5·(E_full − E_up)²

    Time derivative:
        dV/dt = (E_full − E_up) · u · ẋ

    To make dV/dt ≤ 0, choose:
        u = −k_E · (E_full − E_up) · ẋ

    Then:
        dV/dt = −k_E · (E_full − E_up)² · ẋ²  ≤  0  ✓

    Note: convergence requires persistent excitation (ẋ ≠ 0),
    which is generally satisfied during swing-up.

    Comparison with pendulum-energy controller:
    ┌─────────────────────┬──────────────────────┬──────────────────────┐
    │                     │   Pendulum Energy     │    Full Energy       │
    ├─────────────────────┼──────────────────────┼──────────────────────┤
    │ Energy definition   │ Rotational KE + PE    │ Total system KE + PE │
    │ dE/dt expression    │ Complex (δ, f terms)  │ Simple: u·ẋ          │
    │ Control law         │ f + k·ΔE·θ̇·cosθ      │ −k·ΔE·ẋ             │
    │ Singularity         │ cosθ=0 or θ̇=0        │ ẋ=0 (cart stationary)│
    │ Cart coupling       │ Indirect (centering)  │ Natural (through ẋ)  │
    │ Cart drift          │ Needs explicit PD     │ Self-limiting        │
    │ Energy at upright   │ 2·m_p·g·l             │ 2·m_p·g·l (ẋ→0)     │
    └─────────────────────┴──────────────────────┴──────────────────────┘

    ── Stabilization: same LQR as base class ──
    """

    def __init__(
        self,
        system: CartPoleSystem,
        Q: np.ndarray = None,
        R: float = 1.0,
        k_energy: float = 50.0,
        k_center: float = 0.5,
        k_center_d: float = 0.2,
        # Additional gain: adds a θ̇·cosθ term to help when cart is
        # nearly stationary (breaks the ẋ=0 degeneracy)
        k_theta_kick: float = 5.0,
        theta_enter: float = 0.3,
        omega_enter: float = 2.0,
        theta_exit: float = 0.8,
        omega_exit: float = 6.0,
    ):
        self.k_theta_kick = k_theta_kick
        super().__init__(
            system=system,
            Q=Q,
            R=R,
            k_energy=k_energy,
            k_center=k_center,
            k_center_d=k_center_d,
            theta_enter=theta_enter,
            omega_enter=omega_enter,
            theta_exit=theta_exit,
            omega_exit=omega_exit,
        )
        self.controller_name = "Full Energy"
        self._print_info()

    def compute_lyapunov(self, state: np.ndarray) -> float:
        x, x_dot, theta, theta_dot = state
        th = _normalize(theta)
        if self.mode == "stabilization":
            s = np.array([x, x_dot, th, theta_dot])
            return float(s @ self.P @ s)
        else:
            E = self.sys.get_full_energy(state)
            return 0.5 * (E - self.E_up) ** 2

    def _swing_up(self, x, x_dot, theta, theta_dot):
        """
        Full-energy swing-up.

        Primary term (Lyapunov-certified):
            u_energy = −k_E · (E_full − E_up) · ẋ

            => dV/dt = −k_E · (E_full − E_up)² · ẋ² ≤ 0

        Auxiliary kick term (for when ẋ ≈ 0):
            u_kick = k_kick · (E_full − E_up) · θ̇ · cosθ

            This injects a small torque-like signal that gets the
            cart moving so the primary term can take over.
            It's the same form as the pendulum-energy controller
            but applied to the full energy error.

        Centering PD (keeps cart bounded):
            u_center = −k_x · x − k_d · ẋ
        """
        state = np.array([x, x_dot, theta, theta_dot])
        E_full = self.sys.get_full_energy(state)
        E_err = E_full - self.E_up

        # Primary: Lyapunov-certified term
        #   u = -k_E * (E_full - E_up) * x_dot
        #   => dV/dt = -k_E * (E_full - E_up)^2 * x_dot^2 <= 0
        u_energy = -self.k_energy * E_err * x_dot

        # Auxiliary: break ẋ=0 degeneracy using pendulum motion
        # When the cart is nearly still, this nudges it into motion
        # so the primary term can do its job
        cos_t = np.cos(theta)
        u_kick = self.k_theta_kick * E_err * theta_dot * cos_t

        # Centering
        u_center = -self.k_center * x - self.k_center_d * x_dot

        return u_energy + u_kick + u_center