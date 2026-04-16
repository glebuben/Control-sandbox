# system.py
import numpy as np
from dataclasses import dataclass


@dataclass
class CartPoleParams:
    m_c: float = 0.5
    m_p: float = 0.2
    l: float = 0.5
    g: float = 9.81
    b_c: float = 0.0
    b_p: float = 0.0
    max_force: float = 10.0

    @property
    def M(self):
        return self.m_c + self.m_p


class CartPoleSystem:
    def __init__(self, params: CartPoleParams = None):
        self.params = params or CartPoleParams()
        self.state = np.zeros(4)

    def dynamics(self, state, u):
        x, x_dot, theta, theta_dot = state
        m_c = self.params.m_c
        m_p = self.params.m_p
        l = self.params.l
        g = self.params.g
        M = self.params.M

        sin_t, cos_t = np.sin(theta), np.cos(theta)
        delta = M - m_p * cos_t**2

        x_ddot = (
            u + m_p * l * theta_dot**2 * sin_t - m_p * g * cos_t * sin_t
        ) / delta

        theta_ddot = (
            -u * cos_t
            - m_p * l * theta_dot**2 * sin_t * cos_t
            + M * g * sin_t
        ) / (l * delta)

        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

    def step(self, u, dt=0.01):
        u = np.clip(u, -self.params.max_force, self.params.max_force)
        # RK4 integration for better accuracy
        k1 = self.dynamics(self.state, u)
        k2 = self.dynamics(self.state + 0.5 * dt * k1, u)
        k3 = self.dynamics(self.state + 0.5 * dt * k2, u)
        k4 = self.dynamics(self.state + dt * k3, u)
        self.state += (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return self.state.copy()

    def reset(self, state=None):
        self.state = (
            state.copy() if state is not None else np.array([0, 0, np.pi, 0])
        )
        return self.state.copy()

    def get_pendulum_energy(self, state=None):
        """
        Pendulum energy relative to the cart (ignores cart velocity).
        E_pend = 0.5 * m_p * (l * theta_dot)^2 + m_p * g * l * (1 + cos(theta))
        At upright equilibrium (theta=0, theta_dot=0): E_pend = 2*m_p*g*l
        """
        if state is None:
            state = self.state
        _, _, theta, theta_dot = state
        m_p, l, g = self.params.m_p, self.params.l, self.params.g

        T_rel = 0.5 * m_p * (l * theta_dot) ** 2
        V = m_p * g * l * (1 + np.cos(theta))
        return T_rel + V

    def get_full_energy(self, state=None):
        """
        Total mechanical energy of the cart-pendulum system.

        E_full = 0.5*m_c*x_dot^2
               + 0.5*m_p*(x_dot^2 + 2*l*x_dot*theta_dot*cos(theta)
                          + l^2*theta_dot^2)
               + m_p*g*l*(1 + cos(theta))

        At upright equilibrium (all velocities zero, theta=0):
            E_full_up = 2*m_p*g*l  (same potential reference)

        Key property: dE_full/dt = u * x_dot  (power input from control force)
        """
        if state is None:
            state = self.state
        x, x_dot, theta, theta_dot = state
        m_c = self.params.m_c
        m_p = self.params.m_p
        l = self.params.l
        g = self.params.g

        # Cart KE
        T_cart = 0.5 * m_c * x_dot**2

        # Pendulum absolute velocity components:
        #   v_px = x_dot + l*theta_dot*cos(theta)  (horizontal, but in our
        #          convention cos appears because theta=0 is up)
        #   v_py = l*theta_dot*sin(theta)
        #   |v_p|^2 = x_dot^2 + 2*l*x_dot*theta_dot*cos(theta)
        #             + l^2*theta_dot^2
        v_p_sq = (
            x_dot**2
            + 2 * l * x_dot * theta_dot * np.cos(theta)
            + l**2 * theta_dot**2
        )
        T_pend = 0.5 * m_p * v_p_sq

        # Potential energy (same reference as pendulum-only version)
        V = m_p * g * l * (1 + np.cos(theta))

        return T_cart + T_pend + V

    # Keep backward compatibility
    def get_energy(self, state=None):
        return self.get_pendulum_energy(state)