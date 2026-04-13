import numpy as np
from dataclasses import dataclass

@dataclass
class CartPoleParams:
    m_c: float = 0.5        # Масса тележки (полегче для разгона)
    m_p: float = 0.2        # Масса маятника (потятяжелее для инерции)
    l: float = 0.5          # Длина
    g: float = 9.81
    b_c: float = 0.0        # Трение тележки (0 для теста)
    b_p: float = 0.0        # Трение маятника (0 для теста)
    max_force: float = 10.0

    @property
    def M(self): return self.m_c + self.m_p

class CartPoleSystem:
    def __init__(self, params: CartPoleParams = None):
        self.params = params or CartPoleParams()
        self.state = np.zeros(4)

    def dynamics(self, state, u):
        x, x_dot, theta, theta_dot = state
        m_c, m_p, l, g = self.params.m_c, self.params.m_p, self.params.l, self.params.g
        M = self.params.M

        sin_t, cos_t = np.sin(theta), np.cos(theta)
        delta = M - m_p * cos_t**2

        # Динамика (Inverted Pendulum)
        # u > 0 (вправо) -> маятник отклоняется влево (против часовой)
        x_ddot = (u + m_p * l * theta_dot**2 * sin_t - m_p * g * cos_t * sin_t - self.params.b_c * x_dot) / delta
        
        theta_ddot = (-u * cos_t - m_p * l * theta_dot**2 * sin_t * cos_t + 
                      M * g * sin_t + self.params.b_c * x_dot * cos_t - self.params.b_p * theta_dot) / (l * delta)
        
        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

    def step(self, u, dt=0.01):
        u = np.clip(u, -self.params.max_force, self.params.max_force)
        dx = self.dynamics(self.state, u)
        self.state += dx * dt
        return self.state.copy()

    def reset(self, state=None):
        self.state = state.copy() if state is not None else np.array([0, 0, np.pi, 0])
        return self.state.copy()

    def get_energy(self, state=None):
        if state is None: state = self.state
        x, x_dot, theta, theta_dot = state
        m_p, l, g = self.params.m_p, self.params.l, self.params.g
        
        # Кинетическая
        T = 0.5 * m_p * (x_dot**2 + (l * theta_dot)**2 + 2 * l * x_dot * theta_dot * np.cos(theta))
        
        # Потенциальная (0 внизу, 2mgl вверху)
        # theta=0 (верх) -> cos=1 -> V=2mgl
        # theta=pi (низ) -> cos=-1 -> V=0
        V = m_p * g * l * (1 + np.cos(theta)) 
        
        return T + V