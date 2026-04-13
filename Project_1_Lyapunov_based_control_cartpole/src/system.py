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

        x_ddot = (u + m_p * l * theta_dot**2 * sin_t - m_p * g * cos_t * sin_t) / delta
        
        theta_ddot = (-u * cos_t - m_p * l * theta_dot**2 * sin_t * cos_t + 
                      M * g * sin_t) / (l * delta)
        
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
        # ИСПРАВЛЕНИЕ: Считаем энергию маятника ОТНОСИТЕЛЬНО тележки.
        # Скорость тележки (x_dot) не должна влиять на решение о раскачке.
        if state is None: state = self.state
        x, x_dot, theta, theta_dot = state
        m_p, l, g = self.params.m_p, self.params.l, self.params.g
        
        # Кинетическая энергия только вращения (без учета движения тележки)
        T_rel = 0.5 * m_p * (l * theta_dot)**2 
        
        # Потенциальная энергия
        V = m_p * g * l * (1 + np.cos(theta)) 
        
        return T_rel + V