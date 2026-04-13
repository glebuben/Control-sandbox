import numpy as np
from typing import Tuple
from dataclasses import dataclass
from system import CartPoleSystem

@dataclass
class ControllerParams:
    k_energy: float = 80.0    # Умеренная раскачка
    k_x_swing: float = 2.0    # Удержание тележки в центре при раскачке
    k_damp: float = 1.5       # Демпфирование скорости
    
    k_x: float = 8.0          # Стабилизация позиции
    k_x_dot: float = 3.0      # Стабилизация скорости
    k_theta: float = 100.0    # Стабилизация угла
    k_theta_dot: float = 25.0 # Стабилизация угловой скорости
    
    theta_threshold: float = 0.2  # ~11 градусов

class LyapunovController:
    def __init__(self, system: CartPoleSystem, params: ControllerParams = None):
        self.system = system
        self.params = params or ControllerParams()
        self.mode = "swing_up"

    def compute_lyapunov_function(self, state):
        x, x_dot, theta, theta_dot = state
        if self.mode == "swing_up":
            E = self.system.get_energy(state)
            E_des = 2 * self.system.params.m_p * self.system.params.g * self.system.params.l
            return 0.5 * (E - E_des)**2
        else:
            theta_err = self._normalize_angle(theta)
            return 0.5 * (self.params.k_x * x**2 + x_dot**2 + 
                          self.params.k_theta * theta_err**2 + theta_dot**2)

    def swing_up_control(self, state):
        x, x_dot, theta, theta_dot = state
        E = self.system.get_energy(state)
        E_des = 2 * self.system.params.m_p * self.system.params.g * self.system.params.l
        
        # 1. Раскачка по энергии
        u_energy = self.params.k_energy * (E - E_des) * theta_dot * np.cos(theta)
        # 2. Возврат тележки к центру + гашение скорости
        u_center = -self.params.k_x_swing * x - self.params.k_damp * x_dot
        
        return np.clip(u_energy + u_center, -self.system.params.max_force, self.system.params.max_force)

    def stabilization_control(self, state):
        x, x_dot, theta, theta_dot = state
        theta_err = self._normalize_angle(theta)
        # LQR-подобная стабилизация
        return np.clip(-(self.params.k_x * x + self.params.k_x_dot * x_dot + 
                         self.params.k_theta * theta_err + self.params.k_theta_dot * theta_dot),
                       -self.system.params.max_force, self.system.params.max_force)

    def compute_control(self, state):
        theta_norm = self._normalize_angle(state[2])
        
        # Переключаемся, как только угол достаточно мал
        if abs(theta_norm) < self.params.theta_threshold:
            self.mode = "stabilization"
            return self.stabilization_control(state), self.mode
        else:
            self.mode = "swing_up"
            return self.swing_up_control(state), self.mode

    def _normalize_angle(self, theta):
        while theta > np.pi: theta -= 2 * np.pi
        while theta < -np.pi: theta += 2 * np.pi
        return theta