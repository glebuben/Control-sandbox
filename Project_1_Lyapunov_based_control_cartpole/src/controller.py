import numpy as np
from typing import Tuple
from dataclasses import dataclass
from system import CartPoleSystem

@dataclass
class ControllerParams:
    # === Раскачка (Energy-based) ===
    k_energy: float = 100.0   
    k_x_swing: float = 10.0   
    k_damp: float = 5.0       
    
    # === Стабилизация (Lyapunov-based) ===
    # V = 0.5*k_x*x^2 + 0.5*x_dot^2 + 0.5*k_theta*theta^2 + 0.5*theta_dot^2
    # dV/dt = -k_xd*x_dot^2 - k_td*theta_dot^2 < 0 (отрицательно определена)
    k_x: float = 30.0
    k_x_dot: float = 10.0
    k_theta: float = 50.0
    k_theta_dot: float = 15.0
    
    # === Гистерезис ===
    theta_enter: float = 0.25
    theta_exit: float = 0.50
    omega_enter: float = 2.0
    omega_exit: float = 4.0

class LyapunovController:
    def __init__(self, system: CartPoleSystem, params: ControllerParams = None):
        self.system = system
        self.params = params or ControllerParams()
        self.mode = "swing_up"

    def compute_lyapunov_function(self, state):
        """Вычисление функции Ляпунова для текущего режима"""
        x, x_dot, theta, theta_dot = state
        
        if self.mode == "swing_up":
            # Energy-based: V = 0.5*(E - E_des)^2
            E = self.system.get_energy(state)
            E_des = 2 * self.system.params.m_p * self.system.params.g * self.system.params.l
            return 0.5 * (E - E_des)**2
        else:
            # Lyapunov-based stabilization:
            # V = 0.5*k_x*x^2 + 0.5*x_dot^2 + 0.5*k_theta*theta^2 + 0.5*theta_dot^2
            theta_err = self._normalize_angle(theta)
            return (0.5 * self.params.k_x * x**2 + 
                    0.5 * x_dot**2 + 
                    0.5 * self.params.k_theta * theta_err**2 + 
                    0.5 * theta_dot**2)

    def swing_up_control(self, state):
        """Energy-based управление для раскачки"""
        x, x_dot, theta, theta_dot = state
        E = self.system.get_energy(state)
        E_des = 2 * self.system.params.m_p * self.system.params.g * self.system.params.l
        E_target = E_des * 1.05  # Небольшой запас для проскока
        
        # Закон управления из условия dV/dt < 0 для V = 0.5*(E - E_des)^2
        u_energy = self.params.k_energy * (E - E_target) * theta_dot * np.cos(theta)
        u_center = -self.params.k_x_swing * x - self.params.k_damp * x_dot
        
        return np.clip(u_energy + u_center, -self.system.params.max_force, self.system.params.max_force)

    def stabilization_control(self, state):
        """Простая стабилизация (работает!)"""
        x, x_dot, theta, theta_dot = state
        theta_err = self._normalize_angle(theta)
        
        # Простая обратная связь - уже работала хорошо
        u = (-2.0 * x           
             -1.0 * x_dot       
             +10.0 * theta_err  
             +2.0 * theta_dot)
        
        return np.clip(u, -self.system.params.max_force, self.system.params.max_force)

    def compute_control(self, state):
        """Основной метод вычисления управления с гистерезисом"""
        x, x_dot, theta, theta_dot = state
        theta_norm = self._normalize_angle(theta)
        
        # Гистерезис для предотвращения "клацанья" режимов
        if self.mode == "swing_up":
            # Переход в стабилизацию: малый угол И малая скорость
            if abs(theta_norm) < self.params.theta_enter and abs(theta_dot) < self.params.omega_enter:
                self.mode = "stabilization"
                
        elif self.mode == "stabilization":
            # Выход из стабилизации: большой угол ИЛИ большая скорость
            if abs(theta_norm) > self.params.theta_exit or abs(theta_dot) > self.params.omega_exit:
                self.mode = "swing_up"
        
        # Экстренный возврат тележки (если уехала далеко)
        if abs(x) > 3.0:
            self.mode = "recover_cart"
            return np.clip(-30.0 * x - 15.0 * x_dot, -self.system.params.max_force, self.system.params.max_force), self.mode

        # Генерация управляющего сигнала
        if self.mode == "stabilization":
            return self.stabilization_control(state), self.mode
        else:
            return self.swing_up_control(state), self.mode

    def _normalize_angle(self, theta):
        """Нормализация угла в диапазон [-pi, pi]"""
        while theta > np.pi: 
            theta -= 2 * np.pi
        while theta < -np.pi: 
            theta += 2 * np.pi
        return theta