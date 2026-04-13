import numpy as np
from typing import List, Dict
from dataclasses import dataclass
import time

from system import CartPoleSystem
from controller import LyapunovController

@dataclass
class SimulationConfig:
    dt: float = 0.002
    T: float = 15.0
    save_every: int = 1
    random_seed: int = 42

@dataclass
class SimulationResult:
    time: np.ndarray
    states: np.ndarray
    controls: np.ndarray
    modes: List[str]
    lyapunov_values: np.ndarray
    energies: np.ndarray
    computation_time: float

class CartPoleSimulation:
    def __init__(self, system: CartPoleSystem, controller: LyapunovController, config: SimulationConfig):
        self.system = system
        self.controller = controller
        self.config = config

    def run(self, initial_state=None, verbose=True):
        np.random.seed(self.config.random_seed)
        state = self.system.reset(initial_state)
        
        N = int(self.config.T / self.config.dt)
        t_data, s_data, u_data, m_data, v_data, e_data = [], [], [], [], [], []
        
        start = time.time()
        if verbose: print(f"🚀 Симуляция: {self.config.T}с, шаг {self.config.dt}с")
        
        for step in range(N):
            t = step * self.config.dt
            
            # Управление
            u, mode = self.controller.compute_control(state)
            V = self.controller.compute_lyapunov_function(state)
            E = self.system.get_energy(state)
            
            # Сохраняем данные
            if step % self.config.save_every == 0:
                t_data.append(t)
                s_data.append(state.copy())
                u_data.append(u)
                m_data.append(mode)
                v_data.append(V)
                e_data.append(E)
            
            # Интегрирование
            state = self.system.step(u, self.config.dt)
            
            # 🛡️ Защита от численного взрыва
            if not np.all(np.isfinite(state)):
                if verbose: print("⚠️ Симуляция разошлась (NaN/Inf). Останавливаю.")
                break
                
            if verbose and step % 200 == 0:
                print(f"   t={t:.2f} | θ={np.degrees(state[2]):5.1f}° | V={V:.2f} | Mode={mode}")
                
        comp_time = time.time() - start
        if verbose: print(f"✅ Готово за {comp_time:.2f}с")
        
        return SimulationResult(
            time=np.array(t_data), states=np.array(s_data), controls=np.array(u_data),
            modes=m_data, lyapunov_values=np.array(v_data), energies=np.array(e_data),
            computation_time=comp_time
        )