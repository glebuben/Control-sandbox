#!/usr/bin/env python3
import numpy as np, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from system import CartPoleSystem, CartPoleParams
from controller import LyapunovController, ControllerParams
from simulation import CartPoleSimulation, SimulationConfig
from visualization import CartPoleVisualizer

def main():
    print("🚀 CartPole - Stable Swing-Up & Stabilization")
    
    # Параметры системы
    params = CartPoleParams(m_c=0.5, m_p=0.2, l=0.5, g=9.81, max_force=10.0)
    system = CartPoleSystem(params)
    
    # Параметры контроллера
    ctrl_params = ControllerParams(
        k_energy=35.0, k_x_swing=2.0, k_damp=1.5,
        k_theta=100.0, k_theta_dot=25.0, theta_threshold=0.2
    )
    controller = LyapunovController(system, ctrl_params)
    
    config = SimulationConfig(dt=0.002, T=15.0, save_every=1, random_seed=42)
    
    # Старт снизу с легким толчком
    initial_state = np.array([0.0, 0.0, np.pi + 0.1, 0.0])
    
    sim = CartPoleSimulation(system, controller, config)
    result = sim.run(initial_state, verbose=True)
    
    # Визуализация
    vis = CartPoleVisualizer(result)
    base_dir = os.path.dirname(os.path.dirname(__file__))
    os.makedirs(os.path.join(base_dir, 'figures'), exist_ok=True)
    
    vis.plot_results(save_path=os.path.join(base_dir, 'figures', 'results.png'), show=False)
    print("📊 Графики сохранены.")
    
    vis.animate_realtime(show=True)

if __name__ == "__main__":
    main()