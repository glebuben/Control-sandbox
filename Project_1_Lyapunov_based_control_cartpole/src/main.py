#!/usr/bin/env python3
"""
CartPole - Lyapunov-based Control
Проект по управлению перевернутым маятником на тележке

Author: Mikkokiss
Course: Advanced Control Methods, Skoltech 2026
"""
import numpy as np
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from system import CartPoleSystem, CartPoleParams
from controller import LyapunovController, ControllerParams
from simulation import CartPoleSimulation, SimulationConfig
from visualization import CartPoleVisualizer

def main():
    print("="*60)
    print("CartPole - Lyapunov-based Swing-Up & Stabilization")
    print("="*60)
    
    # === Параметры системы ===
    params = CartPoleParams(
        m_c=0.5,        # Масса тележки [kg]
        m_p=0.2,        # Масса маятника [kg]
        l=0.5,          # Длина маятника [m]
        g=9.81,         # Ускорение свободного падения [m/s^2]
        max_force=10.0  # Максимальная сила управления [N]
    )
    system = CartPoleSystem(params)
    print(f"✅ Система создана: m_c={params.m_c}kg, m_p={params.m_p}kg, l={params.l}m")
    
    # === Параметры контроллера ===
    ctrl_params = ControllerParams(
        # Раскачка
        k_energy=100.0, 
        k_x_swing=10.0,   
        k_damp=5.0,
        
        # Стабилизация (Lyapunov)
        k_x=30.0,
        k_x_dot=10.0,
        k_theta=15.0,       # Умеренный коэффициент угла
        k_theta_dot=3.0,   # Умеренное гашение скорости
        
        # Гистерезис переключения режимов
        theta_enter=0.25,   # Вход в стабилизацию: ~14°
        theta_exit=0.50,    # Выход из стабилизации: ~28°
        omega_enter=2.0,    # Макс. угловая скорость для входа [rad/s]
        omega_exit=4.0      # Макс. угловая скорость для выхода [rad/s]
    )
    controller = LyapunovController(system, ctrl_params)
    print("✅ Контроллер создан (Energy-based + Lyapunov-based)")
    
    # === Конфигурация симуляции ===
    config = SimulationConfig(
        dt=0.002,           # Шаг интегрирования [s]
        T=15.0,             # Общее время [s]
        save_every=1,       # Сохранять каждый шаг
        random_seed=42
    )
    
    # === Начальное состояние (маятник внизу) ===
    initial_state = np.array([
        0.0,                    # x: позиция тележки
        0.0,                    # x_dot: скорость тележки
        np.pi + 0.1,           # theta: угол (pi = вниз, + отклонение)
        0.0                     # theta_dot: угловая скорость
    ])
    print(f"📍 Начальное состояние: x={initial_state[0]:.2f}m, θ={np.degrees(initial_state[2]):.1f}°")
    
    # === Запуск симуляции ===
    print("\n🚀 Запуск симуляции...")
    sim = CartPoleSimulation(system, controller, config)
    result = sim.run(initial_state, verbose=True)
    
    # === Статистика ===
    print("\n" + "="*60)
    print("📊 СТАТИСТИКА")
    print("="*60)
    E_des = 2 * params.m_p * params.g * params.l
    print(f"Энергия:")
    print(f"   Начальная: {result.energies[0]:.3f} J")
    print(f"   Конечная: {result.energies[-1]:.3f} J")
    print(f"   Желаемая: {E_des:.3f} J")
    print(f"Режимы:")
    n_stab = sum(1 for m in result.modes if m == 'stabilization')
    n_swing = sum(1 for m in result.modes if m == 'swing_up')
    print(f"   Swing-up: {n_swing} шагов")
    print(f"   Stabilization: {n_stab} шагов")
    
    # === Визуализация ===
    print("\n📊 Визуализация результатов...")
    vis = CartPoleVisualizer(result)
    base_dir = os.path.dirname(os.path.dirname(__file__))
    figures_dir = os.path.join(base_dir, 'figures')
    os.makedirs(figures_dir, exist_ok=True)
    
    # Сохранение графиков
    vis.plot_results(
        save_path=os.path.join(figures_dir, 'results.png'), 
        show=False
    )
    print(f"✅ Графики сохранены: {figures_dir}/results.png")
    
    # Анимация в реальном времени
    print("\n🎬 Запуск анимации...")
    print("   Закройте окно анимации для завершения")
    vis.animate_realtime(show=True)
    
    print("\n✅ Симуляция завершена успешно!")
    print("="*60)

if __name__ == "__main__":
    main()