#!/usr/bin/env python3
"""
CartPole — LQR Stabilization + Energy-based Swing-Up
Course: Advanced Control Methods, Skoltech 2026
"""
import numpy as np
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from system import CartPoleSystem, CartPoleParams
from controller import LQRController
from simulation import CartPoleSimulation, SimulationConfig
from visualization import CartPoleVisualizer


def _normalize(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


def main():
    print("=" * 60)
    print("CartPole — LQR Stabilization + Energy Swing-Up")
    print("=" * 60)

    # ---- System ----
    params = CartPoleParams(
        m_c=0.5,
        m_p=0.2,
        l=0.5,
        g=9.81,
        max_force=10.0,
    )
    system = CartPoleSystem(params)
    E_up = 2.0 * params.m_p * params.g * params.l
    print(f"✅ System: m_c={params.m_c} kg, m_p={params.m_p} kg, l={params.l} m")
    print(f"   E_up = 2·m_p·g·l = {E_up:.4f} J")

    # ---- Controller ----
    controller = LQRController(
        system=system,
        Q=np.diag([1.0, 1.0, 100.0, 10.0]),
        R=1.0,
        k_energy=8.0,
        k_center=1.0,
        k_center_d=0.5,
        theta_enter=0.4,
        omega_enter=2.0,
        theta_exit=0.8,
        omega_exit=5.0,
    )

    # ---- Simulation ----
    config = SimulationConfig(
        dt=0.002,
        T=20.0,
        save_every=1,
        random_seed=42,
    )

    initial_state = np.array([0.0, 0.0, np.pi + 0.1, 0.0])
    print(f"\n📍 Initial state: x=0.0 m, "
          f"θ={np.degrees(_normalize(initial_state[2])):+.1f}° from upright")

    print("\n🚀 Running simulation...")
    sim = CartPoleSimulation(system, controller, config)
    result = sim.run(initial_state, verbose=True)

    # ---- Statistics ----
    print("\n" + "=" * 60)
    print("📊 STATISTICS")
    print("=" * 60)
    theta_f     = result.states[-1, 2]
    theta_f_deg = np.degrees(_normalize(theta_f))
    x_f         = result.states[-1, 0]

    print(f"Energy  — initial: {result.energies[0]:.4f} J  |  "
          f"final: {result.energies[-1]:.4f} J  |  target: {E_up:.4f} J")
    print(f"Lyapunov — initial: {result.lyapunov_values[0]:.4f}  |  "
          f"final: {result.lyapunov_values[-1]:.6f}")
    print(f"Final state — x={x_f:+.4f} m,  θ={theta_f_deg:+.2f}°")

    n_stab  = sum(1 for m in result.modes if m == "stabilization")
    n_swing = sum(1 for m in result.modes if m == "swing_up")
    total   = len(result.modes)
    print(f"Modes — swing: {n_swing} ({100*n_swing/total:.0f}%),  "
          f"stab: {n_stab} ({100*n_stab/total:.0f}%)")

    if abs(theta_f_deg) < 5.0:
        print("\n🎉 SUCCESS — stabilized within 5° of upright!")
    else:
        print(f"\n⚠️  Not stabilized (θ = {theta_f_deg:+.1f}°). "
              f"Try increasing k_energy or widening theta_enter.")

    # ---- Visualise ----
    result.l = params.l
    vis = CartPoleVisualizer(result)

    PROJECT_ROOT = Path(__file__).resolve().parent.parent
    
    # 1. Static plots (figures folder)
    figures_dir = PROJECT_ROOT / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    vis.plot_results(save_path=figures_dir / "results_lqr.png", show=False)
    print(f"\n✅ Plots saved → {figures_dir / 'results_lqr.png'}")
    # 3. Phase Portraits
    vis.plot_phase_portraits(
        save_path=figures_dir / "phase_portraits.png",
        show=False,
    )
    print(f"✅ Phase portraits saved → {figures_dir / 'phase_portraits.png'}")
    
    # 2. GIF Animation (animations folder - already exists)
    anim_dir = PROJECT_ROOT / "animations"
    vis.save_gif(save_path=anim_dir / "cartpole_lqr.gif", fps=30)

    # 3. Real-time animation (optional, shows window)
    print("\n🎬 Starting real-time animation...")
    vis.animate_realtime(show=True, speed=1.0)

    print("\n✅ Done!")
    print("=" * 60)


if __name__ == "__main__":
    main()