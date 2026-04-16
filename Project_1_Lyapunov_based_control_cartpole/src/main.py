#!/usr/bin/env python3
"""
CartPole — Compare Pendulum-Energy vs Full-Energy Swing-Up Controllers
"""
import numpy as np
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from system import CartPoleSystem, CartPoleParams
from controller import LQRController, FullEnergyLQRController
from simulation import CartPoleSimulation, SimulationConfig
from visualization import CartPoleVisualizer, ComparisonVisualizer


def _normalize(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


def run_single(system, controller, config, initial_state, verbose=True):
    """Run one simulation and print summary."""
    name = getattr(controller, "controller_name", "Unknown")
    E_up = 2.0 * system.params.m_p * system.params.g * system.params.l

    print(f"\n{'─'*60}")
    print(f"Running: {name}")
    print(f"{'─'*60}")

    sim = CartPoleSimulation(system, controller, config)
    result = sim.run(initial_state, verbose=verbose)
    result.l = system.params.l

    theta_f_deg = np.degrees(_normalize(result.states[-1, 2]))
    x_f = result.states[-1, 0]

    print(f"\n  Energy  — initial: {result.energies[0]:.4f} J | "
          f"final: {result.energies[-1]:.4f} J | target: {E_up:.4f} J")
    print(f"  Lyapunov — initial: {result.lyapunov_values[0]:.4f} | "
          f"final: {result.lyapunov_values[-1]:.6f}")
    print(f"  Final state — x={x_f:+.4f} m,  θ={theta_f_deg:+.2f}°")

    n_stab = sum(1 for m in result.modes if m == "stabilization")
    n_swing = sum(1 for m in result.modes if m == "swing_up")
    total = len(result.modes)
    print(f"  Modes — swing: {n_swing} ({100*n_swing/total:.0f}%), "
          f"stab: {n_stab} ({100*n_stab/total:.0f}%)")

    if abs(theta_f_deg) < 5.0:
        print(f"  🎉 SUCCESS — stabilized within 5° of upright!")
    else:
        print(f"  ⚠️  Not stabilized (θ = {theta_f_deg:+.1f}°)")

    return result


def main():
    print("=" * 60)
    print("CartPole — Pendulum Energy vs Full Energy Swing-Up")
    print("=" * 60)

    # ── System parameters ──────────────────────────────────────────────────
    params = CartPoleParams(
        m_c=0.5,
        m_p=0.2,
        l=0.5,
        g=9.81,
        max_force=3.0,
    )
    E_up = 2.0 * params.m_p * params.g * params.l
    print(f"System: m_c={params.m_c}, m_p={params.m_p}, "
          f"l={params.l}, g={params.g}")
    print(f"E_up = {E_up:.4f} J,  max_force = {params.max_force} N")

    # ── Shared settings ────────────────────────────────────────────────────
    Q = np.diag([1.0, 1.0, 100.0, 10.0])
    R = 1.0
    config = SimulationConfig(dt=0.002, T=20.0, save_every=1)
    initial_state = np.array([0.0, 0.0, np.pi + 0.1, 0.0])

    print(f"Initial state: θ={np.degrees(_normalize(initial_state[2])):+.1f}° "
          f"from upright")

    # ── Controller A: Pendulum energy ──────────────────────────────────────
    system_a = CartPoleSystem(params)
    ctrl_a = LQRController(
        system=system_a,
        Q=Q, R=R,
        k_energy=8.0,
        k_center=1.0,
        k_center_d=0.5,
        theta_enter=0.4,
        omega_enter=2.0,
        theta_exit=0.8,
        omega_exit=5.0,
    )
    result_a = run_single(system_a, ctrl_a, config, initial_state)

    # ── Controller B: Full energy ──────────────────────────────────────────
    system_b = CartPoleSystem(params)
    ctrl_b = FullEnergyLQRController(
        system=system_b,
        Q=Q, R=R,
        k_energy=8.0,
        k_center=1.0,
        k_center_d=0.5,
        k_theta_kick=5.0,
        theta_enter=0.4,
        omega_enter=2.0,
        theta_exit=0.8,
        omega_exit=5.0,
    )
    result_b = run_single(system_b, ctrl_b, config, initial_state)

    # ── Output directories ─────────────────────────────────────────────────
    base_dir    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    anim_dir    = os.path.join(base_dir, "animations")
    figures_dir = os.path.join(base_dir, "figures")
    os.makedirs(figures_dir, exist_ok=True)
    os.makedirs(anim_dir,    exist_ok=True)

    # ── Visualizers ────────────────────────────────────────────────────────
    vis_a = CartPoleVisualizer(result_a, e_up=E_up)
    vis_b = CartPoleVisualizer(result_b, e_up=E_up)

    # ── Individual static plots ────────────────────────────────────────────
    print("\n📊 Saving static result plots...")

    vis_a.plot_results(
        save_path=os.path.join(figures_dir, "results_pendulum_energy.png"),
        show=False,
    )
    vis_b.plot_results(
        save_path=os.path.join(figures_dir, "results_full_energy.png"),
        show=False,
    )

    # ── Phase portraits ────────────────────────────────────────────────────
    print("\n🌀 Saving phase portraits...")

    vis_a.plot_phase_portraits(
        save_path=os.path.join(figures_dir, "phase_portraits_pendulum_energy.png"),
        show=False,
    )
    vis_b.plot_phase_portraits(
        save_path=os.path.join(figures_dir, "phase_portraits_full_energy.png"),
        show=False,
    )

    # ── Side-by-side comparison ────────────────────────────────────────────
    print("\n📊 Saving comparison plots...")

    comp = ComparisonVisualizer(result_a, result_b, e_up=E_up)
    comp.plot_comparison(
        save_path=os.path.join(figures_dir, "comparison.png"),
        show=False,
    )

    # ── Save GIFs ─────────────────────────────────────────────────────────
    print("\n💾 Saving GIF: Pendulum Energy controller...")
    vis_a.save_gif(
        save_path=os.path.join(anim_dir, "animation_pendulum_energy.gif"),
        speed=1.0,
        fps=30,
        dpi=100,
    )

    print("💾 Saving GIF: Full Energy controller...")
    vis_b.save_gif(
        save_path=os.path.join(anim_dir, "animation_full_energy.gif"),
        speed=1.0,
        fps=30,
        dpi=100,
    )

    print(f"\n✅ All figures saved to:   {figures_dir}/")
    print(f"✅ All animations saved to: {anim_dir}/")
    _print_file_summary(figures_dir, anim_dir)

    # ── Live animations (optional, blocking) ──────────────────────────────
    print("\n🎬 Animating Pendulum Energy controller...")
    print("   (close the window to continue)")
    vis_a.animate_realtime(show=True, speed=1.0)

    print("🎬 Animating Full Energy controller...")
    print("   (close the window to continue)")
    vis_b.animate_realtime(show=True, speed=1.0)

    print("\n✅ Done!")


def _print_file_summary(figures_dir: str, anim_dir: str) -> None:
    """Print a neat table of every output file that was written."""
    rows = []

    expected_figures = [
        ("results_pendulum_energy.png",       "Static plots — Pendulum Energy ctrl"),
        ("results_full_energy.png",            "Static plots — Full Energy ctrl"),
        ("phase_portraits_pendulum_energy.png","Phase portraits — Pendulum Energy ctrl"),
        ("phase_portraits_full_energy.png",    "Phase portraits — Full Energy ctrl"),
        ("comparison.png",                     "Side-by-side comparison"),
    ]
    expected_anims = [
        ("animation_pendulum_energy.gif", "Animation — Pendulum Energy ctrl"),
        ("animation_full_energy.gif",     "Animation — Full Energy ctrl"),
    ]

    for fname, desc in expected_figures:
        fpath = os.path.join(figures_dir, fname)
        size  = _file_size_str(fpath)
        rows.append((desc, fpath, size))

    for fname, desc in expected_anims:
        fpath = os.path.join(anim_dir, fname)
        size  = _file_size_str(fpath)
        rows.append((desc, fpath, size))

    col_desc = max(len(r[0]) for r in rows)
    col_size = max(len(r[2]) for r in rows)

    print()
    print("  " + "─" * (col_desc + col_size + 6))
    for desc, fpath, size in rows:
        print(f"  {desc:<{col_desc}}  {size:>{col_size}}  {fpath}")
    print("  " + "─" * (col_desc + col_size + 6))


def _file_size_str(path: str) -> str:
    """Return human-readable file size, or '(missing)' if not found."""
    try:
        b = os.path.getsize(path)
        if b < 1024:
            return f"{b} B"
        elif b < 1024 ** 2:
            return f"{b / 1024:.1f} KB"
        else:
            return f"{b / 1024**2:.1f} MB"
    except FileNotFoundError:
        return "(missing)"


if __name__ == "__main__":
    main()