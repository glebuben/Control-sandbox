"""
main.py
-------
Entry point for the flexible-joint backstepping simulation toolkit.

Usage
-----
  python main.py --plots --scenario equilibrium     # default: stabilise to θ*=1 rad
  python main.py --plots --scenario step            # smooth sigmoid step trajectory
  python main.py --plots --scenario sin             # sinusoidal tracking
  python main.py --plots --scenario ctrl_compare    # Backstepping vs PD vs PID (step)
  python main.py --plots --scenario ctrl_compare_eq # Backstepping vs PD vs PID (equilibrium)
  python main.py --all --scenario ctrl_compare      # all four plot types
  python main.py --gif --animate --scenario equilibrium

  python main.py --t_end 15   # simulation duration in seconds
"""

from __future__ import annotations
import argparse
import sys
import time
import numpy as np
from pathlib import Path

_SRC = Path(__file__).resolve().parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from system        import SystemParams
from controller    import ControllerGains
from simulation    import (run_simulation,
                           run_equilibrium_stabilisation,
                           run_controller_comparison)
from visualization import (plot_states, plot_phase_portraits,
                            plot_lyapunov, plot_shaft_dynamics,
                            generate_all_plots, create_animation)


# ──────────────────────────────────────────────────────────────────────
# Default configuration
# ──────────────────────────────────────────────────────────────────────

DEFAULT_SYS_PARAMS = SystemParams(
    J_l=0.5, J_m=0.1,
    k=50.0, k3=20.0, b=1.0,
    Fc_l=0.3, Fc_m=0.2, v_s=0.1,
    Bv_l=0.05, Bv_m=0.03,
)

DEFAULT_GAINS    = ControllerGains(k1=5.0, k2=8.0, k3=10.0, k4=15.0)
STEP_REF_KWARGS  = dict(amplitude=1.0, t_rise=1.5, t_start=0.5)
SIN_REF_KWARGS   = dict(amplitude=0.8, frequency=0.4)

# Perturbed initial state for equilibrium scenario:
# load at 0 rad, motor at 0.3 rad, small load velocity → visible transient
EQUIL_X0 = np.array([0.0, 0.5, 0.3, 0.0])


# ──────────────────────────────────────────────────────────────────────
# Scenario builders
# ──────────────────────────────────────────────────────────────────────

def _base(t_end):
    return dict(sys_params=DEFAULT_SYS_PARAMS,
                t_span=(0.0, t_end), dt=5e-4, u_clip=200.0)


def build_equilibrium(t_end: float):
    """Backstepping stabilisation to a constant setpoint — Lyapunov scenario."""
    print("  Running equilibrium stabilisation (Backstepping) …")
    return run_simulation(
        **_base(t_end),
        controller_type = "backstepping",
        ctrl_gains      = DEFAULT_GAINS,
        x0              = EQUIL_X0.copy(),
        reference       = "equilibrium",
        ref_kwargs      = {"setpoint": 1.0},
        label           = "Backstepping",
    )


def build_step(t_end: float):
    print("  Running smooth-step trajectory (Backstepping) …")
    return run_simulation(
        **_base(t_end),
        controller_type = "backstepping",
        ctrl_gains      = DEFAULT_GAINS,
        reference       = "smooth_step",
        ref_kwargs      = STEP_REF_KWARGS,
        label           = "Backstepping",
    )


def build_sin(t_end: float):
    print("  Running sinusoidal tracking (Backstepping) …")
    return run_simulation(
        **_base(t_end),
        controller_type = "backstepping",
        ctrl_gains      = DEFAULT_GAINS,
        reference       = "sinusoidal",
        ref_kwargs      = SIN_REF_KWARGS,
        label           = "Backstepping",
    )


def build_ctrl_compare(t_end: float, equilibrium: bool = False):
    tag = "equilibrium" if equilibrium else "step"
    print(f"  Running controller comparison [{tag}]  PD / PID / Backstepping …")
    return run_controller_comparison(
        base_kwargs     = dict(**_base(t_end),
                               ref_kwargs={"setpoint": 1.0} if equilibrium
                                          else STEP_REF_KWARGS,
                               x0=EQUIL_X0.copy() if equilibrium else None),
        t_end           = t_end,
        use_equilibrium = equilibrium,
    )


# ──────────────────────────────────────────────────────────────────────
# Summary
# ──────────────────────────────────────────────────────────────────────

def print_summary(results):
    print("\n" + "="*60)
    print("  SIMULATION SUMMARY")
    print("="*60)
    for r in results:
        e_rms = float(r.e1[-max(1, len(r.e1)//5):].std())
        print(f"\n  [{r.label}]")
        print(f"    Duration          : {r.t[-1]:.2f} s")
        print(f"    Pos error RMS (ss): {e_rms:.4e} rad")
        print(f"    V(t_end)          : {r.lyapunov[-1]:.4e}")
        print(f"    E(t_end)          : {r.energy[-1]:.4e} J")
        print(f"    |u|_max           : {np.abs(r.u).max():.2f} N·m")
    print("="*60 + "\n")


# ──────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Flexible-Joint Backstepping Simulation Toolkit",
        formatter_class=argparse.RawTextHelpFormatter,
    )

    parser.add_argument("--plots",    action="store_true",
                        help="Time-domain state & error plots")
    parser.add_argument("--phase",    action="store_true",
                        help="Phase portraits")
    parser.add_argument("--lyapunov", action="store_true",
                        help="Lyapunov & energy plots")
    parser.add_argument("--shaft",    action="store_true",
                        help="Shaft-dynamics plots")
    parser.add_argument("--all",      action="store_true",
                        help="All four plot types")
    parser.add_argument("--gif",      action="store_true",
                        help="Save animation as GIF")
    parser.add_argument("--animate",  action="store_true",
                        help="Run interactive pygame animation")

    parser.add_argument(
        "--scenario",
        choices=["equilibrium", "step", "sin",
                 "ctrl_compare", "ctrl_compare_eq"],
        default="equilibrium",
        help=(
            "equilibrium    – backstepping to constant setpoint (Lyapunov proof scenario)\n"
            "step           – backstepping tracking smooth step trajectory\n"
            "sin            – backstepping sinusoidal tracking\n"
            "ctrl_compare   – Backstepping vs PD vs PID on smooth-step\n"
            "ctrl_compare_eq– Backstepping vs PD vs PID on equilibrium"
        ),
    )

    parser.add_argument("--t_end", type=float, default=10.0,
                        help="Simulation end time [s]  (default: 10.0)")

    args = parser.parse_args()

    if args.all:
        args.plots = args.phase = args.lyapunov = args.shaft = True

    any_plot = args.plots or args.phase or args.lyapunov or args.shaft

    print("\n" + "="*60)
    print("  Flexible-Joint Drive · Backstepping Simulation")
    print("="*60)

    t0      = time.perf_counter()
    results = []

    if args.scenario == "equilibrium":
        results.append(build_equilibrium(args.t_end))
    elif args.scenario == "step":
        results.append(build_step(args.t_end))
    elif args.scenario == "sin":
        results.append(build_sin(args.t_end))
    elif args.scenario == "ctrl_compare":
        results.extend(build_ctrl_compare(args.t_end, equilibrium=False))
    elif args.scenario == "ctrl_compare_eq":
        results.extend(build_ctrl_compare(args.t_end, equilibrium=True))

    print(f"\n  Simulation(s) completed in {time.perf_counter()-t0:.2f} s")
    print_summary(results)

    if any_plot:
        prefix = args.scenario
        print("Writing figures to  figures/  …")
        if args.plots:
            plot_states(results, filename=f"{prefix}_states.png")
        if args.phase:
            plot_phase_portraits(results, filename=f"{prefix}_phase_portraits.png")
        if args.lyapunov:
            plot_lyapunov(results, filename=f"{prefix}_lyapunov.png")
        if args.shaft:
            plot_shaft_dynamics(results, filename=f"{prefix}_shaft_dynamics.png")
        print("\nAll requested figures saved.\n")

    if args.animate or args.gif:
        print("\n  Animation requested.")
        create_animation(results, filename="animation.gif", save_gif=args.gif)

    if not any_plot and not args.animate and not args.gif:
        print("  No output flags specified.  Run  python main.py --help\n")


if __name__ == "__main__":
    main()