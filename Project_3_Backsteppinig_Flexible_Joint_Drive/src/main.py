"""
main.py
-------
Entry point for the flexible-joint backstepping simulation toolkit.

Usage
-----
  python main.py                     # run default simulation, print summary
  python main.py --plots             # time-domain state / error plots
  python main.py --phase             # phase portraits
  python main.py --lyapunov          # Lyapunov & energy plots
  python main.py --plots --phase --lyapunov   # all three plot types

  python main.py --gif --animate     # interactive pygame viz + save GIF
                                     # (animation module deferred)

  python main.py --scenario step     # smooth step reference (default)
  python main.py --scenario sin      # sinusoidal tracking
  python main.py --scenario compare  # overlay step + sinusoidal results

  python main.py --t_end 15          # simulation duration in seconds
  python main.py --all               # equivalent to --plots --phase --lyapunov
"""

from __future__ import annotations
import argparse
import sys
import time
from pathlib import Path

# Ensure src/ is on the Python path when called from project root
_SRC = Path(__file__).resolve().parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from system        import SystemParams
from controller    import ControllerGains
from simulation    import (run_simulation, run_step_response,
                           run_sinusoidal_tracking)
from visualization import (plot_states, plot_phase_portraits,
                            plot_lyapunov, plot_shaft_dynamics,
                            generate_all_plots, create_animation)


# ──────────────────────────────────────────────────────────────────────
# Default scenario configurations
# ──────────────────────────────────────────────────────────────────────

DEFAULT_SYS_PARAMS = SystemParams(
    J_l=0.5,  J_m=0.1,
    k=50.0,   k3=20.0,  b=1.0,
    Fc_l=0.3, Fc_m=0.2, v_s=0.1,
    Bv_l=0.05, Bv_m=0.03,
)

DEFAULT_GAINS = ControllerGains(k1=5.0, k2=8.0, k3=10.0, k4=15.0)

STEP_REF_KWARGS = dict(amplitude=1.0, t_rise=1.5, t_start=0.5)
SIN_REF_KWARGS  = dict(amplitude=0.8, frequency=0.4)


# ──────────────────────────────────────────────────────────────────────
# Scenario builders
# ──────────────────────────────────────────────────────────────────────

def build_step_result(t_end: float):
    print("  Running smooth-step scenario …")
    return run_simulation(
        sys_params  = DEFAULT_SYS_PARAMS,
        ctrl_gains  = DEFAULT_GAINS,
        x0          = None,                 # start at rest / zero
        t_span      = (0.0, t_end),
        dt          = 5e-4,
        reference   = "smooth_step",
        ref_kwargs  = STEP_REF_KWARGS,
        u_clip      = 200.0,
        label       = "Step (σ-smooth)",
    )


def build_sin_result(t_end: float):
    print("  Running sinusoidal tracking scenario …")
    return run_simulation(
        sys_params  = DEFAULT_SYS_PARAMS,
        ctrl_gains  = DEFAULT_GAINS,
        x0          = None,
        t_span      = (0.0, t_end),
        dt          = 5e-4,
        reference   = "sinusoidal",
        ref_kwargs  = SIN_REF_KWARGS,
        u_clip      = 200.0,
        label       = "Sinusoidal 0.4 Hz",
    )


# ──────────────────────────────────────────────────────────────────────
# Print helper
# ──────────────────────────────────────────────────────────────────────

def print_summary(results):
    print("\n" + "="*60)
    print("  SIMULATION SUMMARY")
    print("="*60)
    for r in results:
        t_ss  = r.t[-1]
        e_rms = float(r.e1[-len(r.e1)//5:].std())          # RMS of last 20 %
        V_end = float(r.lyapunov[-1])
        E_end = float(r.energy[-1])
        u_max = float(r.u.__abs__().max())
        print(f"\n  [{r.label}]")
        print(f"    Duration          : {t_ss:.2f} s")
        print(f"    Pos error RMS (ss): {e_rms:.4e} rad")
        print(f"    V(t_end)          : {V_end:.4e}")
        print(f"    E(t_end)          : {E_end:.4e} J")
        print(f"    |u|_max           : {u_max:.2f} N·m")
    print("="*60 + "\n")


# ──────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Flexible-Joint Backstepping Simulation Toolkit",
        formatter_class=argparse.RawTextHelpFormatter,
    )

    # Plot flags
    parser.add_argument("--plots",    action="store_true",
                        help="Generate time-domain state & error plots")
    parser.add_argument("--phase",    action="store_true",
                        help="Generate phase portraits")
    parser.add_argument("--lyapunov", action="store_true",
                        help="Generate Lyapunov & energy plots")
    parser.add_argument("--all",      action="store_true",
                        help="Generate all three plot types")

    parser.add_argument("--shaft",    action="store_true",
                        help="Generate shaft-dynamics plots (Δθ, Δω, E_s, internal phase)")
    parser.add_argument("--gif",     action="store_true",
                        help="Save animation as GIF (requires --animate)")
    parser.add_argument("--animate", action="store_true",
                        help="Run interactive pygame animation")

    # Scenario
    parser.add_argument("--scenario",
                        choices=["step", "sin", "compare"],
                        default="step",
                        help=("Scenario to simulate:\n"
                              "  step    – smooth sigmoid step  (default)\n"
                              "  sin     – sinusoidal tracking\n"
                              "  compare – both overlaid"))

    # Duration
    parser.add_argument("--t_end", type=float, default=10.0,
                        help="Simulation end time [s]  (default: 10.0)")

    args = parser.parse_args()

    # Expand --all shorthand
    if args.all:
        args.plots = args.phase = args.lyapunov = args.shaft = True

    any_plot = args.plots or args.phase or args.lyapunov or args.shaft

    print("\n" + "="*60)
    print("  Flexible-Joint Drive · Backstepping Simulation")
    print("="*60)

    # ── Build results ────────────────────────────────────────────────
    t0 = time.perf_counter()
    results = []

    if args.scenario in ("step", "compare"):
        results.append(build_step_result(args.t_end))

    if args.scenario in ("sin", "compare"):
        results.append(build_sin_result(args.t_end))

    elapsed = time.perf_counter() - t0
    print(f"\n  Simulation(s) completed in {elapsed:.2f} s")

    print_summary(results)

    # ── Generate plots ───────────────────────────────────────────────
    if any_plot:
        prefix = args.scenario
        print(f"Writing figures to  figures/  …")

        if args.plots:
            plot_states(results,
                        filename=f"{prefix}_states.png")

        if args.phase:
            plot_phase_portraits(results,
                                 filename=f"{prefix}_phase_portraits.png")

        if args.lyapunov:
            plot_lyapunov(results,
                          filename=f"{prefix}_lyapunov.png")

        if args.shaft:
            plot_shaft_dynamics(results,
                                filename=f"{prefix}_shaft_dynamics.png")

        print("\nAll requested figures saved.\n")

    # ── Animation ────────────────────────────────────────────────────
    if args.animate or args.gif:
        print("\n  Animation requested.")
        create_animation(results,
                         filename="animation.gif",
                         save_gif=args.gif)

    # If no flags given at all, just print summary (already done)
    if not any_plot and not args.animate and not args.gif:
        print("  No output flags specified.  Use --plots, --phase, "
              "--lyapunov, or --all to generate figures.\n"
              "  Run  python main.py --help  for full usage.\n")


if __name__ == "__main__":
    main()