"""
main.py — entry point.

Usage:
    python main.py                            # defaults: 30s, icing at 10s, 30% severity
    python main.py --t-ice 8 --severity 0.4  # custom scenario
    python main.py --no-pygame               # skip interactive dashboard
    python main.py --no-matplotlib           # skip static plots
"""

import argparse, os, sys
import numpy as np

from simulation           import SimConfig, run_simulation
from visualization        import plot_matplotlib
from visualization_pygame import run_pygame_dashboard


def _parse():
    p = argparse.ArgumentParser()
    p.add_argument("--t-end",     type=float, default=200.0)
    p.add_argument("--t-ice",     type=float, default=10.0)
    p.add_argument("--severity",  type=float, default=0.30,
                   help="Fraction of C_La lost to icing (default 0.30 = 30%%)")
    p.add_argument("--V-trim",    type=float, default=60.0)
    p.add_argument("--alpha-ref", type=float, default=None,
                   help="Reference alpha [deg]; default = trim alpha (4 deg)")
    p.add_argument("--dt",        type=float, default=0.01)
    p.add_argument("--no-matplotlib", action="store_true")
    p.add_argument("--no-pygame",     action="store_true")
    p.add_argument("--out-dir",   type=str,   default="results")
    return p.parse_args()


def _summary(results):
    t   = results["t"]
    adp = results["adaptive_mode"]

    sw  = int(np.argmax(adp))
    t_sw = float(t[sw]) if adp[sw] else None

    hat        = results["delta_CL_alpha_hat"][-1]
    true_delta = results["C_La_iced"] - results["C_La_clean"]
    e_max      = np.rad2deg(np.max(np.abs(results["e_alpha"])))
    e_fin      = np.rad2deg(results["e_alpha"][-1])

    print("=" * 58)
    print("  SIMULATION SUMMARY")
    print("=" * 58)
    print(f"  Duration              : {t[-1]:.1f} s")
    print(f"  C_La clean / iced     : {results['C_La_clean']:.3f} / {results['C_La_iced']:.3f}")
    print(f"  True ΔC_La            : {true_delta:.4f}")
    if t_sw:
        print(f"  Adaptive mode ON      : t = {t_sw:.3f} s  "
              f"({t_sw - results['t_ice']:.3f} s after icing)")
    else:
        print("  Adaptive mode         : never triggered")
    print(f"  Max |e_alpha|         : {e_max:.4f} deg")
    print(f"  Final e_alpha         : {e_fin:.4f} deg")
    print(f"  Final ΔĈ_La           : {hat:.4f}  (true: {true_delta:.4f})")
    if abs(true_delta) > 1e-6:
        print(f"  Estimate accuracy     : {100*hat/true_delta:.1f} %")
    print("=" * 58)


def main():
    args = _parse()

    C_La_nominal       = 3.50          # must match system.py
    delta_CL_alpha_ice = -args.severity * C_La_nominal
    alpha_ref_rad      = np.deg2rad(args.alpha_ref) if args.alpha_ref else None

    cfg = SimConfig(
        t_end              = args.t_end,
        t_ice              = args.t_ice,
        dt                 = args.dt,
        V_trim             = args.V_trim,
        alpha_trim         = np.deg2rad(4.0),
        delta_CL_alpha_ice = delta_CL_alpha_ice,
        alpha_ref          = alpha_ref_rad,
    )

    print("\n" + "="*58)
    print("  Lyapunov Adaptive Icing Controller")
    print("="*58)
    results = run_simulation(config=cfg)
    _summary(results)

    if not args.no_matplotlib:
        os.makedirs(args.out_dir, exist_ok=True)
        print()
        plot_matplotlib(results, save_path=os.path.join(args.out_dir, "simulation"))

    if not args.no_pygame:
        print("\nLaunching pygame dashboard …")
        print("  [A] auto-play  [SPACE/→/←] step  [+/-] speed  [R] restart  [Q] quit")
        run_pygame_dashboard(results)

    if not args.no_matplotlib:
        print(f"\nStatic plots saved in '{args.out_dir}/'")


if __name__ == "__main__":
    main()