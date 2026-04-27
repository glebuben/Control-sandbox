"""
main.py — entry point.

Simulation flags
----------------
  --t-end FLOAT       Total simulation time in seconds (default 200)
  --t-ice FLOAT       Icing onset time (default 10)
  --severity FLOAT    Fraction of C_La lost to icing (default 0.30)
  --V-trim FLOAT      Trim airspeed m/s (default 60)
  --alpha-ref FLOAT   Reference AoA in degrees (default = trim alpha 4 deg)
  --dt FLOAT          Integration timestep (default 0.01)

Plot / output flags  (all OFF by default — opt in to what you want)
--------------------------------------------------------------------
  --plots             Time-series matplotlib plots (comparison of both runs)
  --phase             Phase portraits saved to <out-dir>/phase_portraits/
  --lyapunov          Lyapunov function value plots saved to <out-dir>/
  --animation         Open interactive pygame aircraft window
  --animate-controller {adaptive,baseline,both}
                      Which controller to show in animation/GIF (default: adaptive)
  --lya-scale {log,linear}
                      Y-axis scale for Lyapunov strip in animation (default: log)
  --gif               Export animated GIF of aircraft view
  --gif-step INT      Sample every N-th frame for GIF (default 15)
  --gif-fps  INT      GIF frame rate (default 25)

Output directories
------------------
  --out-dir STR       Root output directory (default "results")
                      Plots are written to <out-dir>/figures/
                      Animations and GIFs are written to <out-dir>/animation/

Controller mode flags
---------------------
  --compare           (legacy) same as default — always runs both controllers
  --adaptation        (legacy) run adaptive only, skip baseline for speed

Examples
--------
  python main.py                             # run both, no plots
  python main.py --plots --lyapunov          # time-series + Lyapunov plots
  python main.py --phase                     # phase portraits only
  python main.py --gif                       # GIF only (no window)
  python main.py --animation --gif                          # adaptive window + GIF
  python main.py --animation --animate-controller baseline  # baseline window
  python main.py --gif --animate-controller both            # GIF for both
  python main.py --plots --phase --lyapunov --gif   # everything static + GIF
                                                    #   figures  -> results/figures/
                                                    #   GIF      -> results/animation/
"""

import argparse, os, sys
import numpy as np

from simulation            import SimConfig, run_simulation
from visualization         import plot_matplotlib
from visualization_phase   import plot_phase_portraits
from visualization_lyapunov import plot_lyapunov
from visualization_aircraft import run_aircraft_view, export_gif
import pygame


def _parse():
    p = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__,
    )
    # Simulation
    p.add_argument("--t-end",      type=float, default=200.0)
    p.add_argument("--t-ice",      type=float, default=10.0)
    p.add_argument("--severity",   type=float, default=0.30)
    p.add_argument("--V-trim",     type=float, default=60.0)
    p.add_argument("--alpha-ref",  type=float, default=None)
    p.add_argument("--dt",         type=float, default=0.01)
    p.add_argument("--out-dir",    type=str,   default="results")

    # Output selection  (opt-in)
    p.add_argument("--plots",      action="store_true",
                   help="Time-series matplotlib plots")
    p.add_argument("--phase",      action="store_true",
                   help="Phase portraits (saved in <out-dir>/phase_portraits/)")
    p.add_argument("--lyapunov",   action="store_true",
                   help="Lyapunov function value plots")
    p.add_argument("--animation",  action="store_true",
                   help="Open interactive pygame aircraft window")
    p.add_argument("--gif",        action="store_true",
                   help="Export animated GIF of aircraft visualisation")
    p.add_argument("--gif-step",   type=int, default=15,
                   help="Sample every N-th frame for GIF (default 15)")
    p.add_argument("--gif-fps",    type=int, default=25,
                   help="GIF frame rate (default 25)")
    p.add_argument("--animate-controller", type=str, default="adaptive",
                   choices=["adaptive", "baseline", "both"],
                   help="Controller to show in animation/GIF (default: adaptive)")
    p.add_argument("--lya-scale", type=str, default="log",
                   choices=["log", "linear"],
                   help="Y-axis scale for Lyapunov strip in animation (default: log)")

    # Legacy controller-mode flags (kept for backwards compatibility)
    p.add_argument("--adaptation", action="store_true",
                   help="(legacy) run adaptive controller only")
    p.add_argument("--compare",    action="store_true",
                   help="(legacy) run both controllers and compare — this is the default")

    # Legacy disable flags (kept for backwards compatibility)
    p.add_argument("--no-matplotlib", action="store_true",
                   help="(legacy) suppress all matplotlib output")
    p.add_argument("--no-pygame",     action="store_true",
                   help="(legacy) suppress pygame window")

    return p.parse_args()


def _summary(results, label=""):
    t   = results["t"]
    adp = results["adaptive_mode"]

    sw   = int(np.argmax(adp))
    t_sw = float(t[sw]) if adp[sw] else None

    hat        = results["delta_CL_alpha_hat"][-1]
    true_delta = results["C_La_iced"] - results["C_La_clean"]
    e_max      = np.rad2deg(np.max(np.abs(results["e_alpha"])))
    e_fin      = np.rad2deg(results["e_alpha"][-1])

    print("=" * 58)
    print(f"  SIMULATION SUMMARY  {label}")
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

    C_La_nominal       = 3.50
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

    # ---- Always run both controllers -----------------------------------
    print("\nRunning Baseline Controller (adaptation OFF) ...")
    res_base = run_simulation(config=cfg, use_adaptation=False)

    print("\nRunning Adaptive Controller (adaptation ON) ...")
    res_adp  = run_simulation(config=cfg, use_adaptation=True)

    print("\n--- BASELINE SUMMARY ---")
    _summary(res_base, "BASELINE")
    print("\n--- ADAPTIVE SUMMARY ---")
    _summary(res_adp,  "ADAPTIVE")

    os.makedirs(args.out_dir, exist_ok=True)
    figures_dir = os.path.join(args.out_dir, "figures")
    anim_dir    = os.path.join(args.out_dir, "animation")

    # ---- Legacy --no-matplotlib flag overrides new flags ---------------
    if args.no_matplotlib:
        args.plots = args.phase = args.lyapunov = False

    # ---- Time-series plots ---------------------------------------------
    if args.plots:
        print("\n[plots] Generating time-series plots ...")
        os.makedirs(figures_dir, exist_ok=True)
        plot_matplotlib(
            res_adp,
            res_base=res_base,
            save_path=os.path.join(figures_dir, "comparison"),
        )

    # ---- Phase portraits -----------------------------------------------
    if args.phase:
        phase_dir = os.path.join(figures_dir, "phase_portraits")
        print(f"\n[phase] Generating phase portraits → {phase_dir}/")
        plot_phase_portraits(res_adp, res_base, save_dir=phase_dir)

    # ---- Lyapunov plots ------------------------------------------------
    if args.lyapunov:
        os.makedirs(figures_dir, exist_ok=True)
        print(f"\n[lyapunov] Generating Lyapunov plots → {figures_dir}/")
        plot_lyapunov(res_adp, res_base=res_base, save_dir=figures_dir,
                      lya_scale=args.lya_scale)

    # ---- Determine which result(s) to animate -------------------------
    _anim_ctrl = args.animate_controller
    _anim_runs = []
    if _anim_ctrl in ("adaptive", "both"):
        _anim_runs.append((res_adp,  "Adaptive"))
    if _anim_ctrl in ("baseline", "both"):
        _anim_runs.append((res_base, "Baseline"))

    # ---- GIF export (headless, no display needed) ----------------------
    if args.gif:
        os.makedirs(anim_dir, exist_ok=True)
        pygame.init()
        for _res, _lbl in _anim_runs:
            _gif_path = os.path.join(anim_dir, f"aircraft_{_lbl.lower()}.gif")
            print(f"\n[gif] Exporting {_lbl} GIF -> {_gif_path}")
            export_gif(_res, _gif_path,
                       step=args.gif_step, fps=args.gif_fps, label=_lbl,
                       lya_scale=args.lya_scale)
        pygame.quit()

    # ---- Interactive aircraft window -----------------------------------
    want_window = args.animation and not args.no_pygame
    if want_window:
        os.makedirs(anim_dir, exist_ok=True)
        for _res, _lbl in _anim_runs:
            _gif_path = os.path.join(anim_dir, f"aircraft_{_lbl.lower()}.gif")
            print(f"\n[animation] Opening {_lbl} aircraft window ...")
            run_aircraft_view(_res, label=_lbl, gif_path=_gif_path,
                              lya_scale=args.lya_scale)


if __name__ == "__main__":
    main()