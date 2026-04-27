"""
visualization_lyapunov.py
-------------------------
Plot the Lyapunov function value V(t) over time for both the adaptive and
baseline controllers.

Lyapunov function
-----------------
The Lyapunov candidate used in the controller proof is:

    V(t)  =  (1/2) * r(t)²  +  (1/(2*gamma_C)) * (ΔĈ_Lα - ΔC_La*)²

where
    r            = e_q + lambda_alpha * e_alpha      (filtered tracking error)
    ΔĈ_Lα        = current parameter estimate
    ΔC_La*       = true (unknown) ΔC_La              (used here only for plotting)
    gamma_C      = adaptation gain

For the **baseline** controller, adaptation is OFF, so ΔĈ_Lα ≡ 0 always.
The adaptation term is therefore NOT subtracted and the Lyapunov value
simply tracks the squared filtered error:

    V_base(t)  =  (1/2) * r(t)²

This makes the comparison meaningful: the adaptive controller drives the
estimation error to zero, which should reduce V faster after icing onset.

Usage (standalone):
    python visualization_lyapunov.py

Usage (from other module):
    from visualization_lyapunov import plot_lyapunov
    plot_lyapunov(res_adp, res_base=res_base, save_dir="results")
"""

from __future__ import annotations

import os
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Lyapunov computation
# ---------------------------------------------------------------------------
DEFAULT_GAMMA_C    = 300.0   # matches controller default
DEFAULT_LAMBDA_ALP = 1.5     # matches controller default


def compute_lyapunov(results: dict,
                     gamma_C: float = DEFAULT_GAMMA_C,
                     use_adaptation: bool = True) -> np.ndarray:
    """
    Compute V(t) from logged simulation data.

    Parameters
    ----------
    results        : dict from run_simulation()
    gamma_C        : adaptation gain (must match the controller used)
    use_adaptation : if False, the estimation term is omitted

    Returns
    -------
    V_t : np.ndarray  shape (N,)
    """
    r          = results["r"]                     # filtered error [rad/s]
    delta_hat  = results["delta_CL_alpha_hat"]    # estimate
    true_delta = results["C_La_iced"] - results["C_La_clean"]  # true ΔC_La

    V_tracking   = 0.5 * r ** 2

    if use_adaptation:
        tilde        = delta_hat - true_delta          # estimation error
        V_estimation = 0.5 / gamma_C * tilde ** 2
    else:
        V_estimation = np.zeros_like(r)                # no adaptation term

    return V_tracking + V_estimation, V_tracking, V_estimation


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def plot_lyapunov(
    results: dict,
    res_base: dict | None = None,
    gamma_C: float = DEFAULT_GAMMA_C,
    save_dir: str = "results",
) -> None:
    """
    Produce and save the Lyapunov function plot.

    Parameters
    ----------
    results  : adaptive simulation results dict
    res_base : baseline simulation results dict (optional)
    gamma_C  : adaptation gain used in the controller
    save_dir : output directory
    """
    os.makedirs(save_dir, exist_ok=True)

    _STYLE = {
        "figure.facecolor": "#0d1117",
        "axes.facecolor":   "#111520",
        "text.color":       "#dde1ea",
        "axes.labelcolor":  "#dde1ea",
        "xtick.color":      "#8892a4",
        "ytick.color":      "#8892a4",
        "grid.color":       "#1e2535",
        "grid.linewidth":   0.6,
        "figure.dpi":       150,
        "font.size":        9.5,
    }

    with plt.rc_context(_STYLE):
        fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
        fig.suptitle(
            "Lyapunov Function  V(t) = ½r² + (1/2γ)·ΔC̃²\n"
            f"Icing onset at t = {results['t_ice']:.1f} s",
            fontsize=11, fontweight="bold",
        )

        t     = results["t"]
        t_ice = results["t_ice"]

        # --- Compute adaptive Lyapunov components -----------------------
        V_adp, V_tr_adp, V_est_adp = compute_lyapunov(
            results, gamma_C=gamma_C, use_adaptation=True)

        # --- Compute baseline Lyapunov components -----------------------
        V_bas, V_tr_bas, _ = (
            compute_lyapunov(res_base, gamma_C=gamma_C, use_adaptation=False)
            if res_base is not None else (None, None, None)
        )

        # ---------------------------------------------------------------
        # Panel 1 : total V(t)
        # ---------------------------------------------------------------
        ax = axes[0]
        ax.set_title("Total Lyapunov Value  V(t)", loc="left", fontweight="bold", fontsize=9)

        ax.plot(t, V_adp, color="#3a9eff", lw=1.8, label="Adaptive  V(t)", zorder=4)
        if V_bas is not None:
            ax.plot(res_base["t"], V_bas,
                    color="#e67e22", lw=1.8, ls="--", alpha=0.8,
                    label="Baseline  V(t)  (½r² only)", zorder=3)

        ax.set_ylabel("V(t)  [rad²]", fontsize=9)
        ax.set_yscale("symlog", linthresh=1e-6)
        ax.axvline(t_ice, color="#e74c3c", lw=1.2, ls="-.", alpha=0.7, label=f"Icing onset")
        _shade_icing(ax, t, t_ice)
        _draw_adapt_line(ax, results)
        ax.legend(fontsize=8, loc="upper right", framealpha=0.35, facecolor="#0d1117")
        ax.grid(True)

        # ---------------------------------------------------------------
        # Panel 2 : tracking term ½r²
        # ---------------------------------------------------------------
        ax = axes[1]
        ax.set_title("Tracking Term  ½ r²(t)", loc="left", fontweight="bold", fontsize=9)

        ax.plot(t, V_tr_adp, color="#3a9eff", lw=1.6, label="Adaptive  ½r²")
        if V_tr_bas is not None:
            ax.plot(res_base["t"], V_tr_bas,
                    color="#e67e22", lw=1.6, ls="--", alpha=0.8, label="Baseline  ½r²")

        ax.set_ylabel("½r²  [rad²/s²]", fontsize=9)
        ax.set_yscale("symlog", linthresh=1e-8)
        ax.axvline(t_ice, color="#e74c3c", lw=1.2, ls="-.", alpha=0.7)
        _shade_icing(ax, t, t_ice)
        _draw_adapt_line(ax, results)
        ax.legend(fontsize=8, loc="upper right", framealpha=0.35, facecolor="#0d1117")
        ax.grid(True)

        # ---------------------------------------------------------------
        # Panel 3 : estimation term  1/(2γ) · ΔC̃²
        # ---------------------------------------------------------------
        ax = axes[2]
        ax.set_title(
            "Estimation Term  (1/2γ)·ΔC̃²  [adaptive only — zero for baseline]",
            loc="left", fontweight="bold", fontsize=9,
        )

        ax.plot(t, V_est_adp, color="#80c080", lw=1.6, label="(1/2γ)·ΔC̃²")
        ax.axhline(0, color="#555577", lw=0.8, ls="--")

        # True ΔC_La reference line for context
        true_delta = results["C_La_iced"] - results["C_La_clean"]
        V_true_fixed = 0.5 / gamma_C * true_delta ** 2
        ax.axhline(V_true_fixed, color="#aa5555", lw=0.8, ls=":",
                   label=f"(1/2γ)·ΔC*²  = {V_true_fixed:.5f}  (worst-case)")

        ax.set_ylabel("(1/2γ)·ΔC̃²", fontsize=9)
        ax.set_xlabel("Time  [s]", fontsize=9)
        ax.set_yscale("symlog", linthresh=1e-8)
        ax.axvline(t_ice, color="#e74c3c", lw=1.2, ls="-.", alpha=0.7, label="Icing onset")
        _shade_icing(ax, t, t_ice)
        _draw_adapt_line(ax, results)
        ax.legend(fontsize=8, loc="upper right", framealpha=0.35, facecolor="#0d1117")
        ax.grid(True)

        # ---------------------------------------------------------------
        fig.tight_layout(rect=[0, 0, 1, 0.96])

        for ext in ("png", "pdf"):
            path = os.path.join(save_dir, f"lyapunov.{ext}")
            fig.savefig(path, bbox_inches="tight", facecolor=fig.get_facecolor())
            print(f"[lyapunov] Saved → {path}")

        plt.close(fig)

    print(f"[lyapunov] Plot saved to '{save_dir}/'")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------
def _shade_icing(ax, t, t_ice):
    ax.axvspan(t_ice, t[-1], color="#3a0a0a", alpha=0.30, zorder=0, label="_nolegend_")


def _draw_adapt_line(ax, results):
    adp = results.get("adaptive_mode", np.zeros(len(results["t"]), dtype=bool))
    if np.any(adp):
        idx = int(np.argmax(adp))
        t_ad = results["t"][idx]
        ax.axvline(t_ad, color="#f39c12", lw=1.1, ls="--", alpha=0.7,
                   label=f"Adaptive ON (t={t_ad:.1f}s)")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys
    sys.path.insert(0, os.path.dirname(__file__))
    from simulation import SimConfig, run_simulation

    ap = argparse.ArgumentParser()
    ap.add_argument("--t-end",    type=float, default=60.0)
    ap.add_argument("--t-ice",    type=float, default=10.0)
    ap.add_argument("--severity", type=float, default=0.30)
    ap.add_argument("--gamma-C",  type=float, default=DEFAULT_GAMMA_C)
    ap.add_argument("--out-dir",  type=str,   default="results")
    args = ap.parse_args()

    import numpy as np
    delta_CL_alpha_ice = -args.severity * 3.50
    cfg = SimConfig(t_end=args.t_end, t_ice=args.t_ice,
                    delta_CL_alpha_ice=delta_CL_alpha_ice)

    print("Running adaptive simulation …")
    res_adp  = run_simulation(config=cfg, use_adaptation=True)
    print("Running baseline simulation …")
    res_base = run_simulation(config=cfg, use_adaptation=False)

    plot_lyapunov(res_adp, res_base=res_base, gamma_C=args.gamma_C,
                  save_dir=args.out_dir)