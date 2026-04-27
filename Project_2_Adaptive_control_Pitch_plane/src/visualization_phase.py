"""
visualization_phase.py
----------------------
Phase portraits of the longitudinal aircraft system with a time-based heatmap
colouring the trajectory.

Three portrait types are produced, each saved as TWO separate files:
    *_adaptive.png  — run with adaptation ON
    *_baseline.png  — run with adaptation OFF

Portrait types
--------------
    1.  alpha  vs  q       (angle-of-attack vs pitch-rate)
    2.  e_alpha vs r       (tracking error vs filtered error — Lyapunov error space)
    3.  alpha  vs  theta   (AoA vs pitch angle)

The trajectory colour encodes time: dark/purple = early, bright/yellow = late
(viridis colormap).  Events are marked:
    ◆  red diamond     : icing onset (t_ice)
    ▲  orange triangle : adaptive mode triggered (adaptive plot only)
    ●  white circle    : initial condition
    ★  white star      : equilibrium / reference point

Usage (standalone — runs its own 200 s simulation):
    python visualization_phase.py
    python visualization_phase.py --t-end 200 --t-ice 10 --out-dir results

Usage (from main.py or another module):
    from visualization_phase import plot_phase_portraits
    plot_phase_portraits(res_adp, res_base, save_dir="results")

Both res_adp (adaptation ON) and res_base (adaptation OFF) are required.
"""

from __future__ import annotations

import os
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.collections as mc
import matplotlib.colors as mcolors
from mpl_toolkits.axes_grid1 import make_axes_locatable


# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------

def _deg(r):
    return np.rad2deg(np.asarray(r, dtype=float))


def _colored_line(ax, x, y, t_norm, cmap="viridis", lw=2.0, alpha=0.92):
    """
    Draw a trajectory whose colour varies continuously with normalised time.
    Returns the LineCollection so the caller can attach a colorbar.
    """
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    pts  = np.stack([x, y], axis=1).reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc   = mc.LineCollection(
        segs,
        cmap=cmap,
        norm=mcolors.Normalize(vmin=0, vmax=1),
        linewidth=lw,
        alpha=alpha,
        zorder=3,
    )
    lc.set_array(t_norm[:-1])
    ax.add_collection(lc)
    return lc


def _colorbar(fig, ax, lc, t):
    """Attach a time colorbar to the right of *ax*."""
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="3%", pad=0.08)
    cb  = fig.colorbar(lc, cax=cax)
    cb.set_label("Time [s]", fontsize=9, color="#aab0bc")
    ticks_norm = [0.0, 0.25, 0.5, 0.75, 1.0]
    cb.set_ticks(ticks_norm)
    cb.set_ticklabels([f"{t[0] + v * (t[-1] - t[0]):.0f}" for v in ticks_norm])
    cb.ax.yaxis.set_tick_params(color="#aab0bc", labelsize=8)
    plt.setp(cb.ax.yaxis.get_ticklabels(), color="#aab0bc")
    return cb


def _mark_icing(ax, results, x_arr, y_arr, t):
    """Red diamond at icing onset."""
    t_ice   = results["t_ice"]
    idx_ice = int(np.searchsorted(t, t_ice))
    if 0 < idx_ice < len(x_arr):
        ax.scatter(x_arr[idx_ice], y_arr[idx_ice],
                   color="#e74c3c", s=80, zorder=7, marker="D",
                   label=f"Icing onset  t={t_ice:.0f} s")


def _mark_adaptive(ax, results, x_arr, y_arr, t):
    """Orange triangle at adaptive-mode trigger (adaptive run only)."""
    adp = results.get("adaptive_mode", np.zeros(len(t), dtype=bool))
    if np.any(adp):
        idx = int(np.argmax(adp))
        if 0 < idx < len(x_arr):
            ax.scatter(x_arr[idx], y_arr[idx],
                       color="#f39c12", s=100, zorder=7, marker="^",
                       label=f"Adaptive ON  t={t[idx]:.1f} s")


def _mark_start_end(ax, x_arr, y_arr):
    """White circle at t=0, small arrow showing initial direction."""
    ax.scatter(x_arr[0], y_arr[0],
               color="white", s=55, zorder=8, marker="o", label="t = 0")
    if len(x_arr) >= 5:
        dx = x_arr[4] - x_arr[0]
        dy = y_arr[4] - y_arr[0]
        norm = max(np.hypot(dx, dy), 1e-12)
        scale = max(abs(x_arr.max() - x_arr.min()),
                    abs(y_arr.max() - y_arr.min())) * 0.04
        ax.annotate(
            "", xy=(x_arr[0] + dx / norm * scale,
                    y_arr[0] + dy / norm * scale),
            xytext=(x_arr[0], y_arr[0]),
            arrowprops=dict(arrowstyle="-|>", color="white",
                            lw=1.2, mutation_scale=10),
            zorder=9,
        )


def _mark_equilibrium(ax, x_eq, y_eq):
    """White star at the desired equilibrium."""
    ax.scatter(x_eq, y_eq, color="white", s=120, zorder=9, marker="*",
               label="Equilibrium / ref")


# ---------------------------------------------------------------------------
# Single-run portrait renderer
# ---------------------------------------------------------------------------

_STYLE = {
    "figure.facecolor": "#0b0f1a",
    "axes.facecolor":   "#0f1420",
    "text.color":       "#d8dde8",
    "axes.labelcolor":  "#d8dde8",
    "xtick.color":      "#7a8494",
    "ytick.color":      "#7a8494",
    "axes.edgecolor":   "#2a3040",
    "grid.color":       "#1a2030",
    "grid.linewidth":   0.6,
    "figure.dpi":       160,
    "font.size":        9.5,
}


def _single_portrait(
    results: dict,
    run_label: str,
    x_key: str,
    y_key: str,
    xlabel: str,
    ylabel: str,
    title: str,
    x_deg: bool = False,
    y_deg: bool = False,
    equilibrium: tuple | None = None,
    diag_ref: bool = False,
    is_adaptive: bool = True,
) -> plt.Figure:
    """
    Render one phase portrait for a single simulation run.
    Returns the matplotlib Figure.
    """
    t      = results["t"]
    t_norm = (t - t[0]) / max(t[-1] - t[0], 1e-9)

    x_arr = _deg(results[x_key]) if x_deg else np.asarray(results[x_key], dtype=float)
    y_arr = _deg(results[y_key]) if y_deg else np.asarray(results[y_key], dtype=float)

    with plt.rc_context(_STYLE):
        fig, ax = plt.subplots(figsize=(8, 6))

        lc = _colored_line(ax, x_arr, y_arr, t_norm, cmap="viridis", lw=2.0)
        _colorbar(fig, ax, lc, t)

        _mark_icing(ax, results, x_arr, y_arr, t)
        if is_adaptive:
            _mark_adaptive(ax, results, x_arr, y_arr, t)
        _mark_start_end(ax, x_arr, y_arr)
        if equilibrium is not None:
            _mark_equilibrium(ax, *equilibrium)

        if diag_ref:
            lo = min(x_arr.min(), y_arr.min())
            hi = max(x_arr.max(), y_arr.max())
            ax.plot([lo, hi], [lo, hi],
                    color="#44446a", lw=1.0, ls="--", zorder=2,
                    label="θ = α  (level flight)")

        # Axis limits with padding
        x_span = x_arr.max() - x_arr.min()
        y_span = y_arr.max() - y_arr.min()
        pad_x  = max(x_span * 0.12, 0.1)
        pad_y  = max(y_span * 0.12, 0.1)
        ax.set_xlim(x_arr.min() - pad_x, x_arr.max() + pad_x)
        ax.set_ylim(y_arr.min() - pad_y, y_arr.max() + pad_y)

        ax.set_xlabel(xlabel, fontsize=10)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_title(f"{title}\n[{run_label}]",
                     fontsize=11, fontweight="bold", pad=8)
        ax.tick_params(labelsize=8.5)
        ax.grid(True, zorder=0)

        leg = ax.legend(fontsize=8, loc="best",
                        framealpha=0.45, facecolor="#0b0f1a",
                        edgecolor="#2a3040")
        for txt in leg.get_texts():
            txt.set_color("#d8dde8")

        fig.tight_layout()

    return fig


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def plot_phase_portraits(
    res_adp:  dict,
    res_base: dict,
    save_dir: str = "results",
) -> None:
    """
    Produce 3 portrait types x 2 runs = 6 separate PNG + PDF files.

    Files produced in *save_dir*:
        phase_alpha_q_adaptive.png / .pdf
        phase_alpha_q_baseline.png / .pdf
        phase_error_r_adaptive.png / .pdf
        phase_error_r_baseline.png / .pdf
        phase_alpha_theta_adaptive.png / .pdf
        phase_alpha_theta_baseline.png / .pdf

    Parameters
    ----------
    res_adp  : results dict from run_simulation(use_adaptation=True)
    res_base : results dict from run_simulation(use_adaptation=False)
    save_dir : directory where figures are saved
    """
    os.makedirs(save_dir, exist_ok=True)

    alpha_ref_deg = float(np.rad2deg(res_adp["alpha_ref"][0]))

    runs = [
        (res_adp,  "Adaptive (adaptation ON)",  "adaptive",  True),
        (res_base, "Baseline (adaptation OFF)", "baseline",  False),
    ]

    # Portrait 1 : alpha vs q
    for res, label, tag, is_adp in runs:
        fig = _single_portrait(
            res, label,
            x_key="alpha", y_key="q",
            xlabel="Angle of Attack  α [deg]",
            ylabel="Pitch Rate  q [deg/s]",
            title="Phase Portrait — α vs q",
            x_deg=True, y_deg=True,
            equilibrium=(alpha_ref_deg, 0.0),
            is_adaptive=is_adp,
        )
        _save(fig, save_dir, f"phase_alpha_q_{tag}")

    # Portrait 2 : e_alpha vs r  (Lyapunov error space)
    for res, label, tag, is_adp in runs:
        fig = _single_portrait(
            res, label,
            x_key="e_alpha", y_key="r",
            xlabel="Alpha Tracking Error  eα [deg]",
            ylabel="Filtered Error  r  [deg/s]",
            title="Phase Portrait — eα vs r  (Lyapunov error space)",
            x_deg=True, y_deg=True,
            equilibrium=(0.0, 0.0),
            is_adaptive=is_adp,
        )
        _save(fig, save_dir, f"phase_error_r_{tag}")

    # Portrait 3 : alpha vs theta
    for res, label, tag, is_adp in runs:
        fig = _single_portrait(
            res, label,
            x_key="alpha", y_key="theta",
            xlabel="Angle of Attack  α [deg]",
            ylabel="Pitch Angle  θ [deg]",
            title="Phase Portrait — α vs θ",
            x_deg=True, y_deg=True,
            diag_ref=True,
            is_adaptive=is_adp,
        )
        _save(fig, save_dir, f"phase_alpha_theta_{tag}")

    print(f"[phase] 6 phase portraits (3 types x 2 runs) saved to '{save_dir}/'")


def _save(fig, save_dir, stem):
    for ext in ("png", "pdf"):
        path = os.path.join(save_dir, f"{stem}.{ext}")
        fig.savefig(path, bbox_inches="tight",
                    facecolor=fig.get_facecolor())
        print(f"[phase] Saved -> {path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys
    sys.path.insert(0, os.path.dirname(__file__))
    from simulation import SimConfig, run_simulation

    ap = argparse.ArgumentParser()
    ap.add_argument("--t-end",    type=float, default=200.0,
                    help="Simulation duration [s] (default 200)")
    ap.add_argument("--t-ice",    type=float, default=10.0)
    ap.add_argument("--severity", type=float, default=0.30)
    ap.add_argument("--out-dir",  type=str,   default="results")
    args = ap.parse_args()

    delta_CL_alpha_ice = -args.severity * 3.50
    cfg = SimConfig(
        t_end=args.t_end,
        t_ice=args.t_ice,
        delta_CL_alpha_ice=delta_CL_alpha_ice,
    )

    print("Running adaptive simulation ...")
    res_adp  = run_simulation(config=cfg, use_adaptation=True)
    print("Running baseline simulation ...")
    res_base = run_simulation(config=cfg, use_adaptation=False)

    plot_phase_portraits(res_adp, res_base, save_dir=args.out_dir)