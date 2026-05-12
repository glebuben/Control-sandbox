"""
visualization.py
----------------
Publication-quality plots for the flexible-joint backstepping simulation.

Generates
---------
  • Time-domain state & error plots  (--plots)
  • Phase portraits  θ_l–ω_l and θ_m–ω_m  (--phase)
  • Lyapunov function V(t) and energy E(t)  (--lyapunov)

All figures are saved to the `figures/` directory at the project root.
Animation stubs are present but deferred per spec  (--gif / --animate).
"""

from __future__ import annotations
import os
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")          # headless backend – safe for server/CI
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
from matplotlib.ticker import AutoMinorLocator
from scipy.signal import savgol_filter

from simulation import SimResult


# ──────────────────────────────────────────────────────────────────────
# Paths
# ──────────────────────────────────────────────────────────────────────

_HERE       = Path(__file__).resolve().parent
_PROJ_ROOT  = _HERE.parent
FIGURES_DIR   = _PROJ_ROOT / "figures"
ANIMATIONS_DIR = _PROJ_ROOT / "animations"
FIGURES_DIR.mkdir(exist_ok=True)
ANIMATIONS_DIR.mkdir(exist_ok=True)


# ──────────────────────────────────────────────────────────────────────
# Style constants
# ──────────────────────────────────────────────────────────────────────

PALETTE = {
    "bg":        "#0d0f14",
    "panel":     "#13161e",
    "grid":      "#1f2433",
    "accent0":   "#4fc3f7",   # load / position
    "accent1":   "#f06292",   # motor
    "accent2":   "#aed581",   # reference / desired
    "accent3":   "#ffb74d",   # coupling torque
    "accent4":   "#ce93d8",   # energy / Lyapunov
    "error":     "#ef5350",
    "text":      "#e0e0e0",
    "subtext":   "#90a4ae",
}

_FONT_TITLE = {"fontsize": 13, "fontweight": "bold", "color": PALETTE["text"]}
_FONT_LABEL = {"fontsize": 10, "color": PALETTE["subtext"]}
_FONT_TICK  = {"labelsize": 8,  "colors": PALETTE["subtext"]}


def _apply_dark_style(fig: plt.Figure, axes):
    """Apply consistent dark engineering theme to a figure."""
    fig.patch.set_facecolor(PALETTE["bg"])
    if not hasattr(axes, "__iter__"):
        axes = [axes]
    for ax in np.array(axes).ravel():
        ax.set_facecolor(PALETTE["panel"])
        ax.tick_params(axis="both", **_FONT_TICK)
        ax.xaxis.set_minor_locator(AutoMinorLocator())
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.grid(which="major", color=PALETTE["grid"], linewidth=0.6, linestyle="-")
        ax.grid(which="minor", color=PALETTE["grid"], linewidth=0.3, linestyle=":")
        for spine in ax.spines.values():
            spine.set_edgecolor(PALETTE["grid"])
        ax.xaxis.label.set_color(PALETTE["subtext"])
        ax.yaxis.label.set_color(PALETTE["subtext"])
        ax.title.set_color(PALETTE["text"])


def _label_xy(ax, xlabel: str, ylabel: str):
    ax.set_xlabel(xlabel, **_FONT_LABEL)
    ax.set_ylabel(ylabel, **_FONT_LABEL)


def _save(fig: plt.Figure, name: str):
    path = FIGURES_DIR / name
    fig.savefig(path, dpi=160, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"  [saved] {path}")
    plt.close(fig)


# ──────────────────────────────────────────────────────────────────────
# 1 · Time-domain state & error plots
# ──────────────────────────────────────────────────────────────────────

def plot_states(results: list[SimResult],
                filename: str = "states.png") -> None:
    """
    Six-panel time-domain figure:
      Row 1 – angular positions (θ_l, θ_m) + reference
      Row 2 – angular velocities (ω_l, ω_m)
      Row 3 – coupling torque τ_c
      Row 4 – control input u
      Row 5 – tracking errors e₁, e₂
      Row 6 – torque error e₃
    """
    fig = plt.figure(figsize=(14, 16))
    gs  = gridspec.GridSpec(6, 1, hspace=0.55, figure=fig)
    axes = [fig.add_subplot(gs[i]) for i in range(6)]
    _apply_dark_style(fig, axes)

    for r in results:
        t = r.t
        lw = 1.6

        # ─ positions ─
        axes[0].plot(t, r.x[:, 0], color=PALETTE["accent0"], lw=lw,
                     label=f"θ_l  {r.label}")
        axes[0].plot(t, r.x[:, 2], color=PALETTE["accent1"], lw=lw,
                     linestyle="--", label=f"θ_m  {r.label}", alpha=0.8)
        axes[0].plot(t, r.theta_d, color=PALETTE["accent2"], lw=1.0,
                     linestyle=":", label="θ_d (ref)", alpha=0.9)

        # ─ velocities ─
        axes[1].plot(t, r.x[:, 1], color=PALETTE["accent0"], lw=lw,
                     label=f"ω_l  {r.label}")
        axes[1].plot(t, r.x[:, 3], color=PALETTE["accent1"], lw=lw,
                     linestyle="--", label=f"ω_m  {r.label}", alpha=0.8)
        axes[1].plot(t, r.dtheta_d, color=PALETTE["accent2"], lw=1.0,
                     linestyle=":", label="ω_d (ref)", alpha=0.9)

        # ─ coupling torque ─
        axes[2].plot(t, r.tau_c, color=PALETTE["accent3"], lw=lw,
                     label=f"τ_c  {r.label}")

        # ─ control input ─
        axes[3].plot(t, r.u, color=PALETTE["accent4"], lw=lw,
                     label=f"u  {r.label}")
        axes[3].axhline(0, color=PALETTE["grid"], lw=0.8, linestyle="--")

        # ─ tracking errors ─
        axes[4].plot(t, r.e1, color=PALETTE["accent0"], lw=lw,
                     label=f"e₁=θ_l−θ_d  {r.label}")
        axes[4].plot(t, r.e2, color=PALETTE["accent1"], lw=lw,
                     linestyle="--", label=f"e₂=ω_l−α₁  {r.label}", alpha=0.8)
        axes[4].axhline(0, color=PALETTE["grid"], lw=0.8, linestyle="--")

        # ─ torque error ─
        axes[5].plot(t, r.e3, color=PALETTE["error"], lw=lw,
                     label=f"e₃=τ_c−α₂  {r.label}")
        axes[5].axhline(0, color=PALETTE["grid"], lw=0.8, linestyle="--")

    # Labels & titles
    titles = [
        "Angular Positions  [rad]",
        "Angular Velocities  [rad/s]",
        "Coupling Torque τ_c  [N·m]",
        "Motor Torque Command u  [N·m]",
        "Tracking Errors e₁, e₂",
        "Torque Error e₃  [N·m]",
    ]
    ylabels = [
        "angle [rad]", "velocity [rad/s]",
        "torque [N·m]", "torque [N·m]",
        "error", "torque error [N·m]",
    ]
    for ax, ttl, yl in zip(axes, titles, ylabels):
        ax.set_title(ttl, **_FONT_TITLE)
        _label_xy(ax, "time [s]", yl)
        ax.legend(fontsize=7, framealpha=0.25,
                  labelcolor=PALETTE["text"], facecolor=PALETTE["panel"])

    fig.suptitle("Flexible-Joint Backstepping — Time Domain",
                 fontsize=15, fontweight="bold",
                 color=PALETTE["text"], y=1.005)
    _save(fig, filename)


# ──────────────────────────────────────────────────────────────────────
# 2 · Phase portraits
# ──────────────────────────────────────────────────────────────────────

def _colored_line(ax, x, y, cmap="plasma", lw=1.5):
    """Draw a line whose colour encodes progression in time."""
    pts   = np.array([x, y]).T.reshape(-1, 1, 2)
    segs  = np.concatenate([pts[:-1], pts[1:]], axis=1)
    norm  = plt.Normalize(0, len(x))
    lc    = LineCollection(segs, cmap=cmap, norm=norm, linewidth=lw, alpha=0.9)
    lc.set_array(np.arange(len(x)))
    ax.add_collection(lc)
    ax.autoscale()
    return lc


def plot_phase_portraits(results: list[SimResult],
                         filename: str = "phase_portraits.png") -> None:
    """
    Two-panel phase portrait:
      Left  – load side  (θ_l, ω_l)
      Right – motor side (θ_m, ω_m)

    Trajectory colour encodes time (dark → bright = early → late).
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    _apply_dark_style(fig, axes)

    cmaps = ["plasma", "viridis", "cool", "spring"]

    for idx, r in enumerate(results):
        cm = cmaps[idx % len(cmaps)]

        # Load-side portrait
        lc1 = _colored_line(axes[0], r.x[:, 0], r.x[:, 1], cmap=cm)
        axes[0].scatter(r.x[0, 0], r.x[0, 1], color="white",
                        zorder=5, s=40, marker="o", label=f"IC  {r.label}")
        axes[0].scatter(r.x[-1, 0], r.x[-1, 1], color=PALETTE["accent2"],
                        zorder=5, s=60, marker="*")

        # Motor-side portrait
        lc2 = _colored_line(axes[1], r.x[:, 2], r.x[:, 3], cmap=cm)
        axes[1].scatter(r.x[0, 2], r.x[0, 3], color="white",
                        zorder=5, s=40, marker="o", label=f"IC  {r.label}")
        axes[1].scatter(r.x[-1, 2], r.x[-1, 3], color=PALETTE["accent2"],
                        zorder=5, s=60, marker="*")

        # Colorbars
        cb1 = fig.colorbar(lc1, ax=axes[0], pad=0.02)
        cb1.set_label("time step", color=PALETTE["subtext"], fontsize=8)
        cb1.ax.yaxis.set_tick_params(color=PALETTE["subtext"])
        plt.setp(cb1.ax.yaxis.get_ticklabels(), color=PALETTE["subtext"])

    axes[0].set_title("Load Phase Portrait  (θ_l, ω_l)", **_FONT_TITLE)
    axes[1].set_title("Motor Phase Portrait  (θ_m, ω_m)", **_FONT_TITLE)
    _label_xy(axes[0], "θ_l  [rad]", "ω_l  [rad/s]")
    _label_xy(axes[1], "θ_m  [rad]", "ω_m  [rad/s]")

    for ax in axes:
        ax.legend(fontsize=7, framealpha=0.25,
                  labelcolor=PALETTE["text"], facecolor=PALETTE["panel"])

    fig.suptitle("Flexible-Joint Backstepping — Phase Portraits",
                 fontsize=15, fontweight="bold",
                 color=PALETTE["text"], y=1.01)
    _save(fig, filename)


# ──────────────────────────────────────────────────────────────────────
# 3 · Lyapunov & energy
# ──────────────────────────────────────────────────────────────────────

def plot_lyapunov(results: list[SimResult],
                  filename: str = "lyapunov.png") -> None:
    """
    Three-panel figure:
      Top    – Lyapunov function V(t) on log scale (should be strictly decreasing)
      Middle – Total mechanical energy E(t)
      Bottom – dV/dt (numerical) – should be ≤ 0 almost everywhere
    """
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    _apply_dark_style(fig, axes)
    fig.subplots_adjust(hspace=0.45)

    for r in results:
        t   = r.t
        V   = r.lyapunov
        E   = r.energy
        lw  = 1.8

        # Lyapunov on log scale
        V_safe = np.where(V > 1e-14, V, 1e-14)
        axes[0].semilogy(t, V_safe, lw=lw, color=PALETTE["accent4"],
                         label=r.label)

        # Energy
        axes[1].plot(t, E, lw=lw, color=PALETTE["accent3"], label=r.label)

        # Numerical dV/dt – Savitzky-Golay smoothed
        dV_raw = np.gradient(V, t)
        win = min(51, len(dV_raw) // 10 * 2 + 1)   # odd window ≤ 51
        win = max(win, 5)
        dV  = savgol_filter(dV_raw, window_length=win, polyorder=3)
        axes[2].plot(t, dV, lw=1.2, color=PALETTE["error"], label=r.label,
                     alpha=0.85)
        axes[2].axhline(0, color=PALETTE["grid"], lw=1.0, linestyle="--")
        axes[2].fill_between(t, dV, 0,
                              where=(dV > 0),
                              color=PALETTE["error"], alpha=0.20,
                              label="V̇ > 0 (violation)")

    axes[0].set_title("Composite Lyapunov Function  V(t) = ½e₁² + ½e₂² + ½e₃²",
                      **_FONT_TITLE)
    axes[1].set_title("Total Mechanical Energy  E(t)  [J]", **_FONT_TITLE)
    axes[2].set_title("Lyapunov Rate  dV/dt  (≤ 0 → stable)", **_FONT_TITLE)

    _label_xy(axes[0], "", "V(t)  [log]")
    _label_xy(axes[1], "", "E(t)  [J]")
    _label_xy(axes[2], "time [s]", "dV/dt")

    for ax in axes:
        ax.legend(fontsize=7, framealpha=0.25,
                  labelcolor=PALETTE["text"], facecolor=PALETTE["panel"])

    fig.suptitle("Flexible-Joint Backstepping — Lyapunov & Energy Analysis",
                 fontsize=15, fontweight="bold",
                 color=PALETTE["text"], y=1.005)
    _save(fig, filename)


# ──────────────────────────────────────────────────────────────────────
# 5 · Shaft dynamics plots  (new)
# ──────────────────────────────────────────────────────────────────────

def plot_shaft_dynamics(results: list[SimResult],
                        filename: str = "shaft_dynamics.png",
                        k: float = 50.0,
                        k3: float = 20.0) -> None:
    """
    Four-panel figure focusing on the internal flexible-shaft dynamics:

      (A) Shaft twist  Δθ = θ_m − θ_l
      (B) Relative velocity  Δω = ω_m − ω_l
      (C) Elastic potential energy  E_s = ½k·Δθ² + ¼k₃·Δθ⁴
      (D) Internal phase portrait  (Δθ, Δω)  – nonlinear oscillation
    """
    fig = plt.figure(figsize=(14, 12))
    gs  = gridspec.GridSpec(2, 2, hspace=0.50, wspace=0.35, figure=fig)
    ax_twist   = fig.add_subplot(gs[0, 0])
    ax_nu      = fig.add_subplot(gs[0, 1])
    ax_elastic = fig.add_subplot(gs[1, 0])
    ax_phase   = fig.add_subplot(gs[1, 1])
    axes       = [ax_twist, ax_nu, ax_elastic, ax_phase]
    _apply_dark_style(fig, axes)

    cmaps = ["plasma", "viridis", "cool", "spring"]

    for idx, r in enumerate(results):
        t      = r.t
        delta  = r.x[:, 2] - r.x[:, 0]         # Δθ
        nu     = r.x[:, 3] - r.x[:, 1]          # Δω
        E_s    = 0.5 * k * delta**2 + 0.25 * k3 * delta**4
        lw     = 1.6
        cm     = cmaps[idx % len(cmaps)]

        # (A) Shaft twist
        ax_twist.plot(t, delta, color=PALETTE["accent3"], lw=lw,
                      label=r.label)
        ax_twist.axhline(0, color=PALETTE["grid"], lw=0.8, linestyle="--")

        # (B) Relative velocity
        ax_nu.plot(t, nu, color=PALETTE["accent1"], lw=lw,
                   label=r.label)
        ax_nu.axhline(0, color=PALETTE["grid"], lw=0.8, linestyle="--")

        # (C) Elastic energy  — log scale when range spans orders of magnitude
        E_safe = np.where(E_s > 1e-12, E_s, 1e-12)
        if E_safe.max() / E_safe[E_safe > 1e-12].min() > 1e3:
            ax_elastic.semilogy(t, E_safe, color=PALETTE["accent4"],
                                lw=lw, label=r.label)
        else:
            ax_elastic.plot(t, E_s, color=PALETTE["accent4"],
                            lw=lw, label=r.label)

        # (D) Internal phase portrait  Δθ vs Δω  (time-coloured)
        lc = _colored_line(ax_phase, delta, nu, cmap=cm, lw=1.5)
        ax_phase.scatter(delta[0],  nu[0],  color="white",
                         zorder=5, s=40, marker="o", label=f"IC  {r.label}")
        ax_phase.scatter(delta[-1], nu[-1], color=PALETTE["accent2"],
                         zorder=5, s=60, marker="*")

    ax_twist.set_title("Shaft Twist  Δθ = θ_m − θ_l  [rad]",   **_FONT_TITLE)
    ax_nu.set_title("Relative Velocity  Δω = ω_m − ω_l  [rad/s]", **_FONT_TITLE)
    ax_elastic.set_title(
        "Elastic Potential  E_s = ½k·Δθ² + ¼k₃·Δθ⁴  [J]",     **_FONT_TITLE)
    ax_phase.set_title("Internal Phase Portrait  (Δθ, Δω)",      **_FONT_TITLE)

    _label_xy(ax_twist,   "time [s]", "Δθ [rad]")
    _label_xy(ax_nu,      "time [s]", "Δω [rad/s]")
    _label_xy(ax_elastic, "time [s]", "E_s [J]")
    _label_xy(ax_phase,   "Δθ [rad]", "Δω [rad/s]")

    for ax in axes:
        ax.legend(fontsize=7, framealpha=0.25,
                  labelcolor=PALETTE["text"], facecolor=PALETTE["panel"])

    fig.suptitle("Flexible-Joint Backstepping — Shaft Internal Dynamics",
                 fontsize=15, fontweight="bold",
                 color=PALETTE["text"], y=1.005)
    _save(fig, filename)


# ──────────────────────────────────────────────────────────────────────
# 6 · Updated convenience wrapper
# ──────────────────────────────────────────────────────────────────────

def generate_all_plots(results: list[SimResult],
                        prefix: str = "") -> None:
    """Call all plot functions with an optional filename prefix."""
    p = f"{prefix}_" if prefix else ""
    print("Generating time-domain state plots …")
    plot_states(results, filename=f"{p}states.png")
    print("Generating phase portraits …")
    plot_phase_portraits(results, filename=f"{p}phase_portraits.png")
    print("Generating Lyapunov / energy plots …")
    plot_lyapunov(results, filename=f"{p}lyapunov.png")
    print("Generating shaft dynamics plots …")
    plot_shaft_dynamics(results, filename=f"{p}shaft_dynamics.png")


# ──────────────────────────────────────────────────────────────────────
# 7 · Animation  (delegates to animation.py)
# ──────────────────────────────────────────────────────────────────────

def create_animation(results: list[SimResult],
                     filename: str = "animation.gif",
                     save_gif: bool = False) -> None:
    """
    Launch the pygame-based interactive animation.

    Uses the first SimResult in `results`.  If save_gif is True (or
    filename ends in .gif) the rendered frames are written to
    animations/<filename> via Pillow.

    Delegates all rendering to animation.py so this module stays
    matplotlib-only for static plots.
    """
    from animation import create_animation as _anim_run

    result   = results[0]
    gif_path = ANIMATIONS_DIR / filename if save_gif else None
    print(f"  Launching pygame animation (save_gif={save_gif}) …")
    _anim_run(result,
              save_gif=save_gif,
              gif_path=gif_path,
              gif_fps=20,
              gif_max_frames=500)