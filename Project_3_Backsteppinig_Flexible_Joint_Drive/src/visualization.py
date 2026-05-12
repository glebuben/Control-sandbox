"""
visualization.py
----------------
Publication-quality plots for the flexible-joint backstepping simulation.

Generates
---------
  • Time-domain state & error plots  (--plots)
  • Phase portraits  θ_l–ω_l and θ_m–ω_m  (--phase)
  • Lyapunov function V(t) and energy E(t)  (--lyapunov)
  • Shaft internal dynamics  (--shaft)

All figures are saved to the `figures/` directory at the project root.
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
    "accent0":   "#4fc3f7",   # load / position / cyan
    "accent1":   "#f06292",   # motor / pink
    "accent2":   "#aed581",   # reference / desired / green
    "accent3":   "#ffb74d",   # coupling torque / amber
    "accent4":   "#ce93d8",   # energy / Lyapunov / purple
    "error":     "#ef5350",
    "text":      "#e0e0e0",
    "subtext":   "#90a4ae",
}

_FONT_TITLE = {"fontsize": 13, "fontweight": "bold", "color": PALETTE["text"]}
_FONT_LABEL = {"fontsize": 10, "color": PALETTE["subtext"]}
_FONT_TICK  = {"labelsize": 8,  "colors": PALETTE["subtext"]}

# ── Per-controller colour identity ────────────────────────────────────
# Backstepping gets a vivid cyan/teal family.
# PD gets amber/orange.
# PID gets coral/red.
# Any other result gets purple or a sequential fallback.
#
# Colours are (primary, secondary, reference) triples.
# primary   = main signal  (θ_l, ω_l, τ_c, u, e₁ …)
# secondary = aux signal   (θ_m, ω_m) drawn dashed, same hue but lighter
# ref_col   = reference signal θ_d (always green-ish)
_CTRL_COLORS = {
    "backstepping": ("#00e5ff",  "#80deea",  "#aed581"),   # cyan family
    "pd":           ("#ffab40",  "#ffe0b2",  "#c8e6c9"),   # amber family
    "pid":          ("#ff5252",  "#ff8a80",  "#e8f5e9"),   # red family
}
_FALLBACK_COLORS = [
    ("#ce93d8", "#e1bee7", "#aed581"),   # purple
    ("#80cbc4", "#b2dfdb", "#c8e6c9"),   # teal
    ("#fff176", "#fff9c4", "#dcedc8"),   # yellow
]

_PHASE_CMAPS = {
    "backstepping": "cool",
    "pd":           "autumn",
    "pid":          "hot",
}
_FALLBACK_CMAPS = ["plasma", "viridis", "spring"]


def _result_colors(r: SimResult, idx: int) -> tuple[str, str, str]:
    """Return (primary, secondary, ref) hex colours for a SimResult."""
    lbl = r.label.lower()
    for key, triple in _CTRL_COLORS.items():
        if key in lbl:
            return triple
    return _FALLBACK_COLORS[idx % len(_FALLBACK_COLORS)]


def _result_cmap(r: SimResult, idx: int) -> str:
    lbl = r.label.lower()
    for key, cm in _PHASE_CMAPS.items():
        if key in lbl:
            return cm
    return _FALLBACK_CMAPS[idx % len(_FALLBACK_CMAPS)]


def _sort_results(results: list[SimResult]) -> list[SimResult]:
    """
    Return results sorted so backstepping is plotted LAST (on top).
    Order: PD → PID → everything else → Backstepping.
    """
    def _rank(r: SimResult) -> int:
        lbl = r.label.lower()
        if "backstepping" in lbl:
            return 2      # drawn last = on top
        if "pid" in lbl:
            return 1
        return 0          # PD and unknowns first

    return sorted(results, key=_rank)


# ──────────────────────────────────────────────────────────────────────
# Theme helpers
# ──────────────────────────────────────────────────────────────────────

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


def _legend(ax):
    ax.legend(fontsize=7, framealpha=0.30,
              labelcolor=PALETTE["text"], facecolor=PALETTE["panel"],
              edgecolor=PALETTE["grid"])


# ──────────────────────────────────────────────────────────────────────
# 1 · Time-domain state & error plots
# ──────────────────────────────────────────────────────────────────────

def plot_states(results: list[SimResult],
                filename: str = "states.png") -> None:
    """
    Six-panel time-domain figure.
    Results are sorted so backstepping is drawn last (on top).
    Each controller gets a distinct colour family.
    """
    sorted_r = _sort_results(results)

    fig = plt.figure(figsize=(14, 16))
    gs  = gridspec.GridSpec(6, 1, hspace=0.55, figure=fig)
    axes = [fig.add_subplot(gs[i]) for i in range(6)]
    _apply_dark_style(fig, axes)

    ref_drawn = set()   # avoid duplicate θ_d legend entries

    for idx, r in enumerate(sorted_r):
        t = r.t
        pc, sc, rc = _result_colors(r, idx)
        lw  = 2.0 if "backstepping" in r.label.lower() else 1.4
        alp = 1.0 if "backstepping" in r.label.lower() else 0.75

        # ─ positions ─
        axes[0].plot(t, r.x[:, 0], color=pc, lw=lw, alpha=alp,
                     label=fr"$\theta_l$  [{r.label}]")
        axes[0].plot(t, r.x[:, 2], color=sc, lw=lw, alpha=alp * 0.8,
                     linestyle="--", label=fr"$\theta_m$  [{r.label}]")
        ref_lbl = r"$\theta_d$ (ref)" if "θ_d" not in ref_drawn else "_nolegend_"
        ref_drawn.add("θ_d")
        axes[0].plot(t, r.theta_d, color=rc, lw=1.0,
                     linestyle=":", alpha=0.9, label=ref_lbl)

        # ─ velocities ─
        axes[1].plot(t, r.x[:, 1], color=pc, lw=lw, alpha=alp,
                     label=fr"$\omega_l$  [{r.label}]")
        axes[1].plot(t, r.x[:, 3], color=sc, lw=lw, alpha=alp * 0.8,
                     linestyle="--", label=fr"$\omega_m$  [{r.label}]")

        # ─ coupling torque ─
        axes[2].plot(t, r.tau_c, color=pc, lw=lw, alpha=alp,
                     label=r.label)

        # ─ control input ─
        axes[3].plot(t, r.u, color=pc, lw=lw, alpha=alp,
                     label=r.label)
        axes[3].axhline(0, color=PALETTE["grid"], lw=0.8, linestyle="--")

        # ─ tracking errors ─
        axes[4].plot(t, r.e1, color=pc, lw=lw, alpha=alp,
                     label=fr"$e_1=\theta_l-\theta_d$  [{r.label}]")
        axes[4].plot(t, r.e2, color=sc, lw=lw, alpha=alp * 0.8,
                     linestyle="--", label=fr"$e_2$  [{r.label}]")
        axes[4].axhline(0, color=PALETTE["grid"], lw=0.8, linestyle="--")

        # ─ torque / integral error ─
        axes[5].plot(t, r.e3, color=pc, lw=lw, alpha=alp,
                     label=fr"$e_3$  [{r.label}]")
        axes[5].axhline(0, color=PALETTE["grid"], lw=0.8, linestyle="--")

    titles = [
        r"Angular Positions  [rad]",
        r"Angular Velocities  [rad/s]",
        r"Coupling Torque $\tau_c$  [N$\cdot$m]",
        r"Motor Torque Command $u$  [N$\cdot$m]",
        r"Tracking Errors $e_1$, $e_2$",
        r"Torque / Integral Error $e_3$",
    ]
    ylabels = [
        r"angle $\theta$ [rad]",
        r"velocity $\omega$ [rad/s]",
        r"$\tau_c$ [N$\cdot$m]",
        r"$u$ [N$\cdot$m]",
        r"error",
        r"error",
    ]
    for ax, ttl, yl in zip(axes, titles, ylabels):
        ax.set_title(ttl, **_FONT_TITLE)
        _label_xy(ax, "time [s]", yl)
        _legend(ax)

    fig.suptitle(r"Flexible-Joint Drive — Time Domain Comparison",
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
    Two-panel phase portrait (load side, motor side).
    Backstepping plotted last; each controller gets its own colormap.
    """
    sorted_r = _sort_results(results)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    _apply_dark_style(fig, axes)

    for idx, r in enumerate(sorted_r):
        cm  = _result_cmap(r, idx)
        pc, _, _ = _result_colors(r, idx)
        lw  = 2.0 if "backstepping" in r.label.lower() else 1.4

        lc1 = _colored_line(axes[0], r.x[:, 0], r.x[:, 1], cmap=cm, lw=lw)
        axes[0].scatter(r.x[0, 0], r.x[0, 1], color="white",
                        zorder=5, s=40, marker="o", label=fr"IC  [{r.label}]")
        axes[0].scatter(r.x[-1, 0], r.x[-1, 1], color=pc,
                        zorder=5, s=60, marker="*")

        _colored_line(axes[1], r.x[:, 2], r.x[:, 3], cmap=cm, lw=lw)
        axes[1].scatter(r.x[0, 2], r.x[0, 3], color="white",
                        zorder=5, s=40, marker="o", label=fr"IC  [{r.label}]")
        axes[1].scatter(r.x[-1, 2], r.x[-1, 3], color=pc,
                        zorder=5, s=60, marker="*")

    cb = fig.colorbar(lc1, ax=axes[0], pad=0.02)
    cb.set_label("time step", color=PALETTE["subtext"], fontsize=8)
    cb.ax.yaxis.set_tick_params(color=PALETTE["subtext"])
    plt.setp(cb.ax.yaxis.get_ticklabels(), color=PALETTE["subtext"])

    axes[0].set_title(r"Load Phase Portrait  $(\theta_l,\,\omega_l)$", **_FONT_TITLE)
    axes[1].set_title(r"Motor Phase Portrait  $(\theta_m,\,\omega_m)$", **_FONT_TITLE)
    _label_xy(axes[0], r"$\theta_l$  [rad]", r"$\omega_l$  [rad/s]")
    _label_xy(axes[1], r"$\theta_m$  [rad]", r"$\omega_m$  [rad/s]")
    for ax in axes:
        _legend(ax)

    fig.suptitle(r"Flexible-Joint Drive — Phase Portraits",
                 fontsize=15, fontweight="bold",
                 color=PALETTE["text"], y=1.01)
    _save(fig, filename)


# ──────────────────────────────────────────────────────────────────────
# 3 · Lyapunov & energy
# ──────────────────────────────────────────────────────────────────────

def plot_lyapunov(results: list[SimResult],
                  filename: str = "lyapunov.png") -> None:
    """
    Three-panel figure: V(t) log-scale, E(t), dV/dt.
    Backstepping plotted last; each controller gets its own colour.
    For equilibrium stabilisation V should decay monotonically to zero.
    For trajectory tracking V decays to a residual set.
    """
    sorted_r = _sort_results(results)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    _apply_dark_style(fig, axes)
    fig.subplots_adjust(hspace=0.45)

    for idx, r in enumerate(sorted_r):
        t   = r.t
        V   = r.lyapunov
        E   = r.energy
        pc, _, _ = _result_colors(r, idx)
        lw  = 2.0 if "backstepping" in r.label.lower() else 1.4
        alp = 1.0 if "backstepping" in r.label.lower() else 0.75

        V_safe = np.where(V > 1e-14, V, 1e-14)
        axes[0].semilogy(t, V_safe, lw=lw, alpha=alp, color=pc,
                         label=r.label)
        axes[1].plot(t, E, lw=lw, alpha=alp, color=pc,
                     label=r.label)

        dV_raw = np.gradient(V, t)
        win = min(51, max(5, len(dV_raw) // 10 * 2 + 1))
        dV  = savgol_filter(dV_raw, window_length=win, polyorder=3)
        axes[2].plot(t, dV, lw=max(1.0, lw - 0.4), alpha=alp,
                     color=pc, label=r.label)
        axes[2].fill_between(t, dV, 0, where=(dV > 0),
                             color=pc, alpha=0.15)

    axes[2].axhline(0, color=PALETTE["grid"], lw=1.0, linestyle="--")

    axes[0].set_title(r"Composite Lyapunov Function  $V(t) = \frac{1}{2}e_1^2 + \frac{1}{2}e_2^2 + \frac{1}{2}e_3^2$",
                      **_FONT_TITLE)
    axes[1].set_title(r"Total Mechanical Energy  $E(t)$  [J]", **_FONT_TITLE)
    axes[2].set_title(r"Lyapunov Rate  $\dot{V}$  ($\leq 0 \Rightarrow$ stable)",  **_FONT_TITLE)
    _label_xy(axes[0], "", r"$V(t)$  [log]")
    _label_xy(axes[1], "", r"$E(t)$  [J]")
    _label_xy(axes[2], r"time [s]", r"$\dot{V}$")
    for ax in axes:
        _legend(ax)

    fig.suptitle("Flexible-Joint Drive — Lyapunov & Energy Analysis",
                 fontsize=15, fontweight="bold",
                 color=PALETTE["text"], y=1.005)
    _save(fig, filename)


# ──────────────────────────────────────────────────────────────────────
# 4 · Shaft dynamics plots
# ──────────────────────────────────────────────────────────────────────

def plot_shaft_dynamics(results: list[SimResult],
                        filename: str = "shaft_dynamics.png",
                        k: float = 50.0,
                        k3: float = 20.0) -> None:
    """
    Four-panel figure: Δθ, Δω, elastic energy, internal phase portrait.
    Backstepping plotted last; distinct colours per controller.
    """
    sorted_r = _sort_results(results)

    fig = plt.figure(figsize=(14, 12))
    gs  = gridspec.GridSpec(2, 2, hspace=0.50, wspace=0.35, figure=fig)
    ax_twist   = fig.add_subplot(gs[0, 0])
    ax_nu      = fig.add_subplot(gs[0, 1])
    ax_elastic = fig.add_subplot(gs[1, 0])
    ax_phase   = fig.add_subplot(gs[1, 1])
    axes       = [ax_twist, ax_nu, ax_elastic, ax_phase]
    _apply_dark_style(fig, axes)

    for idx, r in enumerate(sorted_r):
        t      = r.t
        delta  = r.x[:, 2] - r.x[:, 0]
        nu     = r.x[:, 3] - r.x[:, 1]
        E_s    = 0.5 * k * delta**2 + 0.25 * k3 * delta**4
        pc, _, _ = _result_colors(r, idx)
        cm     = _result_cmap(r, idx)
        lw     = 2.0 if "backstepping" in r.label.lower() else 1.4
        alp    = 1.0 if "backstepping" in r.label.lower() else 0.75

        ax_twist.plot(t, delta, color=pc, lw=lw, alpha=alp, label=r.label)
        ax_twist.axhline(0, color=PALETTE["grid"], lw=0.8, linestyle="--")

        ax_nu.plot(t, nu, color=pc, lw=lw, alpha=alp, label=r.label)
        ax_nu.axhline(0, color=PALETTE["grid"], lw=0.8, linestyle="--")

        E_safe = np.where(E_s > 1e-12, E_s, 1e-12)
        if E_safe.max() / E_safe[E_safe > 1e-12].min() > 1e3:
            ax_elastic.semilogy(t, E_safe, color=pc, lw=lw, alpha=alp,
                                label=r.label)
        else:
            ax_elastic.plot(t, E_s, color=pc, lw=lw, alpha=alp,
                            label=r.label)

        _colored_line(ax_phase, delta, nu, cmap=cm, lw=lw)
        ax_phase.scatter(delta[0],  nu[0],  color="white",
                         zorder=5, s=40, marker="o", label=fr"IC  [{r.label}]")
        ax_phase.scatter(delta[-1], nu[-1], color=pc,
                         zorder=5, s=60, marker="*")

    ax_twist.set_title(r"Shaft Twist  $\Delta\theta = \theta_m - \theta_l$  [rad]",   **_FONT_TITLE)
    ax_nu.set_title(r"Relative Velocity  $\Delta\omega = \omega_m - \omega_l$  [rad/s]", **_FONT_TITLE)
    ax_elastic.set_title(r"Elastic Potential  $E_s = \frac{1}{2}k\Delta\theta^2 + \frac{1}{4}k_3\Delta\theta^4$  [J]",
                         **_FONT_TITLE)
    ax_phase.set_title(r"Internal Phase Portrait  $(\Delta\theta,\,\Delta\omega)$", **_FONT_TITLE)
    _label_xy(ax_twist,   r"time [s]", r"$\Delta\theta$ [rad]")
    _label_xy(ax_nu,      r"time [s]", r"$\Delta\omega$ [rad/s]")
    _label_xy(ax_elastic, r"time [s]", r"$E_s$ [J]")
    _label_xy(ax_phase,   r"$\Delta\theta$ [rad]", r"$\Delta\omega$ [rad/s]")
    for ax in axes:
        _legend(ax)

    fig.suptitle(r"Flexible-Joint Drive — Shaft Internal Dynamics",
                 fontsize=15, fontweight="bold",
                 color=PALETTE["text"], y=1.005)
    _save(fig, filename)


# ──────────────────────────────────────────────────────────────────────
# 5 · Convenience wrapper
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
# 6 · Animation  (delegates to animation.py)
# ──────────────────────────────────────────────────────────────────────

def create_animation(results: list[SimResult],
                     filename: str = "animation.gif",
                     save_gif: bool = False) -> None:
    """
    Launch the pygame-based interactive animation for each SimResult.

    When save_gif=True:
      • Each result produces its own GIF stored in animations/.
      • The filename stem is suffixed with a sanitised version of the
        result label so files don't overwrite each other.
      • GIFs are rendered via the offline path (exact playback speed,
        full resolution, Floyd-Steinberg dithering).
    The interactive window then opens for the first result.
    """
    from animation import create_animation as _anim_run

    stem    = Path(filename).stem
    suffix  = Path(filename).suffix or ".gif"

    def _safe(label: str) -> str:
        """Turn a result label into a safe filename fragment."""
        return label.lower().replace(" ", "_").replace("/", "-")[:32]

    # ── Save a GIF for every result ───────────────────────────────────
    if save_gif:
        for r in results:
            gif_name = f"{stem}_{_safe(r.label)}{suffix}"
            gif_path = ANIMATIONS_DIR / gif_name
            print(f"  Rendering GIF for [{r.label}] → {gif_name} …")
            _anim_run(r,
                      save_gif=True,
                      gif_path=gif_path,
                      gif_fps=20)

    # ── Interactive window for the first result ───────────────────────
    first = results[0]
    print(f"  Launching interactive animation for [{first.label}] …")
    _anim_run(first, save_gif=False, gif_fps=20)