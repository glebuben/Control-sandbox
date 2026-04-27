"""
visualization.py
----------------
Publication-quality plots for the adaptive icing simulation.

Two backends
------------
  plot_matplotlib(results, save_path)
      → multi-panel figure  saved as  <save_path>.png  and  <save_path>.pdf

  plot_plotly(results, save_path)
      → interactive multi-panel figure  saved as  <save_path>.html

Both functions accept the dict returned by simulation.run_simulation().
"""

from __future__ import annotations

import os
import numpy as np


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------
def _deg(rad_array):
    """Convert radian array to degrees."""
    return np.rad2deg(rad_array)


def _icing_shade_mpl(ax, t, adaptive_mode, t_ice, color="#e74c3c", alpha=0.10):
    """Add a shaded region from t_ice to end of simulation (Matplotlib)."""
    ax.axvspan(t_ice, t[-1], color=color, alpha=alpha, zorder=0, label="Icing active")


def _mode_switch_line_mpl(ax, t, adaptive_mode, color="#e74c3c", lw=1.2):
    """Draw a vertical dashed line when adaptive mode switches on."""
    switch_idx = np.argmax(adaptive_mode)   # first True
    if adaptive_mode[switch_idx]:
        t_switch = t[switch_idx]
        ax.axvline(t_switch, color=color, lw=lw, ls="--",
                   label=f"Adaptive ON  t={t_switch:.2f}s")


# ---------------------------------------------------------------------------
# Matplotlib
# ---------------------------------------------------------------------------
def plot_matplotlib(results: dict, save_path: str = "results/simulation") -> None:
    """
    Produce a 5-row figure:
      1. Angle of attack  (α, α_ref)
      2. Pitch rate  q
      3. Elevator deflection  (total, nominal, adaptive)
      4. Adaptation estimate  ΔĈ_Lα
      5. Airspeed  V

    Saves as .png and .pdf.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
        from matplotlib.patches import Patch
    except ImportError as e:
        print(f"[visualization] matplotlib not available: {e}")
        return

    os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)

    t   = results["t"]
    adp = results["adaptive_mode"]
    t_ice = results["t_ice"]

    # ---- Style --------------------------------------------------------
    plt.rcParams.update({
        "font.family":      "DejaVu Sans",
        "font.size":        9.5,
        "axes.linewidth":   0.8,
        "axes.spines.top":  False,
        "axes.spines.right":False,
        "lines.linewidth":  1.6,
        "legend.fontsize":  8,
        "legend.framealpha":0.7,
        "figure.dpi":       150,
    })

    fig = plt.figure(figsize=(11, 13))
    fig.suptitle(
        "Lyapunov Adaptive Icing Controller — Simulation Results\n"
        f"(icing onset at t = {t_ice} s,  "
        f"C_Lα: {results['C_La_clean']:.3f} → {results['C_La_iced']:.3f})",
        fontsize=11, fontweight="bold", y=0.98
    )

    gs = gridspec.GridSpec(5, 1, hspace=0.55, left=0.10, right=0.96,
                           top=0.93, bottom=0.05)

    axes = [fig.add_subplot(gs[i]) for i in range(5)]

    col_nom   = "#2c7bb6"
    col_adapt = "#d7191c"
    col_total = "#1a1a2e"
    col_ref   = "#555555"
    col_ice   = "#e74c3c"

    # ---- 1. Angle of attack -------------------------------------------
    ax = axes[0]
    ax.plot(t, _deg(results["alpha_ref"]), color=col_ref, ls="--", lw=1.2,
            label="α_ref")
    ax.plot(t, _deg(results["alpha"]),     color=col_nom,
            label="α (aircraft)")
    _icing_shade_mpl(ax, t, adp, t_ice)
    _mode_switch_line_mpl(ax, t, adp, col_ice)
    ax.set_ylabel("α  [deg]")
    ax.set_title("Angle of Attack", loc="left", fontsize=9)
    ax.legend(loc="upper right", ncol=3)
    ax.set_xlim(t[0], t[-1])

    # ---- 2. Pitch rate ------------------------------------------------
    ax = axes[1]
    ax.plot(t, _deg(results["q"]), color=col_nom, label="q")
    ax.axhline(0, color=col_ref, lw=0.8, ls="--")
    _icing_shade_mpl(ax, t, adp, t_ice)
    _mode_switch_line_mpl(ax, t, adp, col_ice)
    ax.set_ylabel("q  [deg/s]")
    ax.set_title("Pitch Rate", loc="left", fontsize=9)
    ax.legend(loc="upper right", ncol=2)
    ax.set_xlim(t[0], t[-1])

    # ---- 3. Elevator deflection ---------------------------------------
    ax = axes[2]
    ax.plot(t, _deg(results["delta_e_nom"]),   color=col_nom,
            ls="--", lw=1.2, label="δe nominal")
    ax.plot(t, _deg(results["delta_e_adapt"]), color=col_adapt,
            ls=":",  lw=1.5, label="δe adaptive")
    ax.plot(t, _deg(results["delta_e"]),        color=col_total,
            lw=1.8,           label="δe total")
    _icing_shade_mpl(ax, t, adp, t_ice)
    _mode_switch_line_mpl(ax, t, adp, col_ice)
    delta_e_max_deg = np.rad2deg(np.max(np.abs(results["delta_e"]))) * 1.3 + 2
    ax.set_ylim(-delta_e_max_deg, delta_e_max_deg)
    ax.set_ylabel("δe  [deg]")
    ax.set_title("Elevator Deflection", loc="left", fontsize=9)
    ax.legend(loc="upper right", ncol=3)
    ax.set_xlim(t[0], t[-1])

    # ---- 4. Adaptation estimate ---------------------------------------
    ax = axes[3]
    ax.plot(t, results["delta_CL_alpha_hat"], color=col_adapt, label="ΔĈ_Lα")
    ax.axhline(results["C_La_iced"] - results["C_La_clean"],
               color="grey", lw=1.0, ls="--",
               label=f"True ΔC_Lα = {results['C_La_iced'] - results['C_La_clean']:.3f}")
    _icing_shade_mpl(ax, t, adp, t_ice)
    _mode_switch_line_mpl(ax, t, adp, col_ice)
    ax.set_ylabel("ΔĈ_Lα  [–]")
    ax.set_title("Adaptive Estimate of Lift Degradation", loc="left", fontsize=9)
    ax.legend(loc="lower right", ncol=2)
    ax.set_xlim(t[0], t[-1])

    # ---- 5. Airspeed --------------------------------------------------
    ax = axes[4]
    ax.plot(t, results["V"], color=col_nom, label="V")
    _icing_shade_mpl(ax, t, adp, t_ice)
    _mode_switch_line_mpl(ax, t, adp, col_ice)
    ax.set_ylabel("V  [m/s]")
    ax.set_xlabel("Time  [s]")
    ax.set_title("Airspeed", loc="left", fontsize=9)
    ax.legend(loc="upper right", ncol=2)
    ax.set_xlim(t[0], t[-1])

    # ---- Common x-tick & icing annotation ----------------------------
    for ax in axes:
        ax.axvline(t_ice, color=col_ice, lw=0.9, ls="-.", alpha=0.6)

    # ---- Save ---------------------------------------------------------
    for ext in ("png", "pdf"):
        fpath = f"{save_path}.{ext}"
        fig.savefig(fpath, bbox_inches="tight")
        print(f"[visualization] Saved → {fpath}")

    plt.close(fig)


# ---------------------------------------------------------------------------
# Plotly
# ---------------------------------------------------------------------------
def plot_plotly(results: dict, save_path: str = "results/simulation") -> None:
    """
    Interactive Plotly figure with the same five panels.
    Saved as a standalone HTML file.
    """
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError as e:
        print(f"[visualization] plotly not available: {e}")
        return

    os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)

    t     = results["t"]
    adp   = results["adaptive_mode"]
    t_ice = results["t_ice"]

    # Find switch time
    switch_idx = int(np.argmax(adp))
    t_switch   = float(t[switch_idx]) if adp[switch_idx] else None

    # ---- Figure layout -----------------------------------------------
    fig = make_subplots(
        rows=5, cols=1,
        shared_xaxes=True,
        subplot_titles=[
            "Angle of Attack  α",
            "Pitch Rate  q",
            "Elevator Deflection  δe",
            "Adaptive Estimate  ΔĈ_Lα",
            "Airspeed  V",
        ],
        vertical_spacing=0.06,
    )

    col_nom   = "#2c7bb6"
    col_adapt = "#d7191c"
    col_total = "#1a1a2e"
    col_ref   = "#888888"
    col_ice   = "rgba(231,76,60,0.12)"

    # ---- Icing shading (vrect) helper --------------------------------
    def ice_shape():
        return dict(
            type="rect", xref="paper", yref="paper",
            x0=t_ice / t[-1], x1=1.0, y0=0, y1=1,
            fillcolor="rgba(231,76,60,0.07)",
            line_width=0,
            layer="below",
        )

    # ---- 1. Angle of attack ------------------------------------------
    fig.add_trace(go.Scatter(
        x=t, y=_deg(results["alpha_ref"]),
        name="α_ref", line=dict(color=col_ref, dash="dash", width=1.5),
        showlegend=True,
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=t, y=_deg(results["alpha"]),
        name="α", line=dict(color=col_nom, width=2),
        showlegend=True,
    ), row=1, col=1)

    # ---- 2. Pitch rate -----------------------------------------------
    fig.add_trace(go.Scatter(
        x=t, y=_deg(results["q"]),
        name="q", line=dict(color=col_nom, width=2),
        showlegend=False,
    ), row=2, col=1)

    # ---- 3. Elevator deflection --------------------------------------
    fig.add_trace(go.Scatter(
        x=t, y=_deg(results["delta_e_nom"]),
        name="δe nominal", line=dict(color=col_nom, dash="dash", width=1.5),
    ), row=3, col=1)
    fig.add_trace(go.Scatter(
        x=t, y=_deg(results["delta_e_adapt"]),
        name="δe adaptive", line=dict(color=col_adapt, dash="dot", width=1.8),
    ), row=3, col=1)
    fig.add_trace(go.Scatter(
        x=t, y=_deg(results["delta_e"]),
        name="δe total", line=dict(color=col_total, width=2.5),
    ), row=3, col=1)

    # ---- 4. Adaptation estimate -------------------------------------
    true_delta = results["C_La_iced"] - results["C_La_clean"]
    fig.add_trace(go.Scatter(
        x=t, y=results["delta_CL_alpha_hat"],
        name="ΔĈ_Lα", line=dict(color=col_adapt, width=2),
    ), row=4, col=1)
    fig.add_trace(go.Scatter(
        x=[t[0], t[-1]], y=[true_delta, true_delta],
        name=f"True ΔC_Lα={true_delta:.3f}",
        line=dict(color="grey", dash="dash", width=1.4),
    ), row=4, col=1)

    # ---- 5. Airspeed -------------------------------------------------
    fig.add_trace(go.Scatter(
        x=t, y=results["V"],
        name="V [m/s]", line=dict(color=col_nom, width=2),
        showlegend=False,
    ), row=5, col=1)

    # ---- Vertical lines at t_ice and t_switch -----------------------
    vlines = []
    vlines.append(dict(
        type="line", xref="x", yref="paper",
        x0=t_ice, x1=t_ice, y0=0, y1=1,
        line=dict(color="rgba(231,76,60,0.55)", width=1.5, dash="dashdot"),
        label=dict(text="Icing onset", textposition="end", font=dict(size=9)),
    ))
    if t_switch is not None:
        vlines.append(dict(
            type="line", xref="x", yref="paper",
            x0=t_switch, x1=t_switch, y0=0, y1=1,
            line=dict(color="rgba(231,76,60,0.85)", width=1.5, dash="dash"),
            label=dict(text=f"Adapt ON t={t_switch:.2f}s",
                       textposition="end", font=dict(size=9)),
        ))

    # ---- Layout ---------------------------------------------------------
    fig.update_layout(
        title=dict(
            text=(
                "<b>Lyapunov Adaptive Icing Controller — Interactive Results</b><br>"
                f"<sub>Icing onset t={t_ice}s | "
                f"C_Lα: {results['C_La_clean']:.3f} → {results['C_La_iced']:.3f}</sub>"
            ),
            x=0.5, xanchor="center",
        ),
        height=1050,
        template="plotly_white",
        hovermode="x unified",
        legend=dict(orientation="h", yanchor="bottom", y=1.01, x=0),
        shapes=vlines,
        font=dict(family="Arial, sans-serif", size=11),
        margin=dict(l=70, r=30, t=110, b=50),
    )

    # Y-axis labels
    ylabels = {
        "yaxis":  "α  [deg]",
        "yaxis2": "q  [deg/s]",
        "yaxis3": "δe  [deg]",
        "yaxis4": "ΔĈ_Lα  [–]",
        "yaxis5": "V  [m/s]",
    }
    for key, label in ylabels.items():
        fig.update_layout(**{key: dict(title=label)})

    fig.update_xaxes(title_text="Time  [s]", row=5, col=1)

    # ---- Save HTML --------------------------------------------------
    fpath = f"{save_path}.html"
    fig.write_html(fpath, include_plotlyjs="cdn")
    print(f"[visualization] Saved → {fpath}")