from __future__ import annotations
import os
import numpy as np

# --- Helpers ---
def _deg(rad_array):
    return np.rad2deg(rad_array)

def _icing_shade_mpl(ax, t, t_ice, color="#e74c3c", alpha=0.08):
    """Shades the region where icing is active (from t_ice onwards)."""
    ax.axvspan(t_ice, t[-1], color=color, alpha=alpha, zorder=0, label="Icing Active")

def _mode_switch_line_mpl(ax, t, adaptive_mode, color="#e74c3c", lw=1.2):
    """Draws a dashed line when the adaptive controller actually triggers."""
    if adaptive_mode is not None and np.any(adaptive_mode):
        switch_idx = np.argmax(adaptive_mode)
        t_switch = t[switch_idx]
        ax.axvline(t_switch, color=color, lw=lw, ls="--", 
                   label=f"Adaptive ON (t={t_switch:.2f}s)", zorder=5)

# --- Main Plotting Function ---
def plot_matplotlib(results: dict, res_base: dict = None, save_path: str = "results/simulation") -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        return

    os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)

    # Use the primary results for time and icing info
    t = results["t"]
    t_ice = results["t_ice"]
    is_compare = res_base is not None

    # Colors
    col_main  = "#2c7bb6" # Blue (Adaptive or Baseline Primary)
    col_base  = "#e67e22" # Orange (Comparison Baseline)
    col_ref   = "#555555" # Grey
    col_ice   = "#e74c3c" # Red

    plt.rcParams.update({"font.size": 9.5, "lines.linewidth": 1.6, "figure.dpi": 150})
    fig = plt.figure(figsize=(11, 14))
    
    title_prefix = "Comparison: Adaptive vs Baseline" if is_compare else "Controller Results"
    fig.suptitle(f"{title_prefix}\n(Icing onset t = {t_ice}s)", fontsize=12, fontweight="bold", y=0.98)

    gs = gridspec.GridSpec(5, 1, hspace=0.5, left=0.1, right=0.95, top=0.92, bottom=0.05)
    axes = [fig.add_subplot(gs[i]) for i in range(5)]

    # PANEL 1: Alpha
    ax = axes[0]
    ax.plot(t, _deg(results["alpha_ref"]), color=col_ref, ls="--", label="Ref")
    if is_compare:
        ax.plot(t, _deg(res_base["alpha"]), color=col_base, alpha=0.6, label="Baseline")
        ax.plot(t, _deg(results["alpha"]), color=col_main, label="Adaptive")
    else:
        ax.plot(t, _deg(results["alpha"]), color=col_main, label="Alpha")
    ax.set_ylabel("α [deg]")
    ax.set_title("Angle of Attack", loc="left", fontweight="bold")

    # PANEL 2: Pitch Rate
    ax = axes[1]
    if is_compare:
        ax.plot(t, _deg(res_base["q"]), color=col_base, alpha=0.6)
    ax.plot(t, _deg(results["q"]), color=col_main, label="q")
    ax.set_ylabel("q [deg/s]")
    ax.set_title("Pitch Rate", loc="left", fontweight="bold")

    # PANEL 3: Elevator
    ax = axes[2]
    if is_compare:
        ax.plot(t, _deg(res_base["delta_e"]), color=col_base, alpha=0.6, label="Baseline δe")
        ax.plot(t, _deg(results["delta_e"]), color=col_main, label="Adaptive δe")
    else:
        ax.plot(t, _deg(results["delta_e"]), color=col_main, label="δe")
    ax.set_ylabel("δe [deg]")
    ax.set_title("Elevator Deflection", loc="left", fontweight="bold")

    # PANEL 4: Estimation
    ax = axes[3]
    true_delta = results["C_La_iced"] - results["C_La_clean"]
    ax.axhline(true_delta, color="black", ls="--", alpha=0.4, label="True ΔC_La")
    ax.plot(t, results["delta_CL_alpha_hat"], color=col_main, lw=2, label="Estimate ΔĈ_Lα")
    ax.set_ylabel("ΔĈ_Lα")
    ax.set_title("Adaptive Icing Estimation", loc="left", fontweight="bold")

    # PANEL 5: Velocity
    ax = axes[4]
    if is_compare:
        ax.plot(t, res_base["V"], color=col_base, alpha=0.6)
    ax.plot(t, results["V"], color=col_main)
    ax.set_ylabel("V [m/s]")
    ax.set_xlabel("Time [s]")
    ax.set_title("Airspeed", loc="left", fontweight="bold")

    # Apply Shading and Labels to all axes
    for ax in axes:
        _icing_shade_mpl(ax, t, t_ice) # THE RED AREA
        _mode_switch_line_mpl(ax, t, results["adaptive_mode"])
        ax.axvline(t_ice, color=col_ice, lw=1, ls="-.", alpha=0.6)
        ax.set_xlim(t[0], t[-1])
        ax.legend(loc="upper right", fontsize=8, ncol=2)

    # Save
    for ext in ("png", "pdf"):
        fig.savefig(f"{save_path}.{ext}", bbox_inches="tight")
        print(f"[visualization] Saved → {save_path}.{ext}")
    plt.close(fig)