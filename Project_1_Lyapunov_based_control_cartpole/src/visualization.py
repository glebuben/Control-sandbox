# visualization.py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines
from matplotlib.gridspec import GridSpec
from typing import Optional


# ── Helpers ──────────────────────────────────────────────────────────────────

def _normalize(theta):
    """Map theta to [-pi, pi]. theta=0 is upright."""
    return (theta + np.pi) % (2 * np.pi) - np.pi


def _shade_modes(ax, times, modes, alpha=0.15):
    """
    Shade axis background by controller mode.

    Swing-up  → warm orange tint  #fff0d9
    Stabilize → cool blue tint    #ddeeff
    """
    color_map = {
        "swing_up":      "#fff0d9",
        "stabilization": "#ddeeff",
    }
    if not modes:
        return

    i = 0
    n = len(modes)
    while i < n:
        mode = modes[i]
        j = i + 1
        while j < n and modes[j] == mode:
            j += 1
        t_start = times[i]
        t_end = times[j] if j < n else times[-1]
        ax.axvspan(
            t_start, t_end,
            alpha=alpha,
            color=color_map.get(mode, "#eeeeee"),
            linewidth=0,
        )
        i = j


def _mode_legend_handles():
    """Return proxy artists for swing-up / stabilization legend."""
    return [
        patches.Patch(color="#fff0d9", alpha=0.8, label="Swing-up"),
        patches.Patch(color="#ddeeff", alpha=0.8, label="LQR stabilization"),
    ]


# ── Single-run visualizer ─────────────────────────────────────────────────────

class CartPoleVisualizer:
    """
    Plots and animates a single simulation result.
    """

    COLORS = {
        "position":  "tab:blue",
        "angle":     "tab:red",
        "control":   "tab:green",
        "energy":    "tab:purple",
        "e_up":      "tab:gray",
        "lyapunov":  "black",
    }

    def __init__(self, result, e_up=None):
        self.result = result
        if e_up is not None:
            self.e_up = float(e_up)
        elif hasattr(result, "e_up") and result.e_up is not None:
            self.e_up = float(result.e_up)
        else:
            if hasattr(result, "params") and result.params is not None:
                p = result.params
                self.e_up = 2.0 * p.m_p * p.g * p.l
            else:
                self.e_up = float(np.min(result.energies))

    # ── plot_results ──────────────────────────────────────────────────────────

    def plot_results(self, save_path=None, show=True, figsize=(13, 15)):
        r  = self.result
        t  = r.times
        c  = self.COLORS
        name = getattr(r, "controller_name", "") or "Controller"

<<<<<<< HEAD
        fig, axes = plt.subplots(
            5, 1, figsize=figsize, sharex=True,
            gridspec_kw={"height_ratios": [1, 1, 1, 1, 1.2]},
        )
        fig.suptitle(
            "Cart-Pole  -  {}".format(name),
            fontsize=14, fontweight="bold", y=0.995,
=======
        time     = self.result.time
        states   = self.result.states
        ctrls    = self.result.controls
        lyap     = self.result.lyapunov_values
        energies = self.result.energies

        theta_norm_deg = np.degrees(
            (states[:, 2] + np.pi) % (2 * np.pi) - np.pi
>>>>>>> main
        )

        # [0] Cart position
        axes[0].plot(t, r.states[:, 0], color=c["position"], lw=1.0)
        axes[0].axhline(0.0, color="gray", ls="--", lw=0.8, alpha=0.6)
        axes[0].set_ylabel("Cart x  [m]")

<<<<<<< HEAD
        # [1] Pendulum angle
        theta_deg = np.degrees(_normalize(r.states[:, 2]))
        axes[1].plot(t, theta_deg, color=c["angle"], lw=1.0)
        axes[1].axhline(0.0, color="gray", ls="--", lw=0.8, alpha=0.6)
        axes[1].set_ylabel("Angle theta  [deg]")
=======
        axes[0, 0].plot(time, states[:, 0], "b-", linewidth=2)
        axes[0, 0].axhline(0, color="k", linestyle="--", alpha=0.4)
        axes[0, 0].set_ylabel("x [m]")
        axes[0, 0].set_title("Cart Position")
>>>>>>> main

        # [2] Control force
        axes[2].plot(t, r.controls, color=c["control"], lw=1.0)
        axes[2].axhline(0.0, color="gray", ls="--", lw=0.8, alpha=0.6)
        axes[2].set_ylabel("Control u  [N]")

<<<<<<< HEAD
        # [3] Energy
        axes[3].plot(
            t, r.energies,
            color=c["energy"], lw=1.0, label="$E_{pend}$",
        )
        axes[3].axhline(
            self.e_up,
            color=c["e_up"], ls="--", lw=1.2,
            label="$E_{{up}}$ = {:.3f} J".format(self.e_up),
        )
        axes[3].set_ylabel("Pendulum energy  [J]")
        axes[3].legend(fontsize=8, loc="upper right", framealpha=0.7)
=======
        axes[1, 0].plot(time, theta_norm_deg, "g-", linewidth=2)
        axes[1, 0].axhline(0, color="r", linestyle="--", alpha=0.7,
                           label="upright (0°)")
        axes[1, 0].axhline(180, color="gray", linestyle=":", alpha=0.5,
                           label="down (±180°)")
        axes[1, 0].axhline(-180, color="gray", linestyle=":", alpha=0.5)
        axes[1, 0].set_ylim(-200, 200)
        axes[1, 0].set_yticks([-180, -90, 0, 90, 180])
        axes[1, 0].set_ylabel("θ [deg]")
        axes[1, 0].set_title("Pendulum Angle  (0° = upright)")
        axes[1, 0].legend(fontsize=8, loc="upper right")
>>>>>>> main

        # [4] Lyapunov
        axes[4].plot(
            t, r.lyapunov_values,
            color=c["lyapunov"], lw=1.0, label="$V(s)$",
        )
        axes[4].set_yscale("symlog", linthresh=1e-4)
        axes[4].set_ylabel("Lyapunov  V")
        axes[4].set_xlabel("Time  [s]")
        axes[4].legend(fontsize=8, loc="upper right", framealpha=0.7)

<<<<<<< HEAD
        # Background shading & polish
        for ax in axes:
            _shade_modes(ax, t, r.modes)
            ax.grid(True, alpha=0.25, linewidth=0.6)
            ax.margins(x=0.01)
=======
        axes[2, 0].plot(time, ctrls, "c-", linewidth=2)
        axes[2, 0].axhline(0, color="k", linestyle="--", alpha=0.4)
        axes[2, 0].set_ylabel("u [N]")
        axes[2, 0].set_title("Control Input")
>>>>>>> main

        existing_handles, _ = axes[4].get_legend_handles_labels()
        axes[4].legend(
            handles=_mode_legend_handles() + existing_handles,
            fontsize=7, loc="upper right", framealpha=0.8, ncol=2,
        )

<<<<<<< HEAD
        fig.align_ylabels(axes)
        fig.tight_layout(rect=[0, 0, 1, 0.997])

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print("Saved -> {}".format(save_path))
        if show:
            plt.show()
        plt.close(fig)

    # ── animate_realtime ──────────────────────────────────────────────────────

    def animate_realtime(self, show=True, speed=1.0, track_half=2.5, figsize=(11, 5)):
        """
        Real-time cart-pole animation with track, wheels, energy bar.

        Uses matplotlib.patches.Rectangle (not FancyBboxPatch)
        because Rectangle supports set_xy() for position updates.
        """
        r    = self.result
        l    = getattr(r, "l", 0.5)
        name = getattr(r, "controller_name", "") or "Controller"
        dt_data = float(r.times[1] - r.times[0]) if len(r.times) > 1 else 0.002

        target_fps = 30.0
        skip = max(1, round(speed / (target_fps * dt_data)))

        # ── Figure layout ──────────────────────────────────────────────
        fig = plt.figure(figsize=figsize)
        gs  = GridSpec(
            2, 1, figure=fig,
            height_ratios=[7, 1],
            hspace=0.05,
        )
        ax_main = fig.add_subplot(gs[0])
        ax_bar  = fig.add_subplot(gs[1])

        y_top = l + 0.35
        y_bot = -0.45
        ax_main.set_xlim(-track_half - 0.2, track_half + 0.2)
        ax_main.set_ylim(y_bot, y_top)
        ax_main.set_aspect("equal")
        ax_main.axis("off")
        ax_main.set_title("{} - Cart-Pole".format(name), fontsize=11, pad=4)

        # ── Static scenery ─────────────────────────────────────────────
        # Ground line
        ax_main.axhline(-0.18, color="#555555", lw=1.5, zorder=1)
        # Rail
        ax_main.plot(
            [-track_half, track_half], [0, 0],
            color="#888888", lw=5, solid_capstyle="round", zorder=2,
        )
        # Rail end stops
        for xend in [-track_half, track_half]:
            ax_main.plot(
                [xend, xend], [0, 0.08],
                color="#555555", lw=4, solid_capstyle="round", zorder=2,
            )

        # ── Cart body (Rectangle, supports set_xy) ────────────────────
        cart_w, cart_h = 0.38, 0.20
        wheel_r = 0.07
        wheel_y = -wheel_r

        cart_body = patches.Rectangle(
            (-cart_w / 2, 0.0), cart_w, cart_h,
            fc="#2c7bb6", ec="#1a4f78", lw=1.5, zorder=5,
        )
        ax_main.add_patch(cart_body)

        # Rounded corner overlay (cosmetic, drawn once per frame via set_xy)
        # We use Rectangle for reliable set_xy; the slight lack of rounding
        # is an acceptable trade-off for animation stability.

        # Wheels
        wheel_offsets = [-cart_w * 0.28, cart_w * 0.28]
        wheel_patches_list = []
        for wx in wheel_offsets:
            w = patches.Circle(
                (wx, wheel_y), wheel_r,
                fc="#333333", ec="#111111", lw=1.2, zorder=6,
            )
            ax_main.add_patch(w)
            wheel_patches_list.append(w)

        # ── Pendulum ──────────────────────────────────────────────────
        pivot_y = cart_h / 2
        pivot_dot, = ax_main.plot(
            [0], [pivot_y],
            "o", color="#1a4f78", ms=6, zorder=7,
        )
        rod_line, = ax_main.plot(
            [], [], "-",
            color="#c0392b", lw=3.5, solid_capstyle="round", zorder=7,
        )
        bob_dot, = ax_main.plot(
            [], [], "o",
            color="#e74c3c", ms=12, zorder=8,
        )

        # ── Text overlays ─────────────────────────────────────────────
        txt_kw = dict(transform=ax_main.transAxes, fontsize=9, va="top")
        txt_time = ax_main.text(0.02, 0.97, "", **txt_kw)
        txt_mode = ax_main.text(0.02, 0.91, "", **txt_kw)
        txt_vals = ax_main.text(0.02, 0.85, "", **txt_kw)

        # ── Energy progress bar ───────────────────────────────────────
        ax_bar.set_xlim(0, 1)
        ax_bar.set_ylim(0, 1)
        ax_bar.axis("off")
        ax_bar.set_title("Pendulum energy", fontsize=8, pad=1)

        bar_bg = patches.Rectangle(
            (0.05, 0.25), 0.90, 0.50,
            fc="#dddddd", ec="#aaaaaa", lw=1, zorder=1,
        )
        ax_bar.add_patch(bar_bg)
        bar_fill = patches.Rectangle(
            (0.05, 0.25), 0.0, 0.50,
            fc="#8e44ad", ec="none", zorder=2,
        )
        ax_bar.add_patch(bar_fill)
        bar_target = ax_bar.axvline(
            0.05 + 0.90,
            color="#e74c3c", lw=1.5, ls="--", zorder=3,
        )
        txt_bar = ax_bar.text(
            0.5, 0.0, "",
            ha="center", va="bottom", fontsize=7,
            transform=ax_bar.transAxes,
        )

        e_min = 0.0
        e_max = max(float(np.max(r.energies)) * 1.05, self.e_up * 1.1)

        def _e_to_bar(e):
            frac = np.clip((e - e_min) / (e_max - e_min), 0.0, 1.0)
            return 0.05 + 0.90 * frac

        bar_target.set_xdata([_e_to_bar(self.e_up), _e_to_bar(self.e_up)])

        mode_colors = {
            "swing_up":      "#2c7bb6",
            "stabilization": "#27ae60",
        }

        # ── Animation loop ────────────────────────────────────────────
        plt.ion()
        try:
            for i in range(0, len(r.times), skip):
                x     = r.states[i, 0]
                theta = r.states[i, 2]
                mode  = r.modes[i] if i < len(r.modes) else "swing_up"

                # Pendulum tip
                px = x + l * np.sin(theta)
                py = pivot_y + l * np.cos(theta)

                # Update cart position — Rectangle.set_xy works fine
                cart_body.set_xy((x - cart_w / 2, 0.0))
                cart_body.set_facecolor(mode_colors.get(mode, "#2c7bb6"))

                # Update wheels
                for k, wx in enumerate(wheel_offsets):
                    wheel_patches_list[k].center = (x + wx, wheel_y)

                # Pivot
                pivot_dot.set_data([x], [pivot_y])

                # Rod & bob
                rod_line.set_data([x, px], [pivot_y, py])
                bob_dot.set_data([px], [py])

                # Text
                txt_time.set_text("t = {:.2f} s".format(r.times[i]))
                txt_mode.set_text("mode: {}".format(mode))
                txt_mode.set_color(mode_colors.get(mode, "black"))
                theta_d = float(np.degrees(_normalize(theta)))
                txt_vals.set_text(
                    "x = {:+.3f} m     theta = {:+.1f} deg".format(x, theta_d)
                )

                # Energy bar
                e_now = r.energies[i] if i < len(r.energies) else 0.0
                w_fill = max(0.0, _e_to_bar(e_now) - 0.05)
                bar_fill.set_width(w_fill)
                bar_fill.set_facecolor(
                    "#27ae60" if abs(e_now - self.e_up) < 0.05 * self.e_up
                    else "#8e44ad"
                )
                txt_bar.set_text(
                    "E = {:.4f} J   E_up = {:.4f} J".format(e_now, self.e_up)
                )

                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                plt.pause(max(0.001, dt_data * skip / speed))

        except KeyboardInterrupt:
            pass

        plt.ioff()
        if show:
            plt.show()
        plt.close(fig)


# ── Comparison visualizer ─────────────────────────────────────────────────────

class ComparisonVisualizer:
    """Side-by-side OR overlaid comparison of two simulation results."""

    PALETTE = {
        "A": {
            "position": "#1f77b4",
            "angle":    "#d62728",
            "control":  "#2ca02c",
            "energy":   "#9467bd",
            "lyapunov": "#17becf",
            "mode":     "#aec7e8",
        },
        "B": {
            "position": "#ff7f0e",
            "angle":    "#e377c2",
            "control":  "#8c564b",
            "energy":   "#bcbd22",
            "lyapunov": "#7f7f7f",
            "mode":     "#ffbb78",
        },
    }

    def __init__(self, result_a, result_b, e_up=None):
        self.result_a = result_a
        self.result_b = result_b
        if e_up is not None:
            self.e_up = float(e_up)
        else:
            candidates = []
            for r in (result_a, result_b):
                if hasattr(r, "e_up") and r.e_up is not None:
                    candidates.append(float(r.e_up))
                elif hasattr(r, "params") and r.params is not None:
                    p = r.params
                    candidates.append(2.0 * p.m_p * p.g * p.l)
            self.e_up = candidates[0] if candidates else None

    # ── side_by_side ──────────────────────────────────────────────────────────

    def plot_comparison(self, save_path=None, show=True, figsize=(16, 22)):
        ra, rb = self.result_a, self.result_b
        name_a = getattr(ra, "controller_name", None) or "Controller A"
        name_b = getattr(rb, "controller_name", None) or "Controller B"
        pa, pb = self.PALETTE["A"], self.PALETTE["B"]

        fig = plt.figure(figsize=figsize)
        gs  = GridSpec(
            6, 2, figure=fig,
            hspace=0.38, wspace=0.30,
            height_ratios=[1, 1, 1, 1, 1, 0.6],
        )
        fig.suptitle(
            "Cart-Pole  -  Comparison\n{}  vs  {}".format(name_a, name_b),
            fontsize=14, fontweight="bold", y=0.995,
        )

        row_labels = [
            "Cart x  [m]",
            "Angle theta  [deg]",
            "Control u  [N]",
            "Pendulum energy  [J]",
            "Lyapunov  V",
            "Mode",
        ]

        for col, (r, name, pal) in enumerate(
            [(ra, name_a, pa), (rb, name_b, pb)]
        ):
            t = r.times
            theta_deg = np.degrees(_normalize(r.states[:, 2]))

            axes_col = []
            for row in range(6):
                sharex = axes_col[0] if row > 0 else None
                ax = fig.add_subplot(gs[row, col], sharex=sharex)
                axes_col.append(ax)

                if row == 0:
                    ax.set_title(name, fontsize=12, fontweight="bold", pad=6)
                    ax.plot(t, r.states[:, 0], color=pal["position"], lw=1.0)
                    ax.axhline(0, color="gray", ls="--", lw=0.7, alpha=0.6)
                    ax.set_ylabel(row_labels[0])

                elif row == 1:
                    ax.plot(t, theta_deg, color=pal["angle"], lw=1.0)
                    ax.axhline(0, color="gray", ls="--", lw=0.7, alpha=0.6)
                    ax.set_ylabel(row_labels[1])

                elif row == 2:
                    ax.plot(t, r.controls, color=pal["control"], lw=1.0)
                    ax.axhline(0, color="gray", ls="--", lw=0.7, alpha=0.6)
                    ax.set_ylabel(row_labels[2])

                elif row == 3:
                    ax.plot(
                        t, r.energies,
                        color=pal["energy"], lw=1.0, label="$E_{pend}$",
                    )
                    if self.e_up is not None:
                        ax.axhline(
                            self.e_up,
                            color="gray", ls="--", lw=1.0,
                            label="$E_{{up}}$={:.3f} J".format(self.e_up),
                        )
                    ax.set_ylabel(row_labels[3])
                    ax.legend(fontsize=7, loc="upper right", framealpha=0.7)

                elif row == 4:
                    ax.plot(
                        t, r.lyapunov_values,
                        color=pal["lyapunov"], lw=1.0,
                    )
                    ax.set_yscale("symlog", linthresh=1e-4)
                    ax.set_ylabel(row_labels[4])
                    ax.set_xlabel("Time  [s]")

                elif row == 5:
                    mode_num = np.array(
                        [1.0 if m == "stabilization" else 0.0
                         for m in r.modes]
                    )
                    ax.fill_between(
                        t, mode_num,
                        step="post",
                        color=pal["mode"], alpha=0.7,
                        linewidth=0,
                    )
                    ax.set_yticks([0, 1])
                    ax.set_yticklabels(["Swing", "LQR"], fontsize=7)
                    ax.set_ylim(-0.05, 1.15)
                    ax.set_xlabel("Time  [s]")
                    ax.set_ylabel(row_labels[5])

                if row < 5:
                    _shade_modes(ax, t, r.modes)

                ax.grid(True, alpha=0.22, linewidth=0.6)
                ax.margins(x=0.01)

                if row < 4:
                    plt.setp(ax.get_xticklabels(), visible=False)

            fig.align_ylabels(axes_col)

        fig.tight_layout(rect=[0, 0, 1, 0.993])

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print("Saved -> {}".format(save_path))
        if show:
            plt.show()
        plt.close(fig)

    # ── overlay ───────────────────────────────────────────────────────────────

    def plot_overlay(self, save_path=None, show=True, figsize=(13, 15)):
        """Single-column overlay: both controllers on the same axes."""
        ra, rb = self.result_a, self.result_b
        name_a = getattr(ra, "controller_name", None) or "Controller A"
        name_b = getattr(rb, "controller_name", None) or "Controller B"
        pa, pb = self.PALETTE["A"], self.PALETTE["B"]

        fig, axes = plt.subplots(
            5, 1, figsize=figsize, sharex=True,
            gridspec_kw={"height_ratios": [1, 1, 1, 1, 1.2]},
        )
        fig.suptitle(
            "Cart-Pole  -  Overlay\n{}  vs  {}".format(name_a, name_b),
            fontsize=14, fontweight="bold", y=0.998,
        )

        def _plot_pair(ax, ya, yb, key, ylabel, hline=None):
            ax.plot(ra.times, ya, color=pa[key], lw=1.2,
                    label=name_a, zorder=3)
            ax.plot(rb.times, yb, color=pb[key], lw=1.2,
                    label=name_b, zorder=3, ls="--")
            if hline is not None:
                ax.axhline(hline, color="gray", ls=":", lw=1.0, alpha=0.7)
            ax.set_ylabel(ylabel)
            ax.legend(fontsize=8, loc="upper right", framealpha=0.8)
            ax.grid(True, alpha=0.22, linewidth=0.6)
            ax.margins(x=0.01)

        # [0] Position
        _plot_pair(
            axes[0],
            ra.states[:, 0], rb.states[:, 0],
            "position", "Cart x  [m]", hline=0.0,
        )

        # [1] Angle
        theta_a = np.degrees(_normalize(ra.states[:, 2]))
        theta_b = np.degrees(_normalize(rb.states[:, 2]))
        _plot_pair(axes[1], theta_a, theta_b, "angle",
                   "Angle theta  [deg]", hline=0.0)

        # [2] Control
        _plot_pair(axes[2], ra.controls, rb.controls,
                   "control", "Control u  [N]")

        # [3] Energy
        _plot_pair(
            axes[3],
            ra.energies, rb.energies,
            "energy", "Pendulum energy  [J]",
            hline=self.e_up,
        )
        if self.e_up is not None:
            axes[3].annotate(
                "$E_{{up}}$ = {:.3f} J".format(self.e_up),
                xy=(ra.times[-1], self.e_up),
                xytext=(-5, 5), textcoords="offset points",
                ha="right", fontsize=7, color="gray",
            )

        # [4] Lyapunov
        axes[4].plot(
            ra.times, ra.lyapunov_values,
            color=pa["lyapunov"], lw=1.2, label=name_a,
        )
        axes[4].plot(
            rb.times, rb.lyapunov_values,
            color=pb["lyapunov"], lw=1.2, label=name_b, ls="--",
        )
        axes[4].set_yscale("symlog", linthresh=1e-4)
        axes[4].set_ylabel("Lyapunov  V")
        axes[4].set_xlabel("Time  [s]")
        axes[4].grid(True, alpha=0.22, linewidth=0.6)
        axes[4].margins(x=0.01)

        for ax in axes:
            _shade_modes(ax, ra.times, ra.modes, alpha=0.10)

        handles_ctrl = [
            mlines.Line2D([], [], color=pa["position"], lw=1.5,
                          label=name_a),
            mlines.Line2D([], [], color=pb["position"], lw=1.5,
                          ls="--", label=name_b),
        ]
        axes[4].legend(
            handles=handles_ctrl + _mode_legend_handles(),
            fontsize=7, loc="upper right",
            framealpha=0.85, ncol=2,
        )

        fig.align_ylabels(axes)
        fig.tight_layout(rect=[0, 0, 1, 0.996])

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print("Saved -> {}".format(save_path))
        if show:
            plt.show()
=======
        stab_mask = np.array([m == "stabilization" for m in self.result.modes])
        E_up = float(np.mean(energies[stab_mask])) if stab_mask.any() else float(np.mean(energies))

        axes[3, 0].plot(time, energies, color="orange", linewidth=2)
        axes[3, 0].axhline(E_up, color="r", linestyle="--",
                           label=f"E_up ≈ {E_up:.3f} J")
        axes[3, 0].set_ylabel("Energy [J]")
        axes[3, 0].set_title("Pendulum Energy")
        axes[3, 0].legend(fontsize=8)

        modes_num = [0 if m == "swing_up" else 1 for m in self.result.modes]
        axes[3, 1].plot(time, modes_num, "k-", linewidth=2)
        axes[3, 1].set_yticks([0, 1])
        axes[3, 1].set_yticklabels(["Swing-up", "Stabilization"])
        axes[3, 1].set_ylabel("Mode")
        axes[3, 1].set_title("Control Mode")

        for ax in axes.flat:
            ax.grid(True, alpha=0.3)
        for ax in axes[3, :]:
            ax.set_xlabel("Time [s]")

        plt.tight_layout()

        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"✅ Saved to {save_path}")
        if show:
            plt.show(block=False)
            plt.pause(0.1)
        return fig

    # ------------------------------------------------------------------ #
    #  Animation                                                           #
    # ------------------------------------------------------------------ #

    def animate_realtime(self, show: bool = True, speed: float = 1.0):
        print("🎬 Starting animation...  Close the window to exit.")

        states   = self.result.states
        time_arr = self.result.time
        controls = self.result.controls
        modes    = self.result.modes
        lyap     = self.result.lyapunov_values

        dt_sim    = float(time_arr[1] - time_arr[0])
        dt_ms     = dt_sim * 1000.0
        MIN_INTERVAL_MS = 20.0
        frame_skip = max(1, int(np.ceil(MIN_INTERVAL_MS / (dt_ms * speed))))
        interval_ms = dt_ms * frame_skip / speed
        render_frames = np.arange(0, len(time_arr), frame_skip)

        print(f"   Simulation: {len(time_arr)} frames, dt={dt_sim*1000:.1f} ms")
        print(f"   Animation:  {len(render_frames)} frames rendered, "
              f"interval={interval_ms:.1f} ms, speed={speed:.1f}x")

        l = self.l
        cart_w, cart_h = 0.4, 0.2

        fig = plt.figure(figsize=(11, 7))
        fig.suptitle(f"CartPole Animation  (speed = {speed:.1f}×)", fontsize=13, fontweight="bold")

        ax = fig.add_axes([0.05, 0.30, 0.90, 0.65])
        ax.set_xlim(-3.0, 3.0)
        ax.set_ylim(-0.8, 1.2)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.axhline(cart_h / 2, color="#cccccc", linewidth=1.5, zorder=0)
        ax.axvline(-2.5, color="#ffaaaa", linewidth=1, linestyle="--", alpha=0.6)
        ax.axvline( 2.5, color="#ffaaaa", linewidth=1, linestyle="--", alpha=0.6)

        cart_rect = Rectangle((0, 0), cart_w, cart_h, fc="#3b82f6", ec="black", linewidth=2, zorder=3)
        ax.add_patch(cart_rect)
        (pendulum_line,) = ax.plot([], [], "k-", linewidth=3, zorder=4)
        pendulum_bob = Circle((0, l), 0.06, fc="#ef4444", ec="black", zorder=5)
        ax.add_patch(pendulum_bob)
        force_arrow = FancyArrowPatch((0, cart_h / 2), (0.2, cart_h / 2), arrowstyle="->", mutation_scale=20, color="green", linewidth=2, zorder=6)
        ax.add_patch(force_arrow)
        time_label = ax.text(0.98, 0.97, "", transform=ax.transAxes, fontsize=11, ha="right", va="top", bbox=dict(boxstyle="round", fc="white", alpha=0.7))
        info = ax.text(0.02, 0.97, "", transform=ax.transAxes, fontsize=9, va="top", fontfamily="monospace", bbox=dict(boxstyle="round", fc="wheat", alpha=0.85))

        ax_f = fig.add_axes([0.08, 0.05, 0.56, 0.18])
        ax_f.set_xlim(0, time_arr[-1])
        u_absmax = max(abs(controls).max(), 1.0)
        ax_f.set_ylim(-u_absmax * 1.15, u_absmax * 1.15)
        ax_f.axhline(0, color="k", linewidth=0.8)
        ax_f.set_ylabel("u [N]", fontsize=9)
        ax_f.set_xlabel("t [s]",  fontsize=9)
        ax_f.set_title("Control force", fontsize=9)
        ax_f.grid(True, alpha=0.3)
        ax_f.plot(time_arr, controls, color="#cccccc", linewidth=1.0, zorder=1)
        (force_line,) = ax_f.plot([], [], "g-", linewidth=1.8, zorder=2)
        (force_cursor,) = ax_f.plot([], [], "ro", markersize=5, zorder=3)

        ax_v = fig.add_axes([0.70, 0.05, 0.27, 0.18])
        ax_v.set_xlim(0, time_arr[-1])
        lyap_safe = np.clip(lyap, 1e-12, None)
        ax_v.set_ylim(lyap_safe.min() * 0.5, lyap_safe.max() * 2.0)
        ax_v.set_yscale("log")
        ax_v.set_ylabel("V(s)", fontsize=9)
        ax_v.set_xlabel("t [s]",  fontsize=9)
        ax_v.set_title("Lyapunov V (log)", fontsize=9)
        ax_v.grid(True, alpha=0.3)
        ax_v.plot(time_arr, lyap_safe, color="#cccccc", linewidth=1.0, zorder=1)
        (lyap_line,) = ax_v.plot([], [], color="goldenrod", linewidth=1.8, zorder=2)

        def init():
            cart_rect.set_xy((-cart_w / 2, 0))
            pendulum_line.set_data([], [])
            pendulum_bob.center = (0, cart_h / 2 + l)
            force_arrow.set_positions((0, cart_h / 2), (0, cart_h / 2))
            time_label.set_text("")
            info.set_text("")
            force_line.set_data([], [])
            force_cursor.set_data([], [])
            lyap_line.set_data([], [])
            return (cart_rect, pendulum_line, pendulum_bob, force_arrow, time_label, info, force_line, force_cursor, lyap_line)

        def update(render_idx):
            sim_idx = render_frames[render_idx]
            x, x_dot, theta, theta_dot = states[sim_idx]
            u    = controls[sim_idx]
            t    = time_arr[sim_idx]
            mode = modes[sim_idx]
            V    = lyap[sim_idx]

            cart_rect.set_xy((x - cart_w / 2, 0))
            cart_cy = cart_h / 2
            bx = x + l * np.sin(theta)
            by = cart_cy + l * np.cos(theta)
            pendulum_line.set_data([x, bx], [cart_cy, by])
            pendulum_bob.center = (bx, by)
            arrow_len = float(u) * 0.15
            force_arrow.set_positions((x, cart_cy), (x + arrow_len, cart_cy))
            force_arrow.set_color("green" if u >= 0.0 else "red")
            time_label.set_text(f"t = {t:.2f} s")
            theta_deg = np.degrees((theta + np.pi) % (2 * np.pi) - np.pi)
            info.set_text(f"θ    = {theta_deg:+6.1f}°\nx    = {x:+6.3f} m\nẋ    = {x_dot:+6.2f} m/s\nθ̇   = {theta_dot:+6.2f} rad/s\nu    = {u:+6.2f} N\nV    = {V:.4g}\nmode : {mode}")
            force_line.set_data(time_arr[:sim_idx + 1], controls[:sim_idx + 1])
            force_cursor.set_data([t], [u])
            lyap_line.set_data(time_arr[:sim_idx + 1], np.clip(lyap[:sim_idx + 1], 1e-12, None))
            return (cart_rect, pendulum_line, pendulum_bob, force_arrow, time_label, info, force_line, force_cursor, lyap_line)

        ani = animation.FuncAnimation(fig, update, frames=len(render_frames), init_func=init, blit=True, interval=interval_ms, repeat=True)

        if show:
            plt.show()
        plt.close("all")
        return ani

    # ------------------------------------------------------------------ #
    #  GIF Export                                                          #
    # ------------------------------------------------------------------ #
    def save_gif(self, save_path: str, fps: int = 30):
        print(f"💾 Generating GIF (this may take 30-60s)...")
        states   = self.result.states
        time_arr = self.result.time
        controls = self.result.controls
        modes    = self.result.modes
        lyap     = self.result.lyapunov_values
        l = self.l
        cart_w, cart_h = 0.4, 0.2

        # Ограничиваем количество кадров, чтобы GIF не весил слишком много (~10 сек)
        target_frames = fps * 10
        step = max(1, len(time_arr) // target_frames)
        render_idx = np.arange(0, len(time_arr), step)

        fig = plt.figure(figsize=(11, 7))
        fig.suptitle("CartPole Animation", fontsize=13, fontweight="bold")

        ax = fig.add_axes([0.05, 0.30, 0.90, 0.65])
        ax.set_xlim(-3.0, 3.0)
        ax.set_ylim(-0.8, 1.2)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.axhline(cart_h / 2, color="#cccccc", linewidth=1.5, zorder=0)
        ax.axvline(-2.5, color="#ffaaaa", linewidth=1, linestyle="--", alpha=0.6)
        ax.axvline( 2.5, color="#ffaaaa", linewidth=1, linestyle="--", alpha=0.6)

        cart_rect = Rectangle((0, 0), cart_w, cart_h, fc="#3b82f6", ec="black", linewidth=2, zorder=3)
        ax.add_patch(cart_rect)
        (pendulum_line,) = ax.plot([], [], "k-", linewidth=3, zorder=4)
        pendulum_bob = Circle((0, l), 0.06, fc="#ef4444", ec="black", zorder=5)
        ax.add_patch(pendulum_bob)
        force_arrow = FancyArrowPatch((0, cart_h / 2), (0.2, cart_h / 2), arrowstyle="->", mutation_scale=20, color="green", linewidth=2, zorder=6)
        ax.add_patch(force_arrow)
        time_label = ax.text(0.98, 0.97, "", transform=ax.transAxes, fontsize=11, ha="right", va="top", bbox=dict(boxstyle="round", fc="white", alpha=0.7))
        info = ax.text(0.02, 0.97, "", transform=ax.transAxes, fontsize=9, va="top", fontfamily="monospace", bbox=dict(boxstyle="round", fc="wheat", alpha=0.85))

        ax_f = fig.add_axes([0.08, 0.05, 0.56, 0.18])
        ax_f.set_xlim(0, time_arr[-1])
        u_absmax = max(abs(controls).max(), 1.0)
        ax_f.set_ylim(-u_absmax * 1.15, u_absmax * 1.15)
        ax_f.axhline(0, color="k", linewidth=0.8)
        ax_f.set_ylabel("u [N]", fontsize=9)
        ax_f.set_xlabel("t [s]",  fontsize=9)
        ax_f.set_title("Control force", fontsize=9)
        ax_f.grid(True, alpha=0.3)
        ax_f.plot(time_arr, controls, color="#cccccc", linewidth=1.0, zorder=1)
        (force_line,) = ax_f.plot([], [], "g-", linewidth=1.8, zorder=2)
        (force_cursor,) = ax_f.plot([], [], "ro", markersize=5, zorder=3)

        ax_v = fig.add_axes([0.70, 0.05, 0.27, 0.18])
        ax_v.set_xlim(0, time_arr[-1])
        lyap_safe = np.clip(lyap, 1e-12, None)
        ax_v.set_ylim(lyap_safe.min() * 0.5, lyap_safe.max() * 2.0)
        ax_v.set_yscale("log")
        ax_v.set_ylabel("V(s)", fontsize=9)
        ax_v.set_xlabel("t [s]",  fontsize=9)
        ax_v.set_title("Lyapunov V (log)", fontsize=9)
        ax_v.grid(True, alpha=0.3)
        ax_v.plot(time_arr, lyap_safe, color="#cccccc", linewidth=1.0, zorder=1)
        (lyap_line,) = ax_v.plot([], [], color="goldenrod", linewidth=1.8, zorder=2)

        def init():
            cart_rect.set_xy((-cart_w / 2, 0))
            pendulum_line.set_data([], [])
            pendulum_bob.center = (0, cart_h / 2 + l)
            force_arrow.set_positions((0, cart_h / 2), (0, cart_h / 2))
            time_label.set_text("")
            info.set_text("")
            force_line.set_data([], [])
            force_cursor.set_data([], [])
            lyap_line.set_data([], [])
            return (cart_rect, pendulum_line, pendulum_bob, force_arrow, time_label, info, force_line, force_cursor, lyap_line)

        def update(i):
            idx = render_idx[i]
            x, x_dot, theta, theta_dot = states[idx]
            u    = controls[idx]
            t    = time_arr[idx]
            mode = modes[idx]
            V    = lyap[idx]

            cart_rect.set_xy((x - cart_w / 2, 0))
            cart_cy = cart_h / 2
            bx = x + l * np.sin(theta)
            by = cart_cy + l * np.cos(theta)
            pendulum_line.set_data([x, bx], [cart_cy, by])
            pendulum_bob.center = (bx, by)
            arrow_len = float(u) * 0.15
            force_arrow.set_positions((x, cart_cy), (x + arrow_len, cart_cy))
            force_arrow.set_color("green" if u >= 0.0 else "red")
            time_label.set_text(f"t = {t:.2f} s")
            theta_deg = np.degrees((theta + np.pi) % (2 * np.pi) - np.pi)
            info.set_text(f"θ    = {theta_deg:+6.1f}°\nx    = {x:+6.3f} m\nẋ    = {x_dot:+6.2f} m/s\nθ̇   = {theta_dot:+6.2f} rad/s\nu    = {u:+6.2f} N\nV    = {V:.4g}\nmode : {mode}")
            force_line.set_data(time_arr[:idx + 1], controls[:idx + 1])
            force_cursor.set_data([t], [u])
            lyap_line.set_data(time_arr[:idx + 1], np.clip(lyap[:idx + 1], 1e-12, None))
            return (cart_rect, pendulum_line, pendulum_bob, force_arrow, time_label, info, force_line, force_cursor, lyap_line)

        # blit=False надежнее для pillow writer
        ani = animation.FuncAnimation(fig, update, frames=len(render_idx), init_func=init, blit=False, repeat=False)

        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        ani.save(save_path, writer='pillow', fps=fps, dpi=100)
        print(f"✅ GIF saved → {save_path}")
>>>>>>> main
        plt.close(fig)