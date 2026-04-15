# visualization.py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation, PillowWriter
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

        fig, axes = plt.subplots(
            5, 1, figsize=figsize, sharex=True,
            gridspec_kw={"height_ratios": [1, 1, 1, 1, 1.2]},
        )
        fig.suptitle(
            "Cart-Pole  -  {}".format(name),
            fontsize=14, fontweight="bold", y=0.995,
        )

        # [0] Cart position
        axes[0].plot(t, r.states[:, 0], color=c["position"], lw=1.0)
        axes[0].axhline(0.0, color="gray", ls="--", lw=0.8, alpha=0.6)
        axes[0].set_ylabel("Cart x  [m]")

        # [1] Pendulum angle
        theta_deg = np.degrees(_normalize(r.states[:, 2]))
        axes[1].plot(t, theta_deg, color=c["angle"], lw=1.0)
        axes[1].axhline(0.0, color="gray", ls="--", lw=0.8, alpha=0.6)
        axes[1].set_ylabel("Angle theta  [deg]")

        # [2] Control force
        axes[2].plot(t, r.controls, color=c["control"], lw=1.0)
        axes[2].axhline(0.0, color="gray", ls="--", lw=0.8, alpha=0.6)
        axes[2].set_ylabel("Control u  [N]")

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

        # [4] Lyapunov
        axes[4].plot(
            t, r.lyapunov_values,
            color=c["lyapunov"], lw=1.0, label="$V(s)$",
        )
        axes[4].set_yscale("symlog", linthresh=1e-4)
        axes[4].set_ylabel("Lyapunov  V")
        axes[4].set_xlabel("Time  [s]")
        axes[4].legend(fontsize=8, loc="upper right", framealpha=0.7)

        # Background shading & polish
        for ax in axes:
            _shade_modes(ax, t, r.modes)
            ax.grid(True, alpha=0.25, linewidth=0.6)
            ax.margins(x=0.01)

        existing_handles, _ = axes[4].get_legend_handles_labels()
        axes[4].legend(
            handles=_mode_legend_handles() + existing_handles,
            fontsize=7, loc="upper right", framealpha=0.8, ncol=2,
        )

        fig.align_ylabels(axes)
        fig.tight_layout(rect=[0, 0, 1, 0.997])

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print("Saved -> {}".format(save_path))
        if show:
            plt.show()
        plt.close(fig)

    # ── _build_animation_fig ──────────────────────────────────────────────────

    def _build_animation_fig(self, speed=1.0, track_half=2.5, figsize=(11, 5), skip=None):
        """
        Build figure, patches, and update function for the cart-pole animation.
        Returns (fig, update_fn, frame_indices, interval_ms).
        """
        r    = self.result
        l    = getattr(r, "l", 0.5)
        name = getattr(r, "controller_name", "") or "Controller"
        dt_data = float(r.times[1] - r.times[0]) if len(r.times) > 1 else 0.002

        target_fps = 30.0
        if skip is None:
            skip = max(1, round(speed / (target_fps * dt_data)))

        frame_indices = list(range(0, len(r.times), skip))
        interval_ms = max(1, int(dt_data * skip / speed * 1000))

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
        ax_main.axhline(-0.18, color="#555555", lw=1.5, zorder=1)
        ax_main.plot(
            [-track_half, track_half], [0, 0],
            color="#888888", lw=5, solid_capstyle="round", zorder=2,
        )
        for xend in [-track_half, track_half]:
            ax_main.plot(
                [xend, xend], [0, 0.08],
                color="#555555", lw=4, solid_capstyle="round", zorder=2,
            )

        # ── Cart body ────────────────────────────────────────────────
        cart_w, cart_h = 0.38, 0.20
        wheel_r = 0.07
        wheel_y = -wheel_r

        cart_body = patches.Rectangle(
            (-cart_w / 2, 0.0), cart_w, cart_h,
            fc="#2c7bb6", ec="#1a4f78", lw=1.5, zorder=5,
        )
        ax_main.add_patch(cart_body)

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
        pivot_dot, = ax_main.plot([0], [pivot_y], "o", color="#1a4f78", ms=6, zorder=7)
        rod_line,  = ax_main.plot([], [], "-", color="#c0392b", lw=3.5,
                                  solid_capstyle="round", zorder=7)
        bob_dot,   = ax_main.plot([], [], "o", color="#e74c3c", ms=12, zorder=8)

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
            0.05 + 0.90, color="#e74c3c", lw=1.5, ls="--", zorder=3,
        )
        txt_bar = ax_bar.text(
            0.5, 0.0, "", ha="center", va="bottom", fontsize=7,
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

        def update(frame_idx):
            i     = frame_idx
            x     = r.states[i, 0]
            theta = r.states[i, 2]
            mode  = r.modes[i] if i < len(r.modes) else "swing_up"

            px = x + l * np.sin(theta)
            py = pivot_y + l * np.cos(theta)

            cart_body.set_xy((x - cart_w / 2, 0.0))
            cart_body.set_facecolor(mode_colors.get(mode, "#2c7bb6"))

            for k, wx in enumerate(wheel_offsets):
                wheel_patches_list[k].center = (x + wx, wheel_y)

            pivot_dot.set_data([x], [pivot_y])
            rod_line.set_data([x, px], [pivot_y, py])
            bob_dot.set_data([px], [py])

            txt_time.set_text("t = {:.2f} s".format(r.times[i]))
            txt_mode.set_text("mode: {}".format(mode))
            txt_mode.set_color(mode_colors.get(mode, "black"))
            theta_d = float(np.degrees(_normalize(theta)))
            txt_vals.set_text(
                "x = {:+.3f} m     theta = {:+.1f} deg".format(x, theta_d)
            )

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

            return (cart_body, *wheel_patches_list, pivot_dot,
                    rod_line, bob_dot, txt_time, txt_mode, txt_vals,
                    bar_fill, txt_bar)

        return fig, update, frame_indices, interval_ms

    # ── animate_realtime ──────────────────────────────────────────────────────

    def animate_realtime(self, show=True, speed=1.0, track_half=2.5, figsize=(11, 5)):
        """Real-time cart-pole animation with track, wheels, energy bar."""
        r = self.result
        dt_data = float(r.times[1] - r.times[0]) if len(r.times) > 1 else 0.002
        target_fps = 30.0
        skip = max(1, round(speed / (target_fps * dt_data)))

        fig, update, frame_indices, interval_ms = self._build_animation_fig(
            speed=speed, track_half=track_half, figsize=figsize, skip=skip
        )

        plt.ion()
        try:
            for i in frame_indices:
                update(i)
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                plt.pause(interval_ms / 1000.0)
        except KeyboardInterrupt:
            pass

        plt.ioff()
        if show:
            plt.show()
        plt.close(fig)

    # ── save_gif ──────────────────────────────────────────────────────────────

    def save_gif(
        self,
        save_path,
        speed=1.0,
        track_half=2.5,
        figsize=(11, 5),
        fps=30,
        dpi=100,
    ):
        """
        Save the cart-pole animation as a GIF.

        Parameters
        ----------
        save_path : str
            Output path, e.g. "figures/pendulum_energy.gif"
        speed : float
            Playback speed multiplier (>1 = faster).
        track_half : float
            Half-width of the visible track in metres.
        figsize : tuple
            Matplotlib figure size.
        fps : int
            Frames per second in the output GIF.
        dpi : int
            Resolution of each frame.
        """
        r = self.result
        dt_data = float(r.times[1] - r.times[0]) if len(r.times) > 1 else 0.002
        skip = max(1, round(speed / (fps * dt_data)))

        fig, update, frame_indices, _ = self._build_animation_fig(
            speed=speed, track_half=track_half, figsize=figsize, skip=skip
        )

        n = len(frame_indices)
        print("Saving GIF: {} frames → {}".format(n, save_path))

        anim = FuncAnimation(
            fig,
            update,
            frames=frame_indices,
            interval=int(1000 / fps),
            blit=False,
        )

        writer = PillowWriter(fps=fps)
        anim.save(save_path, writer=writer, dpi=dpi)
        plt.close(fig)
        print("Saved -> {}".format(save_path))


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
        plt.close(fig)