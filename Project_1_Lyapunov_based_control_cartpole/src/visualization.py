import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch
import os

from simulation import SimulationResult


class CartPoleVisualizer:
    """Plots and animates a SimulationResult."""

    def __init__(self, result: SimulationResult):
        self.result = result
        self.l = getattr(result, "l", 0.5)

    # ------------------------------------------------------------------ #
    #  Static plots                                                        #
    # ------------------------------------------------------------------ #

    def plot_results(self, save_path: str = None, show: bool = True):
        print("📊 Creating static plots...")

        time     = self.result.time
        states   = self.result.states
        ctrls    = self.result.controls
        lyap     = self.result.lyapunov_values
        energies = self.result.energies

        theta_norm_deg = np.degrees(
            (states[:, 2] + np.pi) % (2 * np.pi) - np.pi
        )

        fig, axes = plt.subplots(4, 2, figsize=(14, 12))
        fig.suptitle("CartPole — LQR Control Results",
                     fontsize=14, fontweight="bold")

        axes[0, 0].plot(time, states[:, 0], "b-", linewidth=2)
        axes[0, 0].axhline(0, color="k", linestyle="--", alpha=0.4)
        axes[0, 0].set_ylabel("x [m]")
        axes[0, 0].set_title("Cart Position")

        axes[0, 1].plot(time, states[:, 1], "r-", linewidth=2)
        axes[0, 1].axhline(0, color="k", linestyle="--", alpha=0.4)
        axes[0, 1].set_ylabel("ẋ [m/s]")
        axes[0, 1].set_title("Cart Velocity")

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

        axes[1, 1].plot(time, states[:, 3], "m-", linewidth=2)
        axes[1, 1].axhline(0, color="k", linestyle="--", alpha=0.4)
        axes[1, 1].set_ylabel("θ̇ [rad/s]")
        axes[1, 1].set_title("Angular Velocity")

        axes[2, 0].plot(time, ctrls, "c-", linewidth=2)
        axes[2, 0].axhline(0, color="k", linestyle="--", alpha=0.4)
        axes[2, 0].set_ylabel("u [N]")
        axes[2, 0].set_title("Control Input")

        lyap_safe = np.clip(lyap, 1e-12, None)
        axes[2, 1].semilogy(time, lyap_safe, color="goldenrod", linewidth=2)
        axes[2, 1].set_ylabel("V(s)")
        axes[2, 1].set_title("Lyapunov Function (log scale)")

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
    #  Phase Portraits                                                     #
    # ------------------------------------------------------------------ #

    def plot_phase_portraits(self, save_path: str = None, show: bool = True):
        """
        Plot phase portraits: (θ, θ̇), (x, ẋ), (θ, ẋ), (θ̇, ẋ)
        """
        print("🌀 Creating phase portraits...")

        states = self.result.states
        time = self.result.time

        # Unpack states
        x, x_dot, theta, theta_dot = states[:, 0], states[:, 1], states[:, 2], states[:, 3]

        # Normalize angle to [-π, π] for cleaner plots
        theta_norm = (theta + np.pi) % (2 * np.pi) - np.pi
        theta_norm_deg = np.degrees(theta_norm)

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("CartPole — Phase Portraits", fontsize=14, fontweight="bold")

        # (a) Pendulum phase portrait (θ, θ̇)
        ax = axes[0, 0]
        ax.plot(theta_norm_deg, theta_dot, "b-", linewidth=0.5, alpha=0.7)
        ax.plot(theta_norm_deg[0], theta_dot[0], "go", markersize=8, label="Start")
        ax.plot(theta_norm_deg[-1], theta_dot[-1], "ro", markersize=8, label="End")
        ax.axhline(0, color="k", linestyle="--", alpha=0.3)
        ax.axvline(0, color="r", linestyle="--", alpha=0.5, label="Upright (0°)")
        ax.set_xlabel("θ [deg]")
        ax.set_ylabel("θ̇ [rad/s]")
        ax.set_title("(a) Pendulum Phase Portrait")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # (b) Cart phase portrait (x, ẋ)
        ax = axes[0, 1]
        ax.plot(x, x_dot, "b-", linewidth=0.5, alpha=0.7)
        ax.plot(x[0], x_dot[0], "go", markersize=8, label="Start")
        ax.plot(x[-1], x_dot[-1], "ro", markersize=8, label="End")
        ax.axhline(0, color="k", linestyle="--", alpha=0.3)
        ax.axvline(0, color="k", linestyle="--", alpha=0.3)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("ẋ [m/s]")
        ax.set_title("(b) Cart Phase Portrait")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # (c) Mixed portrait (θ, ẋ)
        ax = axes[1, 0]
        ax.plot(theta_norm_deg, x_dot, "b-", linewidth=0.5, alpha=0.7)
        ax.axhline(0, color="k", linestyle="--", alpha=0.3)
        ax.axvline(0, color="r", linestyle="--", alpha=0.5)
        ax.set_xlabel("θ [deg]")
        ax.set_ylabel("ẋ [m/s]")
        ax.set_title("(c) Mixed: (θ, ẋ)")
        ax.grid(True, alpha=0.3)

        # (d) Mixed portrait (θ̇, ẋ)
        ax = axes[1, 1]
        ax.plot(theta_dot, x_dot, "b-", linewidth=0.5, alpha=0.7)
        ax.axhline(0, color="k", linestyle="--", alpha=0.3)
        ax.axvline(0, color="k", linestyle="--", alpha=0.3)
        ax.set_xlabel("θ̇ [rad/s]")
        ax.set_ylabel("ẋ [m/s]")
        ax.set_title("(d) Mixed: (θ̇, ẋ)")
        ax.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"✅ Phase portraits saved → {save_path}")
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
        plt.close(fig)