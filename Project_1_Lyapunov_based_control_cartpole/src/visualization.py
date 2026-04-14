import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch
import os
from simulation import SimulationResult


class CartPoleVisualizer:
    def __init__(self, result: SimulationResult):
        self.result = result

    def plot_results(self, save_path: str = None, show: bool = True):
        print("📊 Создание статических графиков...")
        fig, axes = plt.subplots(4, 2, figsize=(14, 10))
        fig.suptitle('CartPole - Lyapunov-based Control Results', fontsize=14, fontweight='bold')
        
        time = self.result.time
        states = self.result.states
        controls = self.result.controls
        
        # Исправленное отображение угла: 0-360°, 0° = верх
        theta_deg = (np.degrees(states[:, 2]) % 360)
        
        axes[0, 0].plot(time, states[:, 0], 'b-', linewidth=2)
        axes[0, 0].set_ylabel('x [m]'); axes[0, 0].set_title('Cart Position'); axes[0, 0].grid(True, alpha=0.3)
        
        axes[0, 1].plot(time, states[:, 1], 'r-', linewidth=2)
        axes[0, 1].set_ylabel('x_dot [m/s]'); axes[0, 1].set_title('Cart Velocity'); axes[0, 1].grid(True, alpha=0.3)
        
        axes[1, 0].plot(time, theta_deg, 'g-', linewidth=2)
        axes[1, 0].axhline(y=0, color='k', linestyle='--', alpha=0.5)
        axes[1, 0].axhline(y=360, color='k', linestyle='--', alpha=0.5)
        axes[1, 0].set_yticks([0, 90, 180, 270, 360])
        axes[1, 0].set_ylabel('θ [deg]'); axes[1, 0].set_title('Pendulum Angle'); axes[1, 0].grid(True, alpha=0.3)
        
        axes[1, 1].plot(time, states[:, 3], 'm-', linewidth=2)
        axes[1, 1].set_ylabel('θ_dot [rad/s]'); axes[1, 1].set_title('Angular Velocity'); axes[1, 1].grid(True, alpha=0.3)
        
        axes[2, 0].plot(time, controls, 'c-', linewidth=2)
        axes[2, 0].set_ylabel('u [N]'); axes[2, 0].set_title('Control Input'); axes[2, 0].grid(True, alpha=0.3)
        
        axes[2, 1].semilogy(time, self.result.lyapunov_values, 'y-', linewidth=2)
        axes[2, 1].set_ylabel('V(x)'); axes[2, 1].set_title('Lyapunov Function'); axes[2, 1].grid(True, alpha=0.3)
        
        axes[3, 0].plot(time, self.result.energies, 'orange', linewidth=2)
        E_des = 2 * self.result.energies.mean() * 0.5
        axes[3, 0].axhline(y=E_des, color='r', linestyle='--', label='E_ref')
        axes[3, 0].set_ylabel('Energy [J]'); axes[3, 0].set_title('Total Energy'); axes[3, 0].grid(True, alpha=0.3)
        
        modes_num = [0 if m == 'swing_up' else 1 for m in self.result.modes]
        axes[3, 1].plot(time, modes_num, 'k-', linewidth=2)
        axes[3, 1].set_ylabel('Mode'); axes[3, 1].set_title('Control Mode')
        axes[3, 1].set_yticks([0, 1]); axes[3, 1].set_yticklabels(['Swing-up', 'Stabilization'])
        axes[3, 1].grid(True, alpha=0.3)
        
        for ax in axes[3, :]: ax.set_xlabel('Time [s]')
        plt.tight_layout()
        
        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"✅ Сохранено в {save_path}")
        if show:
            plt.show(block=False)
            plt.pause(0.1)
        return fig

    def animate_realtime(self, show: bool = True, speed: float = 1.0):
        print("🎬 Запуск анимации... Закройте окно для завершения.")
        
        states = self.result.states
        time = self.result.time
        controls = self.result.controls
        l = 0.5
        cart_w, cart_h = 0.4, 0.2
        
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.set_xlim(-2.5, 2.5)
        ax.set_ylim(-0.5, 1.2)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title('CartPole Real-time + Force Vector')
        
        cart_rect = Rectangle((0, 0), cart_w, cart_h, fc='#3b82f6', ec='black', linewidth=2)
        ax.add_patch(cart_rect)
        pendulum_line, = ax.plot([], [], 'k-', linewidth=3)
        pendulum_circle = Circle((0, 0), 0.06, fc='#ef4444', ec='black')
        ax.add_patch(pendulum_circle)
        
        self.force_arrow = FancyArrowPatch((0, 0.1), (0, 0.1), arrowstyle='->', mutation_scale=20, color='green', linewidth=2)
        ax.add_patch(self.force_arrow)
        
        info_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=11, verticalalignment='top', 
                            bbox=dict(boxstyle='round', fc='wheat', alpha=0.8))
        
        ax_force = fig.add_axes([0.15, 0.05, 0.7, 0.15])
        ax_force.set_ylim(-12, 12)
        ax_force.set_xlim(0, time[-1])
        ax_force.grid(True, alpha=0.3)
        ax_force.set_ylabel('Force u [N]')
        ax_force.set_xlabel('Time [s]')
        force_line, = ax_force.plot([], [], 'g-', linewidth=1.5)
        
        def init():
            cart_rect.set_xy((-cart_w/2, 0))
            pendulum_line.set_data([], [])
            pendulum_circle.center = (0, 0)
            self.force_arrow.set_positions((0, 0.1), (0, 0.1))
            info_text.set_text('')
            force_line.set_data([], [])
            return cart_rect, pendulum_line, pendulum_circle, self.force_arrow, info_text, force_line
        
        def update(frame):
            x, _, theta, _ = states[frame]
            u = controls[frame]
            t = time[frame]
            
            cart_rect.set_xy((x - cart_w/2, 0))
            
            px = x + l * np.sin(theta)
            py = l * np.cos(theta)
            pendulum_line.set_data([x, px], [0, py])
            pendulum_circle.center = (px, py)
            
            arrow_end = (x + u * 0.15, 0.1)
            self.force_arrow.set_positions((x, 0.1), arrow_end)
            self.force_arrow.set_color('green' if u > 0 else 'red')
            
            mode = self.result.modes[frame]
            V = self.result.lyapunov_values[frame]
            
            # 🔧 ИСПРАВЛЕНИЕ: угол 0-360°, 0° = верх
            theta_deg = (np.degrees(theta) % 360)
            
            info_text.set_text(f't = {t:.2f} s\nθ = {theta_deg:.1f}°\nMode: {mode}\nForce = {u:.2f} N')
            
            force_line.set_data(time[:frame+1], controls[:frame+1])
            
            return cart_rect, pendulum_line, pendulum_circle, self.force_arrow, info_text, force_line
        
        ani = animation.FuncAnimation(fig, update, frames=len(time), init_func=init, blit=True, interval=10)
        
        if show:
            plt.show()
        plt.close('all')
        return ani