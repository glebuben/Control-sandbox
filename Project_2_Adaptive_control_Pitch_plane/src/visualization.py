import pygame
import numpy as np
from typing import Tuple, Optional
import sys

# Import the dynamics model from previous code
# (Assume AircraftParameters and LongitudinalDynamics are available)
from system import AircraftParameters, LongitudinalDynamics
from dataclasses import dataclass



class AircraftVisualization:
    """Pygame-based visualization for aircraft longitudinal dynamics."""
    
    # Color palette
    SKY_BLUE = (135, 206, 235)
    GROUND_BROWN = (139, 90, 43)
    HORIZON_WHITE = (255, 255, 255)
    AIRCRAFT_RED = (220, 50, 50)
    VELOCITY_GREEN = (50, 220, 50)
    TEXT_BLACK = (0, 0, 0)
    TEXT_WHITE = (255, 255, 255)
    GRID_GRAY = (200, 200, 200)
    WARNING_YELLOW = (255, 255, 0)
    DANGER_RED = (255, 0, 0)
    
    def __init__(
        self, 
        width: int = 1400, 
        height: int = 800,
        scale: float = 10.0
    ):
        """
        Initialize visualization.
        
        Args:
            width: Window width [pixels]
            height: Window height [pixels]
            scale: Pixels per meter for aircraft rendering
        """
        pygame.init()
        
        self.width = width
        self.height = height
        self.scale = scale
        
        # Create window
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Aircraft Longitudinal Dynamics - Free Flight (No Control)")
        
        # Fonts
        self.font_large = pygame.font.SysFont('monospace', 24, bold=True)
        self.font_medium = pygame.font.SysFont('monospace', 18)
        self.font_small = pygame.font.SysFont('monospace', 14)
        
        # Clock for framerate control
        self.clock = pygame.time.Clock()
        
        # View parameters
        self.center_x = width // 2
        self.center_y = height // 2
        self.altitude_offset = 0.0  # For tracking aircraft vertically
        
        # History for plotting
        self.max_history = 500
        self.time_history = []
        self.V_history = []
        self.alpha_history = []
        self.theta_history = []
        self.q_history = []
        self.altitude_history = []
        
    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to screen coordinates.
        
        Args:
            x: Horizontal position [m] (not used in side view)
            y: Vertical position [m] (altitude)
            
        Returns:
            (screen_x, screen_y) in pixels
        """
        screen_x = self.center_x
        screen_y = int(self.center_y - (y - self.altitude_offset) * self.scale)
        return screen_x, screen_y
    
    def draw_horizon(self, theta: float):
        """
        Draw artificial horizon (rotates with pitch angle).
        
        Args:
            theta: Pitch angle [rad]
        """
        # Sky
        self.screen.fill(self.SKY_BLUE)
        
        # Ground (lower half initially)
        ground_rect = pygame.Rect(0, self.center_y, self.width, self.height // 2)
        pygame.draw.rect(self.screen, self.GROUND_BROWN, ground_rect)
        
        # Horizon line (rotates with theta)
        # For simplicity, we'll draw a straight line at center
        # (In a full implementation, this would tilt)
        horizon_y = self.center_y
        pygame.draw.line(
            self.screen, 
            self.HORIZON_WHITE, 
            (0, horizon_y), 
            (self.width, horizon_y), 
            3
        )
        
        # Pitch ladder (angle reference marks)
        for angle_deg in range(-30, 40, 10):
            if angle_deg == 0:
                continue
            angle_rad = np.deg2rad(angle_deg)
            y_offset = -angle_rad * self.scale * 20  # Scale factor for visibility
            y_pos = int(horizon_y + y_offset)
            
            if 0 < y_pos < self.height:
                line_length = 80 if angle_deg % 20 == 0 else 40
                pygame.draw.line(
                    self.screen,
                    self.GRID_GRAY,
                    (self.center_x - line_length, y_pos),
                    (self.center_x + line_length, y_pos),
                    1
                )
                # Label
                label = self.font_small.render(f"{angle_deg}°", True, self.GRID_GRAY)
                self.screen.blit(label, (self.center_x + line_length + 10, y_pos - 7))
    
    def draw_aircraft(self, theta: float, alpha: float):
        """
        Draw aircraft symbol (side view).
        
        Args:
            theta: Pitch angle [rad]
            alpha: Angle of attack [rad]
        """
        # Aircraft is always at screen center
        x, y = self.center_x, self.center_y
        
        # Aircraft body orientation (pitch angle)
        fuselage_length = 60
        wing_span = 80
        
        # Fuselage
        nose_x = x + fuselage_length * np.cos(theta)
        nose_y = y - fuselage_length * np.sin(theta)
        tail_x = x - fuselage_length * np.cos(theta)
        tail_y = y + fuselage_length * np.sin(theta)
        
        pygame.draw.line(
            self.screen,
            self.AIRCRAFT_RED,
            (int(tail_x), int(tail_y)),
            (int(nose_x), int(nose_y)),
            4
        )
        
        # Wings (perpendicular to fuselage)
        wing_angle = theta + np.pi/2
        wing_dx = wing_span/2 * np.cos(wing_angle)
        wing_dy = wing_span/2 * np.sin(wing_angle)
        
        pygame.draw.line(
            self.screen,
            self.AIRCRAFT_RED,
            (int(x - wing_dx), int(y + wing_dy)),
            (int(x + wing_dx), int(y - wing_dy)),
            3
        )
        
        # Horizontal stabilizer
        stab_length = 30
        stab_x = tail_x - 10 * np.cos(theta)
        stab_y = tail_y + 10 * np.sin(theta)
        stab_dx = stab_length/2 * np.cos(wing_angle)
        stab_dy = stab_length/2 * np.sin(wing_angle)
        
        pygame.draw.line(
            self.screen,
            self.AIRCRAFT_RED,
            (int(stab_x - stab_dx), int(stab_y + stab_dy)),
            (int(stab_x + stab_dx), int(stab_y - stab_dy)),
            2
        )
        
        # Nose marker (circle)
        pygame.draw.circle(self.screen, self.AIRCRAFT_RED, (int(nose_x), int(nose_y)), 6)
    
    def draw_velocity_vector(self, V: float, alpha: float, theta: float):
        """
        Draw velocity vector (shows flight path).
        
        Args:
            V: Airspeed [m/s]
            alpha: Angle of attack [rad]
            theta: Pitch angle [rad]
        """
        # Flight path angle = theta - alpha
        gamma = theta - alpha
        
        # Vector length proportional to speed
        vector_length = V * 3  # Scale for visibility
        
        x, y = self.center_x, self.center_y
        end_x = x + vector_length * np.cos(gamma)
        end_y = y - vector_length * np.sin(gamma)
        
        # Draw arrow
        pygame.draw.line(
            self.screen,
            self.VELOCITY_GREEN,
            (x, y),
            (int(end_x), int(end_y)),
            3
        )
        
        # Arrowhead
        arrow_size = 10
        arrow_angle = np.deg2rad(25)
        
        for sign in [-1, 1]:
            arr_angle = gamma + np.pi + sign * arrow_angle
            arr_x = end_x + arrow_size * np.cos(arr_angle)
            arr_y = end_y - arrow_size * np.sin(arr_angle)
            pygame.draw.line(
                self.screen,
                self.VELOCITY_GREEN,
                (int(end_x), int(end_y)),
                (int(arr_x), int(arr_y)),
                3
            )
    
    def draw_telemetry(
        self, 
        s: np.ndarray, 
        a: np.ndarray, 
        t: float,
        forces: Tuple[float, float, float, float]
    ):
        """
        Draw telemetry panel with state information.
        
        Args:
            s: State vector [V, alpha, q, theta]
            a: Control vector [delta_e, delta_t]
            t: Simulation time [s]
            forces: (L, D, M, T) forces and moments
        """
        V, alpha, q, theta = s
        delta_e, delta_t = a
        L, D, M, T = forces
        
        # Panel background
        panel_width = 350
        panel_height = 400
        panel_x = self.width - panel_width - 10
        panel_y = 10
        
        panel_surface = pygame.Surface((panel_width, panel_height))
        panel_surface.set_alpha(220)
        panel_surface.fill((40, 40, 40))
        self.screen.blit(panel_surface, (panel_x, panel_y))
        
        # Title
        y_offset = panel_y + 10
        title = self.font_large.render("TELEMETRY", True, self.TEXT_WHITE)
        self.screen.blit(title, (panel_x + 10, y_offset))
        y_offset += 35
        
        # Draw separator
        pygame.draw.line(
            self.screen,
            self.GRID_GRAY,
            (panel_x + 10, y_offset),
            (panel_x + panel_width - 10, y_offset),
            1
        )
        y_offset += 10
        
        # State variables
        data = [
            ("Time", f"{t:.2f} s", self.TEXT_WHITE),
            ("", "", self.TEXT_WHITE),
            ("Airspeed (V)", f"{V:.2f} m/s ({V*3.6:.1f} km/h)", self.TEXT_WHITE),
            ("Angle of Attack (α)", f"{np.rad2deg(alpha):.2f}°", 
             self.WARNING_YELLOW if abs(alpha) > np.deg2rad(10) else self.TEXT_WHITE),
            ("Pitch Rate (q)", f"{np.rad2deg(q):.2f}°/s", self.TEXT_WHITE),
            ("Pitch Angle (θ)", f"{np.rad2deg(theta):.2f}°", self.TEXT_WHITE),
            ("Flight Path (γ)", f"{np.rad2deg(theta - alpha):.2f}°", self.TEXT_WHITE),
            ("", "", self.TEXT_WHITE),
            ("Elevator (δe)", f"{np.rad2deg(delta_e):.2f}°", self.TEXT_WHITE),
            ("Throttle (δt)", f"{delta_t:.3f} ({delta_t*100:.1f}%)", self.TEXT_WHITE),
            ("", "", self.TEXT_WHITE),
            ("Lift (L)", f"{L:.1f} N", self.TEXT_WHITE),
            ("Drag (D)", f"{D:.1f} N", self.TEXT_WHITE),
            ("Thrust (T)", f"{T:.1f} N", self.TEXT_WHITE),
            ("Moment (M)", f"{M:.1f} N·m", self.TEXT_WHITE),
        ]
        
        for label, value, color in data:
            if label:  # Skip empty lines for value
                text_label = self.font_small.render(label, True, self.GRID_GRAY)
                text_value = self.font_medium.render(value, True, color)
                self.screen.blit(text_label, (panel_x + 15, y_offset))
                self.screen.blit(text_value, (panel_x + 15, y_offset + 16))
                y_offset += 38
            else:
                y_offset += 10
    
    def draw_plots(self):
        """Draw time-history plots in bottom panel."""
        if len(self.time_history) < 2:
            return
        
        plot_height = 150
        plot_y = self.height - plot_height - 10
        plot_width = (self.width - 40 - 350) // 2  # Split into 2 plots, account for telemetry
        
        # Background
        pygame.draw.rect(
            self.screen,
            (30, 30, 30),
            (10, plot_y, plot_width * 2 + 20, plot_height)
        )
        
        # Plot 1: Airspeed
        self._draw_subplot(
            self.time_history,
            self.V_history,
            10,
            plot_y,
            plot_width,
            plot_height,
            "Airspeed [m/s]",
            self.VELOCITY_GREEN
        )
        
        # Plot 2: Pitch angle and AoA
        self._draw_subplot_dual(
            self.time_history,
            [np.rad2deg(a) for a in self.theta_history],
            [np.rad2deg(a) for a in self.alpha_history],
            20 + plot_width,
            plot_y,
            plot_width,
            plot_height,
            "Angles [deg]",
            self.AIRCRAFT_RED,
            self.WARNING_YELLOW,
            "θ",
            "α"
        )
    
    def _draw_subplot(
        self,
        x_data,
        y_data,
        x_pos,
        y_pos,
        width,
        height,
        title,
        color
    ):
        """Helper to draw a single time-series plot."""
        margin = 30
        plot_x = x_pos + margin
        plot_y = y_pos + margin
        plot_w = width - 2 * margin
        plot_h = height - 2 * margin
        
        # Title
        title_surf = self.font_small.render(title, True, self.TEXT_WHITE)
        self.screen.blit(title_surf, (plot_x, y_pos + 5))
        
        # Axes
        pygame.draw.line(
            self.screen,
            self.GRID_GRAY,
            (plot_x, plot_y + plot_h),
            (plot_x + plot_w, plot_y + plot_h),
            1
        )
        pygame.draw.line(
            self.screen,
            self.GRID_GRAY,
            (plot_x, plot_y),
            (plot_x, plot_y + plot_h),
            1
        )
        
        # Data range
        y_min = min(y_data)
        y_max = max(y_data)
        y_range = y_max - y_min if y_max > y_min else 1.0
        
        x_min = min(x_data)
        x_max = max(x_data)
        x_range = x_max - x_min if x_max > x_min else 1.0
        
        # Plot data
        points = []
        for i, (t, y) in enumerate(zip(x_data, y_data)):
            px = plot_x + int((t - x_min) / x_range * plot_w)
            py = plot_y + plot_h - int((y - y_min) / y_range * plot_h)
            points.append((px, py))
        
        if len(points) > 1:
            pygame.draw.lines(self.screen, color, False, points, 2)
        
        # Labels
        label_min = self.font_small.render(f"{y_min:.1f}", True, self.GRID_GRAY)
        label_max = self.font_small.render(f"{y_max:.1f}", True, self.GRID_GRAY)
        self.screen.blit(label_max, (plot_x - 25, plot_y - 5))
        self.screen.blit(label_min, (plot_x - 25, plot_y + plot_h - 10))
    
    def _draw_subplot_dual(
        self,
        x_data,
        y1_data,
        y2_data,
        x_pos,
        y_pos,
        width,
        height,
        title,
        color1,
        color2,
        label1,
        label2
    ):
        """Helper to draw dual time-series plot."""
        margin = 30
        plot_x = x_pos + margin
        plot_y = y_pos + margin
        plot_w = width - 2 * margin
        plot_h = height - 2 * margin
        
        # Title with legend
        title_surf = self.font_small.render(title, True, self.TEXT_WHITE)
        self.screen.blit(title_surf, (plot_x, y_pos + 5))
        
        leg1 = self.font_small.render(label1, True, color1)
        leg2 = self.font_small.render(label2, True, color2)
        self.screen.blit(leg1, (plot_x + 120, y_pos + 5))
        self.screen.blit(leg2, (plot_x + 150, y_pos + 5))
        
        # Axes
        pygame.draw.line(
            self.screen,
            self.GRID_GRAY,
            (plot_x, plot_y + plot_h),
            (plot_x + plot_w, plot_y + plot_h),
            1
        )
        pygame.draw.line(
            self.screen,
            self.GRID_GRAY,
            (plot_x, plot_y),
            (plot_x, plot_y + plot_h),
            1
        )
        
        # Combined range
        all_y = y1_data + y2_data
        y_min = min(all_y)
        y_max = max(all_y)
        y_range = y_max - y_min if y_max > y_min else 1.0
        
        x_min = min(x_data)
        x_max = max(x_data)
        x_range = x_max - x_min if x_max > x_min else 1.0
        
        # Plot both datasets
        for y_data, color in [(y1_data, color1), (y2_data, color2)]:
            points = []
            for t, y in zip(x_data, y_data):
                px = plot_x + int((t - x_min) / x_range * plot_w)
                py = plot_y + plot_h - int((y - y_min) / y_range * plot_h)
                points.append((px, py))
            if len(points) > 1:
                pygame.draw.lines(self.screen, color, False, points, 2)
    
    def update_history(self, t: float, s: np.ndarray):
        """Update time-history buffers."""
        self.time_history.append(t)
        self.V_history.append(s[0])
        self.alpha_history.append(s[1])
        self.q_history.append(s[2])
        self.theta_history.append(s[3])
        
        # Trim to max length
        if len(self.time_history) > self.max_history:
            self.time_history.pop(0)
            self.V_history.pop(0)
            self.alpha_history.pop(0)
            self.q_history.pop(0)
            self.theta_history.pop(0)
    
    def render(
        self,
        s: np.ndarray,
        a: np.ndarray,
        t: float,
        forces: Tuple[float, float, float, float]
    ):
        """
        Render complete frame.
        
        Args:
            s: State vector
            a: Control vector
            t: Simulation time
            forces: (L, D, M, T)
        """
        V, alpha, q, theta = s
        
        # Draw layers
        self.draw_horizon(theta)
        self.draw_velocity_vector(V, alpha, theta)
        self.draw_aircraft(theta, alpha)
        self.draw_telemetry(s, a, t, forces)
        self.draw_plots()
        
        # Warning overlays
        if abs(alpha) > np.deg2rad(12):
            warning = self.font_large.render("⚠ HIGH ANGLE OF ATTACK", True, self.WARNING_YELLOW)
            self.screen.blit(warning, (20, 20))
        
        if V < 15:
            warning = self.font_large.render("⚠ LOW AIRSPEED", True, self.DANGER_RED)
            self.screen.blit(warning, (20, 60))
        
        # Instructions
        instructions = [
            "ZERO CONTROL MODE (Free Flight)",
            "Press SPACE to reset",
            "Press ESC to quit"
        ]
        y_off = self.height - 100
        for instr in instructions:
            text = self.font_small.render(instr, True, self.TEXT_WHITE)
            self.screen.blit(text, (20, y_off))
            y_off += 20
        
        pygame.display.flip()


def main():
    aircraft = LongitudinalDynamics()
    viz = AircraftVisualization(width=1400, height=800)
    
    # ===== INITIAL CONDITION SELECTOR =====
    print("Select initial condition:")
    print("1. Default (non-trim, shows dive-pullup)")
    print("2. Trim for glide (smooth phugoid)")
    print("3. High speed zoom climb")
    print("4. Custom")
    
    choice = input("Enter choice (1-4): ").strip()
    
    if choice == "1":
        s = np.array([30.0, np.deg2rad(5), 0.0, np.deg2rad(5)])
        a = np.array([0.0, 0.0])
        print("→ Non-equilibrium start (expect dive-pullup)")
        
    elif choice == "2":
        # Compute glide trim (5° descent)
        V_glide = 25.0
        gamma_glide = np.deg2rad(-5)
        
        # Manual trim approximation for glide
        p = aircraft.params
        q_bar = 0.5 * p.rho * V_glide**2
        C_L_req = (p.m * p.g * np.cos(gamma_glide)) / (q_bar * p.S)
        alpha_trim = (C_L_req - p.C_L_0) / p.C_L_alpha
        theta_trim = alpha_trim + gamma_glide
        
        s = np.array([V_glide, alpha_trim, 0.0, theta_trim])
        a = np.array([0.0, 0.0])
        print(f"→ Glide trim: V={V_glide} m/s, γ={np.rad2deg(gamma_glide):.1f}°")
        
    elif choice == "3":
        s = np.array([50.0, np.deg2rad(2), 0.0, np.deg2rad(2)])
        a = np.array([0.0, 0.0])
        print("→ High speed zoom (expect climb then phugoid)")
        
    else:
        # Custom input
        V = float(input("  Airspeed V [m/s]: "))
        alpha_deg = float(input("  Angle of attack α [deg]: "))
        theta_deg = float(input("  Pitch angle θ [deg]: "))
        s = np.array([V, np.deg2rad(alpha_deg), 0.0, np.deg2rad(theta_deg)])
        a = np.array([0.0, 0.0])
    
    # Simulation parameters
    dt = 0.01  # 100 Hz
    sim_speed = 1.0  # Real-time
    t = 0.0
    
    running = True
    paused = False
    
    print("=" * 70)
    print("AIRCRAFT LONGITUDINAL DYNAMICS VISUALIZATION")
    print("=" * 70)
    print("Initial state:")
    print(f"  V = {s[0]:.2f} m/s")
    print(f"  α = {np.rad2deg(s[1]):.2f}°")
    print(f"  q = {np.rad2deg(s[2]):.2f}°/s")
    print(f"  θ = {np.rad2deg(s[3]):.2f}°")
    print()
    print("Controls: ZERO (Free flight simulation)")
    print("Press SPACE to reset, ESC to quit")
    print("=" * 70)
    
    while running:
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    # Reset simulation
                    s = np.array([30.0, np.deg2rad(5), 0.0, np.deg2rad(5)])
                    t = 0.0
                    viz.time_history.clear()
                    viz.V_history.clear()
                    viz.alpha_history.clear()
                    viz.q_history.clear()
                    viz.theta_history.clear()
                    print("\n[RESET] Simulation restarted")
                elif event.key == pygame.K_p:
                    paused = not paused
                    print(f"\n[PAUSE] {'Paused' if paused else 'Resumed'}")
        
        if not paused:
            # Integrate dynamics
            s = aircraft.step(s, a, dt, method='rk4')
            t += dt
            
            # Compute forces for display
            V, alpha, q, theta = s
            delta_e, delta_t = a
            L, D, M, T = aircraft.compute_forces_and_moments(V, alpha, q, delta_e, delta_t)
            forces = (L, D, M, T)
            
            # Update history
            viz.update_history(t, s)
            
            # Check validity
            if not aircraft.is_valid_state(s):
                print(f"\n*** MODEL VALIDITY VIOLATED at t={t:.2f}s ***")
                print(f"State: V={s[0]:.2f}, α={np.rad2deg(s[1]):.2f}°, "
                      f"q={np.rad2deg(s[2]):.2f}°/s, θ={np.rad2deg(s[3]):.2f}°")
                paused = True
        else:
            # Still compute forces for display even when paused
            V, alpha, q, theta = s
            delta_e, delta_t = a
            L, D, M, T = aircraft.compute_forces_and_moments(V, alpha, q, delta_e, delta_t)
            forces = (L, D, M, T)
        
        # Render
        viz.render(s, a, t, forces)
        
        # Control framerate
        viz.clock.tick(60)  # 60 FPS display
    
    pygame.quit()
    print("\nSimulation ended.")
    print(f"Final state at t={t:.2f}s:")
    print(f"  V = {s[0]:.2f} m/s")
    print(f"  α = {np.rad2deg(s[1]):.2f}°")
    print(f"  q = {np.rad2deg(s[2]):.2f}°/s")
    print(f"  θ = {np.rad2deg(s[3]):.2f}°")


if __name__ == "__main__":
    main()