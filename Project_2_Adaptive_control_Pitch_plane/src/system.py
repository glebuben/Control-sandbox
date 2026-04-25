import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class AircraftParameters:
    """Physical and aerodynamic parameters for the aircraft."""
    
    # Mass and inertia
    m: float = 1100.0           # Mass [kg]
    I_y: float = 1800.0          # Pitch moment of inertia [kg·m²]
    
    # Geometric properties
    S: float = 16.2              # Wing reference area [m²]
    c_bar: float = 1.5           # Mean aerodynamic chord [m]
    b: float = 11.0              # Wing span [m]
    
    # Propulsion
    T_max: float = 2500.0        # Maximum thrust [N]
    
    # Aerodynamic coefficients - Lift
    C_L_0: float = 0.28          # Zero-alpha lift coefficient
    C_L_alpha: float = 5.0       # Lift curve slope [1/rad]
    C_L_delta_e: float = 0.4     # Elevator lift effectiveness [1/rad]
    
    # Aerodynamic coefficients - Drag
    C_D_0: float = 0.03          # Zero-lift drag coefficient
    e: float = 0.80              # Oswald efficiency factor
    
    # Aerodynamic coefficients - Pitching moment
    C_m_0: float = 0.00          # Zero pitching moment coefficient
    C_m_alpha: float = -0.6      # Pitch stiffness (static stability) [1/rad]
    C_m_q: float = -8.0          # Pitch damping [1/rad]
    C_m_delta_e: float = -1.2    # Elevator moment effectiveness [1/rad]
    
    # Environmental
    rho: float = 1.225           # Air density [kg/m³] (sea level)
    g: float = 9.81              # Gravitational acceleration [m/s²]
    
    # Control limits
    delta_e_max: float = np.deg2rad(25)  # Max elevator deflection [rad]
    delta_t_min: float = 0.0             # Min throttle [-]
    delta_t_max: float = 1.0             # Max throttle [-]
    
    # Stall limit (model validity)
    alpha_stall: float = np.deg2rad(15)  # Stall angle of attack [rad]
    
    @property
    def AR(self) -> float:
        """Wing aspect ratio."""
        return self.b**2 / self.S
    
    @property
    def K(self) -> float:
        """Induced drag factor."""
        return 1.0 / (np.pi * self.e * self.AR)


class LongitudinalDynamics:
    """
    Nonlinear longitudinal dynamics model for a fixed-wing aircraft.
    
    State vector: s = [V, alpha, q, theta]
        V     : True airspeed [m/s]
        alpha : Angle of attack [rad]
        q     : Pitch rate [rad/s]
        theta : Pitch angle [rad]
    
    Control vector: a = [delta_e, delta_t]
        delta_e : Elevator deflection [rad]
        delta_t : Throttle command [-] (0 to 1)
    
    Dynamics: ds/dt = P(s, a) + d(t)
    
    Reference:
        Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2016).
        Aircraft Control and Simulation (3rd ed.). Wiley.
        Section 2.3, equations 2.3-24 through 2.3-27.
    """
    
    def __init__(self, params: Optional[AircraftParameters] = None):
        """
        Initialize the dynamics model.
        
        Args:
            params: Aircraft parameters. If None, uses Cessna 172 defaults.
        """
        self.params = params if params is not None else AircraftParameters()
        
    def compute_aerodynamic_coefficients(
        self, 
        V: float, 
        alpha: float, 
        q: float, 
        delta_e: float
    ) -> Tuple[float, float, float]:
        """
        Compute lift, drag, and pitching moment coefficients.
        
        Args:
            V: Airspeed [m/s]
            alpha: Angle of attack [rad]
            q: Pitch rate [rad/s]
            delta_e: Elevator deflection [rad]
            
        Returns:
            (C_L, C_D, C_m): Aerodynamic coefficients
        """
        p = self.params
        
        # Lift coefficient (linear in alpha and delta_e)
        C_L = p.C_L_0 + p.C_L_alpha * alpha + p.C_L_delta_e * delta_e
        
        # Drag coefficient (parasitic + induced)
        C_D = p.C_D_0 + p.K * (C_L - p.C_L_0)**2
        
        # Dimensionless pitch rate
        q_hat = (q * p.c_bar) / (2 * V) if V > 1e-3 else 0.0
        
        # Pitching moment coefficient
        C_m = (p.C_m_0 + 
               p.C_m_alpha * alpha + 
               p.C_m_q * q_hat + 
               p.C_m_delta_e * delta_e)
        
        return C_L, C_D, C_m
    
    def compute_forces_and_moments(
        self,
        V: float,
        alpha: float,
        q: float,
        delta_e: float,
        delta_t: float
    ) -> Tuple[float, float, float, float]:
        """
        Compute aerodynamic forces and moments.
        
        Args:
            V: Airspeed [m/s]
            alpha: Angle of attack [rad]
            q: Pitch rate [rad/s]
            delta_e: Elevator deflection [rad]
            delta_t: Throttle command [-]
            
        Returns:
            (L, D, M, T): Lift [N], Drag [N], Pitching moment [N·m], Thrust [N]
        """
        p = self.params
        
        # Dynamic pressure
        q_bar = 0.5 * p.rho * V**2
        
        # Aerodynamic coefficients
        C_L, C_D, C_m = self.compute_aerodynamic_coefficients(V, alpha, q, delta_e)
        
        # Forces
        L = q_bar * p.S * C_L
        D = q_bar * p.S * C_D
        
        # Moment
        M = q_bar * p.S * p.c_bar * C_m
        
        # Thrust (linear with throttle)
        T = p.T_max * delta_t
        
        return L, D, M, T
    
    def dynamics(
        self, 
        s: np.ndarray, 
        a: np.ndarray, 
        d: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        Compute state derivatives: ds/dt = P(s, a) + d(t)
        
        Args:
            s: State vector [V, alpha, q, theta]
            a: Control vector [delta_e, delta_t]
            d: Disturbance vector (optional, same shape as s)
            
        Returns:
            s_dot: State derivatives [dV/dt, dalpha/dt, dq/dt, dtheta/dt]
        """
        p = self.params
        
        # Unpack state
        V, alpha, q, theta = s
        
        # Unpack control (with saturation)
        delta_e = np.clip(a[0], -p.delta_e_max, p.delta_e_max)
        delta_t = np.clip(a[1], p.delta_t_min, p.delta_t_max)
        
        # Compute forces and moments
        L, D, M, T = self.compute_forces_and_moments(V, alpha, q, delta_e, delta_t)
        
        # Avoid division by zero
        V_safe = max(V, 1e-3)
        
        # State derivatives (Stevens & Lewis, 2016, eqs. 2.3-24 to 2.3-27)
        dV_dt = (1/p.m) * (T * np.cos(alpha) - D - p.m * p.g * np.sin(theta))
        
        dalpha_dt = q - (1/(p.m * V_safe)) * (
            T * np.sin(alpha) + L - p.m * p.g * np.cos(theta)
        )
        
        dq_dt = M / p.I_y
        
        dtheta_dt = q
        
        # Construct derivative vector
        s_dot = np.array([dV_dt, dalpha_dt, dq_dt, dtheta_dt])
        
        # Add disturbances if provided
        if d is not None:
            s_dot += d
            
        return s_dot
    
    def step(
        self, 
        s: np.ndarray, 
        a: np.ndarray, 
        dt: float,
        d: Optional[np.ndarray] = None,
        method: str = 'rk4'
    ) -> np.ndarray:
        """
        Integrate dynamics forward by one time step.
        
        Args:
            s: Current state [V, alpha, q, theta]
            a: Control input [delta_e, delta_t]
            dt: Time step [s]
            d: Disturbance (optional)
            method: Integration method ('euler' or 'rk4')
            
        Returns:
            s_next: State at next time step
        """
        if method == 'euler':
            s_next = s + self.dynamics(s, a, d) * dt
            
        elif method == 'rk4':
            # 4th-order Runge-Kutta integration
            k1 = self.dynamics(s, a, d)
            k2 = self.dynamics(s + 0.5*dt*k1, a, d)
            k3 = self.dynamics(s + 0.5*dt*k2, a, d)
            k4 = self.dynamics(s + dt*k3, a, d)
            s_next = s + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
            
        else:
            raise ValueError(f"Unknown integration method: {method}")
        
        return s_next
    
    def trim_state(
        self, 
        V_target: float, 
        gamma: float = 0.0
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute trim state and control for steady flight.
        
        Args:
            V_target: Desired airspeed [m/s]
            gamma: Flight path angle [rad] (0 = level flight)
            
        Returns:
            (s_trim, a_trim): Trim state and control vectors
            
        Note: This is a simplified trim calculation.
        For precise trim, use numerical optimization.
        """
        p = self.params
        
        # For level flight (gamma ≈ 0), approximate trim conditions
        # Weight = Lift, Thrust = Drag
        
        # Required lift coefficient
        q_bar = 0.5 * p.rho * V_target**2
        C_L_req = (p.m * p.g * np.cos(gamma)) / (q_bar * p.S)
        
        # Solve for alpha (ignoring elevator)
        alpha_trim = (C_L_req - p.C_L_0) / p.C_L_alpha
        
        # Required drag
        C_D_trim = p.C_D_0 + p.K * (C_L_req - p.C_L_0)**2
        D_trim = q_bar * p.S * C_D_trim
        
        # Required thrust
        T_req = D_trim + p.m * p.g * np.sin(gamma)
        delta_t_trim = np.clip(T_req / p.T_max, 0.0, 1.0)
        
        # Elevator for trim (simplified: assume C_m ≈ 0)
        delta_e_trim = -(p.C_m_0 + p.C_m_alpha * alpha_trim) / p.C_m_delta_e
        delta_e_trim = np.clip(delta_e_trim, -p.delta_e_max, p.delta_e_max)
        
        # Pitch angle for steady flight
        theta_trim = alpha_trim + gamma
        
        s_trim = np.array([V_target, alpha_trim, 0.0, theta_trim])
        a_trim = np.array([delta_e_trim, delta_t_trim])
        
        return s_trim, a_trim
    
    def is_valid_state(self, s: np.ndarray) -> bool:
        """
        Check if state is within model validity bounds.
        
        Args:
            s: State vector [V, alpha, q, theta]
            
        Returns:
            True if state is valid, False otherwise
        """
        V, alpha, q, theta = s
        
        # Check physical bounds
        if V < 5.0:  # Minimum realistic airspeed
            return False
        if abs(alpha) > self.params.alpha_stall:  # Stall regime
            return False
        if abs(q) > np.deg2rad(100):  # Unrealistic pitch rate
            return False
            
        return True
    
    def get_flight_condition(self, s: np.ndarray) -> str:
        """
        Get human-readable flight condition string.
        
        Args:
            s: State vector
            
        Returns:
            Description of current flight condition
        """
        V, alpha, q, theta = s
        
        alpha_deg = np.rad2deg(alpha)
        theta_deg = np.rad2deg(theta)
        q_deg = np.rad2deg(q)
        
        conditions = []
        
        # Speed regime
        if V < 15:
            conditions.append("STALL SPEED")
        elif V < 25:
            conditions.append("Slow flight")
        elif V > 50:
            conditions.append("High speed")
            
        # Angle of attack
        if abs(alpha_deg) > 12:
            conditions.append("HIGH AoA")
        
        # Pitch attitude
        if theta_deg > 20:
            conditions.append("Climbing")
        elif theta_deg < -20:
            conditions.append("Diving")
            
        # Pitch rate
        if abs(q_deg) > 5:
            conditions.append("Pitching")
            
        return ", ".join(conditions) if conditions else "Normal flight"


# Example usage and testing
if __name__ == "__main__":
    # Create aircraft with default Cessna 172 parameters
    aircraft = LongitudinalDynamics()
    
    # Initial condition: straight and level flight at 30 m/s
    print("=" * 60)
    print("AIRCRAFT LONGITUDINAL DYNAMICS MODEL")
    print("=" * 60)
    print(f"Aircraft: Cessna 172 (default parameters)")
    print(f"Mass: {aircraft.params.m} kg")
    print(f"Wing area: {aircraft.params.S} m²")
    print(f"Aspect ratio: {aircraft.params.AR:.2f}")
    print()
    
    # Compute trim condition
    V_cruise = 30.0  # m/s
    s_trim, a_trim = aircraft.trim_state(V_cruise, gamma=0.0)
    
    print("TRIM CONDITION (level flight at 30 m/s):")
    print(f"  V     = {s_trim[0]:.2f} m/s")
    print(f"  alpha = {np.rad2deg(s_trim[1]):.2f}°")
    print(f"  q     = {np.rad2deg(s_trim[2]):.2f}°/s")
    print(f"  theta = {np.rad2deg(s_trim[3]):.2f}°")
    print(f"  delta_e = {np.rad2deg(a_trim[0]):.2f}°")
    print(f"  delta_t = {a_trim[1]:.3f}")
    print()
    
    # Test zero-control dynamics
    print("SIMULATING ZERO CONTROL (power-off glide):")
    s = np.array([30.0, np.deg2rad(5), 0.0, np.deg2rad(5)])
    a = np.array([0.0, 0.0])  # No control
    
    dt = 0.01
    t = 0.0
    
    print(f"{'Time':>6s} {'V':>8s} {'alpha':>8s} {'q':>8s} {'theta':>8s} {'Condition':>20s}")
    print("-" * 70)
    
    for i in range(500):  # 5 seconds
        if i % 50 == 0:  # Print every 0.5 seconds
            condition = aircraft.get_flight_condition(s)
            print(f"{t:6.2f} {s[0]:8.2f} {np.rad2deg(s[1]):8.2f} "
                  f"{np.rad2deg(s[2]):8.2f} {np.rad2deg(s[3]):8.2f} {condition:>20s}")
        
        # Integrate dynamics
        s = aircraft.step(s, a, dt, method='rk4')
        t += dt
        
        # Check validity
        if not aircraft.is_valid_state(s):
            print(f"\n*** Model validity violated at t={t:.2f}s ***")
            break
    
    print("\n" + "=" * 60)