"""
controller.py
-------------
Lyapunov-based switching adaptive elevator controller for icing-induced
degradation of C_L_alpha.

This controller is written to match the following nonlinear longitudinal
aircraft model:

    x = [V, alpha, q, theta]
    u = [delta_e, delta_t]

    V_dot     = (1/m)  [ T*cos(alpha) - D - m*g*sin(gamma) ]
    alpha_dot = q - (1/(m*V)) [ T*sin(alpha) + L - m*g*cos(gamma) ]
    q_dot     = (1/I_y) [ 0.5*rho*V^2*S*c_bar * C_m ]
    theta_dot = q

where:

    gamma = theta - alpha

Aerodynamics:

    C_L = C_L0 + C_La*alpha + C_Lde*delta_e
    C_D = C_D0 + (C_La*alpha + C_Lde*delta_e)^2 / (pi*e*AR)
    C_m = C_m0 + C_ma*alpha + C_mq*(q*c_bar)/(2*V) + C_mde*delta_e

Icing model:

    C_La_iced = C_La_clean + Delta C_La

where Delta C_La < 0.

Important:
----------
In this aircraft model, Delta C_La affects the lift force L and therefore
alpha_dot. It does NOT directly affect q_dot. Therefore the adaptive
controller is derived for the filtered error:

    r = e_q + lambda_alpha * e_alpha

and for the reduced r-dynamics:

    r_dot = F(s) + B(s)*delta_e + Y(s)*Delta C_La

where:

    F(s) = f_q(s) + lambda_alpha * f_alpha(s)
    B(s) = b_q(s) + lambda_alpha * b_alpha(s)
    Y(s) = lambda_alpha * Y_alpha(s)

The control law is:

    delta_e = ( -F(s) - k_r*r - Y(s)*Delta_C_hat ) / B(s)

The adaptation law is:

    Delta_C_hat_dot = gamma_C * Y(s) * r

Projection keeps Delta_C_hat in [delta_C_min, 0], because icing reduces
C_L_alpha and therefore Delta C_La should be non-positive.

Assumptions for the Lyapunov proof:
-----------------------------------
1. alpha_ref and q_ref are constant, or their derivatives are provided.
2. B(s) is not zero.
3. The elevator is not saturated during the strict Lyapunov proof.
4. After icing onset, Delta C_La is piecewise constant.
5. States remain in the valid flight envelope, especially V > 0.
"""

import numpy as np


# ---------------------------------------------------------------------------
# Default controller parameters
# These values are matched to the system.py parameters provided by the user.
# ---------------------------------------------------------------------------
DEFAULT_CONTROLLER_PARAMS = {
    # Geometry / inertia
    "m":       1200.0,
    "S":       16.2,
    "c_bar":   1.5,
    "I_y":     1800.0,

    # Engine / atmosphere
    "T_max":   3000.0,
    "rho":     1.225,
    "g":       9.81,

    # Clean lift model
    "C_L0":    0.0901,
    "C_La":    3.50,
    "C_Lde":   0.35,

    # Pitching moment model
    "C_m0":    0.02,
    "C_ma":   -0.60,
    "C_mq":   -8.00,
    "C_mde":  -1.10,

    # Default throttle used by controller in f_alpha(s).
    # Ideally this should be set equal to the trim throttle from AircraftSystem.trim().
    # You can pass it via params={"delta_t": delta_t_trim}.
    "delta_t": 0.0,

    # Lyapunov gains
    "lambda_alpha": 1.5,
    "k_r":          2.0,

    # Adaptation gain
    "gamma_C":      300.,

    # Projection bounds for Delta C_La estimate.
    # Icing reduces C_La, so Delta C_La <= 0.
    "delta_C_min": -3.0,

    # Elevator saturation
    "delta_e_max": np.deg2rad(25.0),

    # Detection thresholds
    "e_alpha_thr":  np.deg2rad(3.0),
    "r_thr":        np.deg2rad(2.0),
    "delta_e_thr":  np.deg2rad(10.0),
    "detect_steps": 20,

    # Numerical safety
    "V_min": 1.0,
    "B_min": 1e-6,
}


class LyapunovIcingAdaptiveController:
    """
    Lyapunov-based switching adaptive elevator controller.

    State expected by compute_control:
        state = {
            "V": float,
            "alpha": float,
            "q": float,
            "theta": float,
            optional "delta_t": float
        }

    Reference expected by compute_control:
        reference = {
            "alpha_ref": float,
            optional "q_ref": float,
            optional "alpha_ref_dot": float,
            optional "q_ref_dot": float
        }

    Control output:
        delta_e [rad]
    """

    def __init__(self, params=None, use_adaptation=True):
        self.params = {**DEFAULT_CONTROLLER_PARAMS, **(params or {})}
        self.use_adaptation = use_adaptation # NEW FLAG
        self.delta_CL_alpha_hat = 0.0
        self.adaptive_mode = False
        self.detect_counter = 0


    # ------------------------------------------------------------------
    # Basic error definitions
    # ------------------------------------------------------------------
    def compute_errors(self, state: dict, reference: dict):
        alpha = state["alpha"]
        q = state["q"]

        alpha_ref = reference["alpha_ref"]
        q_ref = reference.get("q_ref", 0.0)

        e_alpha = alpha - alpha_ref
        e_q = q - q_ref

        return e_alpha, e_q

    def compute_r(self, e_alpha: float, e_q: float) -> float:
        lam = self.params["lambda_alpha"]
        return e_q + lam * e_alpha

    # ------------------------------------------------------------------
    # Components of q_dot
    # q_dot = f_q(s) + b_q(s)*delta_e
    # ------------------------------------------------------------------
    def compute_f_q(self, state: dict) -> float:
        """
        Nominal pitch-rate dynamics term f_q(s).

        From:

            q_dot = M / I_y

            M = 0.5*rho*V^2*S*c_bar*C_m

            C_m = C_m0
                  + C_ma*alpha
                  + C_mq*(q*c_bar)/(2V)
                  + C_mde*delta_e

        Therefore, f_q(s) is the part without delta_e:

            f_q = 0.5*rho*V^2*S*c_bar/I_y *
                  [C_m0 + C_ma*alpha + C_mq*(q*c_bar)/(2V)]
        """

        p = self.params

        V = max(float(state["V"]), p["V_min"])
        alpha = float(state["alpha"])
        q = float(state["q"])

        C_m_nom = (
            p["C_m0"]
            + p["C_ma"] * alpha
            + p["C_mq"] * (q * p["c_bar"]) / (2.0 * V)
        )

        f_q = (
            0.5 * p["rho"] * V**2 * p["S"] * p["c_bar"] * C_m_nom
            / p["I_y"]
        )

        return float(f_q)

    def compute_b_q(self, state: dict) -> float:
        """
        Elevator effectiveness in q_dot.

            b_q = 0.5*rho*V^2*S*c_bar*C_mde / I_y
        """

        p = self.params

        V = max(float(state["V"]), p["V_min"])

        b_q = (
            0.5 * p["rho"] * V**2 * p["S"] * p["c_bar"] * p["C_mde"]
            / p["I_y"]
        )

        return float(b_q)

    # ------------------------------------------------------------------
    # Components of alpha_dot
    # alpha_dot = f_alpha(s) + b_alpha(s)*delta_e
    #             + Y_alpha(s)*Delta C_La
    # ------------------------------------------------------------------
    def compute_f_alpha(self, state: dict) -> float:
        """
        Nominal alpha dynamics f_alpha(s), without elevator and without icing.

        Original model:

            alpha_dot =
                q - (1/(mV)) [ T*sin(alpha) + L - m*g*cos(gamma) ]

        with:

            gamma = theta - alpha

        Clean lift without elevator and without icing:

            L_clean_no_de =
                0.5*rho*V^2*S * (C_L0 + C_La*alpha)

        Therefore:

            f_alpha =
                q - (1/(mV)) [
                    T*sin(alpha)
                    + L_clean_no_de
                    - m*g*cos(theta-alpha)
                ]
        """

        p = self.params

        V = max(float(state["V"]), p["V_min"])
        alpha = float(state["alpha"])
        q = float(state["q"])
        theta = float(state["theta"])

        # Prefer delta_t from state, otherwise use controller parameter.
        delta_t = float(state.get("delta_t", p["delta_t"]))
        T = p["T_max"] * delta_t

        gamma = theta - alpha

        L_clean_no_de = (
            0.5 * p["rho"] * V**2 * p["S"]
            * (p["C_L0"] + p["C_La"] * alpha)
        )

        f_alpha = (
            q
            - (
                T * np.sin(alpha)
                + L_clean_no_de
                - p["m"] * p["g"] * np.cos(gamma)
            ) / (p["m"] * V)
        )

        return float(f_alpha)

    def compute_b_alpha(self, state: dict) -> float:
        """
        Elevator effectiveness in alpha_dot.

        Since:

            L_de = 0.5*rho*V^2*S*C_Lde*delta_e

        and L enters alpha_dot with a negative sign:

            alpha_dot = ... - L/(mV)

        we get:

            b_alpha = -0.5*rho*V*S*C_Lde / m
        """

        p = self.params

        V = max(float(state["V"]), p["V_min"])

        b_alpha = -0.5 * p["rho"] * V * p["S"] * p["C_Lde"] / p["m"]

        return float(b_alpha)

    def compute_Y_alpha(self, state: dict) -> float:
        """
        Icing regressor in alpha_dot.

        Icing changes:

            C_La -> C_La + Delta C_La

        Therefore:

            Delta C_L = alpha * Delta C_La

            Delta L = 0.5*rho*V^2*S*alpha*Delta C_La

        Since L enters alpha_dot with a negative sign:

            Delta alpha_dot =
                - Delta L / (mV)
              =
                -0.5*rho*V*S*alpha/m * Delta C_La

        Hence:

            Y_alpha(s) = -0.5*rho*V*S*alpha/m
        """

        p = self.params

        V = max(float(state["V"]), p["V_min"])
        alpha = float(state["alpha"])

        Y_alpha = -0.5 * p["rho"] * V * p["S"] * alpha / p["m"]

        return float(Y_alpha)

    # ------------------------------------------------------------------
    # Reduced r-dynamics:
    # r_dot = F(s) + B(s)*delta_e + Y(s)*Delta C_La
    # ------------------------------------------------------------------
    def compute_F_B_Y(self, state: dict, reference: dict):
        """
        Compute reduced r-dynamics terms:

            r = e_q + lambda_alpha*e_alpha

            r_dot =
                F(s) + B(s)*delta_e + Y(s)*Delta C_La

        For non-constant references:

            e_alpha_dot = alpha_dot - alpha_ref_dot
            e_q_dot     = q_dot     - q_ref_dot

        Therefore:

            F = f_q + lambda*f_alpha - q_ref_dot - lambda*alpha_ref_dot
            B = b_q + lambda*b_alpha
            Y = lambda*Y_alpha
        """

        p = self.params
        lam = p["lambda_alpha"]

        f_q = self.compute_f_q(state)
        b_q = self.compute_b_q(state)

        f_alpha = self.compute_f_alpha(state)
        b_alpha = self.compute_b_alpha(state)
        Y_alpha = self.compute_Y_alpha(state)

        alpha_ref_dot = float(reference.get("alpha_ref_dot", 0.0))
        q_ref_dot = float(reference.get("q_ref_dot", 0.0))

        F = f_q + lam * f_alpha - q_ref_dot - lam * alpha_ref_dot
        B = b_q + lam * b_alpha
        Y = lam * Y_alpha

        return float(F), float(B), float(Y), {
            "f_q": f_q,
            "b_q": b_q,
            "f_alpha": f_alpha,
            "b_alpha": b_alpha,
            "Y_alpha": Y_alpha,
        }

    # ------------------------------------------------------------------
    # Detection logic
    # ------------------------------------------------------------------
    def detect_degradation(self, e_alpha: float, r: float, delta_e_nom: float):
        p = self.params

        # 1. Use OR instead of AND for errors, or lower the thresholds.
        # 2. Usually, just checking 'r' or 'e_alpha' is enough.
        error_condition = (abs(e_alpha) > p["e_alpha_thr"] or abs(r) > p["r_thr"])
        
        # 3. Elevator check: icing usually forces the elevator to work harder.
        control_condition = abs(delta_e_nom) > p["delta_e_thr"]

        # If error is high OR control is unusually high
        if error_condition or control_condition:
            self.detect_counter += 1
        else:
            # Instead of resetting to 0, slowly decrease it (makes it more robust to noise)
            self.detect_counter = max(0, self.detect_counter - 1)

        if self.detect_counter >= p["detect_steps"]:
            self.adaptive_mode = True

    # ------------------------------------------------------------------
    # Adaptation law
    # ------------------------------------------------------------------
    def update_adaptation(self, Y: float, r: float, dt: float):
        """
        Lyapunov adaptation law:

            Delta_C_hat_dot = gamma_C * Y(s) * r

        Projection:

            Delta_C_hat in [delta_C_min, 0]
        """

        p = self.params

        delta_C_hat_dot = p["gamma_C"] * Y * r

        self.delta_CL_alpha_hat += delta_C_hat_dot * dt

        # print(self.delta_CL_alpha_hat)
        # Projection: icing should not increase C_L_alpha.
        self.delta_CL_alpha_hat = float(np.clip(
            self.delta_CL_alpha_hat,
            p["delta_C_min"],
            0.0
        ))

        return float(delta_C_hat_dot)

    # ------------------------------------------------------------------
    # Main control computation
    # ------------------------------------------------------------------
    def compute_control(self, state: dict, reference: dict, dt: float):
        """
        Compute elevator deflection command.

        Parameters
        ----------
        state : dict
            Required:
                "V", "alpha", "q", "theta"
            Optional:
                "delta_t"

        reference : dict
            Required:
                "alpha_ref"
            Optional:
                "q_ref", "alpha_ref_dot", "q_ref_dot"

        dt : float
            Integration time step [s].

        Returns
        -------
        delta_e : float
            Elevator command [rad].

        info : dict
            Diagnostic information.
        """

        p = self.params
        e_alpha, e_q = self.compute_errors(state, reference)
        r = self.compute_r(e_alpha, e_q)
        F, B, Y, parts = self.compute_F_B_Y(state, reference)

        if abs(B) < p["B_min"]:
            B_safe = np.sign(B) * p["B_min"] if B != 0.0 else p["B_min"]
        else:
            B_safe = B

        delta_e_nom = (-F - p["k_r"] * r) / B_safe


        # ------------------------------------------------------------------
        # Nominal Lyapunov control:
        #
        #   delta_e_nom = (-F - k_r*r) / B
        #
        # This imposes:
        #
        #   r_dot = -k_r*r
        #
        # in the clean-wing case.
        # ------------------------------------------------------------------
        # ONLY run detection and adaptation if enabled
        delta_C_hat_dot = 0.0
        delta_e_adapt = 0.0
        
        if self.use_adaptation:
            if not self.adaptive_mode:
                self.detect_degradation(e_alpha, r, delta_e_nom)

            if self.adaptive_mode:
                delta_C_hat_dot = self.update_adaptation(Y, r, dt)
                delta_e_adapt = -Y * self.delta_CL_alpha_hat / B_safe

        delta_e_raw = delta_e_nom + delta_e_adapt
        delta_e = float(np.clip(delta_e_raw, -p["delta_e_max"], p["delta_e_max"]))

        # ------------------------------------------------------------------
        # Adaptive compensation:
        #
        #   delta_e_adapt = -Y*Delta_C_hat / B
        #
        # Total:
        #
        #   delta_e = (-F - k_r*r - Y*Delta_C_hat) / B
        # ------------------------------------------------------------------
        # delta_e_adapt = 0.0
        # if self.adaptive_mode:
        #     delta_e_adapt = -Y * self.delta_CL_alpha_hat / B_safe
            # print("\n\n\n\n\n")
            # print(delta_e_adapt)

        delta_e_raw = delta_e_nom + delta_e_adapt

        # Saturation.
        delta_e = float(np.clip(
            delta_e_raw,
            -p["delta_e_max"],
            p["delta_e_max"]
        ))

        info = {
            "adaptive_mode": self.adaptive_mode,

            # Errors
            "e_alpha": e_alpha,
            "e_q": e_q,
            "r": r,

            # Reduced dynamics terms
            "F": F,
            "B": B,
            "B_safe": B_safe,
            "Y": Y,

            # Components
            "f_q": parts["f_q"],
            "b_q": parts["b_q"],
            "f_alpha": parts["f_alpha"],
            "b_alpha": parts["b_alpha"],
            "Y_alpha": parts["Y_alpha"],

            # Control
            "delta_e_nom": float(delta_e_nom),
            "delta_e_adapt": float(delta_e_adapt),
            "delta_e_raw": float(delta_e_raw),
            "delta_e": delta_e,

            # Adaptation
            "delta_CL_alpha_hat": self.delta_CL_alpha_hat,
            "delta_C_hat_dot": delta_C_hat_dot,

            # Detection
            "detect_counter": self.detect_counter,
        }

        return delta_e, info

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def reset(self):
        self.delta_CL_alpha_hat = 0.0
        self.adaptive_mode = False
        self.detect_counter = 0