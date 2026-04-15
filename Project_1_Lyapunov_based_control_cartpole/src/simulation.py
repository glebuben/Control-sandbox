# simulation.py
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional

from system import CartPoleSystem


def _normalize(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


@dataclass
class SimulationConfig:
    dt: float = 0.002
    T: float = 20.0
    save_every: int = 1
    random_seed: int = 42


@dataclass
class SimulationResult:
    """
    Stores all data from one simulation run.

    Attributes
    ----------
    times           : 1-D array of recorded time stamps
    states          : (N, 4) array  [x, x_dot, theta, theta_dot]
    controls        : 1-D array of applied forces [N]
    energies        : 1-D array of pendulum energy at each step
    lyapunov_values : 1-D array of Lyapunov function value at each step
    modes           : list of mode strings ("swing_up" / "stabilization")
    l               : pendulum length [m]  – needed by visualizer
    e_up            : target upright energy [J] – needed by visualizer
    controller_name : human-readable label for plots
    params          : CartPoleParams instance (optional, for energy calc)
    """
    times:           np.ndarray = field(default_factory=lambda: np.array([]))
    states:          np.ndarray = field(default_factory=lambda: np.zeros((0, 4)))
    controls:        np.ndarray = field(default_factory=lambda: np.array([]))
    energies:        np.ndarray = field(default_factory=lambda: np.array([]))
    lyapunov_values: np.ndarray = field(default_factory=lambda: np.array([]))
    modes:           List[str]  = field(default_factory=list)
    l:               float      = 0.5
    e_up:            Optional[float] = None
    controller_name: str        = ""
    params:          object     = None   # CartPoleParams, kept as object to avoid circular import


class CartPoleSimulation:
    """
    Runs the cart-pole simulation and collects results.

    Parameters
    ----------
    system     : CartPoleSystem
    controller : any controller with .compute_control() and .compute_lyapunov()
    config     : SimulationConfig
    """

    def __init__(self, system, controller, config):
        # type: (CartPoleSystem, object, SimulationConfig) -> None
        self.system     = system
        self.controller = controller
        self.config     = config

    def run(self, initial_state, verbose=False):
        # type: (np.ndarray, bool) -> SimulationResult
        """
        Simulate from initial_state for config.T seconds.

        Returns a fully-populated SimulationResult.
        """
        dt      = self.config.dt
        n_steps = int(self.config.T / dt)
        save    = self.config.save_every

        times_list    = []
        states_list   = []
        controls_list = []
        energies_list = []
        lyapunov_list = []
        modes_list    = []

        state = self.system.reset(initial_state)
        self.controller.mode = "swing_up"

        for i in range(n_steps):
            t = i * dt

            # Always compute control (updates internal mode)
            u_val, mode = self.controller.compute_control(state)

            if i % save == 0:
                lyap = self.controller.compute_lyapunov(state)

                times_list.append(t)
                states_list.append(state.copy())
                controls_list.append(float(u_val))
                energies_list.append(
                    float(self.system.get_pendulum_energy(state))
                )
                lyapunov_list.append(float(lyap))
                modes_list.append(mode)

            state = self.system.step(u_val, dt)

            if verbose and i % max(1, n_steps // 10) == 0:
                th_deg = np.degrees(_normalize(state[2]))
                print(
                    "  t={:6.2f}s  mode={:14s}  "
                    "theta={:+7.2f} deg  x={:+.3f} m".format(
                        t, mode, th_deg, state[0]
                    )
                )

        # Resolve metadata from system / controller
        p    = self.system.params
        e_up = 2.0 * p.m_p * p.g * p.l

        ctrl_name = getattr(self.controller, "controller_name", "")

        result = SimulationResult(
            times           = np.array(times_list),
            states          = np.array(states_list),          # shape (N, 4)
            controls        = np.array(controls_list),
            energies        = np.array(energies_list),
            lyapunov_values = np.array(lyapunov_list),
            modes           = modes_list,
            l               = float(p.l),
            e_up            = float(e_up),
            controller_name = ctrl_name,
            params          = p,
        )
        return result