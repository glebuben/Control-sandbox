import numpy as np
import time
from dataclasses import dataclass
from typing import List

from system import CartPoleSystem
from controller import LyapunovController, PureLyapunovController


@dataclass
class SimulationConfig:
    dt: float = 0.002
    T: float = 15.0
    save_every: int = 1
    random_seed: int = 42


@dataclass
class SimulationResult:
    time: np.ndarray
    states: np.ndarray
    controls: np.ndarray
    modes: List[str]
    lyapunov_values: np.ndarray
    energies: np.ndarray
    computation_time: float


class CartPoleSimulation:
    """
    Runs the cart-pole simulation for any controller that exposes:
        compute_control(state)  -> (u, mode_string)
        compute_lyapunov(state) -> float
    """

    def __init__(self,
                 system: CartPoleSystem,
                 controller,           # LyapunovController or PureLyapunovController
                 config: SimulationConfig):
        self.system = system
        self.controller = controller
        self.config = config

    def run(self, initial_state=None, verbose: bool = True) -> SimulationResult:
        np.random.seed(self.config.random_seed)
        state = self.system.reset(initial_state)

        N = int(self.config.T / self.config.dt)
        t_data, s_data, u_data, m_data, v_data, e_data = [], [], [], [], [], []

        start = time.time()
        if verbose:
            print(f"   Steps: {N}, dt={self.config.dt}s, T={self.config.T}s")

        for step in range(N):
            t = step * self.config.dt

            # --- compute control and Lyapunov value ---
            u, mode = self.controller.compute_control(state)
            V = self.controller.compute_lyapunov(state)   # unified method name
            E = self.system.get_energy(state)

            # --- save data ---
            if step % self.config.save_every == 0:
                t_data.append(t)
                s_data.append(state.copy())
                u_data.append(u)
                m_data.append(mode)
                v_data.append(V)
                e_data.append(E)

            # --- integrate ---
            state = self.system.step(u, self.config.dt)

            # --- guard against numerical blow-up ---
            if not np.all(np.isfinite(state)):
                if verbose:
                    print(f"⚠️  Simulation diverged at t={t:.3f}s (NaN/Inf). Stopping.")
                break

            if verbose and step % 500 == 0:
                theta_deg = np.degrees(state[2]) % 360
                print(f"   t={t:6.2f}s | θ={theta_deg:6.1f}° | "
                      f"x={state[0]:5.2f}m | V={V:.4f} | mode={mode}")

        comp_time = time.time() - start
        if verbose:
            print(f"✅ Done in {comp_time:.2f}s — {len(t_data)} data points saved.")

        return SimulationResult(
            time=np.array(t_data),
            states=np.array(s_data),
            controls=np.array(u_data),
            modes=m_data,
            lyapunov_values=np.array(v_data),
            energies=np.array(e_data),
            computation_time=comp_time,
        )