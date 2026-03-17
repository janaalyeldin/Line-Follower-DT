import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
"""
robot_kinematics.py — Client 1: Plant + Environment
Line-Following Robot Simulation (Differential Drive)

Provides robot state, path reference, and sensor noise to VSI fabric.

Signals published (mirrors invPend signals):
  - x_pos        : float  (robot X position)
  - y_pos        : float  (robot Y position)
  - theta        : float  (robot heading in radians)
  - lateral_error: float  (signed distance from path)
  - heading_error: float  (angle error to path tangent)
  - x_ref        : float  (reference X on path)
  - y_ref        : float  (reference Y on path)

Signals consumed (from controller):
  - v_left       : float  (left wheel velocity command)
  - v_right      : float  (right wheel velocity command)
"""

import numpy as np

from dataclasses import dataclass, field
from typing import List, Tuple
import random

#robot parameters
WHEEL_BASE = 0.2          # L  — distance between wheels (meters)
WHEEL_RADIUS = 0.05       # r  — wheel radius (meters)
DT = 0.01                 # simulation step (seconds), matches VSI 100Hz
TOTAL_TIME = 70.0         # seconds (matches vsiBuildCommands totalSimTime)


#robot state
@dataclass
class RobotState:
    x: float = 0.0         # X position (m)
    y: float = 0.0         # Y position (m)
    theta: float = 0.0     # heading (rad), 0 = facing +X axis

    def __repr__(self):
        return (f"RobotState(x={self.x:.3f}, y={self.y:.3f}, "
                f"theta={np.degrees(self.theta):.1f}°)")



def kinematics_step(state: RobotState,
                    v_left: float,
                    v_right: float,
                    noise_std: float = 0.0) -> RobotState:
    """
    Advance robot state by one timestep DT.
    Optionally adds Gaussian noise to wheel velocities (sensor noise).
    """
    # Add noise if requested (Experiment E3)
    if noise_std > 0:
        v_left  += np.random.normal(0, noise_std)
        v_right += np.random.normal(0, noise_std)

    # Compute linear and angular velocity
    v = (v_right + v_left) / 2.0
    omega = (v_right - v_left) / WHEEL_BASE

    # Euler integration
    new_x     = state.x     + v * np.cos(state.theta) * DT
    new_y     = state.y     + v * np.sin(state.theta) * DT
    new_theta = state.theta + omega * DT

    # Wrap theta to [-pi, pi]
    new_theta = (new_theta + np.pi) % (2 * np.pi) - np.pi

    return RobotState(x=new_x, y=new_y, theta=new_theta)


#st path class
class StraightPath:
    """
    Path Type 1: Straight line from start to end.
    y = m*x + b
    """
    def __init__(self, x_start=0.0, y_start=0.0,
                 x_end=10.0, y_end=0.0):
        self.x_start = x_start
        self.y_start = y_start
        self.x_end   = x_end
        self.y_end   = y_end
        dx = x_end - x_start
        dy = y_end - y_start
        self.length  = np.sqrt(dx**2 + dy**2)
        self.angle   = np.arctan2(dy, dx)   # heading of path

    def nearest_point(self, x: float, y: float) -> Tuple[float, float, float]:
        """
        Returns (x_ref, y_ref, tangent_angle) — closest point on path.
        """
        dx = self.x_end - self.x_start
        dy = self.y_end - self.y_start
        t = ((x - self.x_start) * dx + (y - self.y_start) * dy) / self.length**2
        t = np.clip(t, 0.0, 1.0)
        x_ref = self.x_start + t * dx
        y_ref = self.y_start + t * dy
        return x_ref, y_ref, self.angle

    def sample_points(self, n=200):
        xs = np.linspace(self.x_start, self.x_end, n)
        ys = np.linspace(self.y_start, self.y_end, n)
        return xs, ys

#curved path class uses sine wave
class CurvedPath:
    """
    Path Type 2: Sinusoidal curved path.

    where:
      s in [0, x_end]  — arc parameter (approx. = x for small amplitudes)
      A = amplitude     — peak lateral deviation (meters)
      L = wavelength    — one full S-cycle length (meters)

    Default: x_end=10m, A=1.0m, L=10m → one full S-curve over 10m.
  """
    def __init__(self, x_end: float = 10.0,
                 amplitude: float = 1.0,
                 wavelength: float = 10.0,
                 n_points: int = 500):
        self.x_end      = x_end
        self.amplitude  = amplitude
        self.wavelength = wavelength

        s = np.linspace(0.0, x_end, n_points)
        self._xs      = s
        self._ys      = amplitude * np.sin(2 * np.pi * s / wavelength)
        # dy/dx = A * (2*pi/L) * cos(2*pi*s/L)
        dydx          = amplitude * (2 * np.pi / wavelength) * np.cos(2 * np.pi * s / wavelength)
        self._tangents = np.arctan(dydx)

    def nearest_point(self, x: float, y: float) -> Tuple[float, float, float]:
        """Returns (x_ref, y_ref, tangent_angle) — closest point on path."""
        dists = (self._xs - x)**2 + (self._ys - y)**2
        idx   = np.argmin(dists)
        return float(self._xs[idx]), float(self._ys[idx]), float(self._tangents[idx])

    def sample_points(self):
        return self._xs, self._ys



def compute_errors(state: RobotState,
                   path) -> Tuple[float, float, float, float]:
    """
    Returns (lateral_error, heading_error, x_ref, y_ref)
    """
    x_ref, y_ref, psi = path.nearest_point(state.x, state.y)

    # Lateral error (signed distance to path)
    lateral_error = (-(state.x - x_ref) * np.sin(psi)
                     + (state.y - y_ref) * np.cos(psi))

    # Heading error (wrapped)
    heading_error = state.theta - psi
    heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

    return lateral_error, heading_error, x_ref, y_ref


#added noise for experiment 3
def apply_disturbance(state: RobotState,
                      disturbance_magnitude: float = 0.0) -> RobotState:
    """
    Applies a random lateral push to simulate external disturbance.
    """
    if disturbance_magnitude > 0:
        push = np.random.normal(0, disturbance_magnitude)
        state = RobotState(
            x=state.x + push * np.cos(state.theta + np.pi/2),
            y=state.y + push * np.sin(state.theta + np.pi/2),
            theta=state.theta
        )
    return state


#random spawn function to see how the robot will behave if randomly diverged from the path
def random_spawn(path,
                 lateral_offset_range=(-0.8, 0.8),
                 heading_offset_range=(-0.4, 0.4)) -> RobotState:
    """
    Spawns robot near the start of the path with random offset.
    """
    # Get first point of path
    xs, ys = path.sample_points()
    x_start, y_start = float(xs[0]), float(ys[0])
    _, _, psi = path.nearest_point(x_start, y_start)

    lat_off  = random.uniform(*lateral_offset_range)
    head_off = random.uniform(*heading_offset_range)

    x0 = x_start + lat_off * (-np.sin(psi))
    y0 = y_start + lat_off * ( np.cos(psi))
    theta0 = psi + head_off

    return RobotState(x=x0, y=y0, theta=theta0)


#rsimulator runner
def run_simulation(initial_state: RobotState,
                   path,
                   controller_fn,
                   total_time: float = TOTAL_TIME,
                   dt: float = DT,
                   noise_std: float = 0.0,
                   disturbance_mag: float = 0.0) -> dict:
    """
    Runs full simulation loop.

    Args:
        initial_state  : RobotState — starting position/heading
        path           : StraightPath or CurvedPath
        controller_fn  : callable(lateral_error, heading_error) -> (v_left, v_right)
        total_time     : float — simulation duration (s)
        noise_std      : float — wheel velocity noise std dev
        disturbance_mag: float — random disturbance magnitude

    Returns:
        dict with full history for logging and plotting
    """
    steps = int(total_time / dt)
    state = initial_state

    # History buffers 
    history = {
        'time':          [],
        'x':             [],
        'y':             [],
        'theta':         [],
        'lateral_error': [],
        'heading_error': [],
        'x_ref':         [],
        'y_ref':         [],
        'v_left':        [],
        'v_right':       [],
    }

    for step in range(steps):
        t = step * dt

        # Compute errors → send to controller
        lat_err, head_err, x_ref, y_ref = compute_errors(state, path)

        # Controller output
        v_left, v_right = controller_fn(lat_err, head_err)

        # Log all signals
        history['time'].append(t)
        history['x'].append(state.x)
        history['y'].append(state.y)
        history['theta'].append(state.theta)
        history['lateral_error'].append(lat_err)
        history['heading_error'].append(head_err)
        history['x_ref'].append(x_ref)
        history['y_ref'].append(y_ref)
        history['v_left'].append(v_left)
        history['v_right'].append(v_right)

        # Advance state
        state = kinematics_step(state, v_left, v_right, noise_std)

        # Apply disturbance every 5 seconds
        if disturbance_mag > 0 and step % int(5.0 / dt) == 0 and step > 0:
            state = apply_disturbance(state, disturbance_mag)

    # Convert to numpy arrays
    for key in history:
        history[key] = np.array(history[key])

    return history

#for kpis and experiements
def compute_kpis(history: dict, settling_threshold=0.05) -> dict:
    """
    Computes:
    - overshoot        : negative excursion past zero / initial error (%)
                         Robust to near-zero spawns on curved paths.
    - settling_time    : first time |lateral_error| stays < threshold
    - steady_state_err : mean |lateral_error| in final 10% of run
    """
    lat = history['lateral_error']
    t   = history['time']

    # Use peak of first 10 samples as reference (avoids division by near-zero)
    initial_error = max(np.max(np.abs(lat[:10])), 0.01)

    # Overshoot = how much error crosses past zero
    if lat[0] >= 0:
        overshoot = max(0.0, -np.min(lat)) / initial_error * 100
    else:
        overshoot = max(0.0, np.max(lat)) / initial_error * 100

    # Settling time: first index where |error| stays below threshold
    settling_time = t[-1]
    for i in range(len(lat)):
        if np.all(np.abs(lat[i:]) < settling_threshold):
            settling_time = t[i]
            break

    # Steady-state error: mean of last 10%
    tail = int(len(lat) * 0.9)
    steady_state_err = np.mean(np.abs(lat[tail:]))

    return {
        'overshoot_pct':      round(float(overshoot), 3),
        'settling_time_s':    round(float(settling_time), 3),
        'steady_state_err_m': round(float(steady_state_err), 5),
    }

