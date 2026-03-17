import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
"""
robotPid.py — Client 2: PID Controller
Line-Following Robot

Consumes: lateral_error, heading_error  (from Client 1)
Publishes: v_left, v_right              (wheel velocity commands)

PID Control Law:
  u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt

Two cascaded PID loops:
  - Outer loop: lateral error  → heading correction
  - Inner loop: heading error + correction → differential wheel speed
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List


#robot vel limits
V_BASE = 0.5      # m/s — base forward speed
V_MAX  = 1.0      # m/s — max wheel speed
V_MIN  = -0.2     # m/s — slight reverse allowed


#gain presents for experiement 1 default e1_set3 for rest of simulation
GAIN_PRESETS = {
    "E1_set1": (0.2,  0.02, 0.04,  0.4,  0.01, 0.06),  # Very low  — sluggish, large SSE
    "E1_set2": (0.4,  0.05, 0.06,  0.6,  0.02, 0.09),  # Low-medium — slow settling
    "E1_set3": (0.6,  0.08, 0.08,  0.8,  0.03, 0.11),  # Medium    — good balance
    "E1_set4": (0.9,  0.15, 0.12,  1.1,  0.06, 0.15),  # High      — overshoot visible
    "E1_set5": (1.2,  0.25, 0.18,  1.4,  0.10, 0.20),  # Aggressive — oscillates
    # E4 ablation: PD only (Ki = 0)
    "E4_PD":   (0.6,  0.00, 0.08,  0.8,  0.00, 0.11),
}

DEFAULT_GAINS = GAIN_PRESETS["E1_set3"]


#pid state
@dataclass
class PIDState:
    integral:     float = 0.0
    prev_error:   float = 0.0
    integral_max: float = 2.0
    integral_min: float = -2.0



# SINGLE PID STEP
# u = Kp*e + Ki*integral(e)*dt + Kd*de/dt
# Anti-windup: clamp integral to avoid saturation
def pid_step(error: float,
             dt: float,
             Kp: float,
             Ki: float,
             Kd: float,
             state: PIDState) -> Tuple[float, PIDState]:
    P = Kp * error

    state.integral += error * dt
    state.integral  = np.clip(state.integral,
                               state.integral_min,
                               state.integral_max)
    I = Ki * state.integral

    derivative = (error - state.prev_error) / dt if dt > 1e-9 else 0.0
    D = Kd * derivative

    state.prev_error = error
    return P + I + D, state

#actual controller
class RobotPIDController:
    """
    Cascaded PID for differential-drive line following.

    Loop 1 — lateral PID:
        error  = lateral_error (m)
        output = heading_correction (rad)

    Loop 2 — heading PID:
        error  = heading_error + heading_correction
        output = u_steer

    Wheel commands:
        v_left  = V_BASE + u_steer   (left faster = clockwise = turn right)
        v_right = V_BASE - u_steer
    """

    def __init__(self,
                 Kp_lat=DEFAULT_GAINS[0],
                 Ki_lat=DEFAULT_GAINS[1],
                 Kd_lat=DEFAULT_GAINS[2],
                 Kp_head=DEFAULT_GAINS[3],
                 Ki_head=DEFAULT_GAINS[4],
                 Kd_head=DEFAULT_GAINS[5],
                 dt=0.01):

        self.Kp_lat  = Kp_lat
        self.Ki_lat  = Ki_lat
        self.Kd_lat  = Kd_lat
        self.Kp_head = Kp_head
        self.Ki_head = Ki_head
        self.Kd_head = Kd_head
        self.dt      = dt
        self.lat_state  = PIDState()
        self.head_state = PIDState()

    def reset(self):
        """Reset integrators between experiment runs."""
        self.lat_state  = PIDState()
        self.head_state = PIDState()

    def step(self,
             lateral_error: float,
             heading_error: float) -> Tuple[float, float]:
        """
        One control step.
        Returns (v_left, v_right) wheel velocity commands in m/s.
        """
        # Loop 1: lateral error -> heading correction
        heading_correction, self.lat_state = pid_step(
            lateral_error, self.dt,
            self.Kp_lat, self.Ki_lat, self.Kd_lat,
            self.lat_state
        )

        # Loop 2: combined heading -> steering output
        # (+) because both errors require same turn direction
        combined_heading = heading_error + heading_correction
        u_steer, self.head_state = pid_step(
            combined_heading, self.dt,
            self.Kp_head, self.Ki_head, self.Kd_head,
            self.head_state
        )

        # Positive u_steer -> left faster -> clockwise -> corrects positive lateral error
        v_left  = np.clip(V_BASE + u_steer, V_MIN, V_MAX)
        v_right = np.clip(V_BASE - u_steer, V_MIN, V_MAX)

        return float(v_left), float(v_right)

    @classmethod
    def from_preset(cls, preset_name: str, dt=0.01):
        if preset_name not in GAIN_PRESETS:
            raise ValueError(f"Unknown preset '{preset_name}'. "
                             f"Options: {list(GAIN_PRESETS.keys())}")
        return cls(*GAIN_PRESETS[preset_name], dt=dt)

    def __repr__(self):
        return (f"RobotPIDController("
                f"lat=[{self.Kp_lat},{self.Ki_lat},{self.Kd_lat}] "
                f"head=[{self.Kp_head},{self.Ki_head},{self.Kd_head}])")



def run_experiment(preset_name: str,
                   path,
                   n_spawns: int = 5,
                   noise_std: float = 0.0,
                   disturbance_mag: float = 0.0,
                   total_time: float = 20.0) -> List[dict]:
    from lineRobot.robot_kinematics import run_simulation, compute_kpis, random_spawn

    results = []
    controller = RobotPIDController.from_preset(preset_name)

    for i in range(n_spawns):
        controller.reset()
        state = random_spawn(path)
        history = run_simulation(
            initial_state=state,
            path=path,
            controller_fn=controller.step,
            total_time=total_time,
            noise_std=noise_std,
            disturbance_mag=disturbance_mag,
        )
        kpis = compute_kpis(history)
        kpis['spawn'] = i + 1
        kpis['preset'] = preset_name
        results.append({'kpis': kpis, 'history': history})
        print(f"  [{preset_name}] Spawn {i+1}: {kpis}")

    return results


