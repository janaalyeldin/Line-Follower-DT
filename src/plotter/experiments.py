

import random
import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, src_dir)
from lineRobot.robot_kinematics import (RobotState, StraightPath, CurvedPath,
                                         run_simulation, compute_kpis, random_spawn)
from pidController.robotPid import RobotPIDController, GAIN_PRESETS
from plot import KPILogger, ExperimentReporter
np.random.seed(42)
random.seed(42)

# ── Setup ──────────────────────────────────────────────────────────────
logger   = KPILogger(log_dir="results")
reporter = ExperimentReporter(output_dir="results")
straight = StraightPath(x_end=10.0, y_end=0.0)
curved   = CurvedPath()

# ── E1: PID Gain Sweep ─────────────────────────────────────────────────
print("\n>>> E1: PID Gain Sweep")
e1_results = {}
for preset in GAIN_PRESETS:
    ctrl = RobotPIDController.from_preset(preset)
    hist = run_simulation(RobotState(0.0, 0.8, 0.3), straight, ctrl.step, total_time=20.0)
    kpis = compute_kpis(hist)
    e1_results[preset] = hist
    logger.log("E1", preset, 1, "straight", kpis)
    print(f"  {preset}: {kpis}")
reporter.plot_e1_gain_sweep(e1_results, straight)
logger.print_table("E1")

# ── E2: Curved Path ────────────────────────────────────────────────────
print("\n>>> E2: Curved Path Robustness")
ctrl = RobotPIDController.from_preset("E1_set3")
straight_hist = run_simulation(RobotState(0.0, 0.8, 0.3), straight, ctrl.step, total_time=20.0)
ctrl.reset()
curved_hist = run_simulation(RobotState(0.0, 0.8, 0.3), curved, ctrl.step, total_time=20.0)
logger.log("E2", "E1_set3", 1, "straight", compute_kpis(straight_hist), notes="straight baseline")
logger.log("E2", "E1_set3", 1, "curved",   compute_kpis(curved_hist),   notes="S-curve path")
reporter.plot_e2_curved(straight_hist, curved_hist, straight, curved)
logger.print_table("E2")

# ── E3: Noise & Disturbance Rejection ─────────────────────────────────
print("\n>>> E3: Noise & Disturbance Rejection")
noise_configs = {
    "No Noise":     (0.00, 0.00),
    "Low Noise":    (0.05, 0.02),
    "Medium Noise": (0.15, 0.05),
    "High Noise":   (0.30, 0.10),
}
e3_histories = {}
for label, (noise, dist) in noise_configs.items():
    ctrl = RobotPIDController.from_preset("E1_set3")
    hist = run_simulation(RobotState(0.0, 0.8, 0.3), straight, ctrl.step,
                          total_time=20.0, noise_std=noise, disturbance_mag=dist)
    e3_histories[label] = hist
    logger.log("E3", "E1_set3", 1, "straight", compute_kpis(hist),
               noise_std=noise, disturbance_mag=dist, notes=label)
reporter.plot_e3_noise(e3_histories)
logger.print_table("E3")

# ── E4: PD vs PID Ablation ─────────────────────────────────────────────
print("\n>>> E4: PD vs PID Ablation")
pid_hist = run_simulation(RobotState(0.0, 0.8, 0.3), curved,
                          RobotPIDController.from_preset("E1_set3").step,
                          total_time=20.0, noise_std=0.03)
pd_hist  = run_simulation(RobotState(0.0, 0.8, 0.3), curved,
                          RobotPIDController.from_preset("E4_PD").step,
                          total_time=20.0, noise_std=0.03)
logger.log("E4", "E1_set3 (PID)", 1, "curved", compute_kpis(pid_hist),
           noise_std=0.03, notes="with integral")
logger.log("E4", "E4_PD",         1, "curved", compute_kpis(pd_hist),
           noise_std=0.03, notes="no integral")
reporter.plot_e4_pd_vs_pid(pid_hist, pd_hist)
logger.print_table("E4")

print(f"\n✓ All results saved to: results/")