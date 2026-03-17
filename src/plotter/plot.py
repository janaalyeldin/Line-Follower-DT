import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
"""
visualizer.py — Client 3: Visualizer / Logger
Line-Following Robot

Consumes: x_pos, y_pos, theta, lateral_error, heading_error,
          x_ref, y_ref, v_left, v_right  (from Clients 1 & 2)
"""

import numpy as np
import csv
import os
import sys
from collections import deque
from typing import List, Dict, Deque


try:
    from PyQt6 import QtWidgets, QtCore
except ModuleNotFoundError:
    from PySide6 import QtWidgets, QtCore          # type: ignore

import pyqtgraph as pg

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')


class RealTimePlotter:
    """
    Live 2×2 PyQtGraph dashboard for the line-following robot.

    Layout
    ------
    [Robot Trajectory (XY)]   |  [Lateral Error vs Time]
    [Wheel Velocities vs Time] |  [Heading Error vs Time]

    Parameters
    ----------
    buffer_size : int
        Maximum number of samples kept in the sliding window.
    update_frequency : int
        Redraw the dashboard every N calls to `update_data`.
    path : optional
        Path object with a `.sample_points()` method that returns (px, py).
        If provided, the reference path is drawn on the trajectory plot.
    """

    def __init__(self, path=None, buffer_size: int = 1000, update_frequency: int = 5):
        self.buffer_size      = buffer_size
        self.update_frequency = update_frequency
        self._update_counter  = 0
        self._path            = path

        #fifo buffers
        self.t_buf        : Deque[float] = deque(maxlen=buffer_size)
        self.x_buf        : Deque[float] = deque(maxlen=buffer_size)
        self.y_buf        : Deque[float] = deque(maxlen=buffer_size)
        self.lat_err_buf  : Deque[float] = deque(maxlen=buffer_size)
        self.head_err_buf : Deque[float] = deque(maxlen=buffer_size)
        self.v_left_buf   : Deque[float] = deque(maxlen=buffer_size)
        self.v_right_buf  : Deque[float] = deque(maxlen=buffer_size)

        #qt app
        self._app = (QtWidgets.QApplication.instance()
                     or QtWidgets.QApplication(sys.argv))
        pg.setConfigOptions(antialias=True)

    
        self.win = pg.GraphicsLayoutWidget(title="Line-Following Robot — Live Dashboard")
        self.win.resize(1280, 820)

        # 1) Robot Trajectory (XY)
        self.traj_plot = self.win.addPlot(title="Robot Trajectory")
        self.traj_plot.setLabel('bottom', 'X', units='m')
        self.traj_plot.setLabel('left',   'Y', units='m')
        self.traj_plot.setAspectLocked(True)
        self.traj_plot.addLegend(offset=(10, 10))

        # Reference path (static, drawn once)
        self._path_curve = self.traj_plot.plot(
            pen=pg.mkPen('r', style=QtCore.Qt.PenStyle.DashLine, width=2),
            name="Reference"
        )
        if path is not None:
            try:
                px, py = path.sample_points()
                self._path_curve.setData(px, py)
            except Exception:
                pass

        # Robot trail
        self.curve_robot = self.traj_plot.plot(
            pen=pg.mkPen('#2196F3', width=2), name="Robot"
        )
        # Robot heading arrow (last position marker)
        self.curve_robot_head = self.traj_plot.plot(
            pen=None, symbol='o',
            symbolBrush=pg.mkBrush('#2196F3'), symbolSize=10
        )

        # 2) Lateral Error vs Time
        self.win.nextColumn()
        self.lat_plot = self.win.addPlot(title="Lateral Error vs Time")
        self.lat_plot.setLabel('bottom', 'Time', units='s')
        self.lat_plot.setLabel('left',   'Lateral Error', units='m')
        # Zero reference line
        self.lat_zero = pg.InfiniteLine(
            pos=0, angle=0,
            pen=pg.mkPen('r', style=QtCore.Qt.PenStyle.DashLine, width=1)
        )
        self.lat_plot.addItem(self.lat_zero)
        self.curve_lat_err = self.lat_plot.plot(
            pen=pg.mkPen('#4CAF50', width=2)
        )
# add second row
        self.win.nextRow()

        # 3) Wheel Velocities vs Time
        self.vel_plot = self.win.addPlot(title="Wheel Velocities vs Time")
        self.vel_plot.setLabel('bottom', 'Time', units='s')
        self.vel_plot.setLabel('left',   'Speed', units='m/s')
        self.vel_plot.addLegend(offset=(10, 10))
        # V_BASE reference line
        self.vel_base = pg.InfiniteLine(
            pos=0.5, angle=0,
            pen=pg.mkPen('gray', style=QtCore.Qt.PenStyle.DotLine, width=1)
        )
        self.vel_plot.addItem(self.vel_base)
        self.curve_v_left  = self.vel_plot.plot(
            pen=pg.mkPen('#2196F3', width=2), name="v_left"
        )
        self.curve_v_right = self.vel_plot.plot(
            pen=pg.mkPen('orange', width=2), name="v_right"
        )

        # 4) Heading Error vs Time
        self.win.nextColumn()
        self.head_plot = self.win.addPlot(title="Heading Error vs Time")
        self.head_plot.setLabel('bottom', 'Time', units='s')
        self.head_plot.setLabel('left',   'Heading Error', units='rad')
        self.head_zero = pg.InfiniteLine(
            pos=0, angle=0,
            pen=pg.mkPen('r', style=QtCore.Qt.PenStyle.DashLine, width=1)
        )
        self.head_plot.addItem(self.head_zero)
        self.curve_head_err = self.head_plot.plot(
            pen=pg.mkPen('#E91E63', width=2)
        )

        # show non-blocking so the simulation keeps running
        self.win.show()
        self._app.processEvents()
        print("  [Plotter] Live dashboard started.")

   #public api
    def update_data(self, t: float, x: float, y: float, theta: float,
                    lat_err: float, head_err: float,
                    x_ref: float = 0.0, y_ref: float = 0.0,
                    v_left: float = 0.5, v_right: float = 0.5) -> None:
        """Push one sample; redraws every `update_frequency` calls."""
        self.t_buf.append(t)
        self.x_buf.append(x)
        self.y_buf.append(y)
        self.lat_err_buf.append(lat_err)
        self.head_err_buf.append(head_err)
        self.v_left_buf.append(v_left)
        self.v_right_buf.append(v_right)

        self._update_counter += 1
        if self._update_counter >= self.update_frequency:
            self._update_counter = 0
            self._redraw()

    def close(self) -> None:
        """Cleanly close the dashboard window."""
        self.win.close()

    
    @staticmethod
    def _to_arr(dq: Deque[float]) -> np.ndarray:
        return np.fromiter(dq, dtype=float, count=len(dq))

    def _redraw(self) -> None:
        t_arr    = self._to_arr(self.t_buf)
        x_arr    = self._to_arr(self.x_buf)
        y_arr    = self._to_arr(self.y_buf)
        lat_arr  = self._to_arr(self.lat_err_buf)
        head_arr = self._to_arr(self.head_err_buf)
        vl_arr   = self._to_arr(self.v_left_buf)
        vr_arr   = self._to_arr(self.v_right_buf)

        # Trajectory
        self.curve_robot.setData(x_arr, y_arr)
        if x_arr.size:
            self.curve_robot_head.setData([x_arr[-1]], [y_arr[-1]])

        # Time-series curves
        self.curve_lat_err.setData(t_arr, lat_arr)
        self.curve_head_err.setData(t_arr, head_arr)
        self.curve_v_left.setData(t_arr, vl_arr)
        self.curve_v_right.setData(t_arr, vr_arr)

        # Let Qt paint — non-blocking
        self._app.processEvents()

   
    def finalize(self, title: str = "", save_path: str = None):
        """
        Optionally save a static PNG at the end of simulation.
        The live window stays open until close() is called.
        """
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            from matplotlib.gridspec import GridSpec

            fig = plt.figure(figsize=(13, 7))
            fig.suptitle(title or "Line-Following Robot — Simulation Results", fontsize=14)
            gs = GridSpec(2, 2, figure=fig)

            ax_traj = fig.add_subplot(gs[:, 0])
            ax_lat  = fig.add_subplot(gs[0, 1])
            ax_vel  = fig.add_subplot(gs[1, 1])

            if self._path is not None:
                try:
                    px, py = self._path.sample_points()
                    ax_traj.plot(px, py, 'r--', linewidth=2, label='Path')
                except Exception:
                    pass
            ax_traj.plot(list(self.x_buf), list(self.y_buf),
                         'b-', linewidth=1.5, label='Robot')
            x_list = list(self.x_buf)
            y_list = list(self.y_buf)
            if x_list:
                ax_traj.plot(x_list[0],  y_list[0],  'go', markersize=10, label='Start')
                ax_traj.plot(x_list[-1], y_list[-1], 'rs', markersize=10, label='End')
            ax_traj.set_title("Robot Trajectory vs Path")
            ax_traj.set_xlabel("X (m)"); ax_traj.set_ylabel("Y (m)")
            ax_traj.legend(); ax_traj.grid(True); ax_traj.axis('equal')

            t_list  = list(self.t_buf)
            ax_lat.plot(t_list, list(self.lat_err_buf), 'g-', linewidth=1.5)
            ax_lat.axhline(0, color='r', linestyle='--', linewidth=1)
            ax_lat.set_title("Lateral Error"); ax_lat.set_xlabel("Time (s)")
            ax_lat.set_ylabel("Error (m)"); ax_lat.grid(True)

            ax_vel.plot(t_list, list(self.v_left_buf),  'b-',     linewidth=1.2, label='v_left')
            ax_vel.plot(t_list, list(self.v_right_buf), 'orange', linewidth=1.2, label='v_right')
            ax_vel.axhline(0.5, color='gray', linestyle=':', linewidth=1, label='V_BASE')
            ax_vel.set_title("Wheel Velocities"); ax_vel.set_xlabel("Time (s)")
            ax_vel.set_ylabel("Speed (m/s)"); ax_vel.legend(fontsize=8); ax_vel.grid(True)

            plt.tight_layout()
            out = save_path or "results/vsi_live_run.png"
            os.makedirs(os.path.dirname(out) if os.path.dirname(out) else ".", exist_ok=True)
            fig.savefig(out, dpi=150, bbox_inches='tight')
            print(f"  [Plotter] Static PNG saved to: {out}")
            plt.close(fig)
        except Exception as e:
            print(f"  [Plotter] Could not save static plot: {e}")


#kpi logger
from datetime import datetime

class KPILogger:
    FIELDNAMES = [
        'experiment', 'preset', 'spawn', 'path_type',
        'noise_std', 'disturbance_mag',
        'overshoot_pct', 'settling_time_s', 'steady_state_err_m', 'notes'
    ]

    def __init__(self, log_dir: str = "."):
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(log_dir, f"experiment_kpis_{ts}.csv")
        self.rows: List[Dict] = []
        with open(self.csv_path, 'w', newline='') as f:
            csv.DictWriter(f, fieldnames=self.FIELDNAMES).writeheader()
        print(f"  [Logger] KPI log: {self.csv_path}")

    def log(self, experiment, preset, spawn, path_type, kpis,
            noise_std=0.0, disturbance_mag=0.0, notes=""):
        row = {
            'experiment': experiment, 'preset': preset,
            'spawn': spawn, 'path_type': path_type,
            'noise_std': noise_std, 'disturbance_mag': disturbance_mag,
            'overshoot_pct':      kpis.get('overshoot_pct', ''),
            'settling_time_s':    kpis.get('settling_time_s', ''),
            'steady_state_err_m': kpis.get('steady_state_err_m', ''),
            'notes': notes,
        }
        self.rows.append(row)
        with open(self.csv_path, 'a', newline='') as f:
            csv.DictWriter(f, fieldnames=self.FIELDNAMES).writerow(row)

    def print_table(self, experiment: str):
        rows = [r for r in self.rows if r['experiment'] == experiment]
        if not rows:
            return
        print(f"\n{'='*70}")
        print(f"  {experiment} Results")
        print(f"{'='*70}")
        print(f"{'Preset':<12} {'Spawn':<6} {'Path':<10} "
              f"{'Overshoot%':<12} {'Settling(s)':<13} {'SSE(m)':<10} Notes")
        print("-" * 70)
        for r in rows:
            print(f"{r['preset']:<12} {str(r['spawn']):<6} {r['path_type']:<10} "
                  f"{str(r['overshoot_pct']):<12} {str(r['settling_time_s']):<13} "
                  f"{str(r['steady_state_err_m']):<10} {r['notes']}")
        print(f"{'='*70}\n")


#for plotting experiments
class ExperimentReporter:
    def __init__(self, output_dir: str = "results"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def _get_plt(self):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        return plt

    def plot_e1_gain_sweep(self, results_by_preset: dict, path):
        plt = self._get_plt()
        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        fig.suptitle("E1 — PID Gain Sweep: Lateral Error Over Time", fontsize=14)
        axes = axes.flatten()
        colors = ['#2196F3','#4CAF50','#FF9800','#E91E63','#9C27B0','#00BCD4']
        for idx, (preset, history) in enumerate(results_by_preset.items()):
            ax = axes[idx]
            ax.plot(history['time'], history['lateral_error'], color=colors[idx], linewidth=1.5)
            ax.axhline(0, color='red', linestyle='--', linewidth=1, alpha=0.7)
            ax.fill_between(history['time'], history['lateral_error'], 0, alpha=0.1, color=colors[idx])
            tail = int(len(history['lateral_error']) * 0.9)
            sse  = np.mean(np.abs(history['lateral_error'][tail:]))
            ax.set_title(f"{preset}  |  SSE={sse:.4f}m", fontsize=10)
            ax.set_xlabel("Time (s)"); ax.set_ylabel("Lateral Error (m)"); ax.grid(True, alpha=0.4)
        plt.tight_layout()
        out = os.path.join(self.output_dir, "E1_gain_sweep.png")
        plt.savefig(out, dpi=150, bbox_inches='tight')
        print(f"  [Reporter] Saved: {out}")
        plt.close(fig)

    def plot_e2_curved(self, straight_history, curved_history,
                       straight_path, curved_path, preset_name="E1_set3"):
        plt = self._get_plt()
        fig, axes = plt.subplots(2, 2, figsize=(14, 9))
        fig.suptitle(f"E2 — Curved Path Robustness ({preset_name})", fontsize=13)
        px, py = straight_path.sample_points()
        axes[0,0].plot(px, py, 'r--', linewidth=2, label='Path')
        axes[0,0].plot(straight_history['x'], straight_history['y'], 'b-', linewidth=1.5, label='Robot')
        axes[0,0].set_title("Straight — Trajectory"); axes[0,0].legend(); axes[0,0].axis('equal'); axes[0,0].grid(True)
        cx, cy = curved_path.sample_points()
        axes[0,1].plot(cx, cy, 'r--', linewidth=2, label='Path')
        axes[0,1].plot(curved_history['x'], curved_history['y'], 'b-', linewidth=1.5, label='Robot')
        axes[0,1].set_title("Curved — Trajectory"); axes[0,1].legend(); axes[0,1].axis('equal'); axes[0,1].grid(True)
        axes[1,0].plot(straight_history['time'], straight_history['lateral_error'], 'g-')
        axes[1,0].axhline(0, color='r', linestyle='--'); axes[1,0].set_title("Straight — Lateral Error"); axes[1,0].grid(True)
        axes[1,1].plot(curved_history['time'], curved_history['lateral_error'], 'orange')
        axes[1,1].axhline(0, color='r', linestyle='--'); axes[1,1].set_title("Curved — Lateral Error"); axes[1,1].grid(True)
        plt.tight_layout()
        out = os.path.join(self.output_dir, "E2_curved_path.png")
        plt.savefig(out, dpi=150, bbox_inches='tight'); print(f"  [Reporter] Saved: {out}"); plt.close(fig)

    def plot_e3_noise(self, histories_by_noise: dict):
        plt = self._get_plt()
        fig, axes = plt.subplots(2, 2, figsize=(13, 8))
        fig.suptitle("E3 — Noise & Disturbance Rejection", fontsize=13)
        axes = axes.flatten()
        colors = ['#4CAF50','#2196F3','#FF9800','#F44336']
        for idx, (label, history) in enumerate(histories_by_noise.items()):
            ax = axes[idx]
            ax.plot(history['time'], history['lateral_error'], color=colors[idx], linewidth=1.2)
            ax.axhline(0, color='black', linestyle='--', linewidth=0.8)
            tail = int(len(history['lateral_error']) * 0.9)
            sse  = np.mean(np.abs(history['lateral_error'][tail:]))
            ax.set_title(f"{label}  |  SSE={sse:.4f}m", fontsize=10)
            ax.set_xlabel("Time (s)"); ax.set_ylabel("Lateral Error (m)"); ax.grid(True, alpha=0.4)
        plt.tight_layout()
        out = os.path.join(self.output_dir, "E3_noise_rejection.png")
        plt.savefig(out, dpi=150, bbox_inches='tight'); print(f"  [Reporter] Saved: {out}"); plt.close(fig)

    def plot_e4_pd_vs_pid(self, pid_history: dict, pd_history: dict):
        plt = self._get_plt()
        fig, axes = plt.subplots(1, 2, figsize=(13, 5))
        fig.suptitle("E4 — PD vs PID Ablation (Curved Path + Noise)", fontsize=13)
        for ax, history, label, color in zip(
            axes, [pid_history, pd_history],
            ['PID (E1_set3)', 'PD (Ki=0)'], ['#2196F3','#FF5722']
        ):
            ax.plot(history['time'], history['lateral_error'], color=color, linewidth=1.5, label=label)
            ax.axhline(0, color='black', linestyle='--', linewidth=0.8)
            tail = int(len(history['lateral_error']) * 0.9)
            sse  = np.mean(np.abs(history['lateral_error'][tail:]))
            peak = max(np.abs(history['lateral_error']))
            ax.set_title(f"{label}\nSSE={sse:.4f}m  Peak={peak:.4f}m", fontsize=10)
            ax.set_xlabel("Time (s)"); ax.set_ylabel("Lateral Error (m)")
            ax.legend(); ax.grid(True, alpha=0.4)
        plt.tight_layout()
        out = os.path.join(self.output_dir, "E4_pd_vs_pid.png")
        plt.savefig(out, dpi=150, bbox_inches='tight'); print(f"  [Reporter] Saved: {out}"); plt.close(fig)

