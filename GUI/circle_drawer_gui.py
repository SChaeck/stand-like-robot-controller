"""Circle Drawer GUI

A minimal GUI that lets the user:
1. Specify a circle (center, radius, plane, number of points).
2. Visualise the circle together with the robot.
3. Animate the robot drawing the circle.

The GUI NEVER calls inverse kinematics or motor controllers directly.
It interacts exclusively with RobotSimulator which, in turn, talks to
StandLikeRobot.
"""

from __future__ import annotations

import time
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider, TextBox

from stand_like_robot import StandLikeRobot, RobotSimulator

# ----------------------------------------------------------------------------
# Helper
# ----------------------------------------------------------------------------

PLANE_OPTIONS = {"xy", "xz", "yz"}


def compute_circle_points(center: Tuple[float, float, float], radius: float, num: int, plane: str) -> np.ndarray:
    """Return an (N,3) array of circle points on the chosen *plane* (cm)."""
    if plane not in PLANE_OPTIONS:
        raise ValueError(f"Unsupported plane '{plane}'. Choose from {PLANE_OPTIONS}.")

    cx, cy, cz = center
    angles = np.linspace(0.0, 2 * np.pi, num + 1, endpoint=True)
    pts: List[Tuple[float, float, float]] = []

    for a in angles:
        if plane == "xy":
            pts.append((cx + radius * np.cos(a), cy + radius * np.sin(a), cz))
        elif plane == "xz":
            pts.append((cx + radius * np.cos(a), cy, cz + radius * np.sin(a)))
        else:  # yz
            pts.append((cx, cy + radius * np.cos(a), cz + radius * np.sin(a)))

    return np.asarray(pts)


# ----------------------------------------------------------------------------
# GUI Class
# ----------------------------------------------------------------------------


class CircleDrawerGUI:
    """Simple circle-drawing GUI.

    The class purposefully contains **no kinematics code**. All robot motion is
    delegated to `RobotSimulator`.
    """

    def __init__(self, simulation_mode: bool = True, dual_mode: bool = False):
        # 1) Create StandLikeRobot instance
        self.robot = StandLikeRobot(simulation_mode=simulation_mode, dual_mode=dual_mode)
        # 2) Create RobotSimulator façade
        self.sim = RobotSimulator(self.robot)

        # Circle parameters
        self.circle_points: np.ndarray | None = None
        self.cur_idx: int = 0
        self.is_animating: bool = False
        self.anim: FuncAnimation | None = None
        self.ms_per_step: int = 100  # default 0.3s between points

        # Build the figure
        self._init_gui()

    # ------------------------------------------------------------------ UI --
    def _init_gui(self):
        self.fig = plt.figure(figsize=(14, 8))
        self.ax_3d = self.fig.add_subplot(121, projection="3d")

        # ----------------------- Input widgets ------------------------
        panel_left = 0.70
        width = 0.25
        height = 0.04
        v_gap = 0.05
        y = 0.90

        # Center X,Y,Z
        self.fig.text(panel_left, y, "Center (cm)", weight="bold")
        y -= v_gap
        self.box_x = TextBox(self._ax(panel_left, y, width, height), "X", initial="25")
        y -= v_gap
        self.box_y = TextBox(self._ax(panel_left, y, width, height), "Y", initial="0")
        y -= v_gap
        self.box_z = TextBox(self._ax(panel_left, y, width, height), "Z", initial="20")

        # Radius
        y -= v_gap
        self.fig.text(panel_left, y, "Radius (cm)", weight="bold")
        y -= v_gap
        self.box_radius = TextBox(self._ax(panel_left, y, width, height), "r", initial="5")

        # Number of points
        y -= v_gap
        self.fig.text(panel_left, y, "Points", weight="bold")
        y -= v_gap
        self.box_pts = TextBox(self._ax(panel_left, y, width, height), "n", initial="100")

        # Time per step
        y -= v_gap
        self.fig.text(panel_left, y, "Time/step (ms)", weight="bold")
        y -= v_gap
        self.box_time_step = TextBox(self._ax(panel_left, y, width, height), "ms", initial=str(self.ms_per_step))

        # Plane slider (0=xy,1=xz,2=yz)
        y -= v_gap
        self.fig.text(panel_left, y, "Plane", weight="bold")
        y -= v_gap
        self.slider_plane = Slider(self._ax(panel_left, y, width, height / 3), "", 0, 2, valinit=0, valstep=1)
        self.slider_plane.valtext.set_visible(False)  # hide current value label
        self.slider_plane.on_changed(lambda v: None)
        plane_labels = {0: "xy", 1: "xz", 2: "yz"}

        # Buttons
        y -= v_gap * 2
        self.btn_generate = Button(self._ax(panel_left, y, width, height), "Generate")
        y -= v_gap
        self.btn_start = Button(self._ax(panel_left, y, width, height), "Start Draw")
        y -= v_gap
        self.btn_reset = Button(self._ax(panel_left, y, width, height), "Reset")

        # Bind callbacks
        self.btn_generate.on_clicked(self._on_generate)
        self.btn_start.on_clicked(self._on_start)
        self.btn_reset.on_clicked(self._on_reset)

        # Status text
        y -= v_gap * 2
        self.txt_status = self.fig.text(panel_left, y, "Ready", bbox=dict(facecolor="lightyellow", alpha=0.6))

        # Initial robot render
        self._update_visual()

        print("🚀 CircleDrawerGUI ready. Fill parameters and click 'Generate'.")

    def _ax(self, x, y, w, h):
        return self.fig.add_axes([x, y, w, h])

    # ---------------------------------------------------------------- Events --
    def _on_generate(self, _event):
        try:
            cx = float(self.box_x.text)
            cy = float(self.box_y.text)
            cz = float(self.box_z.text)
            r = float(self.box_radius.text)
            n = int(self.box_pts.text)
            n = max(3, min(200, n))
            plane_idx = int(self.slider_plane.val)
            plane = ("xy", "xz", "yz")[plane_idx]

            self.circle_points = compute_circle_points((cx, cy, cz), r, n, plane)
            self.cur_idx = 0
            self.is_animating = False
            self.txt_status.set_text(f"Circle generated: {n} pts on {plane.upper()} plane.")

            # Visualise
            self._update_visual(show_circle=True)
        except ValueError as e:
            self.txt_status.set_text(f"Input error: {e}")
            print(f"❌ {e}")

    def _on_start(self, _event):
        if self.circle_points is None:
            self.txt_status.set_text("Please generate a circle first.")
            return
        if self.is_animating:
            return

        # Read time per step from GUI
        try:
            time_step = int(self.box_time_step.text)
            if time_step <= 0:
                raise ValueError("Time must be positive.")
            self.ms_per_step = time_step
        except ValueError as e:
            self.txt_status.set_text(f"Invalid time: {e}")
            return

        self.is_animating = True
        self.cur_idx = 0
        interval_ms = self.ms_per_step
        self.anim = FuncAnimation(self.fig, self._animate, interval=interval_ms, blit=False)
        self.anim._start()
        self.txt_status.set_text("Drawing…")

    def _on_reset(self, _event):
        self.circle_points = None
        self.cur_idx = 0
        self.is_animating = False
        if self.anim:
            self.anim.event_source.stop()
            self.anim = None
        # Move robot home via simulator
        self.sim.move_to_home_position()
        self._update_visual()
        self.txt_status.set_text("Reset done.")

    # ---------------------------------------------------------------- update --
    def _animate(self, _frame):
        if not self.is_animating or self.circle_points is None:
            return
        if self.cur_idx >= len(self.circle_points):
            self.is_animating = False
            self.txt_status.set_text("Complete ✔")
            return

        target = self.circle_points[self.cur_idx]
        if self.cur_idx == 0:
            self.sim.move_end_effector([target[0], target[1], target[2] + 3], time_to_go=1.5)
            self.sim.move_end_effector(target, time_to_go=1.5)
        else:
            self.sim.move_end_effector(target, time_to_go=self.ms_per_step / 1000.0)
        self._update_visual(show_circle=True, highlight_idx=self.cur_idx)
        self.cur_idx += 1
        self.fig.canvas.draw_idle()

    # ---------------------------------------------------------------- helper --
    def _update_visual(self, show_circle: bool = False, highlight_idx: int | None = None):
        circle_pts = self.circle_points if show_circle else None
        self.sim.visualize_robot(self.ax_3d, show_circle_points=circle_pts, current_point_idx=highlight_idx)

    # ---------------------------------------------------------------- run --
    def run(self):
        plt.show()

    # Legacy alias (for run_circle_drawer.py that still calls run_demo)
    def run_demo(self):
        """Alias to run() kept for backward compatibility."""
        self.run()


# ----------------------------------------------------------------------------
# CLI Entry
# ----------------------------------------------------------------------------

if __name__ == "__main__":
    gui = CircleDrawerGUI(simulation_mode=True)
    gui.run() 