"""Path Recorder GUI

- Move the robot via XYZ inputs + orientation.
- Record sequences of moves and gripper commands.
- Save / load path files (JSON).
- Play back recorded paths.

This GUI communicates **only** with RobotSimulator and never touches
lower-level components directly.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, TextBox

from stand_like_robot import StandLikeRobot, RobotSimulator

DATA_DIR = Path("GUI/paths")
DATA_DIR.mkdir(exist_ok=True)


class PathRecorderGUI:
    """A simple recorder / player built on top of RobotSimulator."""

    def __init__(self, simulation_mode: bool = True, dual_mode: bool = False):
        self.robot = StandLikeRobot(simulation_mode=simulation_mode, dual_mode=dual_mode)
        self.sim = RobotSimulator(self.robot)

        self.recording: bool = False
        self.steps: List[Dict] = []  # collected steps
        self.play_thread: threading.Thread | None = None
        self.stop_play: threading.Event | None = None
        
        # Gripper state tracking
        self.gripper_open: bool = True  # Track gripper state for visualization

        self._init_gui()
        self._update_visual()
        print("🚀 PathRecorderGUI ready.")

    # ---------------------------------------------------------------- GUI --
    def _init_gui(self):
        self.fig = plt.figure(figsize=(16, 10))
        self.ax_3d = self.fig.add_subplot(121, projection="3d")

        # Controls panel
        x0, y0, w, h, pad = 0.70, 0.95, 0.25, 0.035, 0.04

        # Position inputs
        self.fig.text(x0, y0, "Target XYZ (cm)", weight="bold")
        y0 -= pad
        self.box_x = TextBox(self._ax(x0, y0, w, h), "X", initial="25")
        y0 -= pad
        self.box_y = TextBox(self._ax(x0, y0, w, h), "Y", initial="0")
        y0 -= pad
        self.box_z = TextBox(self._ax(x0, y0, w, h), "Z", initial="20")

        # Orientation
        y0 -= pad
        self.fig.text(x0, y0, "Z-Rotation (deg)", weight="bold")
        y0 -= pad
        self.box_rot = TextBox(self._ax(x0, y0, w, h), "R", initial="90")

        # Time
        y0 -= pad
        self.fig.text(x0, y0, "Time (s)", weight="bold")
        y0 -= pad
        self.box_time = TextBox(self._ax(x0, y0, w, h), "T", initial="2.0")

        # Move & Home
        y0 -= pad * 1.5
        self.btn_move = Button(self._ax(x0, y0, w, h), "Move")
        y0 -= pad
        self.btn_home = Button(self._ax(x0, y0, w, h), "Home")

        # Gripper
        y0 -= pad * 1.5
        self.btn_open = Button(self._ax(x0, y0, w, h), "Open Gripper")
        y0 -= pad
        self.btn_close = Button(self._ax(x0, y0, w, h), "Close Gripper")

        # Recording controls
        y0 -= pad * 1.5
        self.btn_rec = Button(self._ax(x0, y0, w, h), "Start Rec")
        y0 -= pad
        self.btn_stop_rec = Button(self._ax(x0, y0, w, h), "Stop Rec")

        # Save / Load
        y0 -= pad * 1.5
        self.box_path = TextBox(self._ax(x0, y0, w, h), "File", initial="my_path")
        y0 -= pad
        self.btn_save = Button(self._ax(x0, y0, w, h), "Save")
        y0 -= pad
        self.btn_load = Button(self._ax(x0, y0, w, h), "Load")

        # Play
        y0 -= pad * 1.5
        self.btn_play = Button(self._ax(x0, y0, w, h), "Play")
        y0 -= pad
        self.btn_stop_play = Button(self._ax(x0, y0, w, h), "Stop")

        # Status text
        y0 -= pad * 1.5
        self.txt_status = self.fig.text(x0, y0, "Ready", bbox=dict(facecolor="lightyellow", alpha=0.6))

        # Bindings
        self.btn_move.on_clicked(self._on_move)
        self.btn_home.on_clicked(self._on_home)
        self.btn_open.on_clicked(lambda e: self._on_gripper(True))
        self.btn_close.on_clicked(lambda e: self._on_gripper(False))
        self.btn_rec.on_clicked(self._on_start_rec)
        self.btn_stop_rec.on_clicked(self._on_stop_rec)
        self.btn_save.on_clicked(self._on_save)
        self.btn_load.on_clicked(self._on_load)
        self.btn_play.on_clicked(self._on_play)
        self.btn_stop_play.on_clicked(self._on_stop_play)

    def _ax(self, x, y, w, h):
        return self.fig.add_axes([x, y, w, h])

    # ---------------------------------------------------------------- Actions --
    def _on_move(self, _event):
        try:
            pos = [float(self.box_x.text), float(self.box_y.text), float(self.box_z.text)]
            rot = float(self.box_rot.text)
            t = float(self.box_time.text)
            
            # Validate inputs
            if t <= 0:
                raise ValueError("Time must be positive")
            
            self.sim.move_end_effector(pos, z_rotation_deg=rot, time_to_go=t)
            self._update_visual()
            self._status(f"Moved to {pos} with rotation {rot}°")
            
            if self.recording:
                self.steps.append({"type": "move", "pos": pos, "rot": rot, "time": t})
                
        except ValueError as e:
            self._status(f"Input error: {e}")
            print(f"❌ Move error: {e}")

    def _on_home(self, _event):
        try:
            self.sim.move_to_home_position()
            self._update_visual()
            self._status("Moved to home position")
            
            if self.recording:
                self.steps.append({"type": "home"})
                
        except Exception as e:
            self._status(f"Home error: {e}")
            print(f"❌ Home error: {e}")

    def _on_gripper(self, open_: bool):
        try:
            if open_:
                self.sim.open_gripper()
                self.gripper_open = True
                action = "opened"
            else:
                self.sim.close_gripper()
                self.gripper_open = False
                action = "closed"
                
            self._update_visual()
            self._status(f"Gripper {action}")
            
            if self.recording:
                self.steps.append({"type": "gripper", "open": open_})
                
        except Exception as e:
            self._status(f"Gripper error: {e}")
            print(f"❌ Gripper error: {e}")

    def _on_start_rec(self, _event):
        self.recording = True
        self.steps.clear()
        self._status("🔴 Recording started...")

    def _on_stop_rec(self, _event):
        self.recording = False
        self._status(f"⏹️ Recording stopped ({len(self.steps)} steps)")

    def _on_save(self, _event):
        try:
            if not self.steps:
                self._status("No steps to save")
                return
                
            fname = self._filepath()
            with open(fname, "w", encoding="utf-8") as f:
                json.dump(self.steps, f, indent=2)
            self._status(f"💾 Saved {len(self.steps)} steps to {fname.name}")
            
        except Exception as e:
            self._status(f"Save error: {e}")
            print(f"❌ Save error: {e}")

    def _on_load(self, _event):
        try:
            fname = self._filepath()
            if not fname.exists():
                self._status("File not found")
                return
                
            with open(fname, "r", encoding="utf-8") as f:
                self.steps = json.load(f)
            self._status(f"📂 Loaded {len(self.steps)} steps from {fname.name}")
            
        except Exception as e:
            self._status(f"Load error: {e}")
            print(f"❌ Load error: {e}")

    def _on_play(self, _event):
        if not self.steps:
            self._status("No steps to play")
            return
        if self.play_thread and self.play_thread.is_alive():
            self._status("Already playing...")
            return
            
        self.stop_play = threading.Event()
        self.play_thread = threading.Thread(target=self._play_path, daemon=True)
        self.play_thread.start()

    def _on_stop_play(self, _event):
        if self.stop_play:
            self.stop_play.set()
        self._status("⏹️ Playback stopped")

    # ---------------------------------------------------------------- Helpers --
    def _filepath(self):
        name = self.box_path.text.strip() or "my_path"
        return DATA_DIR / f"{name}.json"

    def _play_path(self):
        try:
            self._status("▶️ Playing path...")
            
            for i, step in enumerate(self.steps):
                if self.stop_play and self.stop_play.is_set():
                    break
                    
                self._status(f"▶️ Step {i+1}/{len(self.steps)}: {step['type']}")
                
                if step["type"] == "move":
                    self.sim.move_end_effector(
                        step["pos"], 
                        z_rotation_deg=step.get("rot", 0), 
                        time_to_go=step.get("time", 2),
                        use_cartesian_interpolation=step["use_cartesian_interpolation"] if step.get("is_cartesian_interpolation", False) else False
                    )
                elif step["type"] == "home":
                    self.sim.move_to_home_position()
                elif step["type"] == "gripper":
                    if step.get("open", True):
                        self.sim.open_gripper()
                        self.gripper_open = True
                    else:
                        self.sim.close_gripper()
                        self.gripper_open = False
                        
                self._update_visual()
                
                # Small delay between steps for better visualization
                time.sleep(0.1)
                
            if not (self.stop_play and self.stop_play.is_set()):
                self._status("✅ Playback finished")
            else:
                self._status("⏹️ Playback stopped")
                
        except Exception as e:
            self._status(f"Playback error: {e}")
            print(f"❌ Playback error: {e}")

    # ---------------------------------------------------------------- Visual --
    def _update_visual(self):
        try:
            # Use gripper visualization with current state
            gripper_angle = 180 if self.gripper_open else 120
            self.sim.visualize_robot_with_gripper(
                self.ax_3d, 
                left_finger_angle=gripper_angle,
                right_finger_angle=gripper_angle,
                show_orientation=True
            )
            self.fig.canvas.draw_idle()
        except Exception as e:
            print(f"❌ Visualization error: {e}")

    def _status(self, msg: str):
        self.txt_status.set_text(msg)
        print(msg)

    # ---------------------------------------------------------------- Run --
    def run(self):
        plt.show()

    # Legacy alias (for backward compatibility)
    def run_demo(self):
        self.run()


if __name__ == "__main__":
    gui = PathRecorderGUI(simulation_mode=True)
    gui.run() 