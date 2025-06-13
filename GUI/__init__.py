"""
GUI Package

Contains all user interface modules for robot control:
- circle_drawer_gui: Circle trajectory drawing interface
- path_recorder_gui: Path recording and playback interface
"""

from .circle_drawer_gui import CircleDrawerGUI
from .path_recorder_gui import PathRecorderGUI

__all__ = ['CircleDrawerGUI', 'PathRecorderGUI'] 