"""
Motor Controller Package

Hardware abstraction layer for motor control:
- real_motor_controller: Actual Dynamixel motor control
- mock_motor_controller: Simulation motor control
"""

from .real_motor_controller import RealMotorController
from .mock_motor_controller import MockMotorController

__all__ = ['RealMotorController', 'MockMotorController'] 