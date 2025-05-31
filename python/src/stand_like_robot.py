"""
4 DoF 로봇 컨트롤을 위한 클래스

- 모터 초기 위치 설정
- 모터 현재 위치 읽기
- 모터 이동


Cartesian Space:
    - x: mm
    - y: mm
    - z: mm
    - roll: rad
    - pitch: rad
    - yaw: rad

Joint Space:
    - joint 1: degree
    - joint 2: degree
    - joint 3: degree
    - joint 4: degree
    - joint 5: degree
    - joint 6: degree

"""

import os
import yaml
import time

from dynamixel_sdk.controllerXC import *
from dynamixel_sdk.controllerAX import *
from kinematic_solver import *
from trajectory_planner import TrajectoryPlanner

class StandLikeRobot:
    def __init__(
        self, 
        port_address='/dev/tty.usbserial-FT8ISMU2', 
        metadata_path='stand_like_robot.yaml'
    ):
        self.port_handler = PortHandler(port_address)
        self.metadata = yaml.safe_load(open(metadata_path, 'r'))
        self.init_joint_positions = [
            motor[list(motor.keys())[0]]['initial_position']
            for motor in self.metadata['motors']
        ]
        self.move_lock = False
        
        # Initialize trajectory planner
        self.trajectory_planner = TrajectoryPlanner(time_step=0.01)
        
        # Initialize solver
        self.kinematic_solver = KinematicSolver(self.metadata['dh_parameters'])
        
        if os.name == 'nt':
            import msvcrt
            def getch():
                return msvcrt.getch().decode()
        else:
            import sys, tty, termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            def getch():
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1)
                finally:    
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                return ch
        
        # Open port
        if self.port_handler.openPort():
            print(self.port_handler.getPortName())
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.port_handler.setBaudRate(self.metadata['general_settings']['baudrate']):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
            
        # Initialize motor controllers
        self.motor_controllers = [None] * len(self.metadata['motors'])
        for motor in self.metadata['motors']:
            motor_name = list(motor.keys())[0]
            motor_info = motor[motor_name]
            motor_id = motor_info['id']
            
            if motor_name.startswith('AX'):
                motor_controller = ControllerAX(self.port_handler, self.metadata['general_settings']['baudrate'], motor_id)
                motor_controller.torque_enable()
                motor_controller.set_moving_speed(speed_value=motor_info['speed'])
            elif motor_name.startswith('XC'):
                motor_controller = ControllerXC(self.port_handler, self.metadata['general_settings']['baudrate'], motor_id)
                motor_controller.torque_enable()
                motor_controller.set_position_p_gain(speed_value=motor_info['speed'])
            else:
                raise ValueError(f"Invalid motor name: {motor_name}")
            self.motor_controllers[motor_id] = motor_controller

    def move_to_cartesian_position(self, target_cartesian_positions, time_to_go=None):
        """
        Move to target positions using trajectory planning
        Args:
            target_cartesian_positions: Target position and orientation in cartesian space
                                      (position in mm, orientation in radians)
            time_to_go: Optional time to reach target position
        """
        if self.move_lock:
            print("Move is locked")
            return
        self.move_lock = True
    
        # Get current positions
        current_degrees = self.get_current_joint_degrees()
                
        # Get target joint positions
        # TODO: Jacobian을 쓰는 IK라서 time_to_go가 길어질 수록 오차가 큼 
        #   -> 이 위치에서 하는게 맞는지? 더 정밀한 방법은 없는지? 다른 로봇은 어떻게 하는지?    
        # TODO: current_degrees는 Joint space고 target_cartesian_positions는 Cartesian space 
        #   -> 변경 필요
        target_joint_degrees = self.kinematic_solver.inverse_kinematics(target_cartesian_positions, seed=current_degrees)
            
        # Get max velocities from metadata
        max_velocities = self.get_max_velocities()
        
        # Plan trajectory
        result = self.trajectory_planner.plan(
            start_joint_degrees=current_degrees,
            end_joint_degrees=target_joint_degrees,
            max_velocities=max_velocities,  
            time_to_go=time_to_go
        )
        
        # Execute trajectory
        for i, degrees in enumerate(result['degrees']):
            for joint_id, degree in enumerate(degrees):
                self.motor_controllers[joint_id].move(position=self._degree_to_position(degree))
            # Wait for time_step
            time.sleep(self.trajectory_planner.time_step)
        
        self.move_lock = False

    def move_to_joint_degrees(self, target_joint_degrees, time_to_go=None):
        """
        Move to target joint degrees using trajectory planning
        """
        if self.move_lock:
            print("Move is locked")
            return
        self.move_lock = True
        
        # Get current positions
        current_degrees = self.get_current_joint_degrees()
        
        # Get max velocities from metadata
        max_velocities = self.get_max_velocities() 
        
        # Plan trajectory
        result = self.trajectory_planner.plan(
            start_joint_degrees=current_degrees,
            end_joint_degrees=target_joint_degrees,
            max_velocities=max_velocities,  
            time_to_go=time_to_go
        )
        
        # Execute trajectory
        for i, degrees in enumerate(result['degrees']):
            for joint_id, degree in enumerate(degrees):
                self.motor_controllers[joint_id].move(position=self._degree_to_position(degree))
            # Wait for time_step
            time.sleep(self.trajectory_planner.time_step)
        
        self.move_lock = False

    def reset(self):
        """Reset to initial positions using trajectory planning"""
        self.move_to_joint_degrees(self._position_to_degree(self.init_joint_positions))
    
    def get_current_joint_degrees(self):
        return self._position_to_degree([
            motor_controller.read_current_position()
            for motor_controller in self.motor_controllers
        ])

    def get_max_velocities(self):
        """Get maximum velocities for each motor from metadata"""
        max_velocities = []
        for motor in self.metadata['motors']:
            motor_name = list(motor.keys())[0]
            motor_type = motor_name[:2]  # 'AX' or 'XC'
            max_velocities.append(self.metadata['general_settings']['motor_specs'][motor_type]['max_velocity'])
        return max_velocities

    def get_current_cartesian_position(self):
        """
        Get current position and orientation in cartesian space
        Returns:
            tuple: (position in mm, orientation in radians)
        """
        current_degrees = self.get_current_joint_degrees()
        return self.kinematic_solver.forward_kinematics(current_degrees)

    def _position_to_degree(self, positions):
        """
        Convert position values to degrees for AX and XC series motors
        Args:
            positions: List of position values from motors
        Returns:
            List of angles in degrees
        """
        degrees = []
        for i, (pos, motor) in enumerate(zip(positions, self.metadata['motors'])):
            motor_name = list(motor.keys())[0]
            motor_type = motor_name[:2]  # 'AX' or 'XC'
            
            if motor_type == 'AX':
                # AX series: 0-1023 -> 0-300 degrees
                degrees.append(pos * 300 / 1023)
            elif motor_type == 'XC':
                # XC series: 0-4095 -> 0-360 degrees
                degrees.append(pos * 360 / 4095)
            else:
                raise ValueError(f"Unknown motor type: {motor_type}")
        return degrees
    
    def _degree_to_position(self, degrees):
        """
        Convert degrees to position values for AX and XC series motors
        Args:
            degrees: List of angles in degrees
        Returns:
            List of position values
        """
        positions = []
        for i, (deg, motor) in enumerate(zip(degrees, self.metadata['motors'])):
            motor_name = list(motor.keys())[0]
            motor_type = motor_name[:2]  # 'AX' or 'XC'
            
            if motor_type == 'AX':
                # AX series: 0-300 degrees -> 0-1023
                positions.append(deg * 1023 / 300)
            elif motor_type == 'XC':
                # XC series: 0-360 degrees -> 0-4095
                positions.append(deg * 4095 / 360)
            else:
                raise ValueError(f"Unknown motor type: {motor_type}")
        return positions