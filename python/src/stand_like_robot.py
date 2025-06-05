"""
4 DoF 로봇 컨트롤을 위한 클래스

- 모터 초기 위치 설정
- 모터 현재 위치 읽기
- 모터 이동


Cartesian Space:
    - x: cm
    - y: cm
    - z: cm
    - roll: rad
    - pitch: rad
    - yaw: rad

Joint Space:
    - joint 1: rad
    - joint 2: rad
    - joint 3: rad
    - joint 4: rad  
    - joint 5: rad
    - joint 6: rad

"""

import os
import yaml
import time
import numpy as np

from dynamixel_sdk.controllerXC import *
from dynamixel_sdk.controllerAX import *
from kinematic_solver import *
from trajectory_planner import TrajectoryPlanner
from mock_motor_controller import MockMotorController
from robot_visualizer import RobotVisualizer

class StandLikeRobot:
    def __init__(
        self, 
        port_address='/dev/tty.usbserial-FT8ISMU2', 
        metadata_path='stand_like_robot.yaml',
        simulation_mode=False,
        enable_visualization=False
    ):
        self.simulation_mode = simulation_mode
        self.enable_visualization = enable_visualization
        self.port_handler = PortHandler(port_address) if not simulation_mode else None
        
        # Load metadata with error handling
        try:
            with open(metadata_path, 'r') as file:
                self.metadata = yaml.safe_load(file)
            
            if self.metadata is None:
                raise ValueError(f"YAML file {metadata_path} is empty or invalid")
            
            if 'motors' not in self.metadata:
                raise ValueError(f"YAML file missing 'motors' section")
            
            print(f"Loaded metadata with {len(self.metadata['motors'])} motors")
            
        except FileNotFoundError:
            raise FileNotFoundError(f"YAML file not found: {metadata_path}")
        except yaml.YAMLError as e:
            raise ValueError(f"Invalid YAML file: {e}")
        
        # Extract initial joint radians with error handling
        self.init_joint_radians = []
        self.joint_limits_rad = []  # Store joint limits in radians
        
        print("Debug: Motors section:")
        print(f"Type: {type(self.metadata['motors'])}")
        print(f"Content: {self.metadata['motors']}")
        
        for i, motor in enumerate(self.metadata['motors']):
            print(f"\nDebug motor {i}:")
            print(f"  Raw motor: {motor}")
            print(f"  Type: {type(motor)}")
            print(f"  Keys: {list(motor.keys())}")
            
            try:
                motor_name = list(motor.keys())[0]
                print(f"  Motor name: '{motor_name}'")
                
                motor_config = motor[motor_name]
                print(f"  Motor config: {motor_config}")
                print(f"  Motor config type: {type(motor_config)}")
                
                if motor_config is None:
                    raise ValueError(f"Motor {motor_name} configuration is None")
                
                if 'initial_rad' not in motor_config:
                    raise ValueError(f"Motor {motor_name} missing 'initial_rad' field")
                
                initial_rad = motor_config['initial_rad']
                self.init_joint_radians.append(initial_rad)
                
                # Extract joint limits
                if 'limit_rad_min' in motor_config and 'limit_rad_max' in motor_config:
                    limit_min = motor_config['limit_rad_min']
                    limit_max = motor_config['limit_rad_max']
                    self.joint_limits_rad.append((limit_min, limit_max))
                    print(f"  SUCCESS: Motor {motor_name}: initial_rad = {initial_rad}, limits = [{limit_min:.3f}, {limit_max:.3f}] rad")
                else:
                    # Default limits if not specified
                    self.joint_limits_rad.append((0.0, 2*np.pi))
                    print(f"  WARNING: Motor {motor_name}: no limits specified, using default [0, 2π]")
                    print(f"  SUCCESS: Motor {motor_name}: initial_rad = {initial_rad}")
                
            except Exception as e:
                print(f"  ERROR processing motor {i}: {motor}")
                raise ValueError(f"Error in motor {i} configuration: {e}")
        
        self.move_lock = False
        
        # Initialize trajectory planner
        self.trajectory_planner = TrajectoryPlanner(time_step=0.01)
        
        # Initialize kinematic solver 
        try:
            from kinematic_solver import KinematicSolver
            
            # Extract all DH parameters in sequence, handling dependencies
            dh_params_list = []
            motor_dependency_map = []  # Maps each DH transformation to motor index
            
            print("Processing all links for kinematic solver:")
            for i, link in enumerate(self.metadata['links']):
                print(f"  Link {i}: {link.get('name', 'unnamed')} - Type: {link.get('type', 'unknown')}")
                
                # Process all links that have DH parameters (not just revolute)
                if 'dh_params' in link:
                    dh_params = link['dh_params'].copy()
                    
                    # Determine motor dependency for this transformation
                    motor_idx = None
                    if link.get('type') == 'revolute' and 'joint' in link:
                        # Direct motor dependency
                        motor_joint = link['joint']
                        for idx, motor in enumerate(self.metadata['motors']):
                            motor_name = list(motor.keys())[0]
                            if motor_name == motor_joint:
                                motor_idx = idx
                                break
                        print(f"    Added transformation {len(dh_params_list)}: controlled by motor {motor_joint} (motor_idx: {motor_idx})")
                    
                    elif link.get('type') == 'dependent' and 'depends_on' in link:
                        # Dependent on another motor
                        depends_on = link['depends_on']
                        for idx, motor in enumerate(self.metadata['motors']):
                            motor_name = list(motor.keys())[0]
                            if motor_name == depends_on:
                                motor_idx = idx
                                break
                        print(f"    Added transformation {len(dh_params_list)}: depends on motor {depends_on} (motor_idx: {motor_idx})")
                    
                    elif link.get('type') == 'fixed_revolute':
                        # Fixed transformation (no motor dependency)
                        motor_idx = None
                        print(f"    Added transformation {len(dh_params_list)}: fixed transformation")
                    
                    else:
                        print(f"    Skipped: {link.get('type', 'unknown')} type")
                        continue
                    
                    dh_params_list.append(dh_params)
                    motor_dependency_map.append(motor_idx)
                else:
                    print(f"    Skipped: no dh_params")
            
            print(f"Final DH parameters count: {len(dh_params_list)}")
            print(f"Motor dependency map: {motor_dependency_map}")
            
            if len(dh_params_list) > 0:
                # Create joint limits for all transformations
                # For transformations without motors, use wide limits
                joint_limits_for_solver = []
                for motor_idx in motor_dependency_map:
                    if motor_idx is not None and motor_idx < len(self.joint_limits_rad):
                        joint_limits_for_solver.append(self.joint_limits_rad[motor_idx])
                    else:
                        # Fixed transformation or no motor: use wide limits
                        joint_limits_for_solver.append((-2*np.pi, 2*np.pi))
                
                self.kinematic_solver = KinematicSolver(
                    dh_params_list=dh_params_list,
                    joint_limits=joint_limits_for_solver,
                    motor_dependency_map=motor_dependency_map
                )
                self.motor_dependency_map = motor_dependency_map  # Store the motor dependencies
                print(f"✅ Kinematic solver initialized with {len(dh_params_list)} transformations")
            else:
                print("⚠️ Warning: No valid DH parameters found for kinematic solver")
                self.kinematic_solver = None
                self.motor_dependency_map = []
                
        except Exception as e:
            print(f"⚠️ Warning: Could not initialize kinematic solver: {e}")
            import traceback
            traceback.print_exc()
            self.kinematic_solver = None
            self.motor_dependency_map = []
        
        # Initialize visualizer
        if self.enable_visualization and self.kinematic_solver is not None:
            self.visualizer = RobotVisualizer(self.kinematic_solver, self.motor_dependency_map, self)
            print("🎬 Visualization enabled")
        else:
            self.visualizer = None
            if self.enable_visualization:
                print("⚠️ Visualization requested but kinematic solver not available")
        
        if self.simulation_mode:
            print("🤖 SIMULATION MODE: Motors will be simulated, no hardware connection")
            self._init_simulation_mode()
        else:
            print("🔌 HARDWARE MODE: Connecting to real motors")
            self._init_hardware_mode()

        # Set robot to initial position during initialization
        if len(self.motor_controllers) > 0:
            self.set_to_initial_position()

    def set_to_initial_position(self):
        """Set robot to initial joint positions defined in YAML"""
        print("🏠 Moving robot to initial position...")
        try:
            # Use the initial joint radians from YAML
            if self.simulation_mode:
                # In simulation mode, directly set the positions
                initial_positions = self._radian_to_position(self.init_joint_radians)
                for i, (motor_controller, initial_position) in enumerate(zip(self.motor_controllers, initial_positions)):
                    motor_controller.set_goal_position(initial_position)
                print(f"✅ Robot set to initial position: {np.rad2deg(self.init_joint_radians)} degrees")
            else:
                # In hardware mode, use smooth trajectory movement
                self.move_to_joint_radians(self.init_joint_radians, time_to_go=3.0)
                print("✅ Robot moved to initial position")
        except Exception as e:
            print(f"⚠️ Warning: Could not set initial position: {e}")

    def _init_simulation_mode(self):
        """Initialize simulation mode with mock motor controllers"""
        print("Initializing simulation mode...")
        
        # Create mock motor controllers
        self.motor_controllers = []
        self.simulated_positions = []  # Track simulated motor positions
        
        for motor in self.metadata['motors']:
            motor_name = list(motor.keys())[0]
            motor_info = motor[motor_name]
            
            # Create a simple mock controller
            mock_controller = MockMotorController(
                motor_name=motor_name,
                initial_position=motor_info['initial_position']
            )
            self.motor_controllers.append(mock_controller)
            self.simulated_positions.append(motor_info['initial_position'])
            
        print(f"Created {len(self.motor_controllers)} simulated motors")

    def _init_hardware_mode(self):
        """Initialize hardware mode with real motor controllers"""
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

    def move_to_cartesian_position(self, target_position, time_to_go=2.0):
        """Move the robot to a target cartesian position using inverse kinematics"""
        if not self.kinematic_solver:
            raise ValueError("Kinematic solver not available")
        
        try:
            # Get current joint angles as seed for IK (in kinematics order)
            current_joints = self.get_current_joint_radians_for_kinematics()
            
            # Solve inverse kinematics
            target_joints_kinematics = self.kinematic_solver.inverse_kinematics(
                target_position, 
                seed=current_joints
            )
            
            # Convert back to motor order
            target_joints_motors = self.convert_kinematics_to_motor_order(target_joints_kinematics)
            
            # Apply joint limit checking before movement
            safe_target_joints = self.check_and_apply_limits(target_joints_motors, warn=True, clip=True)
            
            # Check if limits caused significant changes
            joint_differences = np.abs(np.array(target_joints_motors) - np.array(safe_target_joints))
            max_diff = np.max(joint_differences)
            if max_diff > 0.1:  # More than ~6 degrees difference
                print(f"⚠️ Warning: Joint limits caused significant changes (max diff: {np.rad2deg(max_diff):.1f}°)")
                print("The actual end position may differ from the target.")
            
            # Move to the safe joint configuration
            self.move_to_joint_radians(safe_target_joints, time_to_go)
            
        except Exception as e:
            print(f"Error in cartesian movement: {e}")
            raise

    def convert_motor_to_kinematics_order(self, motor_joint_radians):
        """Convert joint angles from motor order to kinematics order"""
        if not hasattr(self, 'motor_dependency_map') or not self.motor_dependency_map:
            # No mapping, use first N joints where N is the number of kinematic joints
            if self.kinematic_solver:
                return motor_joint_radians[:self.kinematic_solver.num_motor_variables]
            else:
                return motor_joint_radians
        
        # Extract only the unique motor variables (AX0, AX1, AX2, AX3)
        # motor_dependency_map = [0, 1, 1, 2, 2, None, 3] means:
        # - 4 unique motor indices: 0, 1, 2, 3
        # - We need to extract motor values at these indices
        kinematic_joint_radians = []
        unique_motor_indices = []
        
        # Find unique motor indices from the dependency map
        for motor_idx in self.motor_dependency_map:
            if motor_idx is not None and motor_idx not in unique_motor_indices:
                unique_motor_indices.append(motor_idx)
        
        # Sort to ensure consistent order
        unique_motor_indices.sort()
        
        # Extract motor values for these indices
        for motor_idx in unique_motor_indices:
            if motor_idx < len(motor_joint_radians):
                kinematic_joint_radians.append(motor_joint_radians[motor_idx])
            else:
                kinematic_joint_radians.append(0.0)
        
        return kinematic_joint_radians

    def convert_kinematics_to_motor_order(self, kinematics_joint_radians):
        """Convert joint angles from kinematics order to motor order"""
        if not hasattr(self, 'motor_dependency_map') or not self.motor_dependency_map:
            # No mapping, assume same order
            return kinematics_joint_radians

        # Initialize with current motor positions
        motor_joint_radians = list(self.get_current_joint_radians())

        # Determine unique motor indices used by the kinematic solver
        unique_motor_indices = []
        for motor_idx in self.motor_dependency_map:
            if motor_idx is not None and motor_idx not in unique_motor_indices:
                unique_motor_indices.append(motor_idx)

        unique_motor_indices.sort()

        # Map kinematic order back to motor order
        for kinematic_idx, motor_idx in enumerate(unique_motor_indices):
            if (kinematic_idx < len(kinematics_joint_radians) and
                motor_idx < len(motor_joint_radians)):
                motor_joint_radians[motor_idx] = kinematics_joint_radians[kinematic_idx]

        return motor_joint_radians

    def move_to_joint_radians(self, joint_radians, time_to_go=2.0):
        """Move the robot to specified joint angles"""
        
        # Apply joint limit checking
        safe_joint_radians = self.check_and_apply_limits(joint_radians, warn=True, clip=True)
        
        try:
            # Get max velocities for trajectory planning
            max_velocities = self.get_max_velocities()
            
            trajectory_result = self.trajectory_planner.plan(
                start_joint_radians=self.get_current_joint_radians(),
                end_joint_radians=safe_joint_radians,
                max_velocities=max_velocities,
                time_to_go=time_to_go
            )
            self.trajectory_data = trajectory_result['radians']
            
            if self.visualizer:
                # Set up visualization for the trajectory
                # The visualizer will handle motor-to-kinematic conversion internally
                self.visualizer.setup_animation(self.trajectory_data)
            
            for i, radian_values in enumerate(self.trajectory_data):
                # Convert all positions at once for efficiency
                positions = self._radian_to_position(radian_values)
                
                # Move all motors
                for motor_idx, position in enumerate(positions):
                    self.motor_controllers[motor_idx].set_goal_position(position)
                
                # Update visualization
                if self.visualizer:
                    # The visualizer will handle motor-to-kinematic conversion internally
                    self.visualizer.update_plot(radian_values)
                
                time.sleep(0.05)  # Small delay between trajectory points
                
        except Exception as e:
            print(f"Error in move_to_joint_radians: {e}")
            raise

    def reset(self):
        """Reset to initial positions using trajectory planning"""
        self.move_to_joint_radians(self.init_joint_radians)
    
    def get_current_joint_degrees(self):
        return self._position_to_degree([
            motor_controller.read_current_position()
            for motor_controller in self.motor_controllers
        ])
        
    def get_current_joint_radians(self):
        return self._position_to_radian([
            motor_controller.read_current_position()
            for motor_controller in self.motor_controllers
        ])

    def get_current_joint_radians_for_kinematics(self):
        """Get joint radians in the order expected by the kinematic solver"""
        if not hasattr(self, 'motor_dependency_map') or not self.motor_dependency_map:
            # Fallback to regular order if no mapping available
            return self.get_current_joint_radians()
        
        all_joint_radians = self.get_current_joint_radians()
        
        # Use the existing conversion method
        return self.convert_motor_to_kinematics_order(all_joint_radians)

    def get_max_velocities(self):
        """Get maximum velocities for each motor in rad/s"""
        max_velocities = []
        for motor in self.metadata['motors']:
            motor_name = list(motor.keys())[0]
            motor_type = motor_name[:2]  # 'AX' or 'XC'

            specs = self.metadata['general_settings']['motor_specs'][motor_type]
            steps_per_sec = specs['max_velocity']
            rad_per_step = specs['rad_range'] / specs['position_range']
            max_velocities.append(steps_per_sec * rad_per_step)

        return max_velocities

    def get_current_cartesian_position(self):
        """
        Get current position and orientation in cartesian space
        Returns:
            tuple: (position in cm, orientation in radians)
        """
        if self.kinematic_solver is None:
            raise ValueError("Kinematic solver not initialized - check DH parameters in YAML file")
        
        current_radians = self.get_current_joint_radians_for_kinematics()
        return self.kinematic_solver.forward_kinematics(current_radians)

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
    
    def _position_to_radian(self, positions):
        """
        Convert position values to radians for AX and XC series motors
        Args:
            positions: List of position values from motors
        Returns:
            List of angles in radians
        """
        radians = []
        for i, (pos, motor) in enumerate(zip(positions, self.metadata['motors'])):
            motor_name = list(motor.keys())[0]
            motor_type = motor_name[:2]  # 'AX' or 'XC'
            
            if motor_type == 'AX':
                # AX series: 0-1023 -> 0-5.235987755982989 radians
                radians.append(pos * 5.235987755982989 / 1023)
            elif motor_type == 'XC':
                # XC series: 0-4095 -> 0-6.283185307179586 radians
                radians.append(pos * 6.283185307179586 / 4095)
        return radians
    
    def _radian_to_position(self, radians):
        """
        Convert radians to position values for AX and XC series motors
        Args:
            radians: List of angles in radians
        Returns:
            List of position values
        """
        positions = [] 
        for i, (rad, motor) in enumerate(zip(radians, self.metadata['motors'])):
            motor_name = list(motor.keys())[0]
            motor_type = motor_name[:2]  # 'AX' or 'XC'
            
            if motor_type == 'AX':
                # AX series: 0-5.235987755982989 radians -> 0-1023
                positions.append(rad * 1023 / 5.235987755982989)
            elif motor_type == 'XC':
                # XC series: 0-6.283185307179586 radians -> 0-4095
                positions.append(rad * 4095 / 6.283185307179586)
        
        return positions    

    def validate_joint_limits(self, joint_radians):
        """
        Validate if joint angles are within their limits
        Args:
            joint_radians: List or array of joint angles in radians
        Returns:
            Tuple of (is_valid, clipped_angles, violations)
        """
        if len(joint_radians) != len(self.joint_limits_rad):
            raise ValueError(f"Expected {len(self.joint_limits_rad)} joint angles, got {len(joint_radians)}")
        
        joint_radians = np.array(joint_radians)
        clipped_angles = joint_radians.copy()
        violations = []
        is_valid = True
        
        for i, (angle, (min_limit, max_limit)) in enumerate(zip(joint_radians, self.joint_limits_rad)):
            if angle < min_limit:
                clipped_angles[i] = min_limit
                violations.append(f"Joint {i} ({angle:.3f} rad) below minimum limit ({min_limit:.3f} rad)")
                is_valid = False
            elif angle > max_limit:
                clipped_angles[i] = max_limit
                violations.append(f"Joint {i} ({angle:.3f} rad) above maximum limit ({max_limit:.3f} rad)")
                is_valid = False
        
        return is_valid, clipped_angles, violations

    def check_and_apply_limits(self, joint_radians, warn=True, clip=True):
        """
        Check joint limits and optionally clip values
        Args:
            joint_radians: Joint angles in radians
            warn: Whether to print warnings for violations
            clip: Whether to clip values to limits
        Returns:
            Valid joint angles (clipped if requested)
        """
        is_valid, clipped_angles, violations = self.validate_joint_limits(joint_radians)
        
        if not is_valid and warn:
            print("🚨 Joint limit violations detected:")
            for violation in violations:
                print(f"  - {violation}")
            if clip:
                print("  ✂️ Angles clipped to safe limits")
        
        return clipped_angles if clip else joint_radians

    def get_joint_limits_degrees(self):
        """Get joint limits in degrees"""
        limits_deg = []
        for min_rad, max_rad in self.joint_limits_rad:
            limits_deg.append((np.rad2deg(min_rad), np.rad2deg(max_rad)))
        return limits_deg

    def print_joint_limits(self):
        """Print joint limits in both degrees and radians"""
        print("\n📐 Joint Limits:")
        print("=" * 60)
        for i, ((min_rad, max_rad), motor) in enumerate(zip(self.joint_limits_rad, self.metadata['motors'])):
            motor_name = list(motor.keys())[0]
            min_deg = np.rad2deg(min_rad)
            max_deg = np.rad2deg(max_rad)
            print(f"Joint {i} ({motor_name}): {min_deg:.1f}° to {max_deg:.1f}° ({min_rad:.3f} to {max_rad:.3f} rad)")
        print("=" * 60)


if __name__ == "__main__":
    # Import here to avoid circular imports when used as module
    from test_robot import main
    main()    