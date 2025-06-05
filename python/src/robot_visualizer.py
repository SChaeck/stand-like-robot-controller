"""
3D Robot arm visualizer for simulation mode
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation


class RobotVisualizer:
    """3D Robot arm visualizer for simulation mode"""
    
    def __init__(self, kinematic_solver, motor_dependency_map=None, robot_instance=None):
        self.kinematic_solver = kinematic_solver
        self.motor_dependency_map = motor_dependency_map or {}
        self.robot_instance = robot_instance  # Reference to robot for conversions
        self.fig = None
        self.ax = None
        self.line = None
        self.points = None
        self.trajectory_data = None
        self.current_frame = 0
        
    def setup_animation(self, trajectory_radians):
        """Setup the animation with trajectory data"""
        # Convert motor trajectory to kinematic trajectory if needed
        if self.robot_instance and hasattr(self.robot_instance, 'convert_motor_to_kinematics_order'):
            # The trajectory_radians is in motor order, convert to kinematic order
            kinematic_trajectory = []
            for motor_joints in trajectory_radians:
                kinematic_joints = self.robot_instance.convert_motor_to_kinematics_order(motor_joints)
                kinematic_trajectory.append(kinematic_joints)
            self.trajectory_data = kinematic_trajectory
        else:
            # Assume trajectory is already in kinematic order
            self.trajectory_data = trajectory_radians
            
        self.current_frame = 0
        
        # Create figure if not exists
        if self.fig is None:
            self.fig = plt.figure(figsize=(12, 8))
            self.ax = self.fig.add_subplot(111, projection='3d')
            
        # Clear previous plots
        self.ax.clear()
        
        # Calculate workspace bounds
        self._calculate_workspace_bounds()
        
        # Setup plot properties
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        self.ax.set_title('Robot Arm Simulation')
        
        # Create initial plot
        self.update_plot(self.trajectory_data[0])
        
        # Enable interactive mode
        plt.ion()
        plt.show()
        
    def update_plot(self, joint_radians):
        """Update the robot arm visualization"""
        if self.ax is None:
            return
            
        # Clear previous frame
        self.ax.clear()
        
        # Ensure joint_radians is in kinematic order and has correct length
        if len(joint_radians) != self.kinematic_solver.num_motor_variables:
            if self.robot_instance and hasattr(self.robot_instance, 'convert_motor_to_kinematics_order'):
                joint_radians = self.robot_instance.convert_motor_to_kinematics_order(joint_radians)
            else:
                # Fallback: just take the first N joints
                joint_radians = joint_radians[:self.kinematic_solver.num_motor_variables]
            
        # Get all joint positions
        joint_positions = self._get_all_joint_positions(joint_radians)
        
        # Plot robot arm links
        self._plot_robot_arm(joint_positions)
        
        # Plot end effector trajectory if available
        if self.trajectory_data is not None:
            self._plot_trajectory_path()
        
        # Update plot properties
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        self.ax.set_title(f'Robot Arm Simulation - Frame {self.current_frame + 1}/{len(self.trajectory_data) if self.trajectory_data is not None else 1}')
        
        # Set consistent axis limits
        self.ax.set_xlim(self.x_limits)
        self.ax.set_ylim(self.y_limits)
        self.ax.set_zlim(self.z_limits)
        
        # Update display
        plt.draw()
        plt.pause(0.001)
        
        self.current_frame += 1
        
    def _get_all_joint_positions(self, joint_radians):
        """Get positions of all joints using forward kinematics"""
        positions = [[0, 0, 0]]  # Base position
        
        # joint_radians should already be in kinematic order with correct length
        if len(joint_radians) != self.kinematic_solver.num_motor_variables:
            raise ValueError(f"Expected {self.kinematic_solver.num_motor_variables} joint angles, got {len(joint_radians)}")
        
        # Calculate cumulative transformations
        T = np.eye(4)
        for i, dh in enumerate(self.kinematic_solver.dh_params):
            # Get the motor variable for this transformation
            motor_idx = self.kinematic_solver.motor_dependency_map[i]
            
            if motor_idx is not None:
                # Variable transformation - get angle from motor variable
                joint_angle = joint_radians[motor_idx]
            else:
                # Fixed transformation - use 0 as base angle
                joint_angle = 0.0
            
            # Calculate actual theta including DH offset
            theta = joint_angle + np.radians(dh['theta'])
            
            # Create transformation matrix for this joint
            T_joint = self.kinematic_solver._create_transformation_matrix(
                theta=theta,
                d=dh['d'],
                a=dh['a'], 
                alpha=np.radians(dh['alpha'])
            )
            
            # Multiply transformations
            T = T @ T_joint
            
            # Store position (convert to cm)
            positions.append((T[:3, 3] * 100).tolist())  # Convert from meters to cm
        
        return positions
    
    def _plot_robot_arm(self, joint_positions):
        """Plot the robot arm links and joints"""
        joint_positions = np.array(joint_positions)
        
        # Plot links between joints
        for i in range(len(joint_positions) - 1):
            self.ax.plot3D(
                [joint_positions[i, 0], joint_positions[i+1, 0]],
                [joint_positions[i, 1], joint_positions[i+1, 1]],
                [joint_positions[i, 2], joint_positions[i+1, 2]],
                'b-', linewidth=3, alpha=0.8
            )
        
        # Plot joints
        for i, pos in enumerate(joint_positions):
            if i == 0:
                # Base joint
                self.ax.scatter(pos[0], pos[1], pos[2], c='red', s=100, marker='s', label='Base')
            elif i == len(joint_positions) - 1:
                # End effector
                self.ax.scatter(pos[0], pos[1], pos[2], c='green', s=100, marker='^', label='End Effector')
            else:
                # Regular joint
                self.ax.scatter(pos[0], pos[1], pos[2], c='blue', s=80, marker='o', alpha=0.8)
        
        self.ax.legend()
    
    def _plot_trajectory_path(self):
        """Plot the end effector trajectory path"""
        if self.trajectory_data is None:
            return
            
        # Calculate end effector positions for entire trajectory  
        # trajectory_data should already be in kinematic order
        end_effector_positions = []
        for kinematic_radians in self.trajectory_data:
            try:
                pos, _ = self.kinematic_solver.forward_kinematics(kinematic_radians)
                end_effector_positions.append(pos)
            except Exception as e:
                print(f"Warning: Could not calculate position for trajectory point: {e}")
                continue
        
        end_effector_positions = np.array(end_effector_positions)
        
        # Plot trajectory path
        self.ax.plot3D(
            end_effector_positions[:, 0],
            end_effector_positions[:, 1],
            end_effector_positions[:, 2],
            'g--', alpha=0.5, linewidth=1, label='Trajectory Path'
        )
        
        # Plot current position on trajectory
        if self.current_frame < len(end_effector_positions):
            current_pos = end_effector_positions[self.current_frame]
            self.ax.scatter(
                current_pos[0], current_pos[1], current_pos[2],
                c='purple', s=80, marker='*', label='Current Target'
            )
    
    def _calculate_workspace_bounds(self):
        """Calculate reasonable workspace bounds for consistent plotting"""
        # Estimate workspace based on link lengths
        total_reach = 0
        for dh in self.kinematic_solver.dh_params:
            total_reach += abs(dh['a']) + abs(dh['d'])
        
        # Convert to cm and add margin
        total_reach_cm = total_reach * 100 * 1.2
        
        self.x_limits = [-total_reach_cm/2, total_reach_cm/2]
        self.y_limits = [-total_reach_cm/2, total_reach_cm/2]
        self.z_limits = [0, total_reach_cm]
    
    def save_animation(self, filename='robot_animation.gif', fps=10):
        """Save the current trajectory as an animated GIF"""
        if self.trajectory_data is None:
            print("No trajectory data to save")
            return
            
        print(f"Saving animation to {filename}...")
        
        # Create animation function
        def animate(frame):
            if frame < len(self.trajectory_data):
                self.update_plot(self.trajectory_data[frame])
            return []
        
        # Create animation
        anim = animation.FuncAnimation(
            self.fig, animate, frames=len(self.trajectory_data),
            interval=1000/fps, blit=False, repeat=True
        )
        
        # Save as GIF
        anim.save(filename, writer='pillow', fps=fps)
        print(f"Animation saved as {filename}")
    
    def close(self):
        """Close the visualization"""
        if self.fig:
            plt.close(self.fig)
            plt.ioff() 