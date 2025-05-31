"""
Forward Kinematics와 Inverse Kinematics을 위한 클래스
    - Forward Kinematics
    - Inverse Kinematics
"""
import numpy as np
from math import pi

class KinematicSolver:
    def __init__(self, dh_params_list):
        """
        Initialize kinematic solver with DH parameters
        Args:
            dh_params_list: List of dictionaries containing DH parameters for each link
                Each dict should have: a, alpha, d, theta
        """
        self.dh_params = dh_params_list
        self.num_links = len(dh_params_list)

    def _dh_transform(self, theta, d, a, alpha):
        """Calculate transformation matrix for a single link"""
        cos_theta = np.cos(np.deg2rad(theta))
        sin_theta = np.sin(np.deg2rad(theta))
        cos_alpha = np.cos(np.deg2rad(alpha))
        sin_alpha = np.sin(np.deg2rad(alpha))
        
        return np.array([
            [cos_theta, -sin_theta*cos_alpha, sin_theta*sin_alpha, a*cos_theta],
            [sin_theta, cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
            [0, sin_alpha, cos_alpha, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_positions):
        """
        Calculate end-effector position using forward kinematics
        Args:
            joint_positions: List of joint angles in radians
        Returns:
            End-effector position [x, y, z] and orientation [roll, pitch, yaw]
        """
        if len(joint_positions) != self.num_links:
            raise ValueError(f"Expected {self.num_links} joint positions, got {len(joint_positions)}")

        # Initialize transformation matrix
        T = np.eye(4)
        
        # Calculate forward kinematics
        for i, (params, theta) in enumerate(zip(self.dh_params, joint_positions)):
            # Update theta in DH parameters
            params['theta'] = theta
            # Calculate transformation matrix for this link
            Ti = self._dh_transform(
                params['theta'],
                params['d'],
                params['a'],
                params['alpha']
            )
            # Multiply transformations
            T = T @ Ti

        # Extract position and orientation
        position = T[:3, 3]
        orientation = self._matrix_to_euler(T[:3, :3])
        
        return position, orientation

    def _matrix_to_euler(self, R):
        """Convert rotation matrix to Euler angles (roll, pitch, yaw)"""
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
        yaw = np.arctan2(R[1, 0], R[0, 0])
        return np.array([roll, pitch, yaw])

    def inverse_kinematics(self, target_cartesian_positions, seed=None):
        """
        Calculate joint angles using inverse kinematics
        Args:
            target_position: Desired end-effector position [x, y, z]
            target_orientation: Optional desired orientation [roll, pitch, yaw] in degrees
            initial_guess: Optional initial guess for joint angles
        Returns:
            Joint angles in degrees
        """
        if seed is None:
            seed = np.zeros(self.num_links)
        
        # Levenberg Method
        max_iter = 100
        step_size = 0.1
        tolerance = 1e-3
        
        current_joint_degrees = seed.copy()
        
        for _ in range(max_iter):
            # Calculate current position
            current_pos, current_orient = self.forward_kinematics(current_joint_degrees)
            
            # Calculate position error
            pos_error = target_cartesian_positions[:3] - current_pos
            # Calculate orientation error
            orient_error = target_cartesian_positions[3:] - current_orient
            # Normalize orientation error to [-pi, pi]
            orient_error = np.arctan2(np.sin(orient_error), np.cos(orient_error))
            # Combine position and orientation errors (6D error)
            error = np.concatenate([pos_error, orient_error])
            
            # Check if we've reached the target
            if np.linalg.norm(error) < tolerance:
                break
                
            # Calculate Jacobian
            J = self._calculate_jacobian(current_joint_degrees)
            
            # Method 1: Simple Transpose Method (JT)
            # This is the basic gradient descent approach
            # delta_angles = step_size * J.T @ error
            
            # Method 2: Levenberg Method
            # This is a modification of the Gauss-Newton method
            # Uses damping factor with identity matrix to improve stability
            # (J^T * J + λI) * Δθ = J^T * e
            damping = 0.1  # λ (lambda) is the damping factor
            JTJ = J.T @ J
            delta_angles = step_size * J.T @ np.linalg.solve(JTJ + damping * np.eye(JTJ.shape[0]), error)
            
            current_joint_degrees += delta_angles
            
            # Keep angles in valid range
            current_joint_degrees = np.clip(current_joint_degrees, -pi, pi)
        
        return np.rad2deg(current_joint_degrees)

    def _calculate_jacobian(self, joint_angles):
        """Calculate the geometric Jacobian matrix"""
        J = np.zeros((6, self.num_links))
        T = np.eye(4)
        
        for i in range(self.num_links):
            # Calculate transformation up to current joint
            for j in range(i):
                params = self.dh_params[j].copy()
                params['theta'] = joint_angles[j]
                Tj = self._dh_transform(
                    params['a'],
                    params['alpha'],
                    params['d'],
                    params['theta']
                )
                T = T @ Tj
            
            # Get z-axis of current joint
            z_axis = T[:3, 2]
            
            # Get position of end-effector
            end_pos = self.forward_kinematics(joint_angles)[0]
            
            # Get position of current joint
            joint_pos = T[:3, 3]
            
            # Calculate linear and angular components
            J[:3, i] = np.cross(z_axis, end_pos - joint_pos)
            J[3:, i] = z_axis
            
            T = np.eye(4)
        
        return J
