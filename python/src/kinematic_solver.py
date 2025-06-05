"""
Forward Kinematics와 Inverse Kinematics을 위한 클래스
    - Forward Kinematics
    - Inverse Kinematics
"""
import numpy as np
from math import pi

class KinematicSolver:
    def __init__(self, dh_params_list, joint_limits=None, motor_dependency_map=None):
        """
        Initialize kinematic solver with DH parameters
        Args:
            dh_params_list: List of dictionaries containing DH parameters for each transformation
                Each dict should have: a, alpha, d, theta
            joint_limits: Optional list of (min, max) tuples for joint limits in radians
            motor_dependency_map: Optional list mapping each transformation to a motor index
                None values indicate fixed transformations
        """
        self.dh_params = dh_params_list
        self.num_links = len(dh_params_list)
        self.motor_dependency_map = motor_dependency_map or list(range(self.num_links))
        
        # Determine number of actual motor variables
        self.num_motor_variables = 0
        if motor_dependency_map:
            motor_indices = [idx for idx in motor_dependency_map if idx is not None]
            self.num_motor_variables = max(motor_indices) + 1 if motor_indices else 0
        else:
            self.num_motor_variables = self.num_links
        
        # Joint limits in radians
        if joint_limits is None:
            # Default limits: -π to π for all transformations
            self.joint_limits = [(-np.pi, np.pi)] * self.num_links
        else:
            self.joint_limits = joint_limits
            
        # Compatibility with old interface
        self.n_joints = self.num_motor_variables

    def _dh_transform(self, theta, d, a, alpha):
        """Calculate transformation matrix for a single link"""
        # theta and alpha should be in radians already
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        
        return np.array([
            [cos_theta, -sin_theta*cos_alpha, sin_theta*sin_alpha, a*cos_theta],
            [sin_theta, cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
            [0, sin_alpha, cos_alpha, d],
            [0, 0, 0, 1]
        ])

    def _create_transformation_matrix(self, theta, d, a, alpha):
        """
        Alias for _dh_transform for compatibility with visualization code
        Create DH transformation matrix
        Args:
            theta: Joint angle (radians)
            d: Link offset
            a: Link length  
            alpha: Link twist (radians)
        Returns:
            4x4 transformation matrix
        """
        return self._dh_transform(theta, d, a, alpha)

    def forward_kinematics(self, joint_radians):
        """
        Calculate forward kinematics
        Args:
            joint_radians: Array of motor variables (length = num_motor_variables)
        Returns:
            End-effector position [x, y, z] and orientation [roll, pitch, yaw]
        """
        if len(joint_radians) != self.num_motor_variables:
            raise ValueError(f"Expected {self.num_motor_variables} motor variables, got {len(joint_radians)}")

        # Initialize transformation matrix
        T = np.eye(4)
        
        # Apply each DH transformation
        for i, dh in enumerate(self.dh_params):
            # Get the motor variable for this transformation
            motor_idx = self.motor_dependency_map[i]
            
            if motor_idx is not None:
                # Variable transformation - get angle from motor variable
                joint_angle = joint_radians[motor_idx]
            else:
                # Fixed transformation - use 0 as base angle
                joint_angle = 0.0
            
            # Calculate actual theta including DH offset
            theta = joint_angle + np.radians(dh['theta'])
            
            # Create transformation matrix for this link
            T_link = self._dh_transform(
                theta=theta,
                d=dh['d'],
                a=dh['a'], 
                alpha=np.radians(dh['alpha'])
            )
            
            # Multiply transformations
            T = T @ T_link
        
        # Extract position (convert to cm for consistency)
        position = T[:3, 3] * 100  # Convert from meters to cm
        
        # Extract orientation (convert rotation matrix to RPY)
        R = T[:3, :3]
        orientation = self._matrix_to_euler(R)
        
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
            target_cartesian_positions: Desired end-effector [x, y, z, roll, pitch, yaw] or [x, y, z]
            seed: Optional initial guess for joint angles
        Returns:
            Joint angles in radians
        """
        if seed is None:
            seed = np.zeros(self.num_motor_variables)
        
        # Check if target is potentially reachable
        target_pos = target_cartesian_positions[:3]
        max_reach = sum([abs(params['a']) for params in self.dh_params]) + sum([abs(params['d']) for params in self.dh_params])
        target_distance = np.linalg.norm(target_pos)
        
        if target_distance > max_reach:
            print(f"Warning: Target may be unreachable. Distance: {target_distance:.1f}, Max reach: {max_reach:.1f}")
        
        # Try multiple seeds if first attempt fails
        seed_attempts = [
            seed,
            np.zeros(self.num_motor_variables),
            np.random.uniform(-np.pi/4, np.pi/4, self.num_motor_variables),
            np.random.uniform(-np.pi/2, np.pi/2, self.num_motor_variables)
        ]
        
        best_result = None
        best_error = float('inf')
        
        for attempt, current_seed in enumerate(seed_attempts):
            result = self._solve_ik_single_attempt(target_cartesian_positions, current_seed)
            
            # Evaluate the result
            test_pos, test_orient = self.forward_kinematics(result)
            if len(target_cartesian_positions) >= 6:
                pos_error = np.linalg.norm(test_pos - target_cartesian_positions[:3])
                orient_error = np.linalg.norm(test_orient - target_cartesian_positions[3:6])
                total_error = pos_error + orient_error
            else:
                total_error = np.linalg.norm(test_pos - target_cartesian_positions[:3])
            
            if total_error < best_error:
                best_error = total_error
                best_result = result
                
            # If we found a good solution, stop
            if total_error < 0.1:  # Good enough threshold
                print(f"IK converged on attempt {attempt+1} with error: {total_error:.6f}")
                break
        
        if best_error > 1.0:  # Still poor convergence
            print(f"Warning: IK convergence poor. Best error: {best_error:.6f}")
        
        return best_result
    
    def _solve_ik_single_attempt(self, target_cartesian_positions, seed):
        """Single IK solving attempt with improved parameters"""
        # Improved parameters for better convergence
        max_iter = 200  # Fewer iterations per attempt
        step_size = 0.05  # Smaller step size for stability
        tolerance = 5e-4  # Tighter tolerance
        
        current_joint_radians = seed.copy()
        prev_error_norm = float('inf')
        
        for iteration in range(max_iter):
            # Calculate current position
            current_pos, current_orient = self.forward_kinematics(current_joint_radians)
            
            # Calculate position error
            pos_error = target_cartesian_positions[:3] - current_pos
            
            # Calculate orientation error (handle 6D vs 3D cases)
            if len(target_cartesian_positions) >= 6:
                orient_error = target_cartesian_positions[3:6] - current_orient
                # Normalize orientation error to [-pi, pi]
                orient_error = np.arctan2(np.sin(orient_error), np.cos(orient_error))
                # Combine position and orientation errors with different weights
                error = np.concatenate([pos_error, 0.1 * orient_error])  # Weight orientation less
            else:
                # Position-only IK (3D)
                error = pos_error
            
            # Check convergence
            error_norm = np.linalg.norm(error)
            if error_norm < tolerance:
                break
                
            # Check if error is increasing (diverging)
            if error_norm > prev_error_norm * 2.0 and iteration > 20:
                step_size *= 0.8  # Reduce step size
                if step_size < 1e-5:
                    break
            
            prev_error_norm = error_norm
                
            # Calculate Jacobian
            J = self._calculate_jacobian(current_joint_radians)
            
            # Use appropriate Jacobian size based on error dimension
            if len(error) == 3:  # Position-only
                J = J[:3, :]  # Use only position part of Jacobian
            elif len(error) == 6:  # Position + orientation with weights
                J[3:, :] *= 0.1  # Weight orientation jacobian less
            
            # Adaptive damping based on error
            damping = 0.001 + 0.01 * min(error_norm, 1.0)
            
            try:
                # Moore-Penrose pseudoinverse with damping
                JTJ = J.T @ J
                if JTJ.shape[0] > 0:
                    delta_joint_radians = step_size * np.linalg.solve(
                        JTJ + damping * np.eye(JTJ.shape[0]), 
                        J.T @ error
                    )
                else:
                    delta_joint_radians = np.zeros(self.num_motor_variables)
            except np.linalg.LinAlgError:
                # Fallback to simple transpose method if matrix is singular
                delta_joint_radians = step_size * 0.01 * J.T @ error
            
            # Limit the step size to prevent large jumps
            max_step = 0.1  # Smaller maximum change per iteration
            delta_norm = np.linalg.norm(delta_joint_radians)
            if delta_norm > max_step:
                delta_joint_radians = delta_joint_radians * (max_step / delta_norm)
            
            # Update joint angles
            current_joint_radians += delta_joint_radians
            
            # Apply joint limits
            for i, (min_limit, max_limit) in enumerate(self.joint_limits):
                current_joint_radians[i] = np.clip(current_joint_radians[i], min_limit, max_limit)
        
        return current_joint_radians

    def _calculate_jacobian(self, joint_radians):
        """Calculate the geometric Jacobian matrix using numerical differentiation"""
        epsilon = 1e-6
        J = np.zeros((6, self.num_motor_variables))
        
        # Get current end-effector pose
        pos0, orient0 = self.forward_kinematics(joint_radians)
        
        for i in range(self.num_motor_variables):
            # Perturb joint i
            joint_radians_plus = joint_radians.copy()
            joint_radians_plus[i] += epsilon
            
            joint_radians_minus = joint_radians.copy()
            joint_radians_minus[i] -= epsilon
            
            # Calculate forward kinematics for perturbed joints
            pos_plus, orient_plus = self.forward_kinematics(joint_radians_plus)
            pos_minus, orient_minus = self.forward_kinematics(joint_radians_minus)
            
            # Numerical differentiation
            J[:3, i] = (pos_plus - pos_minus) / (2 * epsilon)
            J[3:, i] = (orient_plus - orient_minus) / (2 * epsilon)
        
        return J


def main():
    """Test function for KinematicSolver"""
    print("=== Testing KinematicSolver ===")
    
    # More realistic DH parameters for a 6-DOF robot arm
    dh_params = [
        {'a': 0, 'alpha': 0, 'd': 15, 'theta': 0},      # Base joint
        {'a': 0, 'alpha': 90, 'd': 0, 'theta': 0},      # Shoulder joint
        {'a': 20, 'alpha': 0, 'd': 0, 'theta': 0},      # Upper arm
        {'a': 15, 'alpha': 0, 'd': 0, 'theta': 0},      # Forearm
        {'a': 0, 'alpha': 90, 'd': 0, 'theta': 0},      # Wrist pitch
        {'a': 0, 'alpha': 0, 'd': 5, 'theta': 0}        # Wrist roll
    ]
    
    # Initialize solver
    solver = KinematicSolver(dh_params)
    print(f"Initialized solver with {solver.num_motor_variables} links")
    
    # Test 1: Forward kinematics with zero angles
    print("\n--- Test 1: Forward Kinematics (Zero Position) ---")
    joint_radians_zero = np.zeros(6)
    position_zero, orientation_zero = solver.forward_kinematics(joint_radians_zero)
    print(f"Joint angles (rad): {joint_radians_zero}")
    print(f"End-effector position (cm): {position_zero}")
    print(f"End-effector orientation (rad): {orientation_zero}")
    print(f"End-effector orientation (deg): {np.rad2deg(orientation_zero)}")
    
    # Test 2: Forward kinematics with small angles
    print("\n--- Test 2: Forward Kinematics (Small Angles) ---")
    joint_radians_small = np.array([0.1, 0.2, 0.15, 0.1, 0.05, 0.1])
    position_small, orientation_small = solver.forward_kinematics(joint_radians_small)
    print(f"Joint angles (rad): {joint_radians_small}")
    print(f"Joint angles (deg): {np.rad2deg(joint_radians_small)}")
    print(f"End-effector position (cm): {position_small}")
    print(f"End-effector orientation (rad): {orientation_small}")
    print(f"End-effector orientation (deg): {np.rad2deg(orientation_small)}")
    
    # Test 3: Position-only inverse kinematics (3D)
    print("\n--- Test 3: Position-Only Inverse Kinematics ---")
    target_position = np.array([30.0, 5.0, 15.0])  # More reachable target
    print(f"Target position: {target_position}")
    
    solved_radians_3d = solver.inverse_kinematics(target_position, seed=np.zeros(6))
    print(f"Solved joint angles (rad): {solved_radians_3d}")
    print(f"Solved joint angles (deg): {np.rad2deg(solved_radians_3d)}")
    
    # Verify the solution
    verify_position_3d, _ = solver.forward_kinematics(solved_radians_3d)
    print(f"Verification position: {verify_position_3d}")
    
    position_error_3d = np.linalg.norm(target_position - verify_position_3d)
    print(f"Position error: {position_error_3d:.6f} cm")
    
    # Test 4: Full 6D inverse kinematics
    print("\n--- Test 4: Full 6D Inverse Kinematics ---")
    target_cartesian_6d = np.concatenate([position_small, orientation_small])
    print(f"Target 6D pose: {target_cartesian_6d}")
    
    solved_radians_6d = solver.inverse_kinematics(target_cartesian_6d, seed=np.zeros(6))
    print(f"Solved joint angles (rad): {solved_radians_6d}")
    print(f"Solved joint angles (deg): {np.rad2deg(solved_radians_6d)}")
    
    # Verify the solution
    verify_position_6d, verify_orientation_6d = solver.forward_kinematics(solved_radians_6d)
    verify_cartesian_6d = np.concatenate([verify_position_6d, verify_orientation_6d])
    print(f"Verification 6D pose: {verify_cartesian_6d}")
    
    pose_error_6d = np.linalg.norm(target_cartesian_6d - verify_cartesian_6d)
    print(f"6D pose error: {pose_error_6d:.6f}")
    
    # Test 5: Multiple position targets
    print("\n--- Test 5: Multiple Target Test ---")
    test_targets = [
        np.array([32.0, 0.0, 12.0]),     # Forward reach
        np.array([28.0, 8.0, 18.0]),     # Side reach  
        np.array([25.0, -5.0, 8.0]),     # Low reach
    ]
    
    for i, target in enumerate(test_targets):
        print(f"\nTarget {i+1}: {target}")
        try:
            solved = solver.inverse_kinematics(target, seed=np.zeros(6))
            verify_pos, _ = solver.forward_kinematics(solved)
            error = np.linalg.norm(target - verify_pos)
            print(f"  Solved angles (deg): {np.rad2deg(solved)}")
            print(f"  Position error: {error:.6f} cm")
            print(f"  Success: {'Yes' if error < 0.1 else 'No'}")
        except Exception as e:
            print(f"  Failed: {e}")
    
    # Test 6: Workspace analysis
    print("\n--- Test 6: Workspace Analysis ---")
    workspace_angles = [
        np.array([0, 0, 0, 0, 0, 0]),
        np.array([0.5, 0, 0, 0, 0, 0]),
        np.array([0, 0.5, 0, 0, 0, 0]),
        np.array([0, 0, 0.5, 0, 0, 0]),
        np.array([0.2, 0.3, -0.2, 0.1, 0, 0]),
    ]
    
    positions = []
    for angles in workspace_angles:
        pos, _ = solver.forward_kinematics(angles)
        positions.append(pos)
        print(f"Angles {np.rad2deg(angles)[:3]}° -> Position {pos}")
    
    # Calculate workspace extent
    positions = np.array(positions)
    workspace_size = np.ptp(positions, axis=0)  # Range in each dimension
    print(f"Workspace extent (cm): X={workspace_size[0]:.1f}, Y={workspace_size[1]:.1f}, Z={workspace_size[2]:.1f}")
    
    print("\n=== KinematicSolver Testing Complete ===")


if __name__ == "__main__":
    main()
