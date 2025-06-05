import numpy as np
import matplotlib.pyplot as plt

class TrajectoryPlanner:
    def __init__(self, time_step=0.01):
        """
        Initialize trajectory planner
        Args:
            time_step: Time step for trajectory in seconds
        """
        self.time_step = time_step
        self.trajectory = None
        self.velocities = None
        self.accelerations = None

    def _third_order_trajectory(self, t: float, t_total: float, q0: float, qf: float):
        # Normalize time to [0, 1]
        s = t / t_total
        
        # Calculate coefficients for smooth motion
        a0 = q0
        a1 = 0  # Initial velocity is 0
        a2 = 3 * (qf - q0)  # For smooth acceleration
        a3 = -2 * (qf - q0)  # For smooth deceleration
        
        # Calculate position, velocity, and acceleration
        q = a0 + a1*s + a2*(s**2) + a3*(s**3)
        q_dot = (a1 + 2*a2*s + 3*a3*(s**2)) / t_total
        q_ddot = (2*a2 + 6*a3*s) / (t_total**2)
        
        return q, q_dot, q_ddot

    def plan(self, start_joint_radians, end_joint_radians, max_velocities, time_to_go=None):
        """
        Plan trajectory between start and end positions
        Args:
            start_joint_radians: List of starting joint angles in radians
            end_joint_radians: List of target joint angles in radians
            max_velocities: List of maximum angular velocities in rad/s for each joint
            time_to_go: Optional time to complete the trajectory in seconds
        Returns:
            dict: Contains trajectory information
                - 'radians': Joint angles in radians over time
                - 'velocities': Angular velocities in rad/s over time
                - 'accelerations': Angular accelerations in rad/s² over time
                - 'time_points': Time points in seconds
        """
        # Convert to numpy arrays
        start_pos = np.array(start_joint_radians)
        end_pos = np.array(end_joint_radians)
        max_vels = np.array(max_velocities)
        
        # Calculate distances and required times
        distances = np.abs(end_pos - start_pos)
        
        # Handle zero movement case
        if np.allclose(distances, 0, atol=1e-6):
            # No movement needed - return stationary trajectory
            num_points = max(2, int(0.1 / self.time_step))  # Minimum 0.1 second
            time_points = np.linspace(0, 0.1, num_points)
            
            num_joints = len(start_pos)
            self.trajectory = np.tile(start_pos, (num_points, 1))
            self.velocities = np.zeros((num_points, num_joints))
            self.accelerations = np.zeros((num_points, num_joints))
            
            return {
                'radians': self.trajectory,
                'velocities': self.velocities,
                'accelerations': self.accelerations,
                'time_points': time_points
            }
        
        if time_to_go is None:
            # Calculate minimum time needed based on max velocities
            # For third-order polynomial, peak velocity ≈ 1.5 × average velocity
            # So: time ≥ 1.5 × distance / max_velocity to respect velocity limits
            times = 1.5 * distances / max_vels
            time_to_go = np.max(times)
            time_to_go = max(time_to_go, 0.1)  # Minimum 0.1 seconds
        
        # Generate time points
        num_points = int(time_to_go / self.time_step) + 1
        time_points = np.linspace(0, time_to_go, num_points)
        
        # Initialize arrays for all joints
        num_joints = len(start_pos)
        self.trajectory = np.zeros((num_points, num_joints))
        self.velocities = np.zeros((num_points, num_joints))
        self.accelerations = np.zeros((num_points, num_joints))
        
        # Generate trajectories for all joints
        for i in range(num_joints):
            for j, t in enumerate(time_points):
                pos, vel, acc = self._third_order_trajectory(
                    t, time_to_go, start_pos[i], end_pos[i]
                )
                self.trajectory[j, i] = pos
                self.velocities[j, i] = vel
                self.accelerations[j, i] = acc

        return {
            'radians': self.trajectory,
            'velocities': self.velocities,
            'accelerations': self.accelerations,
            'time_points': time_points
        }

    def get_trajectory(self):
        """Get the planned trajectory in radians"""
        return self.trajectory

    def get_velocities(self):
        """Get the planned angular velocities in rad/s"""
        return self.velocities

    def get_accelerations(self):
        """Get the planned angular accelerations in rad/s²"""
        return self.accelerations


def main():
    """Test function for TrajectoryPlanner"""
    print("=== Testing TrajectoryPlanner ===")
    
    # Initialize planner
    planner = TrajectoryPlanner(time_step=0.01)
    print(f"Initialized TrajectoryPlanner with time_step = {planner.time_step}s")
    
    # Test 1: Simple 3-joint trajectory
    print("\n--- Test 1: Simple 3-Joint Trajectory ---")
    start_angles = np.array([0.0, 0.0, 0.0])
    target_angles = np.array([np.pi/4, -np.pi/6, np.pi/3])
    max_vels = np.array([1.0, 1.5, 2.0])  # rad/s
    
    result1 = planner.plan(
        start_joint_radians=start_angles,
        end_joint_radians=target_angles,
        max_velocities=max_vels
    )
    
    print(f"Start angles (deg): {np.rad2deg(start_angles)}")
    print(f"Target angles (deg): {np.rad2deg(target_angles)}")
    print(f"Max velocities (rad/s): {max_vels}")
    print(f"Trajectory duration: {result1['time_points'][-1]:.3f}s")
    print(f"Number of trajectory points: {len(result1['time_points'])}")
    print(f"Final angles (deg): {np.rad2deg(result1['radians'][-1])}")
    
    # Check velocity constraints
    max_actual_vels = np.max(np.abs(result1['velocities']), axis=0)
    print(f"Max actual velocities (rad/s): {max_actual_vels}")
    print(f"Max allowed velocities (rad/s): {max_vels}")
    
    # Check each joint individually with better validation
    constraint_satisfied = True
    tolerance = 0.05  # 5% tolerance for numerical precision
    for i in range(len(max_vels)):
        ratio = max_actual_vels[i] / max_vels[i]
        if max_actual_vels[i] > max_vels[i] * (1 + tolerance):
            print(f"  Joint {i+1}: ❌ VIOLATION - {max_actual_vels[i]:.3f} > {max_vels[i]:.3f} (ratio: {ratio:.2f})")
            constraint_satisfied = False
        else:
            print(f"  Joint {i+1}: ✅ OK - {max_actual_vels[i]:.3f} ≤ {max_vels[i]:.3f} (ratio: {ratio:.2f})")
    
    print(f"Velocity constraint satisfied: {constraint_satisfied}")
    
    # Test 2: Fixed time trajectory
    print("\n--- Test 2: Fixed Time Trajectory ---")
    fixed_time = 3.0  # seconds
    
    result2 = planner.plan(
        start_joint_radians=start_angles,
        end_joint_radians=target_angles,
        max_velocities=max_vels,
        time_to_go=fixed_time
    )
    
    print(f"Fixed time duration: {fixed_time}s")
    print(f"Actual duration: {result2['time_points'][-1]:.3f}s")
    print(f"Final angles (deg): {np.rad2deg(result2['radians'][-1])}")
    
    # Check velocities for fixed time case
    max_actual_vels_fixed = np.max(np.abs(result2['velocities']), axis=0)
    print(f"Max velocities with fixed time (rad/s): {max_actual_vels_fixed}")
    fixed_ratios = max_actual_vels_fixed / max_vels
    print(f"Velocity ratios with fixed time: {fixed_ratios}")
    
    # Test 3: Large angle movement
    print("\n--- Test 3: Large Angle Movement ---")
    large_start = np.array([-np.pi/2, -np.pi/3, -np.pi/4])
    large_target = np.array([np.pi/2, np.pi/3, np.pi/4])
    
    result3 = planner.plan(
        start_joint_radians=large_start,
        end_joint_radians=large_target,
        max_velocities=max_vels
    )
    
    print(f"Start angles (deg): {np.rad2deg(large_start)}")
    print(f"Target angles (deg): {np.rad2deg(large_target)}")
    print(f"Angular distances (deg): {np.rad2deg(np.abs(large_target - large_start))}")
    print(f"Trajectory duration: {result3['time_points'][-1]:.3f}s")
    
    # Test 4: Trajectory smoothness analysis
    print("\n--- Test 4: Trajectory Smoothness Analysis ---")
    
    # Calculate position, velocity, and acceleration statistics
    pos_range = np.ptp(result1['radians'], axis=0)  # peak-to-peak range
    vel_max = np.max(np.abs(result1['velocities']), axis=0)
    acc_max = np.max(np.abs(result1['accelerations']), axis=0)
    
    print("Joint-wise analysis:")
    for i in range(len(start_angles)):
        ratio = vel_max[i] / max_vels[i]
        status = "✅" if ratio <= 1.05 else "❌"
        print(f"  Joint {i+1}: {status}")
        print(f"    Position range (deg): {np.rad2deg(pos_range[i]):.2f}")
        print(f"    Max velocity (rad/s): {vel_max[i]:.3f} (limit: {max_vels[i]:.3f})")
        print(f"    Max acceleration (rad/s²): {acc_max[i]:.3f}")
        print(f"    Velocity ratio: {ratio:.2f}")
    
    # Test 5: Zero movement (should handle gracefully)
    print("\n--- Test 5: Zero Movement Test ---")
    same_angles = np.array([0.5, -0.3, 0.8])
    
    result5 = planner.plan(
        start_joint_radians=same_angles,
        end_joint_radians=same_angles,
        max_velocities=max_vels
    )
    
    print(f"Zero movement duration: {result5['time_points'][-1]:.3f}s")
    print(f"Max velocities in zero movement: {np.max(np.abs(result5['velocities'])):.6f}")
    print(f"Position error: {np.max(np.abs(result5['radians'] - same_angles)):.6f}")
    print(f"Number of trajectory points: {len(result5['time_points'])}")
    print(f"All positions identical: {np.allclose(result5['radians'], same_angles)}")
    
    # Visualization test (if matplotlib is available)
    try:
        print("\n--- Test 6: Trajectory Visualization ---")
        fig, axes = plt.subplots(3, 1, figsize=(10, 8))
        
        time_points = result1['time_points']
        
        # Plot positions
        axes[0].plot(time_points, np.rad2deg(result1['radians']))
        axes[0].set_ylabel('Position (deg)')
        axes[0].set_title('Joint Trajectories')
        axes[0].legend([f'Joint {i+1}' for i in range(len(start_angles))])
        axes[0].grid(True)
        
        # Plot velocities
        axes[1].plot(time_points, result1['velocities'])
        axes[1].set_ylabel('Velocity (rad/s)')
        axes[1].set_title('Joint Velocities')
        axes[1].legend([f'Joint {i+1}' for i in range(len(start_angles))])
        axes[1].grid(True)
        
        # Plot accelerations
        axes[2].plot(time_points, result1['accelerations'])
        axes[2].set_ylabel('Acceleration (rad/s²)')
        axes[2].set_xlabel('Time (s)')
        axes[2].set_title('Joint Accelerations')
        axes[2].legend([f'Joint {i+1}' for i in range(len(start_angles))])
        axes[2].grid(True)
        
        plt.tight_layout()
        plt.savefig('trajectory_test_results.png', dpi=150, bbox_inches='tight')
        print("Trajectory plots saved as 'trajectory_test_results.png'")
        
    except ImportError:
        print("Matplotlib not available, skipping visualization")
    
    print("\n=== TrajectoryPlanner Testing Complete ===")


if __name__ == "__main__":
    main()
