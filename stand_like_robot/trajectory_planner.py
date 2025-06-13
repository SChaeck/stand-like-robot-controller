"""
관절 궤적 계획기

핵심 기능:
- 부드러운 관절 궤적 생성
- 3차 다항식 보간
"""
import numpy as np
import matplotlib.pyplot as plt

class TrajectoryPlanner:
    def __init__(self, time_step=0.01):
        """
        궤적 계획기 초기화
        Args:
            time_step: 시간 간격 (초)
        """
        self.time_step = time_step
        self.trajectory = None

    def _cubic_interpolation(self, t, t_total, q0, qf):
        """
        3차 다항식 보간
        Args:
            t: 현재 시간
            t_total: 전체 시간
            q0: 시작 위치
            qf: 종료 위치
        Returns:
            보간된 위치
        """
        # 시간 정규화 [0, 1]
        s = t / t_total
        
        # 3차 다항식 계수 (부드러운 시작/정지)
        # q(s) = q0 + (qf - q0) * (3s² - 2s³)
        return q0 + (qf - q0) * (3 * s**2 - 2 * s**3)

    def plan(self, start_joint_radians, end_joint_radians, max_velocities, time_to_go=None):
        """
        관절 궤적 계획
        Args:
            start_joint_radians: 시작 관절 각도 (라디안)
            end_joint_radians: 목표 관절 각도 (라디안)
            max_velocities: 각 관절의 최대 각속도 (rad/s) - 시간 계산용
            time_to_go: 이동 시간 (초)
        Returns:
            관절 각도 궤적 배열
        """
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
            
            return {
                'radians': self.trajectory,
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
        
        # Generate trajectories for all joints
        for i in range(num_joints):
            for j, t in enumerate(time_points):
                self.trajectory[j, i] = self._cubic_interpolation(
                    t, time_to_go, start_pos[i], end_pos[i]
                )

        return {
            'radians': self.trajectory,
            'time_points': time_points
        }

    def get_trajectory(self):
        """Get the planned trajectory in radians"""
        return self.trajectory


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
    vel_max = np.max(np.abs(result1['radians']), axis=0)
    
    print("Joint-wise analysis:")
    for i in range(len(start_angles)):
        status = "✅" if vel_max[i] <= max_vels[i] else "❌"
        print(f"  Joint {i+1}: {status}")
        print(f"    Position range (deg): {np.rad2deg(pos_range[i]):.2f}")
        print(f"    Max velocity (rad/s): {vel_max[i]:.3f} (limit: {max_vels[i]:.3f})")
    
    # Test 5: Zero movement (should handle gracefully)
    print("\n--- Test 5: Zero Movement Test ---")
    same_angles = np.array([0.5, -0.3, 0.8])
    
    result5 = planner.plan(
        start_joint_radians=same_angles,
        end_joint_radians=same_angles,
        max_velocities=max_vels
    )
    
    print(f"Zero movement duration: {result5['time_points'][-1]:.3f}s")
    print(f"Max velocities in zero movement: {np.max(np.abs(result5['radians'] - same_angles)):.6f}")
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
        axes[1].plot(time_points, result1['radians'])
        axes[1].set_ylabel('Velocity (rad/s)')
        axes[1].set_title('Joint Velocities')
        axes[1].legend([f'Joint {i+1}' for i in range(len(start_angles))])
        axes[1].grid(True)
        
        # Plot accelerations
        axes[2].plot(time_points, result1['radians'])
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
