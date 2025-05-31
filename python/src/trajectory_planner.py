import numpy as np

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

    def plan(self, start_joint_degrees, end_joint_degrees, max_velocities, time_to_go=None):
        """
        Plan trajectory between start and end positions
        Args:
            start_joint_positions: List of starting joint positions
            end_joint_positions: List of target joint positions
            max_velocities: List of maximum velocities for each joint
            time_to_go: Optional time to complete the trajectory
        """
        # Convert to numpy arrays
        start_pos = np.array(start_joint_degrees)
        end_pos = np.array(end_joint_degrees)
        max_vels = np.array(max_velocities)
        
        # Calculate distances and required times
        distances = np.abs(end_pos - start_pos)
        
        if time_to_go is None:
            # Calculate minimum time needed based on max velocities
            times = distances / max_vels
            time_to_go = np.max(times)
        
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
            'positions': self.trajectory,
            'velocities': self.velocities,
            'accelerations': self.accelerations,
            'time_points': time_points
        }

    def get_trajectory(self):
        return self.trajectory

    def get_velocities(self):
        return self.velocities

    def get_accelerations(self):
        return self.accelerations


if __name__ == "__main__":
    # Example usage
    planner = TrajectoryPlanner()
    
    # Get initial positions
    initial_positions = [0, 0, 0]  # Assuming initial positions are 0 for all joints
    
    # Plan a trajectory to a new position
    target_positions = [100, 100, 100]  # Move 100 steps from initial position
    
    result = planner.plan(
        start_joint_positions=initial_positions,
        end_joint_positions=target_positions,
        max_velocities=[100, 100, 100],  # Assuming max velocities are 100 for all joints
        time_to_go=2.0  # 2 seconds
    )
    print(f"Trajectory shape: {result['positions'].shape}")