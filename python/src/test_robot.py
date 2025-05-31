"""
Test script for robot components:
1. StandLikeRobot
2. TrajectoryPlanner
3. KinematicSolver
"""
import numpy as np
import yaml
import time
import argparse
from stand_like_robot import StandLikeRobot
from trajectory_planner import TrajectoryPlanner
from kinematic_solver import KinematicSolver

def test_trajectory_planner(time_step, start_pos, target_pos, max_velocities, duration):
    """
    Test the TrajectoryPlanner class
    Args:
        time_step: Time step for trajectory planning
        start_pos: Starting position array
        target_pos: Target position array
        max_velocities: Maximum velocities for each joint
        duration: Duration for third-order polynomial trajectory
    """
    print("\n=== Testing TrajectoryPlanner ===")
    
    # Initialize planner
    planner = TrajectoryPlanner(time_step=time_step)
    
    # Test case 1: Simple trajectory
    trajectory = planner.plan(start_pos, target_pos, max_velocities)
    
    print("Test 1 - Simple Trajectory:")
    print(f"Start position: {start_pos}")
    print(f"Target position: {target_pos}")
    print(f"Trajectory length: {len(trajectory)}")
    print(f"Final position: {trajectory[-1]}")
    
    # Test case 2: Third-order polynomial
    trajectory = planner.third_order_trajectory(start_pos, target_pos, duration)
    
    print("\nTest 2 - Third-order Polynomial:")
    print(f"Trajectory length: {len(trajectory)}")
    print(f"Final position: {trajectory[-1]}")
    
    return planner

def test_kinematic_solver(dh_params, test_angles):
    """
    Test the KinematicSolver class
    Args:
        dh_params: DH parameters for the robot
        test_angles: Joint angles to test (in degrees)
    """
    print("\n=== Testing KinematicSolver ===")
    
    # Initialize solver
    solver = KinematicSolver(dh_params)
    
    # Convert test angles to radians
    joint_angles = np.deg2rad(test_angles)
    
    # Test case 1: Forward kinematics
    position, orientation = solver.forward_kinematics(joint_angles)
    
    print("Test 1 - Forward Kinematics:")
    print(f"Joint angles: {test_angles} degrees")
    print(f"Position: {position}")
    print(f"Orientation: {np.rad2deg(orientation)} degrees")
    
    # Test case 2: Inverse kinematics
    solved_angles = solver.inverse_kinematics(position, orientation)
    
    print("\nTest 2 - Inverse Kinematics:")
    print(f"Target position: {position}")
    print(f"Solved angles: {np.rad2deg(solved_angles)} degrees")
    
    return solver

def test_stand_like_robot(robot, target_positions, wait_time, reset_positions):
    """
    Test the StandLikeRobot class
    Args:
        target_positions: Target positions for movement test
        wait_time: Time to wait after each movement
        reset_positions: Positions to reset to
    """
    print("\n=== Testing StandLikeRobot ===")
    
    try:
        # Test case 1: Get current positions
        current_positions = robot.get_current_joint_positions()
        print("Test 1 - Current Positions:")
        print(f"Joint positions: {current_positions}")
        
        # Test case 2: Move to target position
        print("\nTest 2 - Moving to Target Position:")
        print(f"Target positions: {target_positions}")
        
        # Move to target position
        robot.move_to_position(target_positions)
        
        # Wait for movement to complete
        time.sleep(wait_time)
        
        # Check final positions
        final_positions = robot.get_current_joint_positions()
        print(f"Final positions: {final_positions}")
        
        # Test case 3: Reset to initial position
        print("\nTest 3 - Resetting to Initial Position:")
        robot.reset()
        
        # Wait for reset to complete
        time.sleep(wait_time)
        
        # Check reset positions
        reset_positions = robot.get_current_joint_positions()
        print(f"Reset positions: {reset_positions}")
        
    finally:
        # Clean up
        robot.close()
    
    return robot
    

def main():
    """Main test function"""

    # Load robot parameters
    with open('stand_like_robot.yaml', 'r') as file:
        metadata = yaml.safe_load(file)
    
    # Default test parameters
    max_velocities = np.array([100, 100, 100, 100, 100, 100])
    test_angles = np.zeros(6)  # Test angles in degrees
    
    try:
        # Initialize robot
        robot = StandLikeRobot()
        
        # Test trajectory planner
        planner = test_trajectory_planner(
            time_step=0.01,
            start_pos=robot.get_current_joint_positions(),
            target_pos=[0, 0, 0, 0, 0, 0],
            max_velocities=max_velocities,
            duration=2.0
        )
        
        # Test kinematic solver
        kinematic_solver = test_kinematic_solver(
            dh_params=metadata['dh_parameters'],
            test_angles=test_angles
        )
        
        # Test robot
        test_stand_like_robot(
            robot, 
            target_positions=[0, 0, 0, 0, 0, 0],
            wait_time=2.0
        )
        
        print("\nAll tests completed successfully!")
        
    except Exception as e:
        print(f"\nError during testing: {str(e)}")
        raise

if __name__ == "__main__":
    main() 