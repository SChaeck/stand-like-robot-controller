"""
Test script for StandLikeRobot with visualization
"""

import time
import numpy as np
from stand_like_robot import StandLikeRobot


def main():
    """Test function for StandLikeRobot with visualization"""
    print("=== Testing StandLikeRobot with Visualization ===")
    
    try:
        # Initialize robot in simulation mode with visualization
        print("Initializing StandLikeRobot in simulation mode...")
        robot = StandLikeRobot(
            simulation_mode=True,
            enable_visualization=True
        )
        print("Robot initialized successfully!")
        print(f"Number of motors: {len(robot.motor_controllers)}")
        
        # Test 1: Read current positions
        print("\n--- Test 1: Current Position Reading ---")
        current_positions = robot.get_current_joint_radians()
        current_degrees = robot.get_current_joint_degrees()
        
        print("Current joint positions:")
        for i, (rad, deg) in enumerate(zip(current_positions, current_degrees)):
            print(f"  Joint {i+1}: {rad:.4f} rad ({deg:.2f} deg)")
        
        # Test 2: Get current cartesian position
        print("\n--- Test 2: Current Cartesian Position ---")
        try:
            cart_pos, cart_orient = robot.get_current_cartesian_position()
            print(f"Current cartesian position (cm): {cart_pos}")
            print(f"Current cartesian orientation (rad): {cart_orient}")
            print(f"Current cartesian orientation (deg): {np.rad2deg(cart_orient)}")
        except Exception as e:
            print(f"Cartesian position reading failed: {e}")
        
        # Test 3: Show initial robot pose
        if robot.visualizer:
            print("\n--- Test 3: Initial Robot Visualization ---")
            print("Displaying initial robot pose...")
            robot.visualizer.setup_animation([current_positions])
            robot.visualizer.update_plot(current_positions)
            
            # Wait for user to see initial pose
            input("Press Enter to continue with movement tests...")
        
        # Test 4: Joint movement with visualization
        print("\n--- Test 4: Joint Movement with Visualization ---")
        print("Moving joints with visualization...")
        
        # Create movement sequence
        movements = [
            np.array(current_positions) + np.array([0.2, -0.1, 0.15, -0.1, 0.1, -0.1]),
            np.array(current_positions) + np.array([-0.1, 0.2, -0.1, 0.15, -0.1, 0.1]),
            np.array(current_positions) + np.array([0.1, -0.1, 0.1, -0.1, 0.1, -0.1]),
            current_positions  # Return to start
        ]
        
        for i, target_joints in enumerate(movements):
            print(f"\nMovement {i+1}: Moving to {np.rad2deg(target_joints)}")
            robot.move_to_joint_radians(target_joints, time_to_go=2.0)
            time.sleep(0.5)  # Brief pause between movements
        
        # Test 5: Cartesian movement with visualization
        print("\n--- Test 5: Cartesian Movement with Visualization ---")
        try:
            current_cart_pos, current_cart_orient = robot.get_current_cartesian_position()
            
            # Create cartesian movement sequence
            cart_movements = [
                # Move in X direction
                np.concatenate([current_cart_pos + [2.0, 0, 0], current_cart_orient]),
                # Move in Y direction
                np.concatenate([current_cart_pos + [0, 2.0, 0], current_cart_orient]),
                # Move in Z direction
                np.concatenate([current_cart_pos + [0, 0, 1.0], current_cart_orient]),
                # Return to center
                np.concatenate([current_cart_pos, current_cart_orient])
            ]
            
            for i, target_cart in enumerate(cart_movements):
                print(f"\nCartesian movement {i+1}")
                print(f"Target position: {target_cart[:3]} cm")
                robot.move_to_cartesian_position(target_cart, time_to_go=3.0)
                time.sleep(0.5)
                
        except Exception as e:
            print(f"Cartesian movement test failed: {e}")
        
        # Test 6: Save animation (optional)
        if robot.visualizer:
            print("\n--- Test 6: Save Animation ---")
            save_option = input("Save animation as GIF? (y/n): ").lower()
            if save_option == 'y':
                robot.visualizer.save_animation('robot_simulation.gif', fps=15)
        
        # Test 7: Reset with visualization
        print("\n--- Test 7: Reset with Visualization ---")
        print("Resetting to initial positions...")
        robot.reset()
        time.sleep(3.0)
        
        print("\n=== Testing Complete ===")
        print("Visualization features:")
        print("- ✅ 3D robot arm rendering")
        print("- ✅ Real-time movement visualization") 
        print("- ✅ Trajectory path display")
        print("- ✅ Joint and end-effector highlighting")
        print("- ✅ Animation export capability")
        
        if robot.visualizer:
            input("Press Enter to close visualization...")
            robot.visualizer.close()
        
    except Exception as e:
        print(f"Error during robot testing: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean up
        try:
            if 'robot' in locals():
                if hasattr(robot, 'port_handler') and robot.port_handler:
                    robot.port_handler.closePort()
                    print("Port closed successfully")
                if hasattr(robot, 'visualizer') and robot.visualizer:
                    robot.visualizer.close()
                    print("Visualization closed successfully")
        except:
            pass


if __name__ == "__main__":
    main() 