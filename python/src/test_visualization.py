#!/usr/bin/env python3
"""
Robot Visualization Test Suite
Tests robot arm movement and 3D visualization with joint limits and circle drawing
"""

import numpy as np
import time
import sys
from stand_like_robot import StandLikeRobot

def test_basic_visualization():
    """Test basic robot visualization with available positions and circle drawing"""
    print("🤖 Starting Robot Visualization Test")
    print("=" * 50)
    
    try:
        # Initialize robot
        robot = StandLikeRobot(
            simulation_mode=True,
            enable_visualization=True
        )
        
        print("✅ Robot initialized with", len(robot.motor_controllers), "motors")
        print("✅ Kinematic solver:", "Available" if robot.kinematic_solver else "Not available")
        print("✅ Visualizer:", "Available" if robot.visualizer else "Not available")
        
        # Print joint limits
        robot.print_joint_limits()
        
        # Show initial vs current positions
        initial_joints = robot.init_joint_radians
        current_joints = robot.get_current_joint_radians()
        
        print(f"\n📍 Initial joint positions (deg): {np.rad2deg(initial_joints)}")
        print(f"📍 Current joint positions (deg): {np.rad2deg(current_joints)}")
        
        # Check if robot is at initial position
        position_diff = np.abs(np.array(current_joints) - np.array(initial_joints))
        max_diff = np.max(position_diff)
        if max_diff < 0.01:  # Within 0.01 radian (about 0.6 degrees)
            print("✅ Robot is at initial position")
        else:
            print(f"⚠️ Robot position differs from initial by max {np.rad2deg(max_diff):.2f} degrees")
            print("   Moving to initial position...")
            robot.set_to_initial_position()
            # Update current position after move
            current_joints = robot.get_current_joint_radians()
            print(f"📍 Updated current positions (deg): {np.rad2deg(current_joints)}")
        
        # Show initial pose
        print("\n🎬 Displaying initial robot pose...")
        if robot.visualizer:
            robot.visualizer.setup_animation([current_joints])
            input("👀 Check the 3D plot window. Press Enter when ready to start movement...")
        
        # Test 1: Available Position Testing
        print("\n🎯 Test 1: Testing Available Positions Within Joint Limits")
        test_available_positions(robot)
        
        # Test 2: Circle Drawing with End Effector
        print("\n🎯 Test 2: Drawing Circle with End Effector")
        test_circle_drawing(robot)
        
        # Test 3: Workspace Exploration
        print("\n🎯 Test 3: Workspace Exploration")
        test_workspace_exploration(robot)
        
        print("\n✅ All tests completed successfully!")
        
        # Keep visualization open
        if robot.visualizer:
            input("\n📊 Press Enter to close visualization...")
            robot.visualizer.close()
            
    except Exception as e:
        print(f"❌ Error during testing: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("🔒 Visualization closed")

def test_available_positions(robot):
    """Test movements within available joint limits"""
    print("  Testing safe joint movements...")
    
    # Get current joints and limits
    current_joints = np.array(robot.get_current_joint_radians())
    joint_limits = robot.joint_limits_rad
    
    print("  Joint limits (degrees):")
    for i, (min_rad, max_rad) in enumerate(joint_limits):
        print(f"    Joint {i}: {np.rad2deg(min_rad):.0f}° to {np.rad2deg(max_rad):.0f}°")
    
    # Create safe test positions within limits
    safe_positions = []
    
    for i in range(len(joint_limits)):
        min_rad, max_rad = joint_limits[i]
        
        # Create a position that moves joint i to 25%, 50%, 75% of its range
        for percentage in [0.25, 0.5, 0.75]:
            test_joints = current_joints.copy()
            target_angle = min_rad + (max_rad - min_rad) * percentage
            test_joints[i] = target_angle
            safe_positions.append((f"Joint {i} ({np.rad2deg(target_angle):.0f}°)", test_joints))
    
    # Execute safe movements
    for i, (description, target_joints) in enumerate(safe_positions[:6]):  # Limit to 6 movements
        print(f"  Movement {i+1}/{min(6, len(safe_positions))}: {description}")
        
        # Validate the position is within limits
        is_valid, clipped_joints, violations = robot.validate_joint_limits(target_joints)
        
        if is_valid:
            robot.move_to_joint_radians(target_joints, time_to_go=1.5)
            time.sleep(0.5)
        else:
            print(f"    ⚠️ Position not safe, using clipped version")
            robot.move_to_joint_radians(clipped_joints, time_to_go=1.5)
            time.sleep(0.5)
    
    # Return to initial position
    print("  Returning to initial position...")
    robot.move_to_joint_radians(current_joints, time_to_go=2.0)
    time.sleep(1.0)

def test_circle_drawing(robot):
    """Draw a circle using the end effector"""
    print("  Drawing circle with end effector...")
    
    if not robot.kinematic_solver:
        print("  ❌ Cannot draw circle: kinematic solver not available")
        return
    
    try:
        # Get current cartesian position
        current_pos, current_orient = robot.get_current_cartesian_position()
        print(f"  📍 Current end effector position: {current_pos} cm")
        
        # Define circle parameters
        center = current_pos.copy()
        radius = 3.0  # 3 cm radius
        num_points = 16  # Number of points on the circle
        
        print(f"  🎯 Drawing circle: center={center[:2]}, radius={radius}cm, points={num_points}")
        
        # Generate circle points in XY plane
        circle_points = []
        for i in range(num_points + 1):  # +1 to close the circle
            angle = 2 * np.pi * i / num_points
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2]  # Keep Z constant
            
            # Create 6D target (position + orientation)
            target_6d = np.concatenate([[x, y, z], current_orient])
            circle_points.append(target_6d)
        
        # Move through circle points
        for i, target_point in enumerate(circle_points):
            print(f"  Circle point {i+1}/{len(circle_points)}: ({target_point[0]:.1f}, {target_point[1]:.1f}, {target_point[2]:.1f})")
            
            try:
                robot.move_to_cartesian_position(target_point, time_to_go=1.0)
                time.sleep(0.2)  # Brief pause at each point
            except Exception as e:
                print(f"    ⚠️ Could not reach point {i+1}: {e}")
                continue
        
        print("  ✅ Circle drawing completed!")
        
    except Exception as e:
        print(f"  ❌ Circle drawing failed: {e}")

def test_workspace_exploration(robot):
    """Explore the robot's workspace"""
    print("  Exploring robot workspace...")
    
    if not robot.kinematic_solver:
        print("  ❌ Cannot explore workspace: kinematic solver not available")
        return
    
    try:
        # Get current position as starting point
        current_pos, current_orient = robot.get_current_cartesian_position()
        print(f"  📍 Starting position: {current_pos} cm")
        
        # Define exploration targets relative to current position
        exploration_targets = [
            # Move in different directions
            current_pos + np.array([5, 0, 0]),      # +X direction
            current_pos + np.array([0, 5, 0]),      # +Y direction  
            current_pos + np.array([0, 0, 3]),      # +Z direction
            current_pos + np.array([-3, 0, 0]),     # -X direction
            current_pos + np.array([0, -3, 0]),     # -Y direction
            current_pos + np.array([0, 0, -2]),     # -Z direction
            current_pos + np.array([3, 3, 2]),      # Diagonal movement
            current_pos + np.array([-2, -2, -1]),   # Opposite diagonal
            current_pos,                             # Return to start
        ]
        
        successful_moves = 0
        total_moves = len(exploration_targets)
        
        for i, target_pos in enumerate(exploration_targets):
            target_6d = np.concatenate([target_pos, current_orient])
            print(f"  Exploring point {i+1}/{total_moves}: ({target_pos[0]:.1f}, {target_pos[1]:.1f}, {target_pos[2]:.1f})")
            
            try:
                robot.move_to_cartesian_position(target_6d, time_to_go=2.0)
                successful_moves += 1
                time.sleep(0.5)
                print(f"    ✅ Successfully reached target")
            except Exception as e:
                print(f"    ❌ Could not reach target: {e}")
                continue
        
        print(f"  📊 Workspace exploration completed: {successful_moves}/{total_moves} targets reached")
        
    except Exception as e:
        print(f"  ❌ Workspace exploration failed: {e}")

def test_static_poses():
    """Test static robot poses without animation"""
    print("🤖 Starting Static Pose Test")
    print("=" * 50)
    
    try:
        robot = StandLikeRobot(
            simulation_mode=True,
            enable_visualization=False  # No animation for static test
        )
        
        print("✅ Robot initialized for static testing")
        robot.print_joint_limits()
        
        # Test various joint configurations
        current_joints = np.array(robot.get_current_joint_radians())
        joint_limits = robot.joint_limits_rad
        
        test_poses = []
        
        # Create test poses within limits
        for i in range(len(joint_limits)):
            min_rad, max_rad = joint_limits[i]
            
            # Test pose at middle of joint range
            test_joints = current_joints.copy()
            test_joints[i] = (min_rad + max_rad) / 2
            test_poses.append(f"Joint {i} at center")
            
            # Validate and execute
            is_valid, clipped_joints, violations = robot.validate_joint_limits(test_joints)
            if is_valid:
                if robot.kinematic_solver:
                    try:
                        pos, orient = robot.kinematic_solver.forward_kinematics(
                            robot.convert_motor_to_kinematics_order(test_joints)
                        )
                        print(f"  ✅ {test_poses[-1]}: End effector at ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) cm")
                    except Exception as e:
                        print(f"  ❌ {test_poses[-1]}: Kinematic calculation failed: {e}")
                else:
                    print(f"  ✅ {test_poses[-1]}: Joint configuration valid")
            else:
                print(f"  ❌ {test_poses[-1]}: Not within limits")
        
        print("\n✅ Static pose testing completed!")
        
    except Exception as e:
        print(f"❌ Error during static testing: {e}")
        import traceback
        traceback.print_exc()

def main():
    """Main test function"""
    print("🚀 Robot Visualization Test Suite")
    print("=" * 50)
    
    try:
        import matplotlib.pyplot as plt
        print("✅ Matplotlib available")
    except ImportError:
        print("❌ Matplotlib not available - visualization will not work")
        return
    
    print("\nSelect test:")
    print("1. Full visualization test with circle drawing")
    print("2. Static poses only")
    choice = input("Choice (1 or 2): ").strip()
    
    if choice == "1":
        test_basic_visualization()
    elif choice == "2":
        test_static_poses()
    else:
        print("Invalid choice. Running full test...")
        test_basic_visualization()

if __name__ == "__main__":
    main() 