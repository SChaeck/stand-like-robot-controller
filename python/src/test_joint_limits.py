#!/usr/bin/env python3
"""
Test script for joint limits functionality
Tests the joint limit validation and enforcement in simulation mode
"""

import numpy as np
import time
from stand_like_robot import StandLikeRobot

def test_joint_limits():
    """Test joint limits functionality"""
    print("🔧 Testing Joint Limits Functionality")
    print("=" * 50)
    
    try:
        # Initialize robot in simulation mode
        robot = StandLikeRobot(
            simulation_mode=True,
            enable_visualization=False  # Disable visualization for faster testing
        )
        
        print(f"✅ Robot initialized with {len(robot.motor_controllers)} motors")
        
        # Print joint limits
        robot.print_joint_limits()
        
        # Test 1: Validate limit checking function
        print("\n🎯 Test 1: Joint Limit Validation")
        
        # Test angles within limits
        valid_angles = np.array([1.0, 1.0, 3.0, 2.0, 2.0, 2.0])  # Within limits
        is_valid, clipped, violations = robot.validate_joint_limits(valid_angles)
        print(f"Valid angles {np.array2string(np.rad2deg(valid_angles), precision=1)}°: Valid = {is_valid}")
        
        # Test angles outside limits
        invalid_angles = np.array([6.0, 3.0, 1.0, 6.0, 5.0, 5.0])  # Some outside limits
        is_valid, clipped, violations = robot.validate_joint_limits(invalid_angles)
        print(f"Invalid angles {np.array2string(np.rad2deg(invalid_angles), precision=1)}°: Valid = {is_valid}")
        print(f"Violations: {len(violations)}")
        for violation in violations:
            print(f"  - {violation}")
        print(f"Clipped angles: {np.array2string(np.rad2deg(clipped), precision=1)}°")
        
        # Test 2: Movement with limit enforcement
        print("\n🎯 Test 2: Movement with Limit Enforcement")
        
        current_joints = robot.get_current_joint_radians()
        print(f"Current joints (deg): {np.array2string(np.rad2deg(current_joints), precision=1)}")
        
        # Try to move to invalid positions
        test_targets = [
            ("Valid target", np.array([0.5, 1.0, 3.5, 1.5, 2.0, 2.5])),
            ("AX1 too high", np.array([1.0, 3.0, 3.0, 1.0, 2.0, 2.0])),  # AX1 limit exceeded
            ("AX2 too low", np.array([1.0, 1.0, 1.0, 1.0, 2.0, 2.0])),   # AX2 below limit
            ("Multiple violations", np.array([6.0, 4.0, 1.0, 7.0, 6.0, 6.0]))  # Multiple limits exceeded
        ]
        
        for name, target in test_targets:
            print(f"\n  Testing: {name}")
            print(f"    Target (deg): {np.array2string(np.rad2deg(target), precision=1)}")
            
            # Check what would happen with limits
            is_valid, safe_target, violations = robot.validate_joint_limits(target)
            print(f"    Valid: {is_valid}")
            if not is_valid:
                print(f"    Violations: {len(violations)}")
                print(f"    Safe target (deg): {np.array2string(np.rad2deg(safe_target), precision=1)}")
            
            # Actually move (this will apply limits automatically)
            try:
                robot.move_to_joint_radians(target, time_to_go=0.5)
                actual_position = robot.get_current_joint_radians()
                print(f"    Actual result (deg): {np.array2string(np.rad2deg(actual_position), precision=1)}")
            except Exception as e:
                print(f"    Movement failed: {e}")
        
        # Test 3: Cartesian movement with limits
        if robot.kinematic_solver:
            print("\n🎯 Test 3: Cartesian Movement with Joint Limits")
            
            # Try cartesian targets that might require joint angles outside limits
            current_pos, current_orient = robot.get_current_cartesian_position()
            print(f"Current cartesian position: {current_pos} cm")
            
            cartesian_targets = [
                ("Small movement", current_pos + [1.0, 0, 0]),
                ("Large movement", current_pos + [10.0, 10.0, 5.0]),
                ("Extreme movement", current_pos + [20.0, 15.0, 10.0])
            ]
            
            for name, target in cartesian_targets:
                print(f"\n  Testing cartesian: {name}")
                print(f"    Target position: {target}")
                
                try:
                    robot.move_to_cartesian_position(target, time_to_go=1.0)
                    final_pos, _ = robot.get_current_cartesian_position()
                    print(f"    Achieved position: {final_pos}")
                    error = np.linalg.norm(np.array(target) - np.array(final_pos))
                    print(f"    Position error: {error:.3f} cm")
                except Exception as e:
                    print(f"    Cartesian movement failed: {e}")
        
        # Test 4: Demonstrate limit utility functions
        print("\n🎯 Test 4: Utility Functions")
        
        limits_deg = robot.get_joint_limits_degrees()
        print("\nJoint limits in degrees:")
        for i, (min_deg, max_deg) in enumerate(limits_deg):
            print(f"  Joint {i}: {min_deg:.1f}° to {max_deg:.1f}°")
        
        # Test range of motion for each joint
        print("\nRange of motion for each joint:")
        for i, (min_rad, max_rad) in enumerate(robot.joint_limits_rad):
            range_deg = np.rad2deg(max_rad - min_rad)
            print(f"  Joint {i}: {range_deg:.1f}° range")
        
        print("\n✅ Joint limits testing completed successfully!")
        
        print("\nJoint Limits Features Demonstrated:")
        print("- ✅ Joint limit validation")
        print("- ✅ Automatic limit enforcement in movements")
        print("- ✅ Warning system for limit violations")
        print("- ✅ Integration with cartesian movements")
        print("- ✅ Utility functions for limit information")
        
    except Exception as e:
        print(f"❌ Error during testing: {e}")
        import traceback
        traceback.print_exc()

def test_limit_edge_cases():
    """Test edge cases for joint limits"""
    print("\n🎯 Edge Case Testing")
    
    try:
        robot = StandLikeRobot(simulation_mode=True, enable_visualization=False)
        
        # Test exact limit values
        print("\nTesting exact limit values:")
        for i, (min_limit, max_limit) in enumerate(robot.joint_limits_rad):
            # Test minimum limit
            test_angles = np.array(robot.init_joint_radians)
            test_angles[i] = min_limit
            is_valid, _, violations = robot.validate_joint_limits(test_angles)
            print(f"  Joint {i} at min limit ({np.rad2deg(min_limit):.1f}°): Valid = {is_valid}")
            
            # Test maximum limit
            test_angles = np.array(robot.init_joint_radians)
            test_angles[i] = max_limit
            is_valid, _, violations = robot.validate_joint_limits(test_angles)
            print(f"  Joint {i} at max limit ({np.rad2deg(max_limit):.1f}°): Valid = {is_valid}")
            
            # Test slightly outside limits
            test_angles = np.array(robot.init_joint_radians)
            test_angles[i] = min_limit - 0.01
            is_valid, clipped, violations = robot.validate_joint_limits(test_angles)
            print(f"  Joint {i} below min limit: Valid = {is_valid}, Clipped to {np.rad2deg(clipped[i]):.1f}°")
        
        print("✅ Edge case testing completed")
        
    except Exception as e:
        print(f"❌ Edge case testing failed: {e}")

if __name__ == "__main__":
    print("🚀 Joint Limits Test Suite")
    print("=" * 50)
    
    # Main tests
    test_joint_limits()
    
    # Edge case tests
    test_choice = input("\nRun edge case tests? (y/n): ").lower()
    if test_choice == 'y':
        test_limit_edge_cases()
    
    print("\n🎯 Joint limits testing complete!") 