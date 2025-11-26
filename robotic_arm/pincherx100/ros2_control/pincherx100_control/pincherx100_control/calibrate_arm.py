#!/usr/bin/env python3
"""
Calibration script for PincherX100.

This script helps you:
1. Find and verify all servos
2. Check current positions
3. Set home position
4. Test range of motion
5. Save calibration data
"""

import sys
import time
from .dynamixel_interface import DynamixelInterface


def print_header(text):
    """Print formatted header."""
    print("\n" + "=" * 60)
    print(text)
    print("=" * 60)


def scan_servos(dxl, servo_ids):
    """Scan for servos and display their status."""
    print_header("Scanning Servos")
    
    found_servos = []
    for servo_id in servo_ids:
        if dxl.ping_servo(servo_id):
            position = dxl.read_position(servo_id)
            print(f"  ✓ Servo ID {servo_id}: Found (Position: {position})")
            found_servos.append((servo_id, position))
        else:
            print(f"  ✗ Servo ID {servo_id}: Not found")
    
    return found_servos


def test_servo_range(dxl, servo_id, joint_name):
    """Test the range of motion for a servo."""
    print(f"\nTesting {joint_name} (ID {servo_id})")
    print("Press Enter to continue, or 's' to skip...")
    
    response = input().strip().lower()
    if response == 's':
        return
    
    # Get current position
    current_pos = dxl.read_position(servo_id)
    print(f"Current position: {current_pos}")
    
    # Test movements
    test_positions = [
        (2048, "Center"),
        (1024, "CCW limit test"),
        (3072, "CW limit test"),
        (current_pos, "Return to start")
    ]
    
    for pos, description in test_positions:
        print(f"  Moving to {pos} ({description})...")
        dxl.write_position(servo_id, pos)
        time.sleep(2)
        actual_pos = dxl.read_position(servo_id)
        print(f"  Actual position: {actual_pos}")
    
    print(f"{joint_name} test complete")


def find_limits(dxl, servo_id, joint_name):
    """Interactively find safe limits for a joint."""
    print(f"\nFinding limits for {joint_name} (ID {servo_id})")
    print("Move the joint to its MINIMUM safe position")
    print("Use arrow keys or enter position directly")
    
    current_pos = dxl.read_position(servo_id)
    print(f"Current position: {current_pos}")
    
    # Manual positioning
    print("\nEnter target position (0-4095) or 'done': ")
    while True:
        try:
            user_input = input("> ").strip()
            if user_input.lower() == 'done':
                break
            
            pos = int(user_input)
            if 0 <= pos <= 4095:
                dxl.write_position(servo_id, pos)
                time.sleep(1)
                actual = dxl.read_position(servo_id)
                print(f"Moved to {actual}")
            else:
                print("Position must be between 0 and 4095")
        except ValueError:
            print("Invalid input. Enter a number or 'done'")
    
    min_pos = dxl.read_position(servo_id)
    print(f"Minimum position set: {min_pos}")
    
    # Find maximum
    print("\nNow move to MAXIMUM safe position")
    print("Enter target position (0-4095) or 'done': ")
    while True:
        try:
            user_input = input("> ").strip()
            if user_input.lower() == 'done':
                break
            
            pos = int(user_input)
            if 0 <= pos <= 4095:
                dxl.write_position(servo_id, pos)
                time.sleep(1)
                actual = dxl.read_position(servo_id)
                print(f"Moved to {actual}")
            else:
                print("Position must be between 0 and 4095")
        except ValueError:
            print("Invalid input. Enter a number or 'done'")
    
    max_pos = dxl.read_position(servo_id)
    print(f"Maximum position set: {max_pos}")
    
    return min_pos, max_pos


def find_home_position(dxl, servo_ids, joint_names):
    """Find and set home position for all joints."""
    print_header("Finding Home Position")
    print("Position each joint at its desired HOME position")
    
    home_positions = []
    
    for servo_id, joint_name in zip(servo_ids, joint_names):
        print(f"\n{joint_name} (ID {servo_id})")
        print("Enter home position (or press Enter for current): ")
        
        current_pos = dxl.read_position(servo_id)
        print(f"Current: {current_pos}")
        
        user_input = input("> ").strip()
        
        if user_input:
            try:
                home_pos = int(user_input)
                dxl.write_position(servo_id, home_pos)
                time.sleep(1)
                actual_pos = dxl.read_position(servo_id)
                home_positions.append(actual_pos)
                print(f"Home set to: {actual_pos}")
            except ValueError:
                print(f"Invalid input, using current position: {current_pos}")
                home_positions.append(current_pos)
        else:
            home_positions.append(current_pos)
            print(f"Using current position: {current_pos}")
    
    return home_positions


def generate_config(joint_names, limits, home_position, gripper_open, gripper_closed):
    """Generate YAML configuration."""
    config = "# PincherX100 Configuration\n\n"
    
    # Limits
    config += "limits:\n"
    for joint, (min_pos, max_pos) in zip(joint_names, limits):
        config += f"  {joint}:\n"
        config += f"    min: {min_pos}\n"
        config += f"    max: {max_pos}\n"
    
    # Poses
    config += "\nposes:\n"
    config += f"  home: {home_position}\n"
    config += f"  sleep: [2048, 1024, 512, 2048, {gripper_closed}]\n"
    config += f"  ready: [2048, 2500, 2200, 2048, {gripper_open}]\n"
    config += f"  gripper_open: {gripper_open}\n"
    config += f"  gripper_closed: {gripper_closed}\n"
    
    return config


def main():
    print_header("PincherX100 Calibration Tool")
    
    # Configuration
    port = '/dev/ttyUSB0'
    baudrate = 1000000
    servo_ids = [1, 2, 3, 4, 5]
    joint_names = ['base', 'shoulder', 'elbow', 'wrist', 'gripper']
    
    # Connect to servos
    print(f"\nConnecting to {port} at {baudrate} baud...")
    dxl = DynamixelInterface(port, baudrate)
    
    success, message = dxl.connect()
    if not success:
        print(f"Failed to connect: {message}")
        print("\nTroubleshooting:")
        print("  1. Check if arm is powered on")
        print("  2. Verify USB connection")
        print("  3. Check permissions: sudo usermod -a -G dialout $USER")
        return 1
    
    print("Connected successfully!")
    
    # Enable torque for all servos
    print("\nEnabling torque...")
    for servo_id in servo_ids:
        dxl.enable_torque(servo_id)
    
    try:
        # Scan servos
        found_servos = scan_servos(dxl, servo_ids)
        
        if len(found_servos) != len(servo_ids):
            print("\nWarning: Not all servos found!")
            print("Do you want to continue? (y/n): ")
            if input().strip().lower() != 'y':
                return 1
        
        # Main menu
        while True:
            print_header("Calibration Menu")
            print("1. Test servo range of motion")
            print("2. Find joint limits")
            print("3. Set home position")
            print("4. Set gripper positions")
            print("5. Generate configuration file")
            print("6. Exit")
            print("\nSelect option: ")
            
            choice = input("> ").strip()
            
            if choice == '1':
                for servo_id, joint_name in zip(servo_ids, joint_names):
                    test_servo_range(dxl, servo_id, joint_name)
            
            elif choice == '2':
                limits = []
                for servo_id, joint_name in zip(servo_ids, joint_names):
                    min_pos, max_pos = find_limits(dxl, servo_id, joint_name)
                    limits.append((min_pos, max_pos))
                print("\nLimits found:")
                for joint, (min_pos, max_pos) in zip(joint_names, limits):
                    print(f"  {joint}: [{min_pos}, {max_pos}]")
            
            elif choice == '3':
                home_position = find_home_position(dxl, servo_ids, joint_names)
                print(f"\nHome position: {home_position}")
            
            elif choice == '4':
                print("\nGripper calibration")
                gripper_id = servo_ids[4]
                
                print("Move gripper to OPEN position")
                print("Enter position or 'done': ")
                while True:
                    user_input = input("> ").strip()
                    if user_input.lower() == 'done':
                        break
                    try:
                        pos = int(user_input)
                        dxl.write_position(gripper_id, pos)
                        time.sleep(1)
                    except ValueError:
                        print("Invalid input")
                
                gripper_open = dxl.read_position(gripper_id)
                print(f"Open position: {gripper_open}")
                
                print("\nMove gripper to CLOSED position")
                print("Enter position or 'done': ")
                while True:
                    user_input = input("> ").strip()
                    if user_input.lower() == 'done':
                        break
                    try:
                        pos = int(user_input)
                        dxl.write_position(gripper_id, pos)
                        time.sleep(1)
                    except ValueError:
                        print("Invalid input")
                
                gripper_closed = dxl.read_position(gripper_id)
                print(f"Closed position: {gripper_closed}")
            
            elif choice == '5':
                if 'limits' not in locals() or 'home_position' not in locals():
                    print("\nPlease complete calibration steps first!")
                    continue
                
                if 'gripper_open' not in locals():
                    gripper_open = 2448
                    gripper_closed = 1648
                
                config = generate_config(
                    joint_names, limits, home_position,
                    gripper_open, gripper_closed
                )
                
                print("\nGenerated configuration:")
                print(config)
                
                print("\nSave to file? (y/n): ")
                if input().strip().lower() == 'y':
                    with open('/ros2_ws/src/pincherx100_control/config/arm_config.yaml', 'w') as f:
                        f.write(config)
                    print("Saved to config/arm_config.yaml")
            
            elif choice == '6':
                break
            
            else:
                print("Invalid option")
    
    finally:
        # Disable torque
        print("\nDisabling torque...")
        for servo_id in servo_ids:
            dxl.disable_torque(servo_id)
        
        dxl.disconnect()
        print("Disconnected")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())



