#!/usr/bin/env python3
"""
Arm Controller Wrapper

Wraps the low-level arm controller for high-level pick and place operations.
"""

import sys
import os
import time
import math
import numpy as np

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'low-level_control'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'vision'))

from control_arm import ArmController, ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST, ID_GRIPPER
from inverse_kinematics import PincherX100IK, create_pose_matrix


class PickPlaceController:
    """
    High-level controller for pick and place operations.
    """
    
    def __init__(self, baudrate=1000000):
        """
        Initialize pick and place controller.
        
        Args:
            baudrate: Serial baudrate for arm communication
        """
        self.arm = ArmController(baudrate)
        self.ik = PincherX100IK()
        
        self.initialized = False
        
    def initialize(self):
        """Initialize arm connection."""
        if self.arm.initialize():
            self.initialized = True
            return True
        return False
    
    def get_current_joint_positions(self):
        """Get current joint positions in servo units."""
        positions = []
        for servo_id in [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST]:
            pos = self.arm.get_present_position(servo_id)
            if pos is None:
                return None
            positions.append(pos)
        return positions
    
    def get_current_joint_angles(self):
        """Get current joint angles in radians."""
        positions = self.get_current_joint_positions()
        if positions is None:
            return None
        return self.ik.servo_positions_to_angles(np.array(positions))
    
    def move_to_position_horizontal_wrist(self, target_position, wait=True):
        """
        Move to position with wrist parallel to ground (horizontal approach).
        
        Strategy:
        1. Lock shoulder at 90° initially
        2. Adjust elbow and wrist to maintain horizontal wrist
        3. Move shoulder only when needed while keeping wrist horizontal
        
        Args:
            target_position: [x, y, z] in robot base frame (meters)
            wait: If True, wait for movement to complete
        
        Returns:
            success: True if movement successful
        """
        if not self.initialized:
            print("Error: Arm not initialized")
            return False
        
        x, y, z = target_position
        
        # Calculate base rotation (points towards target in XY plane)
        q1 = math.atan2(y, x)
        
        # Distance from base rotation axis to target in XY plane
        r = math.sqrt(x**2 + y**2)
        
        # Account for link offsets
        L1 = 0.0950  # Base height
        L2 = 0.1010  # Shoulder to elbow
        L3 = 0.1010  # Elbow to wrist
        L4 = 0.1090  # Wrist to end effector
        Lm = 0.0315  # Shoulder mechanism offset
        
        # Adjust for shoulder mechanism offset
        r_adj = r - Lm
        z_adj = z - L1
        
        # For horizontal wrist approach:
        # We want the end effector pointing horizontally (+Y direction)
        # This means: q2 + q3 + q4 = 0 (in the vertical plane)
        
        # Start with shoulder at 90 degrees (horizontal)
        q2 = math.pi / 2  # 90 degrees
        
        # Calculate the horizontal reach (L2 + L3 contribution)
        # and vertical reach with shoulder at 90°
        # When shoulder is 90°, it contributes L2 horizontally
        
        # Remaining horizontal distance after shoulder
        r_remaining = r_adj - L2
        
        # Calculate elbow angle using law of cosines
        # We have a triangle: shoulder-elbow-wrist-target
        # with sides L3, L4, and distance from elbow to target
        
        d = math.sqrt(r_remaining**2 + z_adj**2)  # Distance from elbow to target
        
        if d > (L3 + L4):
            print(f"    [WARNING] Target too far: {d:.3f}m > {L3+L4:.3f}m")
            print(f"    [INFO] Will adjust shoulder angle to reach")
            # Need to move shoulder to extend reach
            # Calculate required shoulder angle
            total_reach = math.sqrt(r_adj**2 + z_adj**2)
            cos_q2 = (total_reach**2 - L2**2 - (L3+L4)**2) / (2 * L2 * (L3+L4))
            if abs(cos_q2) <= 1:
                q2 = math.acos(cos_q2)
            # Recalculate with new shoulder angle
        
        # Calculate elbow angle
        # Using law of cosines for the triangle formed by L3, L4, and d
        if d < abs(L3 - L4):
            d = abs(L3 - L4) + 0.01  # Minimum distance
        
        cos_q3 = (d**2 - L3**2 - L4**2) / (2 * L3 * L4)
        if abs(cos_q3) > 1:
            print(f"    [ERROR] Target unreachable with horizontal wrist")
            return False
        
        q3 = math.acos(cos_q3)
        
        # Calculate wrist angle to keep horizontal
        # For horizontal wrist: q2 + q3 + q4 = 0
        q4 = -(q2 + q3)
        
        # Adjust based on target height
        alpha = math.atan2(z_adj, r_remaining)
        q4 = -(q2 + q3 - alpha)
        
        print(f"    [DEBUG] Horizontal Wrist Approach:")
        print(f"    [DEBUG] Target: X={x:.4f}, Y={y:.4f}, Z={z:.4f}")
        print(f"    [DEBUG] Base angle: {math.degrees(q1):.1f}°")
        print(f"    [DEBUG] Shoulder: {math.degrees(q2):.1f}° (started at 90°)")
        print(f"    [DEBUG] Elbow: {math.degrees(q3):.1f}°")
        print(f"    [DEBUG] Wrist: {math.degrees(q4):.1f}° (maintains horizontal)")
        
        # Convert to servo positions
        q = np.array([q1, q2, q3, q4])
        positions = self.ik.angles_to_servo_positions(q)
        
        print(f"    [DEBUG] Servo Positions: base={positions[0]}, shoulder={positions[1]}, elbow={positions[2]}, wrist={positions[3]}")
        
        # Move joints
        success = True
        servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST]
        
        for i, servo_id in enumerate(servo_ids):
            if not self.arm.set_goal_position(servo_id, positions[i]):
                success = False
                print(f"Error moving servo {servo_id}")
            time.sleep(0.1)
        
        if wait and success:
            time.sleep(4.0)  # Increased wait for slower movements to complete
        
        return success
    
    def move_to_position(self, target_position, orientation='horizontal', elbow='up', wait=True, use_horizontal_wrist=False):
        """
        Move end-effector to target position.
        
        Args:
            target_position: [x, y, z] in robot base frame (meters)
            orientation: 'horizontal' or 'vertical'
            elbow: 'up' or 'down'
            wait: If True, wait for movement to complete
        
        Returns:
            success: True if movement successful
        """
        if not self.initialized:
            print("Error: Arm not initialized")
            return False
        
        # Use horizontal wrist approach if requested
        if use_horizontal_wrist:
            return self.move_to_position_horizontal_wrist(target_position, wait=wait)
        
        # Create pose matrix
        T = create_pose_matrix(target_position, orientation=orientation)
        
        # DEBUG: Print target position
        print(f"    [DEBUG] Target Position: X={target_position[0]:.4f}, Y={target_position[1]:.4f}, Z={target_position[2]:.4f}")
        
        try:
            # Solve inverse kinematics
            q = self.ik.solve(T, elbow=elbow)
            
            # DEBUG: Print joint angles
            print(f"    [DEBUG] Joint Angles (rad): q1={q[0]:.4f}, q2={q[1]:.4f}, q3={q[2]:.4f}, q4={q[3]:.4f}")
            print(f"    [DEBUG] Joint Angles (deg): q1={math.degrees(q[0]):.1f}°, q2={math.degrees(q[1]):.1f}°, q3={math.degrees(q[2]):.1f}°, q4={math.degrees(q[3]):.1f}°")
            
            # Convert to servo positions
            positions = self.ik.angles_to_servo_positions(q)
            
            # DEBUG: Print servo positions
            print(f"    [DEBUG] Servo Positions: base={positions[0]}, shoulder={positions[1]}, elbow={positions[2]}, wrist={positions[3]}")
            
            # Move each joint with delays to prevent overload
            # Move joints sequentially with small delays for gentler movement
            success = True
            servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST]
            
            for i, servo_id in enumerate(servo_ids):
                if not self.arm.set_goal_position(servo_id, positions[i]):
                    success = False
                    print(f"Error moving servo {servo_id}")
                # Small delay between joint movements to prevent simultaneous load
                time.sleep(0.1)
            
            if wait and success:
                # Wait for movement to complete (longer wait for slower, gentler movement)
                # With PROFILE_VELOCITY=1000, movements are slower, so need more time
                time.sleep(3.0)  # Increased from 2.0 to 3.0 for gentler movement
            
            return success
            
        except ValueError as e:
            print(f"IK Error: {e}")
            return False
    
    def move_to_joint_positions(self, positions, wait=True):
        """
        Move to specific joint positions.
        
        Args:
            positions: [base, shoulder, elbow, wrist] in servo units
            wait: If True, wait for movement to complete
        """
        if not self.initialized:
            print("Error: Arm not initialized")
            return False
        
        servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST]
        success = True
        
        for i, servo_id in enumerate(servo_ids):
            if not self.arm.set_goal_position(servo_id, positions[i]):
                success = False
        
        if wait and success:
            time.sleep(2.0)
        
        return success
    
    def open_gripper(self):
        """Open gripper."""
        if not self.initialized:
            return False
        return self.arm.set_goal_position(ID_GRIPPER, 2448)  # Open position
    
    def close_gripper(self, position=None):
        """
        Close gripper.
        
        Args:
            position: Gripper position (0-4095). If None, uses default closed position.
        """
        if not self.initialized:
            return False
        if position is None:
            position = 1648  # Default closed position
        return self.arm.set_goal_position(ID_GRIPPER, position)
    
    def move_to_home(self):
        """Move to home position."""
        home_positions = [2048, 2048, 2048, 2048]
        return self.move_to_joint_positions(home_positions)
    
    def pick_object(self, object_position, grip_height, approach_height=0.05, lift_height=0.10):
        """
        Pick up object at specified position.
        
        Args:
            object_position: [x, y, z] in robot base frame (meters)
                           z should be at platform level (0.0) for bottle base
            grip_height: Height above platform to grip the object (meters)
                       This is the middle of the bottle (object_height * grip_height_ratio)
            approach_height: Height above grip position before descending (meters)
            lift_height: Height to lift after grasping (meters, relative to platform)
        
        Returns:
            success: True if pick operation successful
        """
        if not self.initialized:
            print("Error: Arm not initialized")
            return False
        
        print(f"\n{'='*60}")
        print(f"PICK OPERATION")
        print(f"{'='*60}")
        print(f"Object position (bottle base): {object_position}")
        print(f"Grip height: {grip_height*100:.1f}cm above platform")
        
        # Step 1: Move to approach position (above grip position)
        approach_pos = [object_position[0], object_position[1], grip_height + approach_height]
        print(f"\nStep 1: Moving to approach position: ({approach_pos[0]:.3f}, {approach_pos[1]:.3f}, {approach_pos[2]:.3f})")
        if not self.move_to_position(approach_pos, orientation='horizontal', elbow='up'):
            print("  ✗ Failed to move to approach position")
            return False
        
        time.sleep(2.0)  # Wait for movement (increased for gentler movement)
        
        # Step 2: Open gripper
        print("\nStep 2: Opening gripper")
        self.open_gripper()
        time.sleep(0.5)
        
        # Step 3: Descend to grip position (middle of bottle)
        grip_pos = [object_position[0], object_position[1], grip_height]
        print(f"\nStep 3: Descending to grip position: ({grip_pos[0]:.3f}, {grip_pos[1]:.3f}, {grip_pos[2]:.3f})")
        if not self.move_to_position(grip_pos, orientation='horizontal', elbow='up'):
            print("  ✗ Failed to move to grip position")
            return False
        
        time.sleep(2.0)  # Wait for movement (increased for gentler movement)
        
        # Step 4: Close gripper (using default position for now)
        print("\nStep 4: Closing gripper (default position)")
        self.close_gripper()
        time.sleep(1.5)  # Wait for gripper to close (increased)
        
        # Step 5: Lift object
        lift_pos = [object_position[0], object_position[1], lift_height]
        print(f"\nStep 5: Lifting to: ({lift_pos[0]:.3f}, {lift_pos[1]:.3f}, {lift_pos[2]:.3f})")
        if not self.move_to_position(lift_pos, orientation='horizontal', elbow='up'):
            print("  ✗ Failed to lift object")
            return False
        
        time.sleep(2.0)  # Wait for movement (increased for gentler movement)
        
        print(f"\n{'='*60}")
        print("✓ Pick operation complete")
        print(f"{'='*60}\n")
        return True
    
    def place_object(self, drop_position, drop_height=0.05):
        """
        Place object at specified position.
        
        Args:
            drop_position: [x, y, z] in robot base frame (meters)
            drop_height: Height above drop location (meters)
        
        Returns:
            success: True if place operation successful
        """
        if not self.initialized:
            print("Error: Arm not initialized")
            return False
        
        print(f"Placing object at position: {drop_position}")
        
        # Step 1: Move to drop position (above)
        place_pos = [drop_position[0], drop_position[1], drop_position[2] + drop_height]
        print(f"  Moving to drop position: {place_pos}")
        if not self.move_to_position(place_pos, orientation='horizontal', elbow='up'):
            print("  Failed to move to drop position")
            return False
        
        time.sleep(2.0)  # Wait for movement (increased for gentler movement)
        
        # Step 2: Open gripper
        print("  Opening gripper")
        self.open_gripper()
        time.sleep(1.5)  # Wait for gripper to open (increased)
        
        # Step 3: Lift up
        lift_pos = [drop_position[0], drop_position[1], drop_position[2] + 0.10]
        print(f"  Lifting up")
        if not self.move_to_position(lift_pos, orientation='horizontal', elbow='up'):
            print("  Failed to lift after drop")
            return False
        
        time.sleep(2.0)  # Wait for movement (increased for gentler movement)
        
        print("  Place operation complete")
        return True
    
    def shutdown(self):
        """Shutdown arm controller."""
        if self.initialized:
            self.arm.shutdown()
            self.initialized = False


if __name__ == '__main__':
    # Test arm controller
    controller = PickPlaceController()
    
    if controller.initialize():
        print("Arm initialized")
        
        # Test home position
        print("Moving to home...")
        controller.move_to_home()
        time.sleep(2)
        
        # Test position
        test_pos = [0.15, 0.10, 0.10]
        print(f"Moving to test position: {test_pos}")
        controller.move_to_position(test_pos)
        time.sleep(2)
        
        # Return home
        controller.move_to_home()
        
        controller.shutdown()
    else:
        print("Failed to initialize arm")

