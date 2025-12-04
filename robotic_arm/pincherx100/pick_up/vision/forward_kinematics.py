#!/usr/bin/env python3
"""
Forward Kinematics Solver for PincherX100

Calculates end-effector pose (4x4 transformation matrix) from joint angles.
This is the inverse of the IK solver - given joint angles, compute where the gripper is.

Based on Denavit-Hartenberg parameters from px100_description.
"""

import numpy as np
import math


class PincherX100FK:
    """
    Forward kinematics solver for PincherX100 robotic arm.
    
    Robot dimensions (default values from px100_description):
    - L1: 44.5mm (base to shoulder height)
    - L2: 101mm (shoulder to elbow length)
    - L3: 101mm (elbow to wrist length)
    - L4: 109mm (wrist to TCP length)
    - Lm: 31.5mm (shoulder mechanism offset)
    """
    
    def __init__(self, L1=0.0445, L2=0.1010, L3=0.1010, L4=0.1090, Lm=0.0315):
        """
        Initialize FK solver with robot dimensions.
        
        Args:
            L1: Base to shoulder height (meters)
            L2: Shoulder to elbow length (meters)
            L3: Elbow to wrist length (meters)
            L4: Wrist to TCP length (meters)
            Lm: Shoulder mechanism offset (meters)
        """
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L4 = L4
        self.Lm = Lm
        
        # Calculate Lr (combined L2 and Lm)
        self.Lr = math.sqrt(Lm**2 + L2**2)
    
    def servo_positions_to_angles(self, positions):
        """
        Convert servo positions (0-4095) to joint angles (radians).
        
        Args:
            positions: numpy array [pos1, pos2, pos3, pos4] in servo units
            
        Returns:
            q: numpy array [q1, q2, q3, q4] in radians
        """
        q = ((positions - 2048) / 4096) * 2 * math.pi
        return q
    
    def solve(self, q):
        """
        Calculate end-effector pose from joint angles.
        
        Args:
            q: numpy array [q1, q2, q3, q4] in radians
               q1: base rotation (waist)
               q2: shoulder angle
               q3: elbow angle
               q4: wrist angle
        
        Returns:
            T: 4x4 homogeneous transformation matrix (gripper to base)
        """
        if len(q) != 4:
            raise ValueError("q must have 4 elements [q1, q2, q3, q4]")
        
        q1, q2, q3, q4 = q
        
        # Initialize transformation matrix
        T = np.eye(4)
        
        # Base rotation (q1) - rotation around Z axis
        T_base = np.array([
            [math.cos(q1), -math.sin(q1), 0, 0],
            [math.sin(q1),  math.cos(q1), 0, 0],
            [0,             0,            1, 0],
            [0,             0,            0, 1]
        ])
        
        # Shoulder and elbow (2R mechanism)
        # The shoulder-elbow mechanism has an offset Lm
        beta = math.atan2(self.Lm, self.L2)
        
        # Calculate wrist position (before wrist rotation)
        # This is the position of the wrist joint, not the end-effector
        r = self.Lr * math.cos(q2 + beta) + self.L3 * math.cos(q2 + q3 + beta)
        h = self.L1 + self.Lr * math.sin(q2 + beta) + self.L3 * math.sin(q2 + q3 + beta)
        
        # Wrist position in base frame (after base rotation)
        x_wrist = r * math.cos(q1)
        y_wrist = r * math.sin(q1)
        z_wrist = h
        
        # Calculate wrist orientation
        # Wrist angle q4 controls the orientation of the end-effector
        # The approach vector (Z-axis) depends on q2, q3, and q4
        
        # Calculate approach vector direction
        # The wrist is oriented such that the end-effector Z-axis points down
        # when q4 = 0, and rotates around the wrist joint
        
        # Simplified: For horizontal gripper orientation
        # The approach vector is perpendicular to the arm plane
        # More accurate calculation would use DH parameters
        
        # For now, use a simplified model:
        # - X-axis: points forward in base frame (after base rotation)
        # - Y-axis: points left in base frame
        # - Z-axis: points down (perpendicular to arm plane)
        
        # Calculate arm plane normal
        arm_angle = q2 + q3 + beta
        approach_angle = q2 + q3 + q4 + beta - math.pi/2
        
        # End-effector position (wrist + L4 along approach vector)
        # For horizontal gripper, approach is roughly perpendicular to arm
        approach_x = -math.sin(approach_angle) * math.cos(q1)
        approach_y = -math.sin(approach_angle) * math.sin(q1)
        approach_z = -math.cos(approach_angle)
        
        # End-effector position
        x_ee = x_wrist + self.L4 * approach_x
        y_ee = y_wrist + self.L4 * approach_y
        z_ee = z_wrist + self.L4 * approach_z
        
        # Build rotation matrix
        # X-axis: forward direction (in base frame after rotation)
        x_axis = np.array([math.cos(q1), math.sin(q1), 0])
        
        # Z-axis: approach vector (points down for horizontal gripper)
        z_axis = np.array([approach_x, approach_y, approach_z])
        z_axis = z_axis / np.linalg.norm(z_axis)
        
        # Y-axis: cross product (perpendicular to X and Z)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # Recompute X-axis to ensure orthogonality
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Build transformation matrix
        T = np.eye(4)
        T[0:3, 0] = x_axis
        T[0:3, 1] = y_axis
        T[0:3, 2] = z_axis
        T[0:3, 3] = [x_ee, y_ee, z_ee]
        
        return T
    
    def solve_from_servo_positions(self, positions):
        """
        Calculate end-effector pose from servo positions.
        
        Args:
            positions: numpy array [base, shoulder, elbow, wrist] in servo units (0-4095)
        
        Returns:
            T: 4x4 homogeneous transformation matrix (gripper to base)
        """
        q = self.servo_positions_to_angles(positions)
        return self.solve(q)


if __name__ == '__main__':
    # Test forward kinematics
    fk = PincherX100FK()
    
    # Test with known joint angles (all at center = 0 radians)
    q = np.array([0.0, 0.0, 0.0, 0.0])
    T = fk.solve(q)
    print("Test pose (all joints at center):")
    print(f"Position: ({T[0,3]:.4f}, {T[1,3]:.4f}, {T[2,3]:.4f})")
    print(f"Rotation matrix:\n{T[0:3, 0:3]}")
    
    # Test with servo positions
    positions = np.array([2048, 2048, 2048, 2048])  # All at center
    T2 = fk.solve_from_servo_positions(positions)
    print("\nTest from servo positions (all at 2048):")
    print(f"Position: ({T2[0,3]:.4f}, {T2[1,3]:.4f}, {T2[2,3]:.4f})")

