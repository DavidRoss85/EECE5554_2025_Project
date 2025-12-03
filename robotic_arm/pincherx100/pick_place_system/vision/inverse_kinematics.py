#!/usr/bin/env python3
"""
Inverse Kinematics Solver for PincherX100

Based on the geometric approach with wrist decoupling from:
- https://github.com/cychitivav/px100_ikine
- https://github.com/cychitivav/px100_description

The IK solver calculates joint angles (q1, q2, q3, q4) from a desired
end-effector pose (4x4 homogeneous transformation matrix).
"""

import numpy as np
import math


class PincherX100IK:
    """
    Inverse kinematics solver for PincherX100 robotic arm.
    
    Robot dimensions (default values from px100_description):
    - L1: 44.5mm (base to shoulder height)
    - L2: 101mm (shoulder to elbow length)
    - L3: 101mm (elbow to wrist length)
    - L4: 109mm (wrist to TCP length)
    - Lm: 31.5mm (shoulder mechanism offset)
    """
    
    def __init__(self, L1=0.0445, L2=0.1010, L3=0.1010, L4=0.1090, Lm=0.0315):
        """
        Initialize IK solver with robot dimensions.
        
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
        
    def solve(self, T, elbow='up'):
        """
        Solve inverse kinematics for given end-effector pose.
        
        Args:
            T: 4x4 homogeneous transformation matrix
               T[0:3, 3] is the position (x, y, z)
               T[0:3, 0:3] is the rotation matrix
            elbow: 'up' or 'down' - elbow configuration preference
            
        Returns:
            q: numpy array [q1, q2, q3, q4] in radians
               q1: base rotation (waist)
               q2: shoulder angle
               q3: elbow angle
               q4: wrist angle
        """
        if T.shape != (4, 4):
            raise ValueError("T must be a 4x4 homogeneous transformation matrix")
        
        q = np.zeros(4)
        
        # Extract position and rotation
        p = T[0:3, 3]  # Position vector
        R = T[0:3, 0:3]  # Rotation matrix
        
        # DEBUG: Print position and angle calculation
        print(f"    [DEBUG] IK Input Position: X={p[0]:.4f}, Y={p[1]:.4f}, Z={p[2]:.4f}")
        
        # q1 (Waist) - rotation around base
        q[0] = math.atan2(p[1], p[0])
        
        # DEBUG: Print base rotation calculation
        print(f"    [DEBUG] Base Rotation Calculation:")
        print(f"      atan2(Y={p[1]:.4f}, X={p[0]:.4f}) = {q[0]:.4f} rad = {math.degrees(q[0]):.1f}°")
        
        # Wrist decoupling
        # Approach vector (z-axis of end-effector frame)
        a = R[:, 2]
        
        # Wrist position (move back L4 along approach vector)
        w = p - self.L4 * a
        
        # 2R mechanism solution
        r = math.sqrt(w[0]**2 + w[1]**2)
        h = w[2] - self.L1
        
        c = math.sqrt(r**2 + h**2)
        
        # Geometric parameters
        beta = math.atan2(self.Lm, self.L2)
        psi = math.pi / 2 - beta
        Lr = math.sqrt(self.Lm**2 + self.L2**2)
        
        # Check if position is reachable
        if c > (Lr + self.L3):
            raise ValueError(f"Position unreachable: distance {c:.4f}m exceeds max reach {Lr + self.L3:.4f}m")
        
        if c < abs(Lr - self.L3):
            raise ValueError(f"Position unreachable: distance {c:.4f}m is less than min reach {abs(Lr - self.L3):.4f}m")
        
        phi = math.acos((c**2 - self.L3**2 - Lr**2) / (-2 * Lr * self.L3))
        gamma = math.atan2(h, r)
        alpha = math.acos((self.L3**2 - Lr**2 - c**2) / (-2 * Lr * c))
        
        # q2 (Shoulder) and q3 (Elbow)
        if elbow == 'up':
            q[1] = math.pi / 2 - beta - alpha - gamma
            q[2] = math.pi - psi - phi
        else:  # elbow == 'down'
            q[1] = math.pi / 2 - (gamma - alpha + beta)
            q[2] = -math.pi + (phi - psi)
        
        # q4 (Wrist)
        # Angle between approach vector and vertical
        angA = math.atan2(math.sqrt(R[1, 2]**2 + R[0, 2]**2), R[2, 2])
        q[3] = angA - q[1] - math.pi / 2 - q[2]
        
        return q
    
    def angles_to_servo_positions(self, q):
        """
        Convert joint angles (radians) to servo positions (0-4095).
        
        Args:
            q: numpy array [q1, q2, q3, q4] in radians
            q1: base rotation (0 = right/+X, π/2 = forward/+Y, π = left/-X)
            
        Returns:
            positions: numpy array [pos1, pos2, pos3, pos4] in servo units (0-4095)
        
        Base servo mapping (inverse relationship):
        - q1 = 0° (right, +X) → servo 999
        - q1 = 90° (forward, +Y) → servo 2048
        - q1 = 180° (left, -X) → servo 3093
        """
        positions = np.zeros(4, dtype=int)
        
        # Base rotation (q1) - special mapping
        # IK solver: q1 = atan2(Y, X)
        #   q1 = 0 → +X (right)
        #   q1 = π/2 → +Y (forward)
        #   q1 = π → -X (left)
        #
        # Servo mapping (from user specification):
        #   servo 999 = 180° (right, +X)
        #   servo 2048 = 90° (forward, +Y)
        #   servo 3093 = 0° (left, -X)
        #
        # Mapping: q1=0→servo 999, q1=π/2→servo 2048, q1=π→servo 3093
        # Linear interpolation: servo = 999 - (q1 / π) * (999 - 3093)
        q1 = q[0]
        if q1 < 0:
            q1 += 2 * math.pi  # Normalize to 0-2π
        if q1 > math.pi:
            # For q1 > π, wrap around (q1=3π/2 maps to q1=-π/2)
            q1 = q1 - 2 * math.pi
        
        # Map q1 to servo position
        # q1=0 → servo 999, q1=π/2 → servo 2048, q1=π → servo 3093
        positions[0] = int(999 - (q1 / math.pi) * (999 - 3093))
        
        # DEBUG: Print servo position calculation
        print(f"    [DEBUG] Servo Position Calculation:")
        print(f"      q1 (normalized) = {q1:.4f} rad = {math.degrees(q1):.1f}°")
        print(f"      Base servo position = {positions[0]} (expected: 999=0°, 2048=90°, 3093=180°)")
        
        # Other joints: standard mapping (2048 = 0 radians)
        for i in range(1, 4):
            positions[i] = int((q[i] / (2 * math.pi)) * 4096 + 2048)
        
        return positions
    
    def servo_positions_to_angles(self, positions):
        """
        Convert servo positions (0-4095) to joint angles (radians).
        
        Args:
            positions: numpy array [pos1, pos2, pos3, pos4] in servo units
            
        Returns:
            q: numpy array [q1, q2, q3, q4] in radians
        
        Base servo mapping (inverse):
        - servo 999 → q1 = 0° (right, +X)
        - servo 2048 → q1 = 90° (forward, +Y)
        - servo 3093 → q1 = 180° (left, -X)
        """
        q = np.zeros(4)
        
        # Base rotation: inverse mapping
        # servo = 999 + (q1 / π) * 2094
        # Solving: q1 = (servo - 999) / 2094 * π
        pos_base = positions[0]
        q[0] = ((pos_base - 999) / 2094) * math.pi
        # Normalize to -π to π range
        if q[0] > math.pi:
            q[0] -= 2 * math.pi
        
        # Other joints: standard mapping (2048 = 0 radians)
        for i in range(1, 4):
            q[i] = ((positions[i] - 2048) / 4096) * 2 * math.pi
        
        return q


def create_pose_matrix(position, orientation='horizontal'):
    """
    Create a 4x4 homogeneous transformation matrix.
    
    Args:
        position: [x, y, z] in meters (robot base frame)
        orientation: 'horizontal' (gripper parallel to ground) or 'vertical'
        
    Returns:
        T: 4x4 homogeneous transformation matrix
    """
    T = np.eye(4)
    T[0:3, 3] = position
    
    if orientation == 'horizontal':
        # Gripper parallel to ground, approaching from side
        # Z-axis points down, X-axis points forward, Y-axis points left
        T[0:3, 0] = [1, 0, 0]  # X-axis (forward)
        T[0:3, 1] = [0, 1, 0]  # Y-axis (left)
        T[0:3, 2] = [0, 0, 1]  # Z-axis (down)
    elif orientation == 'vertical':
        # Gripper vertical, approaching from top
        T[0:3, 0] = [1, 0, 0]
        T[0:3, 1] = [0, 0, -1]
        T[0:3, 2] = [0, 1, 0]
    else:
        raise ValueError("orientation must be 'horizontal' or 'vertical'")
    
    return T


if __name__ == '__main__':
    # Test IK solver
    ik = PincherX100IK()
    
    # Test position: 15cm forward, 10cm left, 10cm above base
    test_position = [0.15, 0.10, 0.10]
    T = create_pose_matrix(test_position, orientation='horizontal')
    
    try:
        q = ik.solve(T, elbow='up')
        print(f"Test position: {test_position}")
        print(f"Joint angles (rad): {q}")
        print(f"Joint angles (deg): {np.degrees(q)}")
        
        positions = ik.angles_to_servo_positions(q)
        print(f"Servo positions: {positions}")
    except ValueError as e:
        print(f"Error: {e}")

