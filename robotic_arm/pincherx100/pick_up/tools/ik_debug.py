#!/usr/bin/env python3
"""
Inverse Kinematics Debugging Tool

Comprehensive tool to debug the IK calculation process from Base[ADJ] coordinates
to servo positions. Shows all intermediate steps, multiple solutions, and verification.

Usage:
    # Manual mode - input coordinates
    python ik_debug.py --mode manual --x 0.10 --y 0.15 --z 0.05
    python tools/ik_debug.py --mode manual --x 10 --y 15 --z 5 --unit cm
    
    Default unit is meters.
    
    # Live feed mode - from AprilTag detection
    python ik_debug.py --mode live_feed --size 0.0254

Modes:
    manual: Input X, Y, Z coordinates manually (in meters or cm)
    live_feed: Detect AprilTags and calculate IK for each detected tag
"""

import sys
import os
import argparse
import cv2
import numpy as np
import yaml
import math
from datetime import datetime

# Add parent directories to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'vision'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from inverse_kinematics import PincherX100IK, create_pose_matrix
from forward_kinematics import PincherX100FK
from apriltag_detector import AprilTagDetector
from coordinate_transform import CoordinateTransformer


class IKDebugger:
    """Comprehensive IK debugging tool."""
    
    def __init__(self, config_file=None):
        """Initialize IK debugger with robot configuration."""
        base_dir = os.path.join(os.path.dirname(__file__), '..')
        
        if config_file is None:
            config_file = os.path.join(base_dir, 'configs', 'robot_config.yaml')
        
        if not os.path.exists(config_file):
            raise FileNotFoundError(f"Config file not found: {config_file}")
        
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Load robot dimensions
        dims = self.config['robot_dimensions']
        self.L1 = dims['L1']
        self.L2 = dims['L2']
        self.L3 = dims['L3']
        self.L4 = dims['L4']
        self.Lm = dims['Lm']
        
        # Load position limits
        self.position_limits = self.config['position_limits']
        
        # Initialize IK and FK solvers
        self.ik = PincherX100IK(L1=self.L1, L2=self.L2, L3=self.L3, L4=self.L4, Lm=self.Lm)
        self.fk = PincherX100FK(L1=self.L1, L2=self.L2, L3=self.L3, L4=self.L4, Lm=self.Lm)
        
        print("="*80)
        print("IK DEBUGGER INITIALIZED")
        print("="*80)
        print(f"Robot Dimensions:")
        print(f"  L1 (base height): {self.L1*1000:.1f}mm")
        print(f"  L2 (shoulder-elbow): {self.L2*1000:.1f}mm")
        print(f"  L3 (elbow-wrist): {self.L3*1000:.1f}mm")
        print(f"  L4 (wrist-TCP): {self.L4*1000:.1f}mm")
        print(f"  Lm (shoulder offset): {self.Lm*1000:.1f}mm")
        Lr = math.sqrt(self.Lm**2 + self.L2**2)
        print(f"  Lr (combined L2+Lm): {Lr*1000:.1f}mm")
        print()
        print(f"Position Limits:")
        for joint, limits in self.position_limits.items():
            print(f"  {joint}: {limits[0]} - {limits[1]}")
        print()
        max_reach_strict = Lr + self.L3
        max_reach_conservative = max_reach_strict * 1.5
        print(f"WORKSPACE EXPLANATION:")
        print(f"  The arm can reach positions in a 3D workspace:")
        print(f"  - Base rotation (q1) allows reaching in ANY horizontal direction (360°)")
        print(f"  - Maximum horizontal reach (strict): Lr + L3 = {max_reach_strict*100:.2f}cm")
        print(f"  - Maximum horizontal reach (conservative, with 1.5x safety): {max_reach_conservative*100:.2f}cm")
        print(f"    (IK solver uses conservative limit to account for shoulder rotation)")
        print(f"  - Maximum vertical reach: L1 + conservative_max = {((self.L1 + max_reach_conservative)*100):.2f}cm")
        print(f"  - Maximum 3D reach: sqrt(conservative_max² + L1²) = {math.sqrt(max_reach_conservative**2 + self.L1**2)*100:.2f}cm")
        print(f"  - The 2R mechanism (shoulder-elbow) has max reach: {max_reach_strict*100:.2f}cm (strict)")
        print(f"    or {max_reach_conservative*100:.2f}cm (conservative) in its own plane")
        print(f"    which can be oriented by base and shoulder rotation")
        print("="*80)
    
    def calculate_reachability(self, position):
        """
        Calculate workspace reachability metrics.
        
        The workspace is a 3D volume:
        - Base can rotate 360° (or within joint limits), allowing reach in any horizontal direction
        - Maximum horizontal reach: Lr + L3 (when arm fully extended horizontally)
        - Maximum vertical reach: L1 + Lr + L3 (when arm fully extended upward)
        - Maximum 3D reach: sqrt((Lr+L3)² + L1²) (diagonal, fully extended)
        
        For a given position, we check:
        1. Horizontal distance r = sqrt(x² + y²) <= Lr + L3
        2. Height z is within reachable range
        3. The 2R mechanism (shoulder-elbow) can reach the distance c = sqrt(r² + h²)
        """
        x, y, z = position
        r = math.sqrt(x**2 + y**2)  # Horizontal distance from base center
        h = z - self.L1  # Height relative to base platform (h=0 at platform level)
        
        c = math.sqrt(r**2 + h**2)  # Distance from shoulder to wrist (2R mechanism)
        
        beta = math.atan2(self.Lm, self.L2)
        Lr = math.sqrt(self.Lm**2 + self.L2**2)
        
        # Maximum reach in shoulder-elbow plane (2R mechanism)
        # Note: IK solver uses 1.5x safety factor to account for shoulder rotation
        max_reach_2r_strict = Lr + self.L3
        max_reach_2r_conservative = max_reach_2r_strict * 1.5  # Match IK solver
        min_reach_2r = abs(Lr - self.L3)
        
        # Maximum horizontal reach (base can rotate to point in any direction)
        # Use conservative limit to match IK solver behavior
        max_horizontal_reach = max_reach_2r_conservative
        
        # Maximum vertical reach (arm fully extended upward)
        max_vertical_reach = self.L1 + max_reach_2r_conservative
        
        # Maximum 3D reach (diagonal, fully extended)
        max_3d_reach = math.sqrt(max_reach_2r_conservative**2 + self.L1**2)
        
        # Check if position is within workspace
        # 1. Horizontal reach check
        horizontal_reachable = r <= max_horizontal_reach
        
        # 2. Height check (must be above minimum Z, below maximum)
        # Use conservative limit for height range
        min_z = self.L1 - max_reach_2r_conservative  # Can reach below base if extended down
        max_z = self.L1 + max_reach_2r_conservative  # Can reach above base if extended up
        height_reachable = min_z <= z <= max_z
        
        # 3. 2R mechanism reachability (triangle inequality)
        # Use conservative limit to match IK solver (allows shoulder rotation)
        triangle_valid = min_reach_2r <= c <= max_reach_2r_conservative
        
        # Overall reachability
        reachable = horizontal_reachable and height_reachable and triangle_valid
        
        return {
            'r': r,
            'h': h,
            'c': c,
            'max_reach_2r': max_reach_2r_conservative,  # Conservative limit (matches IK solver)
            'max_reach_2r_strict': max_reach_2r_strict,  # Strict limit (theoretical max)
            'min_reach_2r': min_reach_2r,
            'max_horizontal_reach': max_horizontal_reach,  # Max horizontal distance
            'max_vertical_reach': max_vertical_reach,  # Max height
            'max_3d_reach': max_3d_reach,  # Max 3D distance
            'min_z': min_z,
            'max_z': max_z,
            'horizontal_reachable': horizontal_reachable,
            'height_reachable': height_reachable,
            'triangle_valid': triangle_valid,
            'reachable': reachable,
            'Lr': Lr
        }
    
    def check_joint_limits(self, servo_positions):
        """Check if servo positions are within limits."""
        limits = {
            'base': self.position_limits['base'],
            'shoulder': self.position_limits['shoulder'],
            'elbow': self.position_limits['elbow'],
            'wrist': self.position_limits['wrist']
        }
        
        results = {}
        joint_names = ['base', 'shoulder', 'elbow', 'wrist']
        
        for i, name in enumerate(joint_names):
            pos = servo_positions[i]
            min_pos, max_pos = limits[name]
            results[name] = {
                'position': pos,
                'min': min_pos,
                'max': max_pos,
                'within_limits': min_pos <= pos <= max_pos
            }
        
        return results
    
    def solve_ik_detailed(self, position, orientation='horizontal'):
        """
        Solve IK with detailed step-by-step output.
        
        Returns:
            dict with all calculation details
        """
        print("\n" + "="*80)
        print("STEP 1: INPUT POSITION")
        print("="*80)
        print(f"Target Position (Base Frame):")
        print(f"  X: {position[0]*100:.2f}cm ({position[0]:.4f}m)")
        print(f"  Y: {position[1]*100:.2f}cm ({position[1]:.4f}m)")
        print(f"  Z: {position[2]*100:.2f}cm ({position[2]:.4f}m)")
        print(f"Orientation: {orientation}")
        if orientation == 'horizontal':
            print(f"  Note: 'horizontal' orientation means gripper parallel to ground,")
            print(f"        but approach vector is VERTICAL (for top-down picking).")
            print(f"        For side-picking, wrist position calculation may differ.")
        
        # Step 2: Create pose matrix
        print("\n" + "="*80)
        print("STEP 2: CREATE POSE MATRIX")
        print("="*80)
        T = create_pose_matrix(position, orientation=orientation)
        print("4x4 Homogeneous Transformation Matrix:")
        print(T)
        print(f"\nPosition vector: [{T[0,3]:.4f}, {T[1,3]:.4f}, {T[2,3]:.4f}]")
        print(f"Rotation matrix (approach vector = Z-axis):")
        print(T[0:3, 0:3])
        
        # Step 3: Workspace reachability check
        print("\n" + "="*80)
        print("STEP 3: WORKSPACE REACHABILITY CHECK")
        print("="*80)
        
        # Calculate wrist position (end-effector moved back L4)
        R = T[0:3, 0:3]
        a = R[:, 2]  # Approach vector
        p = T[0:3, 3]
        w = p - self.L4 * a  # Wrist position
        
        # Check reachability based on wrist position
        r_wrist = math.sqrt(w[0]**2 + w[1]**2)
        h_wrist = w[2] - self.L1
        c_wrist = math.sqrt(r_wrist**2 + h_wrist**2)
        
        beta = math.atan2(self.Lm, self.L2)
        Lr = math.sqrt(self.Lm**2 + self.L2**2)
        max_reach = Lr + self.L3
        min_reach = abs(Lr - self.L3)
        reachable = min_reach <= c_wrist <= max_reach
        
        print(f"End-effector position: [{position[0]*100:.2f}, {position[1]*100:.2f}, {position[2]*100:.2f}] cm")
        print(f"Wrist position (moved back L4={self.L4*1000:.1f}mm): [{w[0]*100:.2f}, {w[1]*100:.2f}, {w[2]*100:.2f}] cm")
        
        # Calculate reachability for end-effector (for display)
        reach_ee = self.calculate_reachability(position)
        
        # Calculate reachability for wrist position (for IK)
        reach_wrist = self.calculate_reachability(w)
        
        # Use wrist-based values for 2R mechanism checks
        r_wrist = math.sqrt(w[0]**2 + w[1]**2)
        h_wrist = w[2] - self.L1
        c_wrist = math.sqrt(r_wrist**2 + h_wrist**2)
        
        print(f"\nWORKSPACE ANALYSIS (End-Effector):")
        print(f"  Horizontal distance (r): {reach_ee['r']*100:.2f}cm")
        print(f"  Height relative to base (h): {reach_ee['h']*100:.2f}cm")
        print(f"  2R mechanism distance (c): {reach_ee['c']*100:.2f}cm")
        
        print(f"\nWORKSPACE ANALYSIS (Wrist - for IK):")
        print(f"  Horizontal distance (r): {r_wrist*100:.2f}cm")
        print(f"  Height relative to base (h): {h_wrist*100:.2f}cm")
        print(f"  2R mechanism distance (c): {c_wrist*100:.2f}cm")
        
        print(f"\nWORKSPACE LIMITS (considering base rotation and 3D reach):")
        print(f"  Max horizontal reach: {reach_ee['max_horizontal_reach']*100:.2f}cm (in any direction)")
        print(f"  Max vertical reach: {reach_ee['max_vertical_reach']*100:.2f}cm (above base)")
        print(f"  Max 3D reach: {reach_ee['max_3d_reach']*100:.2f}cm (diagonal)")
        print(f"  Height range: {reach_ee['min_z']*100:.2f}cm to {reach_ee['max_z']*100:.2f}cm")
        print(f"  2R mechanism range: {reach_ee['min_reach_2r']*100:.2f}cm to {reach_ee['max_reach_2r']*100:.2f}cm")
        
        print(f"\nREACHABILITY CHECKS (End-Effector):")
        print(f"  Horizontal: {'✓ YES' if reach_ee['horizontal_reachable'] else '✗ NO'} "
              f"(r={reach_ee['r']*100:.2f}cm <= {reach_ee['max_horizontal_reach']*100:.2f}cm)")
        print(f"  Height: {'✓ YES' if reach_ee['height_reachable'] else '✗ NO'} "
              f"(Z={position[2]*100:.2f}cm in range [{reach_ee['min_z']*100:.2f}, {reach_ee['max_z']*100:.2f}]cm)")
        
        # Check 2R mechanism based on wrist position
        triangle_valid_wrist = reach_ee['min_reach_2r'] <= c_wrist <= reach_ee['max_reach_2r']
        print(f"  2R mechanism (wrist): {'✓ YES' if triangle_valid_wrist else '✗ NO'} "
              f"(c={c_wrist*100:.2f}cm in range [{reach_ee['min_reach_2r']*100:.2f}, {reach_ee['max_reach_2r']*100:.2f}]cm)")
        
        overall_reachable = reach_ee['horizontal_reachable'] and reach_ee['height_reachable'] and triangle_valid_wrist
        print(f"  Overall: {'✓ REACHABLE' if overall_reachable else '✗ UNREACHABLE'}")
        
        if not overall_reachable:
            print(f"\n⚠ WARNING: Position is UNREACHABLE!")
            if not reach_ee['horizontal_reachable']:
                excess = (reach_ee['r'] - reach_ee['max_horizontal_reach']) * 100
                print(f"  ✗ Horizontal distance {reach_ee['r']*100:.2f}cm exceeds max {reach_ee['max_horizontal_reach']*100:.2f}cm by {excess:.2f}cm")
                print(f"    → Move closer to base center")
            if not reach_ee['height_reachable']:
                if position[2] < reach_ee['min_z']:
                    deficit = (reach_ee['min_z'] - position[2]) * 100
                    print(f"  ✗ Height {position[2]*100:.2f}cm is below minimum {reach_ee['min_z']*100:.2f}cm by {deficit:.2f}cm")
                else:
                    excess = (position[2] - reach_ee['max_z']) * 100
                    print(f"  ✗ Height {position[2]*100:.2f}cm exceeds maximum {reach_ee['max_z']*100:.2f}cm by {excess:.2f}cm")
                print(f"    → Adjust height")
            if not triangle_valid_wrist:
                if c_wrist > reach_ee['max_reach_2r']:
                    excess = (c_wrist - reach_ee['max_reach_2r']) * 100
                    print(f"  ✗ 2R distance {c_wrist*100:.2f}cm exceeds max {reach_ee['max_reach_2r']*100:.2f}cm by {excess:.2f}cm")
                    print(f"    → Combination of horizontal and height is too far")
                else:
                    deficit = (reach_ee['min_reach_2r'] - c_wrist) * 100
                    print(f"  ✗ 2R distance {c_wrist*100:.2f}cm is less than min {reach_ee['min_reach_2r']*100:.2f}cm by {deficit:.2f}cm")
                    print(f"    → Too close to base")
        
        # Store values for IK solving (include all keys needed for summary)
        reach = {
            'wrist_pos': w,
            'r': r_wrist,
            'h': h_wrist,
            'c': c_wrist,
            'min_reach_2r': reach_ee['min_reach_2r'],
            'max_reach_2r': reach_ee['max_reach_2r'],
            'max_horizontal_reach': reach_ee['max_horizontal_reach'],
            'max_vertical_reach': reach_ee['max_vertical_reach'],
            'max_3d_reach': reach_ee['max_3d_reach'],
            'min_z': reach_ee['min_z'],
            'max_z': reach_ee['max_z'],
            'reachable': overall_reachable,
            'Lr': reach_ee['Lr']
        }
        
        # Step 4: Solve IK for both elbow configurations
        print("\n" + "="*80)
        print("STEP 4: INVERSE KINEMATICS SOLVING")
        print("="*80)
        
        solutions = {}
        
        for elbow_config in ['up', 'down']:
            print(f"\n--- Elbow {elbow_config.upper()} Configuration ---")
            try:
                # Extract position and rotation for manual calculation
                p = T[0:3, 3]
                R = T[0:3, 0:3]
                
                # q1: Base rotation
                q1 = math.atan2(p[1], p[0])
                print(f"q1 (Base): atan2(Y={p[1]:.4f}, X={p[0]:.4f}) = {q1:.4f} rad = {math.degrees(q1):.1f}°")
                
                # Wrist decoupling (use already calculated wrist position from reachability check)
                w = reach['wrist_pos']
                print(f"Wrist position (moved back L4={self.L4*1000:.1f}mm):")
                print(f"  W = [{w[0]:.4f}, {w[1]:.4f}, {w[2]:.4f}]")
                
                # Calculate 2R mechanism parameters from wrist position
                r = math.sqrt(w[0]**2 + w[1]**2)  # Horizontal distance from base
                h = w[2] - self.L1  # Height relative to base platform
                c = math.sqrt(r**2 + h**2)  # Distance from shoulder to wrist
                
                # Get reach limits
                min_reach = reach['min_reach_2r']
                max_reach = reach['max_reach_2r']
                
                print(f"2R mechanism parameters:")
                print(f"  r (horizontal): {r*100:.2f}cm")
                print(f"  h (vertical): {h*100:.2f}cm")
                print(f"  c (distance): {c*100:.2f}cm")
                
                beta = math.atan2(self.Lm, self.L2)
                psi = math.pi / 2 - beta
                Lr = math.sqrt(self.Lm**2 + self.L2**2)
                
                print(f"Geometric parameters:")
                print(f"  beta = atan2(Lm={self.Lm*1000:.1f}mm, L2={self.L2*1000:.1f}mm) = {math.degrees(beta):.2f}°")
                print(f"  psi = π/2 - beta = {math.degrees(psi):.2f}°")
                print(f"  Lr = sqrt(Lm² + L2²) = {Lr*1000:.1f}mm")
                
                # Law of cosines
                cos_phi = (c**2 - self.L3**2 - Lr**2) / (-2 * Lr * self.L3)
                cos_alpha = (self.L3**2 - Lr**2 - c**2) / (-2 * Lr * c)
                
                print(f"Law of cosines calculations:")
                print(f"  cos(phi) = (c²-L3²-Lr²)/(-2LrL3) = {cos_phi:.4f}")
                print(f"  cos(alpha) = (L3²-Lr²-c²)/(-2Lrc) = {cos_alpha:.4f}")
                
                # Check bounds for acos() - must be in [-1, 1]
                # Note: max_reach here is the conservative limit, but triangle inequality
                # requires the strict limit (Lr + L3)
                strict_max_reach = Lr + self.L3
                
                if abs(cos_phi) > 1.0:
                    error_msg = f"Position UNREACHABLE: cos(phi)={cos_phi:.4f} outside [-1, 1]"
                    error_msg += f"\n  This means: c={c*100:.2f}cm violates triangle inequality"
                    error_msg += f"\n  Strict limit: {min_reach*100:.2f}cm <= c <= {strict_max_reach*100:.2f}cm"
                    error_msg += f"\n  Conservative limit: <= {max_reach*100:.2f}cm (for workspace check only)"
                    error_msg += f"\n  Actual: c={c*100:.2f}cm"
                    if c > strict_max_reach:
                        error_msg += f"\n  ✗ Exceeds strict max reach by {(c-strict_max_reach)*100:.2f}cm"
                        error_msg += f"\n  → Wrist position is too far (likely due to approach vector direction)"
                    elif c < min_reach:
                        error_msg += f"\n  ✗ Below min reach by {(min_reach-c)*100:.2f}cm"
                        error_msg += f"\n  → Wrist position is too close to base"
                    else:
                        error_msg += f"\n  ✗ Triangle inequality violated (geometry doesn't work out)"
                    raise ValueError(error_msg)
                
                if abs(cos_alpha) > 1.0:
                    error_msg = f"Position UNREACHABLE: cos(alpha)={cos_alpha:.4f} outside [-1, 1]"
                    error_msg += f"\n  This means: c={c*100:.2f}cm violates triangle inequality"
                    error_msg += f"\n  Strict limit: {min_reach*100:.2f}cm <= c <= {strict_max_reach*100:.2f}cm"
                    error_msg += f"\n  Conservative limit: <= {max_reach*100:.2f}cm (for workspace check only)"
                    error_msg += f"\n  Actual: c={c*100:.2f}cm"
                    if c > strict_max_reach:
                        error_msg += f"\n  ✗ Exceeds strict max reach by {(c-strict_max_reach)*100:.2f}cm"
                        error_msg += f"\n  → Wrist position is too far (likely due to approach vector direction)"
                    elif c < min_reach:
                        error_msg += f"\n  ✗ Below min reach by {(min_reach-c)*100:.2f}cm"
                        error_msg += f"\n  → Wrist position is too close to base"
                    else:
                        error_msg += f"\n  ✗ Triangle inequality violated (geometry doesn't work out)"
                    raise ValueError(error_msg)
                
                phi = math.acos(cos_phi)
                gamma = math.atan2(h, r)
                alpha = math.acos(cos_alpha)
                
                print(f"Angles:")
                print(f"  phi = acos({cos_phi:.4f}) = {math.degrees(phi):.2f}°")
                print(f"  gamma = atan2(h={h*100:.2f}cm, r={r*100:.2f}cm) = {math.degrees(gamma):.2f}°")
                print(f"  alpha = acos({cos_alpha:.4f}) = {math.degrees(alpha):.2f}°")
                
                # Calculate q2 and q3
                if elbow_config == 'up':
                    q2 = math.pi / 2 - beta - alpha - gamma
                    q3 = math.pi - psi - phi
                else:
                    q2 = math.pi / 2 - (gamma - alpha + beta)
                    q3 = -math.pi + (phi - psi)
                
                print(f"q2 (Shoulder): {q2:.4f} rad = {math.degrees(q2):.1f}°")
                print(f"q3 (Elbow): {q3:.4f} rad = {math.degrees(q3):.1f}°")
                
                # q4: Wrist
                angA = math.atan2(math.sqrt(R[1, 2]**2 + R[0, 2]**2), R[2, 2])
                q4 = angA - q2 - math.pi / 2 - q3
                print(f"q4 (Wrist): {q4:.4f} rad = {math.degrees(q4):.1f}°")
                print(f"  (angA = {math.degrees(angA):.2f}°, q2+q3 = {math.degrees(q2+q3):.2f}°)")
                
                q = np.array([q1, q2, q3, q4])
                
                # Convert to servo positions
                print(f"\nServo Position Conversion:")
                # Base (special mapping)
                q1_norm = q1
                if q1_norm < 0:
                    q1_norm += 2 * math.pi
                if q1_norm > math.pi:
                    q1_norm = q1_norm - 2 * math.pi
                
                base_servo = int(999 - (q1_norm / math.pi) * (999 - 3093))
                print(f"  Base: q1={math.degrees(q1):.1f}° → servo={base_servo}")
                print(f"    (Mapping: 0°→999, 90°→2048, 180°→3093)")
                
                servo_positions = np.array([
                    base_servo,
                    int((q[1] / (2 * math.pi)) * 4096 + 2048),
                    int((q[2] / (2 * math.pi)) * 4096 + 2048),
                    int((q[3] / (2 * math.pi)) * 4096 + 2048)
                ])
                
                print(f"  Shoulder: q2={math.degrees(q[1]):.1f}° → servo={servo_positions[1]}")
                print(f"  Elbow: q3={math.degrees(q[2]):.1f}° → servo={servo_positions[2]}")
                print(f"  Wrist: q4={math.degrees(q[3]):.1f}° → servo={servo_positions[3]}")
                
                # Check joint limits
                print(f"\nJoint Limit Check:")
                limit_check = self.check_joint_limits(servo_positions)
                all_within = True
                for name, check in limit_check.items():
                    status = "✓" if check['within_limits'] else "✗"
                    print(f"  {status} {name.capitalize()}: {check['position']} "
                          f"(limits: {check['min']}-{check['max']})")
                    if not check['within_limits']:
                        all_within = False
                
                solutions[elbow_config] = {
                    'q': q,
                    'servo_positions': servo_positions,
                    'within_limits': all_within,
                    'valid': True
                }
                
            except ValueError as e:
                print(f"✗ IK Solution FAILED: {e}")
                solutions[elbow_config] = {
                    'valid': False,
                    'error': str(e)
                }
        
        # Step 5: Forward kinematics verification
        print("\n" + "="*80)
        print("STEP 5: FORWARD KINEMATICS VERIFICATION")
        print("="*80)
        
        for elbow_config, sol in solutions.items():
            if not sol['valid']:
                continue
            
            print(f"\n--- Verifying {elbow_config.upper()} solution ---")
            q = sol['q']
            
            # Use FK to calculate actual end-effector position
            T_actual = self.fk.solve(q)
            p_actual = T_actual[0:3, 3]
            
            print(f"Calculated end-effector position:")
            print(f"  X: {p_actual[0]*100:.2f}cm ({p_actual[0]:.4f}m)")
            print(f"  Y: {p_actual[1]*100:.2f}cm ({p_actual[1]:.4f}m)")
            print(f"  Z: {p_actual[2]*100:.2f}cm ({p_actual[2]:.4f}m)")
            
            # Calculate error
            error = np.linalg.norm(p_actual - position)
            error_x = abs(p_actual[0] - position[0])
            error_y = abs(p_actual[1] - position[1])
            error_z = abs(p_actual[2] - position[2])
            
            print(f"\nPosition Error:")
            print(f"  X error: {error_x*100:.2f}cm ({error_x*1000:.2f}mm)")
            print(f"  Y error: {error_y*100:.2f}cm ({error_y*1000:.2f}mm)")
            print(f"  Z error: {error_z*100:.2f}cm ({error_z*1000:.2f}mm)")
            print(f"  Total 3D error: {error*100:.2f}cm ({error*1000:.2f}mm)")
            
            if error < 0.001:  # 1mm
                print(f"  ✓ Excellent accuracy (< 1mm)")
            elif error < 0.005:  # 5mm
                print(f"  ✓ Good accuracy (< 5mm)")
            elif error < 0.01:  # 1cm
                print(f"  ⚠ Acceptable accuracy (< 1cm)")
            else:
                print(f"  ✗ Poor accuracy (> 1cm)")
            
            sol['fk_position'] = p_actual
            sol['error'] = error
            sol['error_components'] = [error_x, error_y, error_z]
        
        # Summary
        print("\n" + "="*80)
        print("SUMMARY")
        print("="*80)
        print(f"Target Position: X={position[0]*100:.2f}cm, Y={position[1]*100:.2f}cm, Z={position[2]*100:.2f}cm")
        print(f"Workspace: {'✓ REACHABLE' if reach['reachable'] else '✗ UNREACHABLE'}")
        
        # Compare with config value
        config_max_reach = self.config.get('workspace', {}).get('max_reach', None)
        if config_max_reach:
            print(f"\nNote: Config max_reach = {config_max_reach*100:.2f}cm")
            print(f"  Calculated max horizontal reach = {reach['max_horizontal_reach']*100:.2f}cm")
            if abs(config_max_reach - reach['max_horizontal_reach']) > 0.01:
                print(f"  Difference: {abs(config_max_reach - reach['max_horizontal_reach'])*100:.2f}cm")
                print(f"  (Config may include safety margin or different calculation)")
        print()
        
        for elbow_config, sol in solutions.items():
            if sol['valid']:
                print(f"{elbow_config.upper()} Solution:")
                print(f"  Joint angles: q1={math.degrees(sol['q'][0]):.1f}°, "
                      f"q2={math.degrees(sol['q'][1]):.1f}°, "
                      f"q3={math.degrees(sol['q'][2]):.1f}°, "
                      f"q4={math.degrees(sol['q'][3]):.1f}°")
                print(f"  Servo positions: base={sol['servo_positions'][0]}, "
                      f"shoulder={sol['servo_positions'][1]}, "
                      f"elbow={sol['servo_positions'][2]}, "
                      f"wrist={sol['servo_positions'][3]}")
                print(f"  Within limits: {'✓ YES' if sol['within_limits'] else '✗ NO'}")
                print(f"  FK error: {sol['error']*1000:.2f}mm")
            else:
                print(f"{elbow_config.upper()} Solution: ✗ FAILED - {sol.get('error', 'Unknown error')}")
        
        return {
            'position': position,
            'reachability': reach,
            'solutions': solutions,
            'pose_matrix': T
        }
    
    def run_manual_mode(self, x, y, z, unit='m'):
        """Run in manual mode with specified coordinates."""
        if unit == 'cm':
            x, y, z = x/100, y/100, z/100
        
        position = np.array([x, y, z])
        return self.solve_ik_detailed(position)
    
    def run_live_feed_mode(self, tag_size_m, camera_index=0):
        """Run in live feed mode with AprilTag detection."""
        base_dir = os.path.join(os.path.dirname(__file__), '..')
        calib_file = os.path.join(base_dir, 'calibration', 'camera_calibration.npz')
        config_file = os.path.join(base_dir, 'configs', 'robot_config.yaml')
        
        # Load calibration
        calib_data = np.load(calib_file)
        camera_matrix = calib_data['camera_matrix']
        dist_coeffs = calib_data['dist_coeffs']
        
        # Initialize detector and transformer
        tag_detector = AprilTagDetector(camera_matrix, dist_coeffs, tag_size_m)
        transformer = CoordinateTransformer(calib_file, config_file)
        
        # Get offsets
        coord_config = self.config.get('coordinate_transform', {})
        x_offset = coord_config.get('x_offset', 0.0)
        y_offset = coord_config.get('y_offset', 0.0)
        z_offset = coord_config.get('z_offset', 0.0)
        
        # Open camera
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open camera {camera_index}")
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        print("\n" + "="*80)
        print("LIVE FEED MODE - AprilTag Detection")
        print("="*80)
        print("Press Q to quit, S to save frame")
        print("Detected tags will trigger IK calculation")
        print("="*80)
        
        frame_count = 0
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                detections = tag_detector.detect_tags(frame)
                
                if len(detections) > 0:
                    print(f"\n{'='*80}")
                    print(f"FRAME {frame_count} - {len(detections)} tag(s) detected")
                    print(f"{'='*80}")
                    
                    for det in detections:
                        tag_id = det['tag_id']
                        pos_camera = det['position_camera']
                        
                        # Transform to base frame
                        pos_base_raw = transformer.camera_3d_to_base_3d(pos_camera)
                        
                        # Apply offsets
                        pos_base_adj = pos_base_raw.copy()
                        pos_base_adj[0] += x_offset
                        pos_base_adj[1] += y_offset
                        pos_base_adj[2] += z_offset
                        
                        print(f"\nTag ID {tag_id}:")
                        print(f"  Camera frame: [{pos_camera[0]*100:.2f}, {pos_camera[1]*100:.2f}, {pos_camera[2]*100:.2f}] cm")
                        print(f"  Base[RAW]: [{pos_base_raw[0]*100:.2f}, {pos_base_raw[1]*100:.2f}, {pos_base_raw[2]*100:.2f}] cm")
                        print(f"  Base[ADJ]: [{pos_base_adj[0]*100:.2f}, {pos_base_adj[1]*100:.2f}, {pos_base_adj[2]*100:.2f}] cm")
                        
                        # Calculate IK
                        self.solve_ik_detailed(pos_base_adj)
                
                cv2.imshow('IK Debug - Live Feed (Press Q to quit)', frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q'):
                    break
                elif key == ord('s') or key == ord('S'):
                    filename = f"ik_debug_{frame_count:04d}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"\nSaved frame to {filename}")
                
                frame_count += 1
        
        finally:
            cap.release()
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description='IK Debugging Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Manual mode with coordinates in meters
  python ik_debug.py --mode manual --x 0.10 --y 0.15 --z 0.05
  
  # Manual mode with coordinates in centimeters
  python ik_debug.py --mode manual --x 10 --y 15 --z 5 --unit cm
  
  # Live feed mode
  python ik_debug.py --mode live_feed --size 0.0254
        """
    )
    
    parser.add_argument('--mode', choices=['manual', 'live_feed'], required=True,
                       help='Operation mode')
    parser.add_argument('--x', type=float, default=None,
                       help='X coordinate (for manual mode)')
    parser.add_argument('--y', type=float, default=None,
                       help='Y coordinate (for manual mode)')
    parser.add_argument('--z', type=float, default=None,
                       help='Z coordinate (for manual mode)')
    parser.add_argument('--unit', choices=['m', 'cm'], default='m',
                       help='Unit for manual coordinates (default: m)')
    parser.add_argument('--size', type=float, default=None,
                       help='AprilTag size in meters (for live_feed mode)')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera index (for live_feed mode, default: 0)')
    parser.add_argument('--config', type=str, default=None,
                       help='Path to robot_config.yaml')
    
    args = parser.parse_args()
    
    try:
        debugger = IKDebugger(config_file=args.config)
        
        if args.mode == 'manual':
            if args.x is None or args.y is None or args.z is None:
                print("Error: --x, --y, --z required for manual mode")
                return 1
            
            debugger.run_manual_mode(args.x, args.y, args.z, unit=args.unit)
        
        elif args.mode == 'live_feed':
            if args.size is None:
                # Try to get from config
                config_path = args.config or os.path.join(
                    os.path.dirname(__file__), '..', 'configs', 'robot_config.yaml'
                )
                if os.path.exists(config_path):
                    with open(config_path, 'r') as f:
                        config = yaml.safe_load(f)
                    args.size = config.get('apriltags', {}).get('tag_size', 0.0254)
                    print(f"Using tag size from config: {args.size*1000:.1f}mm")
                else:
                    args.size = 0.0254
                    print(f"Using default tag size: {args.size*1000:.1f}mm")
            
            debugger.run_live_feed_mode(args.size, args.camera)
    
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

