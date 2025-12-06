#!/usr/bin/env python3
"""
FULLY CORRECTED Inverse Kinematics for PincherX 100

Implements BOTH approaches for phi control:
1. Phi as optimization constraint (theta2 + theta3 + theta4 = phi)
2. Phi in target pose orientation (rotation matrix)

Based on corrected DH parameters matching physical robot.
"""

import numpy as np
from scipy.optimize import minimize, NonlinearConstraint
from dynamixel_sdk import *
import time

DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

POSITION_TO_RAD = (2 * np.pi) / 4096
RAD_TO_POSITION = 4096 / (2 * np.pi)

def dh(alpha_prev, a_prev, d, theta):
    """DH transformation matrix"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha_prev)
    sa = np.sin(alpha_prev)
    
    return np.array([
        [ct, -st, 0, a_prev],
        [st*ca, ct*ca, -sa, -sa*d],
        [st*sa, ct*sa, ca, ca*d],
        [0, 0, 0, 1]
    ])

def fk_px100(theta_values):
    """
    CORRECTED Forward kinematics for PincherX 100
    
    Args:
        theta_values: [theta1, theta2, theta3, theta4] in radians
    
    Returns:
        4x4 transformation matrix of end-effector pose
    """
    theta1, theta2, theta3, theta4 = theta_values
    
    # CORRECTED DH parameters
    T1 = dh(0,        0,      0.18945,  theta1)  # Base + upper arm: 189.45mm
    T2 = dh(-np.pi/2, 0,      0,        theta2)  # Shoulder twist
    T3 = dh(0,        0.035,  0,        theta3)  # Elbow offset: 35mm
    T4 = dh(0,        0.1,    0,        theta4)  # Forearm: 100mm
    
    T04 = T1 @ T2 @ T3 @ T4
    
    # Gripper transform: 107.6mm
    T4e = np.array([
        [1, 0, 0, 0.1076],
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    T0e = T04 @ T4e
    return T0e

def create_target_pose_with_phi(x, y, z, phi):
    """
    Create target pose with desired pitch angle
    
    Args:
        x, y, z: Position in meters
        phi: Pitch angle in radians (rotation around Y-axis)
    
    Returns:
        4x4 transformation matrix
    """
    pose = np.eye(4)
    pose[:3, 3] = [x, y, z]
    
    # Rotation around Y-axis for pitch
    c = np.cos(phi)
    s = np.sin(phi)
    R_pitch = np.array([
        [c,  0, s],
        [0,  1, 0],
        [-s, 0, c]
    ])
    
    pose[:3, :3] = R_pitch
    return pose

def pose_error(current_pose, target_pose, weights):
    """Calculate weighted error between poses"""
    # Position error
    pos_current = current_pose[:3, 3]
    pos_target = target_pose[:3, 3]
    pos_error = pos_current - pos_target
    
    # Orientation error (Frobenius norm of rotation difference)
    rot_current = current_pose[:3, :3]
    rot_target = target_pose[:3, :3]
    rot_error_matrix = rot_current @ rot_target.T - np.eye(3)
    rot_error = np.linalg.norm(rot_error_matrix, 'fro')
    
    # Weighted sum
    position_error = np.sqrt(
        weights[0] * pos_error[0]**2 + 
        weights[1] * pos_error[1]**2 + 
        weights[2] * pos_error[2]**2
    )
    
    orientation_error = (weights[3] + weights[4] + weights[5]) * rot_error / 3.0
    
    return position_error + orientation_error

def ik_px100_numerical(target_pose, phi_constraint=None, initial_guess=None, 
                       weights=None, max_iterations=5000, tol=1e-10, 
                       num_restarts=100, verbose=False):
    """
    Numerical inverse kinematics with phi control
    
    Args:
        target_pose: 4x4 transformation matrix or [x, y, z] position
        phi_constraint: Desired phi angle (theta2+theta3+theta4), or None to not constrain
        initial_guess: [theta1, theta2, theta3, theta4] starting guess
        weights: [wx, wy, wz, wroll, wpitch, wyaw]
        max_iterations: Maximum optimization iterations
        tol: Tolerance for convergence
        num_restarts: Number of random restarts
        verbose: Print optimization progress
    
    Returns:
        joint_angles: [theta1, theta2, theta3, theta4] in radians
        success: Boolean indicating if IK succeeded
        final_error: Final position error in meters
    """
    
    # Handle input
    if isinstance(target_pose, (list, tuple, np.ndarray)) and len(target_pose) == 3:
        target_pose_matrix = np.eye(4)
        target_pose_matrix[:3, 3] = target_pose
    else:
        target_pose_matrix = target_pose
    
    # Smart initial guess
    if initial_guess is None:
        target_pos = target_pose_matrix[:3, 3]
        theta1_guess = np.arctan2(target_pos[1], target_pos[0])
        z_rel = target_pos[2] - 0.18945
        r = np.sqrt(target_pos[0]**2 + target_pos[1]**2)
        theta2_guess = np.arctan2(z_rel, r) if r > 0.01 else 0.0
        
        # If phi is constrained, use it to set theta3 and theta4
        if phi_constraint is not None:
            # Simple heuristic: split the remaining angle between theta3 and theta4
            remaining = phi_constraint - theta2_guess
            initial_guess = np.array([theta1_guess, theta2_guess, remaining/2, remaining/2])
        else:
            initial_guess = np.array([theta1_guess, theta2_guess, 0.0, 0.0])
    
    # Weights - heavily prioritize position
    if weights is None:
        weights = np.array([1000.0, 1000.0, 1000.0, 1.0, 1.0, 1.0])
    
    # Joint limits with safety margins
    joint_limits = np.array([
        [-1.55, 1.48],
        [-1.63, 1.50],
        [-1.63, 1.50],
        [-1.81, 2.02]
    ])
    
    # Cost function
    def cost_function(theta):
        current_pose = fk_px100(theta)
        error = pose_error(current_pose, target_pose_matrix, weights)
        
        # Add phi penalty if constrained
        if phi_constraint is not None:
            phi_actual = theta[1] + theta[2] + theta[3]
            phi_error = abs(phi_actual - phi_constraint)
            # Heavy penalty for phi deviation (100x weight)
            error += 10.0 * phi_error
        
        return error
    
    # Bounds
    bounds = [(joint_limits[i, 0], joint_limits[i, 1]) for i in range(4)]
    
    # Constraints list for scipy
    constraints = []
    
    # Add phi constraint if specified (using NonlinearConstraint)
    if phi_constraint is not None:
        def phi_constraint_func(theta):
            return theta[1] + theta[2] + theta[3]
        
        # Allow small tolerance on phi constraint
        phi_tol = 0.05  # 0.05 rad ≈ 3°
        constraints.append(
            NonlinearConstraint(
                phi_constraint_func,
                phi_constraint - phi_tol,
                phi_constraint + phi_tol
            )
        )
    
    best_result = None
    best_error = float('inf')
    
    if verbose:
        print("Running IK optimization...")
        print(f"Target position: {target_pose_matrix[:3, 3]}")
        if phi_constraint is not None:
            print(f"Phi constraint: {np.degrees(phi_constraint):.2f}°")
    
    # Try with initial guess using SLSQP (supports constraints better than L-BFGS-B)
    if constraints:
        # Use SLSQP for constrained optimization
        result = minimize(
            cost_function,
            initial_guess,
            method='SLSQP',
            bounds=bounds,
            constraints=constraints,
            options={'maxiter': max_iterations, 'ftol': tol}
        )
    else:
        # Use L-BFGS-B for unconstrained (faster)
        result = minimize(
            cost_function,
            initial_guess,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': max_iterations, 'ftol': tol, 'gtol': tol}
        )
    
    if result.fun < best_error:
        best_error = result.fun
        best_result = result
    
    # Random restarts
    if num_restarts > 0:
        if verbose:
            print(f"Trying {num_restarts} random restarts...")
        
        for i in range(num_restarts):
            random_guess = np.array([
                np.random.uniform(joint_limits[j, 0], joint_limits[j, 1])
                for j in range(4)
            ])
            
            # For phi constraint, adjust the guess
            if phi_constraint is not None:
                # Force the guess to approximately satisfy phi constraint
                current_phi = random_guess[1] + random_guess[2] + random_guess[3]
                diff = phi_constraint - current_phi
                # Distribute the difference across theta2, theta3, theta4
                random_guess[2] += diff / 2
                random_guess[3] += diff / 2
            
            if constraints:
                result = minimize(
                    cost_function,
                    random_guess,
                    method='SLSQP',
                    bounds=bounds,
                    constraints=constraints,
                    options={'maxiter': max_iterations, 'ftol': tol}
                )
            else:
                result = minimize(
                    cost_function,
                    random_guess,
                    method='L-BFGS-B',
                    bounds=bounds,
                    options={'maxiter': max_iterations, 'ftol': tol, 'gtol': tol}
                )
            
            if result.fun < best_error:
                best_error = result.fun
                best_result = result
                if verbose and i < 10:
                    print(f"  Restart {i+1}: Improved error to {best_error:.6f}")
        
        # Global optimizer backup for difficult cases
        if best_error > 0.05:
            if verbose:
                print("Trying global optimizer (differential_evolution)...")
            
            from scipy.optimize import differential_evolution
            
            result = differential_evolution(
                cost_function,
                bounds,
                maxiter=1000,
                popsize=15,
                tol=tol,
                atol=tol,
                seed=42
            )
            
            if result.fun < best_error:
                best_error = result.fun
                best_result = result
                if verbose:
                    print(f"  Global optimizer improved error to {best_error:.6f}")
    
    if best_result is None:
        return None, False, float('inf')
    
    # Verify solution
    final_pose = fk_px100(best_result.x)
    position_error = np.linalg.norm(final_pose[:3, 3] - target_pose_matrix[:3, 3])
    
    # Check phi constraint if specified
    phi_error = None
    if phi_constraint is not None:
        phi_actual = best_result.x[1] + best_result.x[2] + best_result.x[3]
        phi_error = abs(phi_actual - phi_constraint)
    
    success = position_error < 0.02
    
    if verbose:
        print(f"\nOptimization complete:")
        print(f"  Success: {success}")
        print(f"  Final cost: {best_error:.6e}")
        print(f"  Position error: {position_error*1000:.3f} mm")
        print(f"  Joint angles (deg): {np.degrees(best_result.x)}")
        if phi_constraint is not None:
            print(f"  Target phi: {np.degrees(phi_constraint):.2f}°")
            print(f"  Actual phi: {np.degrees(phi_actual):.2f}°")
            print(f"  Phi error: {np.degrees(phi_error):.2f}°")
    
    return best_result.x, success, position_error

def ik_px100(x, y, z, phi=0, verbose=False, method='numerical'):
    """
    Inverse kinematics with phi control
    
    Args:
        x, y, z: Target position in meters
        phi: End-effector pitch angle in radians (0 = horizontal)
        verbose: Print detailed information
        method: 'numerical' (full), 'fast' (fewer restarts), or 'constrained' (strict phi)
    
    Returns:
        [theta1, theta2, theta3, theta4] in radians, or None if unreachable
    """
    
    # Create target pose with phi orientation
    target_pose = create_target_pose_with_phi(x, y, z, phi)
    
    # Set parameters based on method
    if method == 'fast':
        # Faster but less thorough - no phi constraint, just in cost
        joint_angles, success, error = ik_px100_numerical(
            target_pose,
            phi_constraint=None,  # Phi only in orientation, not constrained
            num_restarts=10,
            max_iterations=1000,
            verbose=verbose
        )
    elif method == 'constrained':
        # Strict phi constraint using NonlinearConstraint
        joint_angles, success, error = ik_px100_numerical(
            target_pose,
            phi_constraint=phi,  # Hard constraint on phi
            num_restarts=50,
            max_iterations=5000,
            verbose=verbose
        )
    else:  # 'numerical' - default
        # Both approaches: phi in pose orientation AND as soft constraint
        joint_angles, success, error = ik_px100_numerical(
            target_pose,
            phi_constraint=phi,  # Soft constraint (penalty in cost function)
            num_restarts=100,
            max_iterations=5000,
            verbose=verbose
        )
    
    if not success:
        print(f"IK failed to converge!")
        print(f"Target: X={x:.4f}, Y={y:.4f}, Z={z:.4f}, phi={np.degrees(phi):.2f}°")
        print(f"Final position error: {error*1000:.3f} mm")
        return None
    
    # Verify phi
    if joint_angles is not None:
        phi_actual = joint_angles[1] + joint_angles[2] + joint_angles[3]
        phi_error = abs(phi_actual - phi)
        
        if phi_error > 0.1:  # More than 5.7°
            print(f"⚠ Warning: Phi error = {np.degrees(phi_error):.2f}°")
            print(f"  Target phi: {np.degrees(phi):.2f}°")
            print(f"  Actual phi: {np.degrees(phi_actual):.2f}°")
    
    return joint_angles

def move_servo(packetHandler, portHandler, servo_id, angle_rad):
    """Move servo to specified angle"""
    position = int(angle_rad * RAD_TO_POSITION + 2048)
    position = max(0, min(4095, position))
    
    packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, position)

if __name__ == "__main__":
    print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))
    
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)
    
    for servo_id in [1, 2, 3, 4]:
        packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, 1)
    
    print("\n" + "="*60)
    print("PincherX 100 Inverse Kinematics with Phi Control")
    print("CORRECTED DH Parameters + Phi Constraint")
    print("="*60)
    
    x = float(input("\nTarget X (m): "))
    y = float(input("Target Y (m): "))
    z = float(input("Target Z (m): "))
    phi_deg = float(input("End-effector pitch angle (degrees, 0=horizontal): ") or 0)
    phi = np.radians(phi_deg)
    
    print("\nChoose method:")
    print("  1. numerical (default) - Both phi constraint + orientation")
    print("  2. constrained - Strict phi constraint only")
    print("  3. fast - Quick solve, phi in orientation only")
    method_choice = input("Enter choice (1/2/3, default=1): ") or "1"
    
    method_map = {'1': 'numerical', '2': 'constrained', '3': 'fast'}
    method = method_map.get(method_choice, 'numerical')
    
    print(f"\nCalculating IK with method='{method}'...")
    joints = ik_px100(x, y, z, phi=phi, verbose=True, method=method)
    
    if joints is None:
        print("\nInverse kinematics failed!")
        portHandler.closePort()
        exit(1)
    
    theta1, theta2, theta3, theta4 = joints
    
    print("\n" + "="*60)
    print("Calculated joint angles:")
    print("="*60)
    print(f"  theta1 = {theta1:.6f} rad ({np.degrees(theta1):8.4f}°)")
    print(f"  theta2 = {theta2:.6f} rad ({np.degrees(theta2):8.4f}°)")
    print(f"  theta3 = {theta3:.6f} rad ({np.degrees(theta3):8.4f}°)")
    print(f"  theta4 = {theta4:.6f} rad ({np.degrees(theta4):8.4f}°)")
    
    phi_actual = theta2 + theta3 + theta4
    print(f"\n  Actual phi = theta2 + theta3 + theta4 = {np.degrees(phi_actual):.2f}°")
    print(f"  Target phi = {phi_deg:.2f}°")
    
    # Verify with FK
    T = fk_px100(joints)
    pos = T[:3, 3]
    
    print("\n" + "="*60)
    print("Verification (forward kinematics):")
    print("="*60)
    print(f"  Target:  X={x:.6f}, Y={y:.6f}, Z={z:.6f}")
    print(f"  Reached: X={pos[0]:.6f}, Y={pos[1]:.6f}, Z={pos[2]:.6f}")
    
    error = np.sqrt((pos[0]-x)**2 + (pos[1]-y)**2 + (pos[2]-z)**2)
    print(f"  Position error: {error:.6f} m = {error*1000:.3f} mm")
    
    # Move the robot
    confirm = input("\nMove robot to calculated position? (y/n): ")
    
    if confirm.lower() == 'y':
        print("\nMoving robot...")
        move_servo(packetHandler, portHandler, 1, theta1)
        move_servo(packetHandler, portHandler, 2, theta2)
        move_servo(packetHandler, portHandler, 3, theta3)
        move_servo(packetHandler, portHandler, 4, theta4)
        
        time.sleep(2)
        print("Movement complete!")
        print(f"\nExpected gripper pitch: {phi_deg:.2f}°")
        print(f"Actual phi from joints: {np.degrees(phi_actual):.2f}°")
    else:
        print("\nMovement cancelled.")
    
    portHandler.closePort()