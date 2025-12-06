#!/usr/bin/env python3
"""
Standalone test for numerical IK (no dynamixel_sdk needed)
"""

import numpy as np
from scipy.optimize import minimize

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
    """Forward kinematics"""
    theta1, theta2, theta3, theta4 = theta_values
    
    T1 = dh(0,          0,          0.0931,     theta1)
    T2 = dh(-np.pi/2,   0,          0,          theta2)
    T3 = dh(0,          0.1059,     0,          theta3)
    T4 = dh(0,          0.1,        0,          theta4)
    
    T04 = T1 @ T2 @ T3 @ T4
    
    T4e = np.array([
        [1, 0, 0, 0.1090],
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    T0e = T04 @ T4e
    return T0e

def pose_error(current_pose, target_pose, weights):
    """Calculate weighted error between poses"""
    # Position error
    pos_current = current_pose[:3, 3]
    pos_target = target_pose[:3, 3]
    pos_error = pos_current - pos_target
    
    # Orientation error
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

def ik_px100_numerical(target_pose, initial_guess=None, weights=None, 
                       max_iterations=5000, tol=1e-10, num_restarts=10,
                       verbose=False):
    """Numerical IK using scipy optimization"""
    
    # Handle input
    if isinstance(target_pose, (list, tuple, np.ndarray)) and len(target_pose) == 3:
        target_pose_matrix = np.eye(4)
        target_pose_matrix[:3, 3] = target_pose
    else:
        target_pose_matrix = target_pose
    
    # Defaults - smart initial guess based on target position
    if initial_guess is None:
        target_pos = target_pose_matrix[:3, 3]
        # Estimate theta1 from target x, y
        theta1_guess = np.arctan2(target_pos[1], target_pos[0])
        initial_guess = np.array([theta1_guess, 0.0, 0.0, 0.0])
    
    if weights is None:
        weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    
    # Joint limits
    joint_limits = np.array([
        [-1.6, 1.53],
        [-1.68, 1.55],
        [-1.68, 1.55],
        [-1.86, 2.07]
    ])
    
    def cost_function(theta):
        current_pose = fk_px100(theta)
        return pose_error(current_pose, target_pose_matrix, weights)
    
    bounds = [(joint_limits[i, 0], joint_limits[i, 1]) for i in range(4)]
    
    # Try with initial guess
    best_result = None
    best_error = float('inf')
    
    if verbose:
        print("Running IK optimization...")
        print(f"Target position: {target_pose_matrix[:3, 3]}")
    
    result = minimize(
        cost_function,
        initial_guess,
        method='L-BFGS-B',
        bounds=bounds,
        options={
            'maxiter': max_iterations,
            'ftol': tol,
            'gtol': tol
        }
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
            
            result = minimize(
                cost_function,
                random_guess,
                method='L-BFGS-B',
                bounds=bounds,
                options={
                    'maxiter': max_iterations,
                    'ftol': tol,
                    'gtol': tol
                }
            )
            
            if result.fun < best_error:
                best_error = result.fun
                best_result = result
                if verbose:
                    print(f"  Restart {i+1}: Improved error to {best_error:.6f}")
    
    if best_result is None:
        return None, False, float('inf')
    
    # Verify
    final_pose = fk_px100(best_result.x)
    position_error = np.linalg.norm(final_pose[:3, 3] - target_pose_matrix[:3, 3])
    
    success = position_error < 0.02  # Relax tolerance slightly (2cm instead of 1cm)
    
    if verbose:
        print(f"\nOptimization complete:")
        print(f"  Success: {success}")
        print(f"  Position error: {position_error*1000:.3f} mm")
    
    return best_result.x, success, position_error

def test_ik():
    """Test the numerical IK"""
    print("="*70)
    print("Testing Numerical IK (scipy-based, similar to MATLAB)")
    print("="*70)
    
    test_cases = [
        ([0, 0, 0, 0], "Zero configuration (home)"),
        ([0.3149, 0, 0.0931], "Zero config position"),
        ([0.25, 0, 0.15], "Forward, medium height"),
        ([0.2, 0.1, 0.1], "Forward-left"),
        ([0.2, -0.1, 0.1], "Forward-right"),
    ]
    
    passed = 0
    failed = 0
    
    for test_input, description in test_cases:
        print(f"\n{'='*70}")
        print(f"Test: {description}")
        print(f"{'='*70}")
        
        if len(test_input) == 4:
            target_pose = fk_px100(test_input)
            target_pos = target_pose[:3, 3]
            print(f"Input joint angles: {np.degrees(test_input)}")
            print(f"Target position: X={target_pos[0]:.6f}, Y={target_pos[1]:.6f}, Z={target_pos[2]:.6f}")
        else:
            target_pos = np.array(test_input)
            target_pose = np.eye(4)
            target_pose[:3, 3] = target_pos
            print(f"Target position: X={target_pos[0]:.6f}, Y={target_pos[1]:.6f}, Z={target_pos[2]:.6f}")
        
        # Run IK with more restarts and position-focused weights
        # Weight position more heavily than orientation
        weights = np.array([10.0, 10.0, 10.0, 0.1, 0.1, 0.1])  # Prioritize position
        joint_angles, success, error = ik_px100_numerical(
            target_pose,
            num_restarts=30,
            weights=weights,
            verbose=False
        )
        
        if not success or joint_angles is None:
            print("IK FAILED")
            failed += 1
            continue
        
        print(f"\nIK solution:")
        for i, angle in enumerate(joint_angles):
            print(f"  theta{i+1} = {angle:.6f} rad ({np.degrees(angle):8.4f}°)")
        
        # Verify
        result_pose = fk_px100(joint_angles)
        result_pos = result_pose[:3, 3]
        
        print(f"\nVerification:")
        print(f"  Target:  X={target_pos[0]:.6f}, Y={target_pos[1]:.6f}, Z={target_pos[2]:.6f}")
        print(f"  Reached: X={result_pos[0]:.6f}, Y={result_pos[1]:.6f}, Z={result_pos[2]:.6f}")
        print(f"  Position error: {error*1000:.3f} mm")
        
        if error < 0.001:
            print("✓ EXCELLENT - Error < 1mm")
            passed += 1
        elif error < 0.01:
            print("✓ GOOD - Error < 1cm")
            passed += 1
        else:
            print("⚠ POOR - Error > 1cm")
            failed += 1
    
    print(f"\n{'='*70}")
    print(f"SUMMARY")
    print(f"{'='*70}")
    print(f"Passed: {passed}/{len(test_cases)}")
    print(f"Failed: {failed}/{len(test_cases)}")

if __name__ == "__main__":
    test_ik()