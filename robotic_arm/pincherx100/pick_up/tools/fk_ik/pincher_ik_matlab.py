#!/usr/bin/env python3
"""
PincherX 100 - Exact MATLAB Implementation in Python
Based on: https://github.com/Aryaman22102002/PincherX100_Pick_and_Place

Uses:
- MATLAB's exact DH parameters
- scipy numerical IK (matches MATLAB's inverseKinematics)
- RRT-Connect for path planning
"""

import numpy as np
from scipy.optimize import minimize
from dynamixel_sdk import *
import time

# Dynamixel settings
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

POSITION_TO_RAD = (2 * np.pi) / 4096
RAD_TO_POSITION = 4096 / (2 * np.pi)

# Joint limits (from MATLAB code)
JOINT_LIMITS = np.array([
    [-1.6, 1.53],
    [-1.68, 1.55],
    [-1.68, 1.55],
    [-1.86, 2.07]
])

def fk_px100_matlab(a, alpha, d, theta):
    """
    Forward kinematics - EXACT MATLAB implementation
    Function signature matches MATLAB: fk_px100(a, alpha, d, theta)
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -d*sa],
        [st*sa, ct*sa, ca, d*ca],
        [0, 0, 0, 1]
    ])

def fk_px100(theta_values):
    """
    Complete forward kinematics using MATLAB's exact DH parameters
    
    From MATLAB code:
    a0=0, a1=0, a2=0.1, a3=0.1
    alpha0=0, alpha1=-pi/2, alpha2=0, alpha3=0
    d1=0.0931, d2=0, d3=0.035, d4=0
    T4e = [1,0,0,0.1136; 0,0,-1,0; 0,1,0,0; 0,0,0,1]
    """
    theta1, theta2, theta3, theta4 = theta_values
    
    # MATLAB's exact DH parameters
    a0, a1, a2, a3 = 0, 0, 0.1, 0.1
    alpha0, alpha1, alpha2, alpha3 = 0, -np.pi/2, 0, 0
    d1, d2, d3, d4 = 0.0931, 0, 0.035, 0
    
    # Build transformation matrices
    T01 = fk_px100_matlab(a0, alpha0, d1, theta1)
    T12 = fk_px100_matlab(a1, alpha1, d2, theta2)
    T23 = fk_px100_matlab(a2, alpha2, d3, theta3)
    T34 = fk_px100_matlab(a3, alpha3, d4, theta4)
    
    # End-effector transform (from MATLAB)
    T4e = np.array([
        [1, 0, 0, 0.1136],
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    # Compute full transform
    T04 = T01 @ T12 @ T23 @ T34
    T0e = T04 @ T4e
    
    return T0e

def ik_px100_numerical(target_pose, initial_guess=None, max_iterations=5000, 
                       num_restarts=100, verbose=False):
    """
    Numerical IK - matches MATLAB's inverseKinematics
    
    Uses BFGS optimization with:
    - MaxIterations = 5000
    - NumRandomRestarts = 100
    - Tolerance = 1e-10
    - Weights = [1, 0, 0, 1, 1, 1] (prioritize position)
    """
    
    # Handle input
    if isinstance(target_pose, (list, tuple, np.ndarray)) and len(target_pose) == 3:
        target_pose_matrix = np.eye(4)
        target_pose_matrix[:3, 3] = target_pose
    else:
        target_pose_matrix = target_pose
    
    # Default initial guess - home configuration
    if initial_guess is None:
        initial_guess = np.array([0.0, 0.0, 0.0, 0.0])
    
    # MATLAB weights: [1, 0, 0, 1, 1, 1]
    # This means: prioritize x position, don't care much about y,z, care about orientation
    # We'll use: prioritize ALL position heavily
    weights = np.array([1000.0, 1000.0, 1000.0, 1.0, 1.0, 1.0])
    
    def cost_function(theta):
        """Cost function matching MATLAB's behavior"""
        current_pose = fk_px100(theta)
        
        # Position error
        pos_error = current_pose[:3, 3] - target_pose_matrix[:3, 3]
        position_cost = np.sqrt(
            weights[0] * pos_error[0]**2 +
            weights[1] * pos_error[1]**2 +
            weights[2] * pos_error[2]**2
        )
        
        # Orientation error (Frobenius norm)
        rot_current = current_pose[:3, :3]
        rot_target = target_pose_matrix[:3, :3]
        rot_error = np.linalg.norm(rot_current @ rot_target.T - np.eye(3), 'fro')
        orientation_cost = (weights[3] + weights[4] + weights[5]) * rot_error / 3.0
        
        return position_cost + orientation_cost
    
    # Bounds from joint limits
    bounds = [(JOINT_LIMITS[i, 0], JOINT_LIMITS[i, 1]) for i in range(4)]
    
    best_result = None
    best_error = float('inf')
    
    # Try with initial guess
    result = minimize(
        cost_function,
        initial_guess,
        method='L-BFGS-B',
        bounds=bounds,
        options={'maxiter': max_iterations, 'ftol': 1e-10, 'gtol': 1e-10}
    )
    
    if result.fun < best_error:
        best_error = result.fun
        best_result = result
    
    # Random restarts (like MATLAB's NumRandomRestarts)
    if num_restarts > 0:
        if verbose:
            print(f"Running {num_restarts} random restarts...")
        
        for i in range(num_restarts):
            random_guess = np.array([
                np.random.uniform(JOINT_LIMITS[j, 0], JOINT_LIMITS[j, 1])
                for j in range(4)
            ])
            
            result = minimize(
                cost_function,
                random_guess,
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': max_iterations, 'ftol': 1e-10, 'gtol': 1e-10}
            )
            
            if result.fun < best_error:
                best_error = result.fun
                best_result = result
                if verbose and i < 5:
                    print(f"  Restart {i+1}: improved to {best_error:.6e}")
    
    # Check success
    if best_result is None:
        return None, False
    
    # Verify position error
    final_pose = fk_px100(best_result.x)
    pos_error = np.linalg.norm(final_pose[:3, 3] - target_pose_matrix[:3, 3])
    success = pos_error < 0.01  # Within 1cm
    
    if verbose:
        print(f"\nIK Solution:")
        print(f"  Position error: {pos_error*1000:.3f} mm")
        print(f"  Joints: {np.degrees(best_result.x)}")
    
    return best_result.x, success

class RRTConnect:
    """RRT-Connect path planner"""
    
    def __init__(self, step_size=0.1, max_iter=5000):
        self.step_size = step_size
        self.max_iter = max_iter
    
    def distance(self, q1, q2):
        return np.linalg.norm(np.array(q1) - np.array(q2))
    
    def steer(self, from_q, to_q):
        direction = np.array(to_q) - np.array(from_q)
        dist = np.linalg.norm(direction)
        
        if dist < self.step_size:
            return to_q
        
        new_q = np.array(from_q) + self.step_size * (direction / dist)
        
        # Clip to limits
        for i in range(4):
            new_q[i] = np.clip(new_q[i], JOINT_LIMITS[i, 0], JOINT_LIMITS[i, 1])
        
        return new_q
    
    def nearest(self, tree, q):
        return min(tree, key=lambda node: self.distance(node, q))
    
    def plan(self, start, goal, verbose=False):
        """RRT-Connect algorithm"""
        tree_a = [start]
        tree_b = [goal]
        parent_a = {tuple(start): None}
        parent_b = {tuple(goal): None}
        
        for iteration in range(self.max_iter):
            # Sample random configuration
            q_rand = np.array([
                np.random.uniform(JOINT_LIMITS[i, 0], JOINT_LIMITS[i, 1])
                for i in range(4)
            ])
            
            # Extend tree_a
            q_near = self.nearest(tree_a, q_rand)
            q_new = self.steer(q_near, q_rand)
            
            tree_a.append(q_new)
            parent_a[tuple(q_new)] = q_near
            
            # Try to connect to tree_b
            q_near_b = self.nearest(tree_b, q_new)
            q_connect = self.steer(q_near_b, q_new)
            
            if self.distance(q_connect, q_new) < self.step_size:
                # Trees connected!
                if verbose:
                    print(f"  Path found in {iteration} iterations")
                
                # Reconstruct path
                path_a = self._reconstruct(parent_a, q_new, start)
                path_b = self._reconstruct(parent_b, q_connect, goal)
                
                return path_a + path_b[::-1]
            
            tree_b.append(q_connect)
            parent_b[tuple(q_connect)] = q_near_b
            
            # Swap trees
            tree_a, tree_b = tree_b, tree_a
            parent_a, parent_b = parent_b, parent_a
        
        if verbose:
            print(f"  RRT failed after {self.max_iter} iterations")
        return None
    
    def _reconstruct(self, parent, end, start):
        path = []
        current = end
        
        while current is not None:
            path.append(current)
            current = parent.get(tuple(current))
            
            if current is not None and np.allclose(current, start):
                path.append(current)
                break
        
        return path[::-1]

def move_to_position(portHandler, packetHandler, target_x, target_y, target_z, 
                     use_rrt=True, verbose=True):
    """
    Main function: Move arm to (x,y,z) position
    
    Args:
        portHandler, packetHandler: Dynamixel handlers
        target_x, target_y, target_z: Target position in meters
        use_rrt: Use RRT-Connect (True) or linear interpolation (False)
        verbose: Print progress
    
    Returns:
        success: Boolean
    """
    
    # Step 1: Get current configuration
    if verbose:
        print("\n" + "="*60)
        print("Step 1: Reading current configuration...")
    
    current_config = []
    for servo_id in range(1, 5):
        position, _, _ = packetHandler.read4ByteTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION)
        angle = -np.pi + (position * 2 * np.pi / 4095)
        current_config.append(angle)
    
    current_config = np.array(current_config)
    current_pos = (fk_px100(current_config) @ [0, 0, 0, 1])[:3]
    
    if verbose:
        print(f"  Current: X={current_pos[0]:.4f}, Y={current_pos[1]:.4f}, Z={current_pos[2]:.4f}")
        print(f"  Joints: {np.degrees(current_config)}")
    
    # Step 2: Solve IK for target
    if verbose:
        print("\n" + "="*60)
        print(f"Step 2: Solving IK for target ({target_x}, {target_y}, {target_z})...")
    
    target_pose = np.eye(4)
    target_pose[:3, 3] = [target_x, target_y, target_z]
    
    target_config, success = ik_px100_numerical(
        target_pose,
        initial_guess=current_config,
        num_restarts=100,
        verbose=verbose
    )
    
    if not success or target_config is None:
        print("  ✗ IK failed!")
        return False
    
    if verbose:
        target_pos_check = (fk_px100(target_config) @ [0, 0, 0, 1])[:3]
        print(f"  ✓ IK succeeded!")
        print(f"  Target joints: {np.degrees(target_config)}")
        print(f"  Will reach: X={target_pos_check[0]:.4f}, Y={target_pos_check[1]:.4f}, Z={target_pos_check[2]:.4f}")
    
    # Step 3: Plan path
    if verbose:
        print("\n" + "="*60)
        print("Step 3: Planning path...")
    
    if use_rrt:
        planner = RRTConnect(step_size=0.15, max_iter=5000)
        path = planner.plan(current_config, target_config, verbose=verbose)
        
        if path is None:
            if verbose:
                print("  RRT failed, using linear interpolation")
            path = [current_config, target_config]
    else:
        path = [current_config, target_config]
    
    # Interpolate to smooth waypoints
    num_waypoints = 50
    waypoints = []
    
    for i in range(len(path) - 1):
        start = path[i]
        end = path[i + 1]
        
        for t in np.linspace(0, 1, num_waypoints // len(path)):
            waypoint = start + t * (end - start)
            waypoints.append(waypoint)
    
    waypoints.append(target_config)
    
    if verbose:
        print(f"  Generated {len(waypoints)} waypoints")
    
    # Step 4: Execute trajectory
    if verbose:
        print("\n" + "="*60)
        print("Step 4: Executing trajectory...")
    
    for i, waypoint in enumerate(waypoints):
        # Move all servos
        for servo_id in range(4):
            position = int(waypoint[servo_id] * RAD_TO_POSITION + 2048)
            position = max(0, min(4095, position))
            packetHandler.write4ByteTxRx(portHandler, servo_id + 1, ADDR_GOAL_POSITION, position)
        
        # Print progress
        if i % 10 == 0 or i == len(waypoints) - 1:
            pos = (fk_px100(waypoint) @ [0, 0, 0, 1])[:3]
            if verbose:
                print(f"  Waypoint {i:3d}/{len(waypoints)}: X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f}")
        
        time.sleep(0.05)
    
    if verbose:
        print("\n✓ Movement complete!")
    
    return True

if __name__ == "__main__":
    print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))
    
    # Initialize robot
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    if not portHandler.openPort():
        print("Failed to open port!")
        exit(1)
    
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to set baudrate!")
        exit(1)
    
    # Enable torque
    for servo_id in [1, 2, 3, 4]:
        packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, 1)
    
    print("\n" + "="*60)
    print("PincherX 100 Movement Planner")
    print("Using MATLAB's exact DH parameters + RRT-Connect")
    print("="*60)
    
    # Get target position
    x = float(input("\nTarget X (m): "))
    y = float(input("Target Y (m): "))
    z = float(input("Target Z (m): "))
    
    use_rrt = input("Use RRT-Connect path planning? (y/n, default=y): ").lower() != 'n'
    
    # Move to position
    success = move_to_position(
        portHandler, packetHandler,
        x, y, z,
        use_rrt=use_rrt,
        verbose=True
    )
    
    if success:
        print("\n✓ Successfully moved to target position!")
    else:
        print("\n✗ Movement failed!")
    
    # Cleanup
    portHandler.closePort()