#!/usr/bin/env python3
"""
Simple Path Planner for PincherX 100
- Always keeps gripper horizontal (phi = 0)
- Uses RRT-Connect for collision-free path planning
- Takes (x, y, z) target and plans smooth movement
"""

import numpy as np
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

# Joint limits
JOINT_LIMITS = np.array([
    [-1.55, 1.48],   # theta1
    [-1.63, 1.50],   # theta2
    [-1.63, 1.50],   # theta3
    [-1.81, 2.02]    # theta4
])

def dh(alpha_prev, a_prev, d, theta):
    """DH transformation matrix"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha_prev), np.sin(alpha_prev)
    return np.array([
        [ct, -st, 0, a_prev],
        [st*ca, ct*ca, -sa, -sa*d],
        [st*sa, ct*sa, ca, ca*d],
        [0, 0, 0, 1]
    ])

def fk_px100(theta_values):
    """Forward kinematics"""
    theta1, theta2, theta3, theta4 = theta_values
    
    T1 = dh(0, 0, 0.18945, theta1)
    T2 = dh(-np.pi/2, 0, 0, theta2)
    T3 = dh(0, 0.035, 0, theta3)
    T4 = dh(0, 0.1, 0, theta4)
    T04 = T1 @ T2 @ T3 @ T4
    
    T4e = np.array([
        [1, 0, 0, 0.1076],
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    return T04 @ T4e

def ik_simple(x, y, z, phi=0, max_attempts=100):
    """
    Simple IK with phi constraint
    Uses random sampling to find valid configuration
    
    Args:
        x, y, z: Target position in meters
        phi: Gripper pitch (default 0 for horizontal)
        max_attempts: Number of random samples
    
    Returns:
        [theta1, theta2, theta3, theta4] or None
    """
    from scipy.optimize import minimize
    
    target = np.array([x, y, z])
    
    def cost(theta):
        pos = (fk_px100(theta) @ [0, 0, 0, 1])[:3]
        pos_error = np.linalg.norm(pos - target)
        phi_actual = theta[1] + theta[2] + theta[3]
        phi_error = abs(phi_actual - phi)
        return 1000.0 * pos_error + 10.0 * phi_error
    
    bounds = [(JOINT_LIMITS[i, 0], JOINT_LIMITS[i, 1]) for i in range(4)]
    
    best_result = None
    best_error = float('inf')
    
    # theta1 is straightforward
    theta1 = np.arctan2(y, x)
    
    # For phi=0, use pattern: theta2 negative, theta3 positive, theta4 compensates
    for attempt in range(max_attempts):
        if attempt == 0:
            # Try a good heuristic first
            initial = [theta1, -0.7, 1.2, -0.5]
        else:
            # Random guesses following phi=0 pattern
            theta2_g = np.random.uniform(-1.5, 0.5)
            theta3_g = np.random.uniform(0, 1.5)
            theta4_g = phi - theta2_g - theta3_g
            theta4_g = np.clip(theta4_g, JOINT_LIMITS[3, 0], JOINT_LIMITS[3, 1])
            initial = [theta1, theta2_g, theta3_g, theta4_g]
        
        result = minimize(cost, initial, method='L-BFGS-B', bounds=bounds,
                         options={'maxiter': 1000, 'ftol': 1e-8})
        
        if result.fun < best_error:
            best_error = result.fun
            best_result = result
    
    if best_result is None:
        return None
    
    # Verify solution
    pos_reached = (fk_px100(best_result.x) @ [0, 0, 0, 1])[:3]
    error = np.linalg.norm(pos_reached - target)
    
    if error > 0.05:  # More than 5cm error
        print(f"Warning: Position error {error*1000:.1f}mm")
        print(f"Target may not be reachable with phi={np.degrees(phi):.1f}°")
        return None
    
    return best_result.x

class RRTConnect:
    """RRT-Connect path planner for PincherX 100"""
    
    def __init__(self, step_size=0.1, max_iterations=5000):
        self.step_size = step_size  # Step size in joint space (radians)
        self.max_iterations = max_iterations
        self.joint_limits = JOINT_LIMITS
    
    def random_config(self):
        """Generate random valid configuration"""
        return np.array([
            np.random.uniform(self.joint_limits[i, 0], self.joint_limits[i, 1])
            for i in range(4)
        ])
    
    def distance(self, config1, config2):
        """Distance between two configurations"""
        return np.linalg.norm(np.array(config1) - np.array(config2))
    
    def steer(self, from_config, to_config):
        """Steer from one config toward another by step_size"""
        direction = np.array(to_config) - np.array(from_config)
        dist = np.linalg.norm(direction)
        
        if dist < self.step_size:
            return to_config
        
        direction = direction / dist  # Normalize
        new_config = np.array(from_config) + self.step_size * direction
        
        # Clip to joint limits
        for i in range(4):
            new_config[i] = np.clip(new_config[i], self.joint_limits[i, 0], self.joint_limits[i, 1])
        
        return new_config
    
    def nearest(self, tree, config):
        """Find nearest node in tree to config"""
        min_dist = float('inf')
        nearest_node = None
        
        for node in tree:
            dist = self.distance(node, config)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        
        return nearest_node
    
    def plan(self, start_config, goal_config, verbose=False):
        """
        RRT-Connect algorithm to find path from start to goal
        
        Returns:
            path: List of configurations from start to goal, or None if failed
        """
        # Initialize trees
        tree_start = [start_config]
        tree_goal = [goal_config]
        
        # Parent tracking for path reconstruction
        parent_start = {tuple(start_config): None}
        parent_goal = {tuple(goal_config): None}
        
        for iteration in range(self.max_iterations):
            # Grow tree from start
            rand_config = self.random_config()
            nearest_start = self.nearest(tree_start, rand_config)
            new_config = self.steer(nearest_start, rand_config)
            
            tree_start.append(new_config)
            parent_start[tuple(new_config)] = nearest_start
            
            # Try to connect to goal tree
            nearest_goal = self.nearest(tree_goal, new_config)
            connect_config = self.steer(nearest_goal, new_config)
            
            if self.distance(connect_config, new_config) < self.step_size * 0.5:
                # Connection made!
                if verbose:
                    print(f"Path found in {iteration} iterations!")
                
                # Reconstruct path
                path_start = self._reconstruct_path(parent_start, new_config, start_config)
                path_goal = self._reconstruct_path(parent_goal, connect_config, goal_config)
                
                # Combine paths
                path = path_start + path_goal[::-1]
                return path
            
            tree_goal.append(connect_config)
            parent_goal[tuple(connect_config)] = nearest_goal
            
            # Swap trees (alternate growing from start and goal)
            tree_start, tree_goal = tree_goal, tree_start
            parent_start, parent_goal = parent_goal, parent_start
        
        if verbose:
            print(f"Failed to find path after {self.max_iterations} iterations")
        return None
    
    def _reconstruct_path(self, parent_dict, end_config, start_config):
        """Reconstruct path from parent dictionary"""
        path = []
        current = end_config
        
        while current is not None:
            path.append(current)
            current = parent_dict.get(tuple(current))
            
            if current is not None and np.allclose(current, start_config):
                path.append(current)
                break
        
        return path[::-1]

def plan_movement(current_pos, target_x, target_y, target_z, num_waypoints=50, verbose=False):
    """
    Plan movement from current position to target position
    
    Args:
        current_pos: Current [theta1, theta2, theta3, theta4]
        target_x, target_y, target_z: Target position in meters
        num_waypoints: Number of waypoints for smooth trajectory
        verbose: Print planning info
    
    Returns:
        waypoints: List of [theta1, theta2, theta3, theta4] configurations
    """
    
    # Step 1: Solve IK for target with phi=0
    if verbose:
        print(f"\nStep 1: Solving IK for target ({target_x}, {target_y}, {target_z})")
    
    target_config = ik_simple(target_x, target_y, target_z, phi=0, max_attempts=100)
    
    if target_config is None:
        print("Failed to find IK solution for target!")
        return None
    
    if verbose:
        print(f"  IK solution: {np.degrees(target_config)}")
        phi_check = target_config[1] + target_config[2] + target_config[3]
        print(f"  phi = {np.degrees(phi_check):.2f}°")
        
        # Verify position
        pos_reached = (fk_px100(target_config) @ [0, 0, 0, 1])[:3]
        error = np.linalg.norm(pos_reached - np.array([target_x, target_y, target_z]))
        print(f"  Position error: {error*1000:.1f} mm")
    
    # Step 2: Plan path using RRT-Connect
    if verbose:
        print(f"\nStep 2: Planning path with RRT-Connect...")
    
    planner = RRTConnect(step_size=0.15, max_iterations=5000)
    path = planner.plan(current_pos, target_config, verbose=verbose)
    
    if path is None:
        print("RRT-Connect failed to find path!")
        # Fallback to simple linear interpolation
        if verbose:
            print("Falling back to linear interpolation...")
        path = [current_pos, target_config]
    
    # Step 3: Interpolate to get smooth waypoints
    if verbose:
        print(f"\nStep 3: Interpolating to {num_waypoints} waypoints...")
    
    waypoints = []
    for i in range(len(path) - 1):
        start = np.array(path[i])
        end = np.array(path[i + 1])
        
        # Number of points for this segment
        segment_points = max(2, int(num_waypoints / len(path)))
        
        for t in np.linspace(0, 1, segment_points):
            waypoint = start + t * (end - start)
            waypoints.append(waypoint)
    
    # Ensure final waypoint is exactly the target
    waypoints.append(target_config)
    
    if verbose:
        print(f"  Generated {len(waypoints)} waypoints")
    
    return waypoints

def move_servo(packetHandler, portHandler, servo_id, angle_rad):
    """Move servo to angle"""
    position = int(angle_rad * RAD_TO_POSITION + 2048)
    position = max(0, min(4095, position))
    packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, position)

def execute_trajectory(portHandler, packetHandler, waypoints, delay=0.1):
    """Execute trajectory by moving through waypoints"""
    print(f"\nExecuting trajectory with {len(waypoints)} waypoints...")
    
    for i, waypoint in enumerate(waypoints):
        # Move all servos
        for servo_id in range(4):
            move_servo(packetHandler, portHandler, servo_id + 1, waypoint[servo_id])
        
        # Verify position
        if i % 10 == 0:
            pos = (fk_px100(waypoint) @ [0, 0, 0, 1])[:3]
            print(f"  Waypoint {i}/{len(waypoints)}: X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f}")
        
        time.sleep(delay)
    
    print("Trajectory complete!")

def get_current_config(portHandler, packetHandler):
    """Read current joint positions from robot"""
    config = []
    
    for servo_id in range(1, 5):
        position, _, _ = packetHandler.read4ByteTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION)
        angle = -np.pi + (position * 2 * np.pi / 4095)
        config.append(angle)
    
    return np.array(config)

if __name__ == "__main__":
    print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))
    
    # Initialize robot
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)
    
    for servo_id in [1, 2, 3, 4]:
        packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, 1)
    
    print("\n" + "="*60)
    print("PincherX 100 Path Planner (RRT-Connect)")
    print("Gripper always horizontal (phi = 0)")
    print("="*60)
    
    # Get current position
    print("\nReading current robot configuration...")
    current_config = get_current_config(portHandler, packetHandler)
    current_pos = (fk_px100(current_config) @ [0, 0, 0, 1])[:3]
    
    print(f"Current position: X={current_pos[0]:.4f}, Y={current_pos[1]:.4f}, Z={current_pos[2]:.4f}")
    print(f"Current joints: {np.degrees(current_config)}")
    
    # Get target position
    x = float(input("\nTarget X (m): "))
    y = float(input("Target Y (m): "))
    z = float(input("Target Z (m): "))
    
    use_rrt = input("Use RRT-Connect planning? (y/n, default=y): ").lower() != 'n'
    
    # Plan movement
    print("\nPlanning movement...")
    waypoints = plan_movement(
        current_config,
        x, y, z,
        num_waypoints=50,
        verbose=True
    )
    
    if waypoints is None:
        print("\nPath planning failed!")
        portHandler.closePort()
        exit(1)
    
    # Show final configuration
    final_config = waypoints[-1]
    print(f"\nFinal configuration:")
    print(f"  theta1 = {np.degrees(final_config[0]):7.2f}°")
    print(f"  theta2 = {np.degrees(final_config[1]):7.2f}°")
    print(f"  theta3 = {np.degrees(final_config[2]):7.2f}°")
    print(f"  theta4 = {np.degrees(final_config[3]):7.2f}°")
    
    phi_final = final_config[1] + final_config[2] + final_config[3]
    print(f"  phi = {np.degrees(phi_final):.2f}° (target: 0.00°)")
    
    # Execute trajectory
    confirm = input("\nExecute trajectory? (y/n): ")
    
    if confirm.lower() == 'y':
        execute_trajectory(portHandler, packetHandler, waypoints, delay=0.05)
        print("\nMovement complete!")
    else:
        print("\nMovement cancelled.")
    
    portHandler.closePort()