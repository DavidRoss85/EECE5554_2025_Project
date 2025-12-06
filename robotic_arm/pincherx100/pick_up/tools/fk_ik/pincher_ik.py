#!/usr/bin/env python3
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

def dh(alpha_prev, a_prev, d, theta):
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

def ik_px100(x, y, z, phi=0):
    """
    Inverse kinematics for PincherX 100
    
    Args:
        x, y, z: Desired end-effector position (in meters)
        phi: Desired pitch angle of end-effector (in radians, default=0)
    
    Returns:
        [theta1, theta2, theta3, theta4] in radians, or None if unreachable
    """
    
    # DH parameters (matching your working FK)
    d1 = 0.0931   # Base height
    a2 = 0.1059   # Link 2 length
    a3 = 0.1      # Link 3 length  
    gripper_length = 0.109  # From T4e
    
    # Step 1: Approximate wrist position
    # The gripper extends forward by ~0.109m
    x_wrist = x - gripper_length
    y_wrist = y
    z_wrist = z
    
    # Step 2: Calculate theta1 (base rotation)
    theta1 = np.arctan2(y_wrist, x_wrist)
    
    # Step 3: Calculate distance in XY plane
    r = np.sqrt(x_wrist**2 + y_wrist**2)
    
    # Step 4: Height relative to shoulder
    z_rel = z_wrist - d1
    
    # Step 5: 2D planar arm IK
    D = np.sqrt(r**2 + z_rel**2)
    
    # Check reachability
    L_max = a2 + a3
    L_min = abs(a2 - a3)
    
    if D > L_max + 0.001:
        print(f"Target unreachable! Too far.")
        print(f"Distance required: {D:.4f} m")
        print(f"Maximum reach: {L_max:.4f} m")
        return None
    
    if D < L_min:
        print(f"Target unreachable! Too close.")
        print(f"Distance required: {D:.4f} m")
        print(f"Minimum reach: {L_min:.4f} m")
        return None
    
    # Step 6: Calculate theta3 using law of cosines
    cos_theta3 = (D**2 - a2**2 - a3**2) / (2 * a2 * a3)
    cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
    theta3 = np.arccos(cos_theta3)
    
    # Step 7: Calculate theta2
    alpha = np.arctan2(z_rel, r)
    beta = np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))
    theta2 = alpha - beta
    
    # Step 8: Calculate theta4
    theta4 = phi - theta2 - theta3
    
    # Normalize angles to [-pi, pi]
    theta1 = np.arctan2(np.sin(theta1), np.cos(theta1))
    theta2 = np.arctan2(np.sin(theta2), np.cos(theta2))
    theta3 = np.arctan2(np.sin(theta3), np.cos(theta3))
    theta4 = np.arctan2(np.sin(theta4), np.cos(theta4))
    
    return [theta1, theta2, theta3, theta4]

def move_servo(packetHandler, portHandler, servo_id, angle_rad):
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
    print("PincherX 100 Inverse Kinematics")
    print("="*60)
    
    x = float(input("\nTarget X (m): "))
    y = float(input("Target Y (m): "))
    z = float(input("Target Z (m): "))
    phi = float(input("End-effector pitch angle (rad, default=0): ") or 0)
    
    print("\nCalculating inverse kinematics...")
    joints = ik_px100(x, y, z, phi)
    
    if joints is None:
        print("\nInverse kinematics failed! Target is unreachable.")
        portHandler.closePort()
        exit(1)
    
    theta1, theta2, theta3, theta4 = joints
    
    print("\nCalculated joint angles:")
    print(f"  theta1 = {theta1:.6f} rad ({np.degrees(theta1):8.4f}°)")
    print(f"  theta2 = {theta2:.6f} rad ({np.degrees(theta2):8.4f}°)")
    print(f"  theta3 = {theta3:.6f} rad ({np.degrees(theta3):8.4f}°)")
    print(f"  theta4 = {theta4:.6f} rad ({np.degrees(theta4):8.4f}°)")
    
    # Verify with forward kinematics
    T = fk_px100(joints)
    pos = T[:3, 3]
    
    print("\nVerification (forward kinematics):")
    print(f"  Target:  X={x:.6f}, Y={y:.6f}, Z={z:.6f}")
    print(f"  Reached: X={pos[0]:.6f}, Y={pos[1]:.6f}, Z={pos[2]:.6f}")
    
    error = np.sqrt((pos[0]-x)**2 + (pos[1]-y)**2 + (pos[2]-z)**2)
    print(f"  Position error: {error:.6f} m = {error*1000:.3f} mm")
    
    if error > 0.01:
        print("\n⚠ WARNING: Position error > 1cm detected!")
    
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
    else:
        print("\nMovement cancelled.")
    
    portHandler.closePort()