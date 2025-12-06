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
    
    # T1 = dh(0, 0, 0.0931, theta1)
    # T2 = dh(np.pi/2, 0, 0.1, theta2)
    # T3 = dh(0, 0.1, 0.035, theta3)
    # T4 = dh(0, 0.1, 0, theta4)
    
    T1 = dh(0,          0,          0.090,     theta1)
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
    
    theta1 = float(input("theta1 (rad): "))
    theta2 = float(input("theta2 (rad): "))
    theta3 = float(input("theta3 (rad): "))
    theta4 = float(input("theta4 (rad): "))
    
    joints = [theta1, theta2, theta3, theta4]
    
    move_servo(packetHandler, portHandler, 1, theta1)
    move_servo(packetHandler, portHandler, 2, theta2)
    move_servo(packetHandler, portHandler, 3, theta3)
    move_servo(packetHandler, portHandler, 4, theta4)
    
    time.sleep(2)
    
    T = fk_px100(joints)
    pos = T[:3, 3]
    
    print(f"\nMoved to: X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f}")
    print(T)
    
    portHandler.closePort()
