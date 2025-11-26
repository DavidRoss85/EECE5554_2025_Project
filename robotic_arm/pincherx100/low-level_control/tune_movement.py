#!/usr/bin/env python3
"""
Quick tuning script to adjust movement smoothness.
Run this to try different profile velocity/acceleration settings.
"""

import sys
from dynamixel_sdk import *

# Control table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108
ADDR_PWM_LIMIT = 36
ADDR_CURRENT_LIMIT = 38

PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000

# Servo IDs
SERVO_IDS = [1, 2, 3, 4, 5]

def set_profile(velocity, acceleration):
    """Set profile velocity, acceleration, and MAXIMUM POWER for all servos."""
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    if not portHandler.openPort():
        print(f"Failed to open {DEVICENAME}")
        return False
    
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"Failed to set baudrate to {BAUDRATE}")
        return False
    
    print(f"\nSetting Profile Velocity: {velocity}")
    print(f"Setting Profile Acceleration: {acceleration}")
    print(f"Setting MAXIMUM POWER (PWM=885, Current=1193)\n")
    
    for servo_id in SERVO_IDS:
        # Disable torque
        packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, 0)
        
        # Set MAXIMUM power limits
        packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_PWM_LIMIT, 885)
        packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_CURRENT_LIMIT, 1193)
        
        # Set profile velocity
        packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_PROFILE_VELOCITY, velocity)
        
        # Set profile acceleration
        packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_PROFILE_ACCELERATION, acceleration)
        
        # Enable torque
        packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, 1)
        
        print(f"  Configured servo ID {servo_id} with MAX POWER")
    
    portHandler.closePort()
    print("\n" + "=" * 60)
    print("Done! Maximum power enabled for all servos.")
    print("=" * 60)
    print("Now run: python control_arm.py")
    return True

if __name__ == "__main__":
    print("=" * 60)
    print("Movement Tuning for PincherX100")
    print("=" * 60)
    print("\nAll presets include MAXIMUM POWER settings.")
    print("Lower velocity = more torque (better for lifting)")
    print("\nPresets:")
    print("  1. Maximum Power (velocity=150, accel=80) - BEST FOR LIFTING")
    print("  2. Balanced (velocity=100, accel=50)")
    print("  3. Fast & Snappy (velocity=50, accel=20)")
    print("  4. Maximum Speed (velocity=0, accel=0)")
    print("  5. Custom values")
    
    choice = input("\nSelect preset (1-5): ").strip()
    
    if choice == "1":
        velocity, acceleration = 150, 80  # Maximum power for lifting
    elif choice == "2":
        velocity, acceleration = 100, 50  # Balanced
    elif choice == "3":
        velocity, acceleration = 50, 20   # Fast & Snappy
    elif choice == "4":
        velocity, acceleration = 0, 0     # Maximum Speed
    elif choice == "5":
        try:
            velocity = int(input("Profile Velocity (0-1023, lower=faster): "))
            acceleration = int(input("Profile Acceleration (0-32767, lower=faster): "))
        except ValueError:
            print("Invalid input!")
            sys.exit(1)
    else:
        print("Invalid choice!")
        sys.exit(1)
    
    set_profile(velocity, acceleration)

