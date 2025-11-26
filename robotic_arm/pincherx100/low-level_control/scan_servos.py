#!/usr/bin/env python3
"""
Scan for Dynamixel servos on the bus and display their IDs and models.
"""

import os
from dynamixel_sdk import *

# Control table addresses for XL430-W250 (Protocol 2.0)
ADDR_MODEL_NUMBER = 0
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings
DEVICENAME = '/dev/ttyUSB0'
BAUDRATES = [57600, 1000000, 115200, 3000000]  # Common baudrates to try

def scan_servos(baudrate):
    """Scan for servos at given baudrate."""
    print(f"\nScanning at baudrate {baudrate}...")
    
    # Initialize PortHandler and PacketHandler
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    # Open port
    if not portHandler.openPort():
        print(f"  Failed to open port {DEVICENAME}")
        return []
    
    # Set baudrate
    if not portHandler.setBaudRate(baudrate):
        print(f"  Failed to set baudrate to {baudrate}")
        portHandler.closePort()
        return []
    
    found_servos = []
    
    # Scan IDs from 0 to 20 (typical range for robot arms)
    for dxl_id in range(0, 21):
        # Try to ping
        dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)
        
        if dxl_comm_result == COMM_SUCCESS:
            print(f"  [ID:{dxl_id:02d}] Model: {dxl_model_number}")
            
            # Read current position
            present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
                portHandler, dxl_id, ADDR_PRESENT_POSITION
            )
            if dxl_comm_result == COMM_SUCCESS:
                print(f"         Current Position: {present_position}")
            
            found_servos.append((dxl_id, dxl_model_number))
    
    portHandler.closePort()
    return found_servos

def main():
    print("=" * 60)
    print("Dynamixel Servo Scanner for PincherX100")
    print("=" * 60)
    print(f"Port: {DEVICENAME}")
    
    all_found = []
    
    for baudrate in BAUDRATES:
        found = scan_servos(baudrate)
        if found:
            all_found.extend([(baudrate, sid, model) for sid, model in found])
    
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    if all_found:
        print(f"\nFound {len(all_found)} servo(s):")
        for baudrate, sid, model in all_found:
            print(f"  Baudrate: {baudrate}, ID: {sid}, Model: {model}")
        
        print("\n" + "=" * 60)
        print("Typical PincherX100 Configuration:")
        print("  ID 1: Base/Waist (left/right rotation)")
        print("  ID 2: Shoulder (up/down)")
        print("  ID 3: Elbow (up/down)")
        print("  ID 4: Wrist (up/down)")
        print("  ID 5: Gripper (open/close)")
        print("=" * 60)
    else:
        print("\nNo servos found!")
        print("Please check:")
        print("  1. Is the arm powered on?")
        print("  2. Is /dev/ttyUSB0 the correct device?")
        print("  3. Do you have permission? (Try: sudo usermod -a -G dialout $USER)")

if __name__ == "__main__":
    main()

