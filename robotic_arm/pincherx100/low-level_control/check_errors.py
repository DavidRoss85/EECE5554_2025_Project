#!/usr/bin/env python3
"""
Quick error checker for PincherX100 servos.
Displays current status and errors for all servos.
Can also clear errors by rebooting servos.
"""

import sys
import time
from dynamixel_sdk import *

# Control table addresses
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_TEMPERATURE = 146
ADDR_PRESENT_CURRENT = 126
ADDR_TORQUE_ENABLE = 64
ADDR_PWM_LIMIT = 36
ADDR_CURRENT_LIMIT = 38
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108

# Power settings
PWM_LIMIT = 885
CURRENT_LIMIT = 1193
PROFILE_VELOCITY = 150
PROFILE_ACCELERATION = 80

PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
SERVO_IDS = [1, 2, 3, 4, 5]
SERVO_NAMES = {1: "Base", 2: "Shoulder", 3: "Elbow", 4: "Wrist", 5: "Gripper"}

def decode_error(error_status):
    """Decode hardware error status byte."""
    if error_status == 0:
        return "✓ No errors"
    
    errors = []
    if error_status & 0x01:
        errors.append("⚠️  Input Voltage Error")
    if error_status & 0x04:
        errors.append("🔥 Overheating Error")
    if error_status & 0x08:
        errors.append("⚙️  Motor Encoder Error")
    if error_status & 0x10:
        errors.append("⚡ Electrical Shock Error")
    if error_status & 0x20:
        errors.append("💪 OVERLOAD Error (most common)")
    
    return "\n       ".join(errors)

def check_servos():
    """Check all servos for errors and status."""
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    if not portHandler.openPort():
        print(f"❌ Failed to open {DEVICENAME}")
        return
    
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"❌ Failed to set baudrate to {BAUDRATE}")
        return
    
    print("=" * 70)
    print("PincherX100 Servo Error Check")
    print("=" * 70)
    print()
    
    any_errors = False
    
    for servo_id in SERVO_IDS:
        name = SERVO_NAMES.get(servo_id, f"Servo {servo_id}")
        
        # Read error status
        error_status, comm_result, _ = packetHandler.read1ByteTxRx(
            portHandler, servo_id, ADDR_HARDWARE_ERROR_STATUS
        )
        
        if comm_result != COMM_SUCCESS:
            print(f"[ID {servo_id}] {name:12} - ❌ Communication failed")
            continue
        
        # Read position
        position, _, _ = packetHandler.read4ByteTxRx(
            portHandler, servo_id, ADDR_PRESENT_POSITION
        )
        
        # Read temperature
        temperature, _, _ = packetHandler.read1ByteTxRx(
            portHandler, servo_id, ADDR_PRESENT_TEMPERATURE
        )
        
        # Read current (in 2-byte signed format)
        current, _, _ = packetHandler.read2ByteTxRx(
            portHandler, servo_id, ADDR_PRESENT_CURRENT
        )
        
        print(f"[ID {servo_id}] {name:12}")
        print(f"    Position: {position:4d}")
        print(f"    Temperature: {temperature}°C")
        print(f"    Current: {current} (raw)")
        print(f"    Status: {decode_error(error_status)}")
        
        if error_status != 0:
            any_errors = True
            print(f"    → 🔧 Press 'X' in control_arm.py to clear")
        
        print()
    
    print("=" * 70)
    
    if any_errors:
        print("\n⚠️  ERRORS DETECTED!")
        print("\nCommon causes:")
        print("  • OVERLOAD: Motor working too hard (lifting heavy load)")
        print("  • Overheating: Motor getting too hot (let it cool)")
        print("  • Voltage: Check power supply (should be ~11.1V)")
        print("\nOverload errors are NORMAL when lifting - just clear and continue!")
        
        # Offer to clear errors
        response = input("\nClear errors now? (y/n): ").strip().lower()
        if response == 'y':
            clear_errors(portHandler, packetHandler)
        else:
            print("\nTo clear later:")
            print("  1. Run: python control_arm.py")
            print("  2. Press 'X' to clear errors")
    else:
        print("\n✅ All servos OK! No errors detected.")
    
    portHandler.closePort()

def clear_errors(portHandler, packetHandler):
    """Clear all servo errors by rebooting them."""
    print("\n" + "=" * 70)
    print("Clearing errors by rebooting servos...")
    print("=" * 70)
    
    for servo_id in SERVO_IDS:
        name = SERVO_NAMES.get(servo_id, f"Servo {servo_id}")
        
        # Check if this servo has errors
        error_status, comm_result, _ = packetHandler.read1ByteTxRx(
            portHandler, servo_id, ADDR_HARDWARE_ERROR_STATUS
        )
        
        if comm_result == COMM_SUCCESS and error_status != 0:
            print(f"\n{name} (ID {servo_id}): Rebooting...", end=" ")
            
            # Reboot the servo
            comm_result, _ = packetHandler.reboot(portHandler, servo_id)
            
            if comm_result == COMM_SUCCESS:
                print("✓")
                time.sleep(0.5)
                
                # Reconfigure after reboot
                packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_PWM_LIMIT, PWM_LIMIT)
                packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_CURRENT_LIMIT, CURRENT_LIMIT)
                packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
                packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION)
                packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, 1)
                
                print(f"  → Reconfigured with max power settings")
            else:
                print("✗ Failed")
    
    print("\n" + "=" * 70)
    print("Done! Checking status...")
    print("=" * 70)
    
    # Verify
    time.sleep(0.5)
    any_remaining = False
    for servo_id in SERVO_IDS:
        name = SERVO_NAMES.get(servo_id, f"Servo {servo_id}")
        error_status, comm_result, _ = packetHandler.read1ByteTxRx(
            portHandler, servo_id, ADDR_HARDWARE_ERROR_STATUS
        )
        
        if comm_result == COMM_SUCCESS:
            if error_status == 0:
                print(f"{name:12} (ID {servo_id}): ✓ OK")
            else:
                print(f"{name:12} (ID {servo_id}): ⚠️  Still has errors")
                any_remaining = True
    
    if not any_remaining:
        print("\n✅ All errors cleared successfully!")
    else:
        print("\n⚠️  Some errors could not be cleared.")
        print("This may indicate a hardware or power issue.")

if __name__ == "__main__":
    check_servos()

