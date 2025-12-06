#!/usr/bin/env python3
"""
Simple keyboard control for PincherX100 robotic arm.

Controls:
  A/D - Base rotation (left/right)
  W/S - Shoulder movement (up/down)
  E/C - Elbow movement (up/down)
  R/V - Wrist movement (up/down)
  J/K - Gripper (close/open)
  H   - Home (center all servos to safe position)
  P   - Show all positions & error status
  X   - Clear hardware errors (red flashing LED = overload)
  Q   - Quit program

MAXIMUM POWER MODE ENABLED:
  - PWM Limit: 885/885 (100% voltage)
  - Current Limit: 1193/1193 (2.69A max for XL430-W250)
  - Profile Velocity: 150 (slower movement = more torque for lifting)
  - This gives maximum possible torque for the shoulder to lift against gravity

ERROR MONITORING:
  - Red flashing LED = Hardware error (usually OVERLOAD)
  - Press 'X' to reboot servos and clear errors
  - Servos are automatically reconfigured after clearing
  - Position limits prevent extreme/dangerous positions
  - Overload errors are NORMAL when lifting heavy loads
  
  If errors persist after clearing:
  - Check power supply (should be ~11.1V)
  - Let servos cool down if overheating
  - Reduce load or use slower movements
  - Check for mechanical obstructions

Tuning Parameters (edit below):
  PROFILE_VELOCITY: Higher = slower movement BUT MORE TORQUE (range: 0-1023)
                    Currently: 250 (gentle, high torque)
  PROFILE_ACCELERATION: Higher = smoother, gentler acceleration (range: 0-32767)
                        Currently: 150 (smooth acceleration)
  DEGREES_PER_STEP: Per-joint step sizes:
                    - Base: 2° (gentle, precise)
                    - Shoulder: 8° (needs larger steps to generate torque)
                    - Elbow: 3° (gentle)
                    - Wrist: 3° (gentle)
  PWM_LIMIT: Maximum power (885 = 100%, DO NOT EXCEED)
  CURRENT_LIMIT: Maximum current (1193 = 2.69A max, DO NOT EXCEED)
  
NOTE: Dynamixel servos have built-in PID controllers. Profile Velocity and
      Acceleration settings control movement smoothness and torque availability.
      Higher values = slower but more controlled movement with better torque.
"""

import os
import sys
import tty
import termios
import threading
import time
import yaml
from pathlib import Path
from dynamixel_sdk import *

# Control table addresses for XL430-W250 (Protocol 2.0)
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11
ADDR_PROFILE_VELOCITY = 112       # Controls movement speed
ADDR_PROFILE_ACCELERATION = 108   # Controls acceleration
ADDR_GOAL_PWM = 100               # Goal PWM (power)
ADDR_PWM_LIMIT = 36               # Maximum PWM limit
ADDR_GOAL_CURRENT = 102           # Goal current (torque)
ADDR_CURRENT_LIMIT = 38           # Maximum current limit
ADDR_HARDWARE_ERROR_STATUS = 70   # Hardware error status
ADDR_MIN_POSITION_LIMIT = 52      # Minimum position limit
ADDR_MAX_POSITION_LIMIT = 48      # Maximum position limit

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000  # Default to 1000000 (confirmed working)

# Config file path
CONFIG_FILE = Path(__file__).parent / 'servo_config.yaml'

# Servo IDs - Default PincherX100 configuration
ID_BASE = 1      # Waist/Base rotation (A/D keys)
ID_SHOULDER = 2  # Shoulder joint (W/S keys)
ID_ELBOW = 3     # Elbow joint (E/C keys)
ID_WRIST = 4     # Wrist joint (R/V keys)
ID_GRIPPER = 5   # Gripper (J/K keys)

# Servo configuration (loaded from YAML file)
SERVO_CONFIG = None
POSITION_LIMITS = {}

# Movement parameters - Per-joint degree steps
# Each joint has different step size based on load and desired smoothness
# Base: 2 degrees (gentle, precise movement)
# Shoulder: 8 degrees (needs larger steps to generate enough torque to lift)
# Elbow: 3 degrees (gentle movement)
# Wrist: 3 degrees (gentle movement)
DEGREES_PER_STEP = {
    ID_BASE: 2,        # Gentle, precise base rotation
    ID_SHOULDER: 8,    # Larger steps for torque (can't lift at 6 degrees)
    ID_ELBOW: 8,       # Gentle elbow movement
    ID_WRIST: 6,       # Gentle wrist movement
}

# Convert degrees to position steps (XL430 has 4096 positions per 360 degrees)
POSITION_STEPS = {
    servo_id: int((4096 / 360) * degrees) 
    for servo_id, degrees in DEGREES_PER_STEP.items()
}

# Position limits will be loaded from config file
# These prevent the arm from moving into dangerous positions

# Profile settings for smooth, gentle movement
# Profile Velocity: 0-1023 (0 = max speed, unlimited). Higher = slower
# IMPORTANT: Higher velocity = slower movement BUT MORE TORQUE available
# We use higher values for slower, gentler movement while maintaining torque
PROFILE_VELOCITY = 1000        # Slower movement (higher = slower but MORE torque available)
                              # Range: 0-1023. Increase this to slow down movement speed
                              # Was 250, increased for even slower, gentler movement

# Profile Acceleration: 0-32767 (0 = instant, no acceleration control)
# Higher values = smoother, gentler acceleration (less jerky)
PROFILE_ACCELERATION = 32000    # Smoother acceleration (higher = gentler acceleration)
                              # Was 150, increased to prevent fast acceleration
                              # Increase this to make acceleration less aggressive

# Per-joint profile settings (optional - can override global settings)
# Shoulder needs special attention due to heavy load
PROFILE_VELOCITY_PER_JOINT = {
    ID_SHOULDER: 1000,  # Even slower for shoulder to maximize torque
    # Other joints use global PROFILE_VELOCITY
}

PROFILE_ACCELERATION_PER_JOINT = {
    ID_SHOULDER: 32000,  # Smoother acceleration for shoulder
    # Other joints use global PROFILE_ACCELERATION
}

# Power/Torque settings (MAXIMUM values for XL430-W250)
PWM_LIMIT = 885              # Max PWM (0-885 = 0-100% voltage) - MAXIMUM
CURRENT_LIMIT = 1193         # Max current (0-1193 = 0-2.69A) - MAXIMUM
# Note: These are the absolute maximum safe values for XL430-W250

# Gripper parameters (will be loaded from config)
GRIPPER_CLOSED_POS = None
GRIPPER_OPEN_POS = None
GRIPPER_POS_RANGE = None
GRIPPER_GAP_RANGE = None
GRIPPER_STEP_CM = 0.2  # Adjust by 0.2cm per keypress
GRIPPER_STEP_POS = None

def load_servo_config():
    """Load servo configuration from YAML file."""
    global SERVO_CONFIG, POSITION_LIMITS
    global GRIPPER_CLOSED_POS, GRIPPER_OPEN_POS, GRIPPER_POS_RANGE, GRIPPER_GAP_RANGE, GRIPPER_STEP_POS
    
    if not CONFIG_FILE.exists():
        print(f"ERROR: Config file not found: {CONFIG_FILE}")
        print("Please create servo_config.yaml in the same directory as control_arm.py")
        sys.exit(1)
    
    try:
        with open(CONFIG_FILE, 'r') as f:
            SERVO_CONFIG = yaml.safe_load(f)
        
        # Build position limits dictionary from config
        servo_mapping = {
            'base': ID_BASE,
            'shoulder': ID_SHOULDER,
            'elbow': ID_ELBOW,
            'wrist': ID_WRIST,
            'gripper': ID_GRIPPER
        }
        
        POSITION_LIMITS = {}
        for name, servo_id in servo_mapping.items():
            if name in SERVO_CONFIG['servos']:
                config = SERVO_CONFIG['servos'][name]
                pos_min = config['position_range']['min']
                pos_max = config['position_range']['max']
                # Ensure min < max
                POSITION_LIMITS[servo_id] = (min(pos_min, pos_max), max(pos_min, pos_max))
        
        # Load gripper parameters
        if 'gripper' in SERVO_CONFIG['servos']:
            gripper_config = SERVO_CONFIG['servos']['gripper']
            GRIPPER_CLOSED_POS = gripper_config['position_range']['min']
            GRIPPER_OPEN_POS = gripper_config['position_range']['max']
            GRIPPER_POS_RANGE = abs(GRIPPER_OPEN_POS - GRIPPER_CLOSED_POS)
            GRIPPER_GAP_RANGE = gripper_config['gap_range']['max'] - gripper_config['gap_range']['min']
            GRIPPER_STEP_POS = int(GRIPPER_STEP_CM * GRIPPER_POS_RANGE / GRIPPER_GAP_RANGE)
        
        print(f"Loaded servo configuration from {CONFIG_FILE}")
        return True
        
    except Exception as e:
        print(f"ERROR: Failed to load config file: {e}")
        sys.exit(1)

# Load configuration on import
load_servo_config()

class ArmController:
    def __init__(self, baudrate=57600):
        self.baudrate = baudrate
        self.portHandler = None
        self.packetHandler = None
        self.running = False
        self.error_count = 0
        
    def initialize(self):
        """Initialize connection to servos."""
        print(f"Initializing connection to {DEVICENAME} at {self.baudrate} baud...")
        
        # Initialize PortHandler and PacketHandler
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        # Open port
        if not self.portHandler.openPort():
            print(f"Failed to open port {DEVICENAME}")
            return False
        
        print(f"Opened port {DEVICENAME}")
        
        # Set baudrate
        if not self.portHandler.setBaudRate(self.baudrate):
            print(f"Failed to set baudrate to {self.baudrate}")
            return False
        
        print(f"Set baudrate to {self.baudrate}")
        
        # Configure all servos for smooth movement
        servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST, ID_GRIPPER]
        
        print("\nConfiguring servos for MAXIMUM POWER...")
        for servo_id in servo_ids:
            # Disable torque to change settings
            self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, ADDR_TORQUE_ENABLE, 0
            )
            
            # Set MAXIMUM PWM Limit (power/voltage)
            self.packetHandler.write2ByteTxRx(
                self.portHandler, servo_id, ADDR_PWM_LIMIT, PWM_LIMIT
            )
            
            # Set MAXIMUM Current Limit (torque)
            self.packetHandler.write2ByteTxRx(
                self.portHandler, servo_id, ADDR_CURRENT_LIMIT, CURRENT_LIMIT
            )
            
            # Set Profile Velocity (use per-joint if specified, else global)
            # Higher velocity = slower movement BUT MORE TORQUE available
            velocity = PROFILE_VELOCITY_PER_JOINT.get(servo_id, PROFILE_VELOCITY)
            self.packetHandler.write4ByteTxRx(
                self.portHandler, servo_id, ADDR_PROFILE_VELOCITY, velocity
            )
            
            # Set Profile Acceleration (use per-joint if specified, else global)
            # Higher acceleration = smoother, gentler movement
            acceleration = PROFILE_ACCELERATION_PER_JOINT.get(servo_id, PROFILE_ACCELERATION)
            self.packetHandler.write4ByteTxRx(
                self.portHandler, servo_id, ADDR_PROFILE_ACCELERATION, acceleration
            )
            
            # Enable torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, ADDR_TORQUE_ENABLE, 1
            )
            
            if dxl_comm_result == COMM_SUCCESS:
                print(f"  Servo ID {servo_id}: MAX POWER configured and enabled")
            else:
                print(f"  Warning: Could not enable servo ID {servo_id}")
        
        print("\n" + "=" * 60)
        print("MAXIMUM POWER MODE ENABLED - GENTLE MOVEMENT")
        print("=" * 60)
        print(f"PWM Limit: {PWM_LIMIT}/885 (100% voltage)")
        print(f"Current Limit: {CURRENT_LIMIT}/1193 (2.69A max)")
        print(f"\nProfile Settings (Higher = Slower but MORE Torque):")
        print(f"  Global Velocity: {PROFILE_VELOCITY} (gentle movement)")
        print(f"  Global Acceleration: {PROFILE_ACCELERATION} (smooth)")
        if ID_SHOULDER in PROFILE_VELOCITY_PER_JOINT:
            print(f"  Shoulder Velocity: {PROFILE_VELOCITY_PER_JOINT[ID_SHOULDER]} (extra torque)")
            print(f"  Shoulder Acceleration: {PROFILE_ACCELERATION_PER_JOINT[ID_SHOULDER]} (smoother)")
        print(f"\nPer-Joint Step Sizes:")
        print(f"  Base: {DEGREES_PER_STEP[ID_BASE]}° per press")
        print(f"  Shoulder: {DEGREES_PER_STEP[ID_SHOULDER]}° per press (needs larger steps)")
        print(f"  Elbow: {DEGREES_PER_STEP[ID_ELBOW]}° per press")
        print(f"  Wrist: {DEGREES_PER_STEP[ID_WRIST]}° per press")
        print("\nShoulder optimized for maximum lifting power!")
        print("=" * 60)
        return True
    
    def get_present_position(self, servo_id):
        """Read current position of a servo with retry logic."""
        max_retries = 3
        for attempt in range(max_retries):
            position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, servo_id, ADDR_PRESENT_POSITION
            )
            
            if dxl_comm_result == COMM_SUCCESS:
                # Validate position is in reasonable range
                if 0 <= position <= 4095:
                    return position
                else:
                    # Invalid position, retry
                    time.sleep(0.01)
                    continue
            
            # Communication failed, retry
            if attempt < max_retries - 1:
                time.sleep(0.01)
        
        # All retries failed
        return None
    
    def check_hardware_error(self, servo_id):
        """Check for hardware errors on a servo."""
        error_status, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(
            self.portHandler, servo_id, ADDR_HARDWARE_ERROR_STATUS
        )
        
        if dxl_comm_result == COMM_SUCCESS and error_status != 0:
            error_msgs = []
            if error_status & 0x01:
                error_msgs.append("Input Voltage")
            if error_status & 0x04:
                error_msgs.append("Overheating")
            if error_status & 0x08:
                error_msgs.append("Motor Encoder")
            if error_status & 0x10:
                error_msgs.append("Electrical Shock")
            if error_status & 0x20:
                error_msgs.append("OVERLOAD")  # Most common with lifting
            
            if error_msgs:
                return True, error_msgs
        
        return False, []
    
    def clear_hardware_error(self, servo_id):
        """Clear hardware error by rebooting the servo."""
        print(f"  Rebooting servo {servo_id}...", end=" ")
        
        # Use the reboot instruction to clear errors
        dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, servo_id)
        
        if dxl_comm_result == COMM_SUCCESS:
            print("OK")
            # Wait for servo to restart
            time.sleep(0.5)
            
            # Reconfigure the servo after reboot
            # Set PWM and Current limits
            self.packetHandler.write2ByteTxRx(self.portHandler, servo_id, ADDR_PWM_LIMIT, PWM_LIMIT)
            self.packetHandler.write2ByteTxRx(self.portHandler, servo_id, ADDR_CURRENT_LIMIT, CURRENT_LIMIT)
            
            # Set profile settings (use per-joint if specified, else global)
            velocity = PROFILE_VELOCITY_PER_JOINT.get(servo_id, PROFILE_VELOCITY)
            acceleration = PROFILE_ACCELERATION_PER_JOINT.get(servo_id, PROFILE_ACCELERATION)
            self.packetHandler.write4ByteTxRx(self.portHandler, servo_id, ADDR_PROFILE_VELOCITY, velocity)
            self.packetHandler.write4ByteTxRx(self.portHandler, servo_id, ADDR_PROFILE_ACCELERATION, acceleration)
            
            # Enable torque
            self.packetHandler.write1ByteTxRx(self.portHandler, servo_id, ADDR_TORQUE_ENABLE, 1)
            return True
        else:
            print("Failed")
            return False
    
    def set_goal_position(self, servo_id, position):
        """Set goal position for a servo with safety checks."""
        # Apply position limits for safety
        if servo_id in POSITION_LIMITS:
            min_pos, max_pos = POSITION_LIMITS[servo_id]
            position = max(min_pos, min(max_pos, position))
        else:
            # Clamp to valid range if no specific limit
            position = max(0, min(4095, position))
        
        # Check for errors before moving
        has_error, error_msgs = self.check_hardware_error(servo_id)
        if has_error:
            self.error_count += 1
            print(f"\nServo {servo_id} ERROR: {', '.join(error_msgs)}")
            print(f"   Error count: {self.error_count}. Press 'X' to clear errors.")
            return False
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, servo_id, ADDR_GOAL_POSITION, position
        )
        return dxl_comm_result == COMM_SUCCESS
    
    def move_servo_relative(self, servo_id, delta):
        """Move servo by relative amount with safety checks."""
        current_pos = self.get_present_position(servo_id)
        
        if current_pos is None:
            print(f"\nCannot read position from servo {servo_id}")
            return False
        
        # Validate current position is reasonable
        if current_pos < 0 or current_pos > 4095:
            print(f"\nInvalid position {current_pos} from servo {servo_id}")
            return False
        
        new_pos = current_pos + delta
        
        # Debug: Show what we're doing
        # Uncomment for debugging:
        # print(f"\nServo {servo_id}: {current_pos} -> {new_pos} (delta: {delta})")
        
        return self.set_goal_position(servo_id, new_pos)
    
    def move_base_left(self):
        """Rotate base left."""
        step = POSITION_STEPS[ID_BASE]
        before = self.get_present_position(ID_BASE)
        success = self.move_servo_relative(ID_BASE, step)
        after = self.get_present_position(ID_BASE)
        
        if success and before and after:
            deg_before = self.position_to_degree(ID_BASE, before)
            deg_after = self.position_to_degree(ID_BASE, after)
            if deg_before is not None and deg_after is not None:
                print(f"Base: LEFT  pos: {after:4d}  {deg_after:5.1f}° (Δ{deg_after-deg_before:+.1f}°)    ", end='\r')
            else:
                print(f"Base: LEFT  {before:4d} -> {after:4d} (Δ{after-before:+4d}, {DEGREES_PER_STEP[ID_BASE]}°)    ", end='\r')
        else:
            print(f"Base: LEFT  (error reading position)              ", end='\r')
    
    def move_base_right(self):
        """Rotate base right."""
        step = POSITION_STEPS[ID_BASE]
        before = self.get_present_position(ID_BASE)
        success = self.move_servo_relative(ID_BASE, -step)
        after = self.get_present_position(ID_BASE)
        
        if success and before and after:
            deg_before = self.position_to_degree(ID_BASE, before)
            deg_after = self.position_to_degree(ID_BASE, after)
            if deg_before is not None and deg_after is not None:
                print(f"Base: RIGHT pos: {after:4d}  {deg_after:5.1f}° (Δ{deg_after-deg_before:+.1f}°)    ", end='\r')
            else:
                print(f"Base: RIGHT {before:4d} -> {after:4d} (Δ{after-before:+4d}, {DEGREES_PER_STEP[ID_BASE]}°)    ", end='\r')
        else:
            print(f"Base: RIGHT (error reading position)              ", end='\r')
    
    def move_arm_up(self):
        """Move arm up (shoulder)."""
        step = POSITION_STEPS[ID_SHOULDER]
        self.move_servo_relative(ID_SHOULDER, -step)
        pos = self.get_present_position(ID_SHOULDER)
        degree = self.position_to_degree(ID_SHOULDER, pos)
        if degree is not None:
            print(f"Shoulder: UP pos: {pos:4d}  {degree:5.1f}°     ", end='\r')
        else:
            print(f"Shoulder: UP (pos: {pos}, {DEGREES_PER_STEP[ID_SHOULDER]}°)     ", end='\r')
    
    def move_arm_down(self):
        """Move arm down (shoulder)."""
        step = POSITION_STEPS[ID_SHOULDER]
        self.move_servo_relative(ID_SHOULDER, step)
        pos = self.get_present_position(ID_SHOULDER)
        degree = self.position_to_degree(ID_SHOULDER, pos)
        if degree is not None:
            print(f"Shoulder: DOWN pos: {pos:4d}  {degree:5.1f}°   ", end='\r')
        else:
            print(f"Shoulder: DOWN (pos: {pos}, {DEGREES_PER_STEP[ID_SHOULDER]}°)   ", end='\r')
    
    def move_elbow_up(self):
        """Move elbow up."""
        step = POSITION_STEPS[ID_ELBOW]
        self.move_servo_relative(ID_ELBOW, -step)  # Inverted like shoulder
        pos = self.get_present_position(ID_ELBOW)
        degree = self.position_to_degree(ID_ELBOW, pos)
        if degree is not None:
            print(f"Elbow: UP pos: {pos:4d}  {degree:5.1f}°        ", end='\r')
        else:
            print(f"Elbow: UP (pos: {pos}, {DEGREES_PER_STEP[ID_ELBOW]}°)        ", end='\r')
    
    def move_elbow_down(self):
        """Move elbow down."""
        step = POSITION_STEPS[ID_ELBOW]
        self.move_servo_relative(ID_ELBOW, step)   # Inverted like shoulder
        pos = self.get_present_position(ID_ELBOW)
        degree = self.position_to_degree(ID_ELBOW, pos)
        if degree is not None:
            print(f"Elbow: DOWN pos: {pos:4d}  {degree:5.1f}°      ", end='\r')
        else:
            print(f"Elbow: DOWN (pos: {pos}, {DEGREES_PER_STEP[ID_ELBOW]}°)      ", end='\r')
    
    def move_wrist_up(self):
        """Move wrist up."""
        step = POSITION_STEPS[ID_WRIST]
        self.move_servo_relative(ID_WRIST, -step)  # Inverted like shoulder
        pos = self.get_present_position(ID_WRIST)
        degree = self.position_to_degree(ID_WRIST, pos)
        if degree is not None:
            print(f"Wrist: UP pos: {pos:4d}  {degree:5.1f}°        ", end='\r')
        else:
            print(f"Wrist: UP (pos: {pos}, {DEGREES_PER_STEP[ID_WRIST]}°)        ", end='\r')
    
    def move_wrist_down(self):
        """Move wrist down."""
        step = POSITION_STEPS[ID_WRIST]
        self.move_servo_relative(ID_WRIST, step)   # Inverted like shoulder
        pos = self.get_present_position(ID_WRIST)
        degree = self.position_to_degree(ID_WRIST, pos)
        if degree is not None:
            print(f"Wrist: DOWN pos: {pos:4d}  {degree:5.1f}°      ", end='\r')
        else:
            print(f"Wrist: DOWN (pos: {pos}, {DEGREES_PER_STEP[ID_WRIST]}°)      ", end='\r')
    
    def position_to_degree(self, servo_id, position):
        """Convert servo position to degrees based on config."""
        if servo_id not in POSITION_LIMITS:
            return None
        
        servo_name_map = {
            ID_BASE: 'base',
            ID_SHOULDER: 'shoulder',
            ID_ELBOW: 'elbow',
            ID_WRIST: 'wrist'
        }
        
        if servo_id not in servo_name_map:
            return None
        
        servo_name = servo_name_map[servo_id]
        if servo_name not in SERVO_CONFIG['servos']:
            return None
        
        config = SERVO_CONFIG['servos'][servo_name]
        pos_min = config['position_range']['min']
        pos_max = config['position_range']['max']
        deg_min = config['degree_range']['min']
        deg_max = config['degree_range']['max']
        
        # Clamp position to range
        position = max(min(pos_min, pos_max), min(max(pos_min, pos_max), position))
        
        # Base has inverse relationship (position decreases as degree increases)
        if servo_id == ID_BASE:
            # Position 3093 = 0°, Position 999 = 180°
            if position >= pos_max:  # pos_max is 3093 (0°)
                return deg_min
            elif position <= pos_min:  # pos_min is 999 (180°)
                return deg_max
            else:
                # Inverse linear interpolation
                degree = deg_min + (pos_max - position) * (deg_max - deg_min) / (pos_max - pos_min)
        else:
            # Normal relationship (position increases as degree increases)
            if position <= pos_min:
                return deg_min
            elif position >= pos_max:
                return deg_max
            else:
                # Linear interpolation
                degree = deg_min + (position - pos_min) * (deg_max - deg_min) / (pos_max - pos_min)
        
        return round(degree, 1)
    
    def position_to_gap(self, position):
        """Convert gripper position to gap in cm."""
        if GRIPPER_CLOSED_POS is None or GRIPPER_OPEN_POS is None:
            return None
        
        if position < GRIPPER_CLOSED_POS:
            if 'gripper' in SERVO_CONFIG['servos']:
                return SERVO_CONFIG['servos']['gripper']['gap_range']['min']
            return 4.0
        elif position > GRIPPER_OPEN_POS:
            if 'gripper' in SERVO_CONFIG['servos']:
                return SERVO_CONFIG['servos']['gripper']['gap_range']['max']
            return 6.8
        else:
            # Linear interpolation
            if GRIPPER_POS_RANGE == 0:
                return None
            gap_min = SERVO_CONFIG['servos']['gripper']['gap_range']['min']
            gap_max = SERVO_CONFIG['servos']['gripper']['gap_range']['max']
            gap = gap_min + (position - GRIPPER_CLOSED_POS) * (gap_max - gap_min) / GRIPPER_POS_RANGE
            return round(gap, 2)
    
    def gripper_open(self):
        """Open gripper by 0.2cm."""
        current_pos = self.get_present_position(ID_GRIPPER)
        if current_pos is None:
            print(f"Gripper: Cannot read position              ", end='\r')
            return
        
        # Move towards open position (increase position)
        new_pos = current_pos + GRIPPER_STEP_POS
        
        # Clamp to limits
        if new_pos > GRIPPER_OPEN_POS:
            new_pos = GRIPPER_OPEN_POS
        
        success = self.set_goal_position(ID_GRIPPER, new_pos)
        if success:
            gap = self.position_to_gap(new_pos)
            print(f"Gripper: OPEN  pos: {new_pos:4d}  gap: {gap:.2f}cm    ", end='\r')
        else:
            print(f"Gripper: OPEN  (error)                    ", end='\r')
    
    def gripper_close(self):
        """Close gripper by 0.2cm."""
        current_pos = self.get_present_position(ID_GRIPPER)
        if current_pos is None:
            print(f"Gripper: Cannot read position              ", end='\r')
            return
        
        # Move towards closed position (decrease position)
        new_pos = current_pos - GRIPPER_STEP_POS
        
        # Clamp to limits
        if new_pos < GRIPPER_CLOSED_POS:
            new_pos = GRIPPER_CLOSED_POS
        
        success = self.set_goal_position(ID_GRIPPER, new_pos)
        if success:
            gap = self.position_to_gap(new_pos)
            print(f"Gripper: CLOSE pos: {new_pos:4d}  gap: {gap:.2f}cm    ", end='\r')
        else:
            print(f"Gripper: CLOSE (error)                    ", end='\r')
    
    def show_all_positions(self):
        """Display current positions and status of all servos."""
        print("\n" + "=" * 70)
        servo_ids = [
            (ID_BASE, "Base"),
            (ID_SHOULDER, "Shoulder"),
            (ID_ELBOW, "Elbow"),
            (ID_WRIST, "Wrist"),
            (ID_GRIPPER, "Gripper")
        ]
        
        for servo_id, name in servo_ids:
            pos = self.get_present_position(servo_id)
            has_error, error_msgs = self.check_hardware_error(servo_id)
            
            if servo_id in POSITION_LIMITS:
                min_pos, max_pos = POSITION_LIMITS[servo_id]
                limit_info = f"[{min_pos}-{max_pos}]"
            else:
                limit_info = "[0-4095]"
            
            if pos is not None:
                status = "ERROR" if has_error else "OK"
                # Show gap for gripper
                if servo_id == ID_GRIPPER:
                    gap = self.position_to_gap(pos)
                    if gap is not None:
                        print(f"  {name:12} (ID {servo_id}): {pos:4d} {limit_info:15} gap: {gap:.2f}cm  {status}")
                    else:
                        print(f"  {name:12} (ID {servo_id}): {pos:4d} {limit_info:15} {status}")
                else:
                    # Show degrees for other servos
                    degree = self.position_to_degree(servo_id, pos)
                    if degree is not None:
                        print(f"  {name:12} (ID {servo_id}): {pos:4d} {limit_info:15} {degree:5.1f}°  {status}")
                    else:
                        print(f"  {name:12} (ID {servo_id}): {pos:4d} {limit_info:15} {status}")
                if has_error:
                    print(f"               -> {', '.join(error_msgs)}")
        
        print("=" * 70)
    
    def center_all_servos(self):
        """Move all servos to center/safe position."""
        print("\n" + "=" * 70)
        print("Moving all servos to center position...")
        print("=" * 70)
        
        # Safe center positions for each servo
        center_positions = {
            ID_BASE: 2048,      # Center (180 degrees)
            ID_SHOULDER: 2048,  # Center
            ID_ELBOW: 2048,     # Center
            ID_WRIST: 2048,     # Center
            ID_GRIPPER: 2048,   # Center (half open)
        }
        
        servo_names = {
            ID_BASE: "Base",
            ID_SHOULDER: "Shoulder",
            ID_ELBOW: "Elbow",
            ID_WRIST: "Wrist",
            ID_GRIPPER: "Gripper"
        }
        
        for servo_id, target_pos in center_positions.items():
            name = servo_names.get(servo_id, f"Servo {servo_id}")
            current_pos = self.get_present_position(servo_id)
            
            if current_pos is not None:
                print(f"{name:12} (ID {servo_id}): {current_pos:4d} -> {target_pos:4d}...", end=" ")
                success = self.set_goal_position(servo_id, target_pos)
                if success:
                    print("OK")
                else:
                    print("Failed")
            else:
                print(f"{name:12} (ID {servo_id}): Cannot read position")
        
        print("\nWaiting for movement to complete...")
        time.sleep(2)
        
        print("\nVerifying positions:")
        for servo_id in center_positions.keys():
            name = servo_names.get(servo_id, f"Servo {servo_id}")
            pos = self.get_present_position(servo_id)
            if pos:
                print(f"  {name:12} (ID {servo_id}): {pos:4d}")
        
        print("=" * 70)
        print()
    
    def clear_all_errors(self):
        """Clear errors on all servos by rebooting them."""
        print("\n" + "=" * 70)
        print("Clearing errors (rebooting servos with errors)...")
        print("=" * 70)
        
        servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST, ID_GRIPPER]
        servo_names = {
            ID_BASE: "Base",
            ID_SHOULDER: "Shoulder", 
            ID_ELBOW: "Elbow",
            ID_WRIST: "Wrist",
            ID_GRIPPER: "Gripper"
        }
        
        errors_found = False
        
        for servo_id in servo_ids:
            has_error, error_msgs = self.check_hardware_error(servo_id)
            if has_error:
                errors_found = True
                name = servo_names.get(servo_id, f"Servo {servo_id}")
                print(f"\n{name} (ID {servo_id}): {', '.join(error_msgs)}")
                success = self.clear_hardware_error(servo_id)
                if success:
                    print(f"  -> Error cleared")
                else:
                    print(f"  -> Failed to clear error")
        
        if not errors_found:
            print("\nNo errors found on any servos.")
        else:
            print("\n" + "=" * 70)
            print("Reconfiguration complete!")
            print("=" * 70)
            # Verify errors are cleared
            print("\nVerifying...")
            time.sleep(0.5)
            remaining_errors = False
            for servo_id in servo_ids:
                has_error, error_msgs = self.check_hardware_error(servo_id)
                if has_error:
                    remaining_errors = True
                    name = servo_names.get(servo_id, f"Servo {servo_id}")
                    print(f"  {name} (ID {servo_id}): Still has errors: {', '.join(error_msgs)}")
            
            if not remaining_errors:
                print("All errors successfully cleared!")
            else:
                print("\nSome errors remain. This may indicate:")
                print("  - Hardware problem (check connections)")
                print("  - Power supply issue (check voltage)")
                print("  - Mechanical obstruction (check for binding)")
        
        self.error_count = 0
        print()
    
    def shutdown(self):
        """Disable torque and close port."""
        print("\n\nShutting down...")
        
        # Disable torque for all servos
        servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST, ID_GRIPPER]
        for servo_id in servo_ids:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, ADDR_TORQUE_ENABLE, 0
            )
        
        # Close port
        if self.portHandler:
            self.portHandler.closePort()
        
        print("Arm controller shutdown complete.")

def get_key():
    """Get a single keypress from terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def print_instructions():
    """Print control instructions."""
    print("\n" + "=" * 70)
    print("PincherX100 Keyboard Control")
    print("=" * 70)
    print("\nControls:")
    print("  A/D - Base rotation (Left/Right)")
    print("  W/S - Shoulder movement (Up/Down)")
    print("  E/C - Elbow movement (Up/Down)")
    print("  R/V - Wrist movement (Up/Down)")
    print("  J/K - Gripper (Close/Open by 0.2cm per press)")
    print("\nUtilities:")
    print("  H   - Home (center all servos to safe position)")
    print("  P   - Show all positions & status")
    print("  X   - Clear hardware errors (red flashing LED)")
    print("  Q   - Quit")
    print("\n" + "=" * 70)
    print("\nSafety:")
    print("  • Position limits enabled to prevent damage")
    print("  • Red flashing LED = Overload error (press X to clear)")
    print("  • Per-joint step sizes:")
    print(f"    - Base: {DEGREES_PER_STEP[ID_BASE]}° per press (gentle)")
    print(f"    - Shoulder: {DEGREES_PER_STEP[ID_SHOULDER]}° per press (needs larger steps for torque)")
    print(f"    - Elbow: {DEGREES_PER_STEP[ID_ELBOW]}° per press (gentle)")
    print(f"    - Wrist: {DEGREES_PER_STEP[ID_WRIST]}° per press (gentle)")
    print("  • Press H if arm is in strange position")
    print("\n" + "=" * 70)
    print("\nPress keys to control the arm...\n")

def main():
    # Check if alternative baudrate provided as argument
    baudrate = BAUDRATE
    if len(sys.argv) > 1:
        try:
            baudrate = int(sys.argv[1])
            print(f"Using baudrate: {baudrate}")
        except ValueError:
            print(f"Invalid baudrate argument. Using default: {BAUDRATE}")
    
    # Initialize controller
    controller = ArmController(baudrate)
    
    if not controller.initialize():
        print("\nFailed to initialize arm controller.")
        print("Please run 'python scan_servos.py' first to find your servos.")
        sys.exit(1)
    
    print_instructions()
    
    try:
        while True:
            key = get_key().lower()
            
            if key == 'q':
                break
            elif key == 'h':
                controller.center_all_servos()
            elif key == 'p':
                controller.show_all_positions()
            elif key == 'x':
                controller.clear_all_errors()
            elif key == 'a':
                controller.move_base_left()
            elif key == 'd':
                controller.move_base_right()
            elif key == 'w':
                controller.move_arm_up()
            elif key == 's':
                controller.move_arm_down()
            elif key == 'e':
                controller.move_elbow_up()
            elif key == 'c':
                controller.move_elbow_down()
            elif key == 'r':
                controller.move_wrist_up()
            elif key == 'v':
                controller.move_wrist_down()
            elif key == 'j':
                controller.gripper_close()
            elif key == 'k':
                controller.gripper_open()
            elif key == '\x03':  # Ctrl+C
                break
            
            time.sleep(0.01)  # Small delay for responsive control
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()

