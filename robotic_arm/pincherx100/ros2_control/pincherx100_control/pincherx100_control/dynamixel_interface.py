#!/usr/bin/env python3
"""
Dynamixel hardware interface for PincherX100.
Handles low-level communication with Dynamixel servos.
"""

from dynamixel_sdk import *
import time

# Control table addresses for XL430-W250 (Protocol 2.0)
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_CURRENT = 126
ADDR_OPERATING_MODE = 11
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108
ADDR_PWM_LIMIT = 36
ADDR_CURRENT_LIMIT = 38
ADDR_VELOCITY_LIMIT = 44
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52

# Protocol version
PROTOCOL_VERSION = 2.0


class DynamixelInterface:
    """Interface for communicating with Dynamixel servos."""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000):
        """Initialize Dynamixel interface."""
        self.port = port
        self.baudrate = baudrate
        self.portHandler = None
        self.packetHandler = None
        self.connected = False
        
    def connect(self):
        """Connect to Dynamixel servos."""
        try:
            self.portHandler = PortHandler(self.port)
            self.packetHandler = PacketHandler(PROTOCOL_VERSION)
            
            if not self.portHandler.openPort():
                return False, f"Failed to open port {self.port}"
            
            if not self.portHandler.setBaudRate(self.baudrate):
                return False, f"Failed to set baudrate to {self.baudrate}"
            
            self.connected = True
            return True, "Connected successfully"
        except Exception as e:
            return False, f"Connection error: {str(e)}"
    
    def disconnect(self):
        """Disconnect from servos."""
        if self.portHandler:
            self.portHandler.closePort()
        self.connected = False
    
    def ping_servo(self, servo_id):
        """Ping a servo to check if it's responding."""
        if not self.connected:
            return False
        
        model_number, result, error = self.packetHandler.ping(
            self.portHandler, servo_id
        )
        return result == COMM_SUCCESS
    
    def enable_torque(self, servo_id):
        """Enable torque for a servo."""
        result, error = self.packetHandler.write1ByteTxRx(
            self.portHandler, servo_id, ADDR_TORQUE_ENABLE, 1
        )
        return result == COMM_SUCCESS
    
    def disable_torque(self, servo_id):
        """Disable torque for a servo."""
        result, error = self.packetHandler.write1ByteTxRx(
            self.portHandler, servo_id, ADDR_TORQUE_ENABLE, 0
        )
        return result == COMM_SUCCESS
    
    def set_position_limits(self, servo_id, min_pos, max_pos):
        """Set position limits for a servo."""
        # Disable torque first
        self.disable_torque(servo_id)
        
        # Set min position
        result1, _ = self.packetHandler.write4ByteTxRx(
            self.portHandler, servo_id, ADDR_MIN_POSITION_LIMIT, min_pos
        )
        
        # Set max position
        result2, _ = self.packetHandler.write4ByteTxRx(
            self.portHandler, servo_id, ADDR_MAX_POSITION_LIMIT, max_pos
        )
        
        # Re-enable torque
        self.enable_torque(servo_id)
        
        return result1 == COMM_SUCCESS and result2 == COMM_SUCCESS
    
    def set_profile_velocity(self, servo_id, velocity):
        """Set profile velocity for smooth movement."""
        result, _ = self.packetHandler.write4ByteTxRx(
            self.portHandler, servo_id, ADDR_PROFILE_VELOCITY, velocity
        )
        return result == COMM_SUCCESS
    
    def set_profile_acceleration(self, servo_id, acceleration):
        """Set profile acceleration for smooth movement."""
        result, _ = self.packetHandler.write4ByteTxRx(
            self.portHandler, servo_id, ADDR_PROFILE_ACCELERATION, acceleration
        )
        return result == COMM_SUCCESS
    
    def set_pwm_limit(self, servo_id, limit):
        """Set PWM limit (power limit)."""
        self.disable_torque(servo_id)
        result, _ = self.packetHandler.write2ByteTxRx(
            self.portHandler, servo_id, ADDR_PWM_LIMIT, limit
        )
        self.enable_torque(servo_id)
        return result == COMM_SUCCESS
    
    def set_current_limit(self, servo_id, limit):
        """Set current limit (torque limit)."""
        self.disable_torque(servo_id)
        result, _ = self.packetHandler.write2ByteTxRx(
            self.portHandler, servo_id, ADDR_CURRENT_LIMIT, limit
        )
        self.enable_torque(servo_id)
        return result == COMM_SUCCESS
    
    def read_position(self, servo_id):
        """Read current position of a servo."""
        position, result, _ = self.packetHandler.read4ByteTxRx(
            self.portHandler, servo_id, ADDR_PRESENT_POSITION
        )
        if result == COMM_SUCCESS:
            return position
        return None
    
    def read_velocity(self, servo_id):
        """Read current velocity of a servo."""
        velocity, result, _ = self.packetHandler.read4ByteTxRx(
            self.portHandler, servo_id, ADDR_PRESENT_VELOCITY
        )
        if result == COMM_SUCCESS:
            # Convert to signed value
            if velocity > 0x7fffffff:
                velocity = velocity - 4294967296
            return velocity
        return None
    
    def read_current(self, servo_id):
        """Read current (load) of a servo."""
        current, result, _ = self.packetHandler.read2ByteTxRx(
            self.portHandler, servo_id, ADDR_PRESENT_CURRENT
        )
        if result == COMM_SUCCESS:
            # Convert to signed value
            if current > 0x7fff:
                current = current - 65536
            return current
        return None
    
    def write_position(self, servo_id, position):
        """Write goal position to a servo."""
        # Clamp position to valid range
        position = max(0, min(4095, int(position)))
        
        result, _ = self.packetHandler.write4ByteTxRx(
            self.portHandler, servo_id, ADDR_GOAL_POSITION, position
        )
        return result == COMM_SUCCESS
    
    def sync_write_positions(self, servo_ids, positions):
        """Write positions to multiple servos simultaneously."""
        groupSyncWrite = GroupSyncWrite(
            self.portHandler, self.packetHandler,
            ADDR_GOAL_POSITION, 4  # 4 bytes for position
        )
        
        for servo_id, position in zip(servo_ids, positions):
            position = max(0, min(4095, int(position)))
            param = [
                DXL_LOBYTE(DXL_LOWORD(position)),
                DXL_HIBYTE(DXL_LOWORD(position)),
                DXL_LOBYTE(DXL_HIWORD(position)),
                DXL_HIBYTE(DXL_HIWORD(position))
            ]
            groupSyncWrite.addParam(servo_id, param)
        
        result = groupSyncWrite.txPacket()
        groupSyncWrite.clearParam()
        return result == COMM_SUCCESS
    
    def sync_read_positions(self, servo_ids):
        """Read positions from multiple servos simultaneously."""
        groupSyncRead = GroupSyncRead(
            self.portHandler, self.packetHandler,
            ADDR_PRESENT_POSITION, 4  # 4 bytes for position
        )
        
        for servo_id in servo_ids:
            groupSyncRead.addParam(servo_id)
        
        result = groupSyncRead.txRxPacket()
        positions = []
        
        if result == COMM_SUCCESS:
            for servo_id in servo_ids:
                if groupSyncRead.isAvailable(servo_id, ADDR_PRESENT_POSITION, 4):
                    position = groupSyncRead.getData(servo_id, ADDR_PRESENT_POSITION, 4)
                    positions.append(position)
                else:
                    positions.append(None)
        
        groupSyncRead.clearParam()
        return positions if all(p is not None for p in positions) else None



