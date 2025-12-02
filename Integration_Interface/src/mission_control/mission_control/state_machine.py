#!/usr/bin/env python3
"""
Mission Control State Machine 
Uses object_location_interfaces messages
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from object_location_interfaces.msg import RSyncDetectionList, RSyncLocationList
from enum import Enum
import time


class MissionState(Enum):
    """Robot mission states"""
    IDLE = "IDLE"
    DETECTING = "DETECTING"
    APPROACHING = "APPROACHING"
    PICKING = "PICKING"
    DONE = "DONE"


class StateMachineNode(Node):
    """
    Central coordinator that manages robot mission states.
    Integrates detection, navigation, and manipulation modules.
    """
    
    def __init__(self):
        super().__init__('state_machine')
        
        # Current state
        self.state = MissionState.IDLE
        self.target_object = None
        self.state_start_time = time.time()
        
        # Timeouts for each state (seconds)
        self.DETECTION_TIMEOUT = 10.0
        self.APPROACH_TIMEOUT = 30.0
        self.PICKING_TIMEOUT = 15.0
        self.DONE_TIMEOUT = 3.0
        
        # Status flags from other modules
        self.object_detected = False
        self.detected_object_position = None  # Will store (x, y, w, h)
        self.robot_aligned = False
        self.object_picked = False
        self.current_distance = 999.0
        self.current_yaw = 0.0
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/mission/command', self.command_callback, 10)
        
        # Subscribe to David's detection topic (plural 'objects')
        self.detection_sub = self.create_subscription(
            RSyncDetectionList, '/objects/detections', self.detection_callback, 10)
        
        # Subscribe to David's location topic (plural 'objects')
        self.distance_sub = self.create_subscription(
            RSyncLocationList, '/objects/locations', self.distance_callback, 10)
        
        # Subscribe to visual servo status
        self.servo_sub = self.create_subscription(
            String, '/visual_servo/status', self.servo_callback, 10)
        
        # Subscribe to arm status
        self.arm_sub = self.create_subscription(
            String, '/arm/status', self.arm_callback, 10)
        
        # State machine timer (runs at 10 Hz)
        self.timer = self.create_timer(0.1, self.state_machine_loop)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Mission Control State Machine Started')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Subscribed topics:')
        self.get_logger().info('  - /mission/command (commands from web UI)')
        self.get_logger().info('  - /objects/detections (David\'s detections)')
        self.get_logger().info('  - /objects/locations (David\'s locations)')
        self.get_logger().info('  - /visual_servo/status (alignment status)')
        self.get_logger().info('  - /arm/status (arm operation status)')
        self.get_logger().info('')
        self.get_logger().info('Published topics:')
        self.get_logger().info('  - /mission/state (current state)')
        self.get_logger().info('  - /mission/status (human-readable status)')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Ready for commands!')
    
    def command_callback(self, msg):
        """Handle commands from web interface or other sources"""
        command = msg.data.lower()
        
        self.get_logger().info(f'📥 Received command: {command}')
        
        if command == 'stop' or command == 'reset':
            self.get_logger().warn('⛔ STOP command - returning to IDLE')
            self.transition_to(MissionState.IDLE)
            self.target_object = None
            
        elif command.startswith('pick_'):
            # Extract object name (e.g., 'pick_bottle' -> 'bottle')
            self.target_object = command.replace('pick_', '')
            self.get_logger().info(f'🎯 Starting mission to pick: {self.target_object}')
            self.transition_to(MissionState.DETECTING)
            
        elif command == 'home':
            self.get_logger().info('🏠 Return home command received')
            self.transition_to(MissionState.IDLE)
    
    def detection_callback(self, msg):
        """
        Handle object detections from David's detection system
        Message type: RSyncDetectionList
        Structure: msg.detections.item_list contains DetectedItem[]
        """
        if self.state != MissionState.DETECTING:
            return
        
        # Access detection list from David's message structure
        item_list = msg.detections.item_list
        
        # Check if target object is in the detection list
        for item in item_list:
            item_name = item.name.lower()
            
            self.get_logger().debug(f'Detected: {item_name} (confidence: {item.confidence:.2f})')
            
            # Check if this is our target object
            if self.target_object and self.target_object in item_name:
                self.object_detected = True
                
                # Store object position for later use
                if len(item.xywh) >= 4:
                    self.detected_object_position = item.xywh
                    x, y, w, h = item.xywh
                    
                    self.get_logger().info(
                        f'✓ Target "{self.target_object}" detected! '
                        f'Position: x={x}, y={y}, size: {w}x{h}, '
                        f'confidence: {item.confidence:.2f}'
                    )
                break
    
    def distance_callback(self, msg):
        """
        Handle distance and yaw measurements from David's distance node
        Message type: RSyncLocationList
        Structure: msg.locations.location_list contains ItemLocation[]
        """
        # If we're not tracking a target, set default distance
        if not self.target_object:
            self.current_distance = 999.0
            return
        
        # Access location list from David's message structure
        location_list = msg.locations.location_list
        
        # Find our target object in the location list
        for item_location in location_list:
            if self.target_object in item_location.name.lower():
                self.current_distance = item_location.distance
                self.current_yaw = item_location.relative_yaw
                
                # Log detailed info during approach
                if self.state == MissionState.APPROACHING:
                    self.get_logger().debug(
                        f'Target "{self.target_object}": '
                        f'distance={item_location.distance:.3f}m, '
                        f'yaw={item_location.relative_yaw:.1f}°'
                    )
                return
        
        # Target not in location list
        self.current_distance = 999.0
    
    def servo_callback(self, msg):
        """Handle status from visual servo controller"""
        if msg.data == 'aligned':
            self.robot_aligned = True
            self.get_logger().info('✓ Robot aligned with target')
        elif msg.data == 'aligning':
            self.get_logger().debug('⟳ Robot aligning...')
    
    def arm_callback(self, msg):
        """Handle status from robotic arm"""
        if msg.data == 'picked':
            self.object_picked = True
            self.get_logger().info('✓ Object picked successfully!')
        elif msg.data == 'error':
            self.get_logger().error('✗ Arm reported error - returning to IDLE')
            self.transition_to(MissionState.IDLE)
        elif msg.data == 'picking':
            self.get_logger().debug('🦾 Arm executing pick...')
    
    def transition_to(self, new_state):
        """Transition to a new state"""
        if self.state != new_state:
            self.get_logger().info(
                f'🔄 State transition: {self.state.value} → {new_state.value}'
            )
            self.state = new_state
            self.state_start_time = time.time()
            
            # Reset flags on state change
            if new_state == MissionState.IDLE:
                self.object_detected = False
                self.detected_object_position = None
                self.robot_aligned = False
                self.object_picked = False
                self.target_object = None
            elif new_state == MissionState.APPROACHING:
                self.robot_aligned = False
            elif new_state == MissionState.PICKING:
                self.object_picked = False
            
            # Publish new state
            state_msg = String()
            state_msg.data = self.state.value
            self.state_pub.publish(state_msg)
    
    def get_state_duration(self):
        """Get time spent in current state"""
        return time.time() - self.state_start_time
    
    def publish_status(self, message):
        """Publish human-readable status"""
        status_msg = String()
        status_msg.data = f"{self.state.value}: {message}"
        self.status_pub.publish(status_msg)
    
    def state_machine_loop(self):
        """Main state machine logic - runs at 10 Hz"""
        
        if self.state == MissionState.IDLE:
            self.publish_status("Ready for commands")
        
        elif self.state == MissionState.DETECTING:
            if self.object_detected:
                self.publish_status(f"Object detected: {self.target_object}")
                self.transition_to(MissionState.APPROACHING)
            elif self.get_state_duration() > self.DETECTION_TIMEOUT:
                self.get_logger().warn(
                    f'⏱️ Detection timeout - {self.target_object} not found'
                )
                self.publish_status(f"Timeout - {self.target_object} not found")
                self.transition_to(MissionState.IDLE)
            else:
                remaining = int(self.DETECTION_TIMEOUT - self.get_state_duration())
                self.publish_status(
                    f"Searching for {self.target_object}... ({remaining}s)"
                )
        
        elif self.state == MissionState.APPROACHING:
            if self.robot_aligned:
                self.get_logger().info('✓ Approach complete - ready to pick')
                self.publish_status("Aligned - ready to pick")
                self.transition_to(MissionState.PICKING)
            elif self.get_state_duration() > self.APPROACH_TIMEOUT:
                self.get_logger().warn('⏱️ Approach timeout')
                self.publish_status("Approach timeout")
                self.transition_to(MissionState.IDLE)
            else:
                if self.current_distance < 900:
                    self.publish_status(
                        f"Approaching (distance: {self.current_distance:.2f}m)"
                    )
                else:
                    self.publish_status("Approaching target...")
        
        elif self.state == MissionState.PICKING:
            if self.object_picked:
                self.get_logger().info('🎉 Pick successful!')
                self.publish_status("Pick successful!")
                self.transition_to(MissionState.DONE)
            elif self.get_state_duration() > self.PICKING_TIMEOUT:
                self.get_logger().warn('⏱️ Pick timeout')
                self.publish_status("Pick timeout")
                self.transition_to(MissionState.IDLE)
            else:
                remaining = int(self.PICKING_TIMEOUT - self.get_state_duration())
                self.publish_status(f"Picking... ({remaining}s)")
        
        elif self.state == MissionState.DONE:
            if self.get_state_duration() > self.DONE_TIMEOUT:
                self.get_logger().info('✓ Mission complete - returning to IDLE')
                self.publish_status("Mission complete")
                self.transition_to(MissionState.IDLE)
            else:
                self.publish_status(f"Mission complete: {self.target_object}")


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down state machine...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()