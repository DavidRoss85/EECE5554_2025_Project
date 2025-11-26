#!/usr/bin/env python3
"""
Test script for individual joint control.
Useful for verifying servo connections and directions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import time
import math


class JointTester(Node):
    """Node for testing individual joints."""
    
    def __init__(self):
        super().__init__('joint_tester')
        
        self.joint_names = ['base', 'shoulder', 'elbow', 'wrist', 'gripper']
        self.current_positions = [0.0] * 5
        
        # Publisher for joint commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            'joint_position_commands',
            10
        )
        
        # Subscriber for joint states
        self.state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('Joint Tester initialized')
    
    def joint_state_callback(self, msg):
        """Update current joint positions."""
        self.current_positions = list(msg.position)
    
    def send_command(self, positions):
        """Send position command."""
        msg = Float64MultiArray()
        msg.data = positions
        self.cmd_pub.publish(msg)
    
    def test_joint(self, joint_idx, amplitude=0.5, cycles=2):
        """Test a single joint by moving it back and forth."""
        joint_name = self.joint_names[joint_idx]
        self.get_logger().info(f'Testing {joint_name} (index {joint_idx})')
        
        center = self.current_positions[joint_idx]
        
        for i in range(cycles * 4):
            # Alternate between center+amplitude and center-amplitude
            if i % 2 == 0:
                target = center + amplitude
            else:
                target = center - amplitude
            
            positions = list(self.current_positions)
            positions[joint_idx] = target
            
            self.get_logger().info(f'Moving to {target:.2f} rad')
            self.send_command(positions)
            time.sleep(1.5)
        
        # Return to center
        positions = list(self.current_positions)
        positions[joint_idx] = center
        self.send_command(positions)
        self.get_logger().info(f'{joint_name} test complete')
    
    def run_test_sequence(self):
        """Run test sequence for all joints."""
        self.get_logger().info('Starting joint test sequence')
        self.get_logger().info('Each joint will move back and forth')
        
        time.sleep(2)  # Wait for initial joint state
        
        for joint_idx in range(len(self.joint_names)):
            input(f"\nPress Enter to test {self.joint_names[joint_idx]}...")
            self.test_joint(joint_idx, amplitude=0.5, cycles=2)
        
        self.get_logger().info('Test sequence complete')


def main(args=None):
    rclpy.init(args=args)
    
    tester = JointTester()
    
    # Run test in a separate thread
    import threading
    test_thread = threading.Thread(target=tester.run_test_sequence)
    test_thread.start()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



