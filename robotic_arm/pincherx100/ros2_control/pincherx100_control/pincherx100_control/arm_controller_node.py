#!/usr/bin/env python3
"""
ROS2 Controller Node for PincherX100 Robotic Arm.

Features:
- Joint state publishing
- Joint position command subscription
- Joint velocity command subscription
- Trajectory execution (via action server)
- Preset pose services
- Safety features (joint limits)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import threading
import time

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from .dynamixel_interface import DynamixelInterface


class ArmControllerNode(Node):
    """Main controller node for PincherX100 arm."""
    
    def __init__(self):
        super().__init__('arm_controller')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyUSB0'),
                ('baudrate', 1000000),
                ('joint_names', ['base', 'shoulder', 'elbow', 'wrist', 'gripper']),
                ('servo_ids', [1, 2, 3, 4, 5]),
                ('publish_rate', 50.0),
                ('profile_velocity', 150),
                ('profile_acceleration', 80),
                ('pwm_limit', 885),
                ('current_limit', 1193),
            ]
        )
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.joint_names = self.get_parameter('joint_names').value
        self.servo_ids = self.get_parameter('servo_ids').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.profile_velocity = self.get_parameter('profile_velocity').value
        self.profile_acceleration = self.get_parameter('profile_acceleration').value
        self.pwm_limit = self.get_parameter('pwm_limit').value
        self.current_limit = self.get_parameter('current_limit').value
        
        # Load joint limits from parameters
        self._load_joint_limits()
        
        # Initialize Dynamixel interface
        self.dxl = DynamixelInterface(self.port, self.baudrate)
        
        # Current joint states
        self.current_positions = [2048] * len(self.joint_names)
        self.current_velocities = [0.0] * len(self.joint_names)
        self.current_efforts = [0.0] * len(self.joint_names)
        self.state_lock = threading.Lock()
        
        # Trajectory execution state
        self.executing_trajectory = False
        self.trajectory_lock = threading.Lock()
        
        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', 10
        )
        self.status_pub = self.create_publisher(
            String, 'arm_status', 10
        )
        
        # Subscribers
        self.position_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'joint_position_commands',
            self.position_command_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.velocity_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'joint_velocity_commands',
            self.velocity_command_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.trajectory_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Action Server for trajectory following
        self.trajectory_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            self.execute_trajectory_action,
            callback_group=self.callback_group
        )
        
        # Services for preset poses
        self.home_service = self.create_service(
            Trigger, 'go_home', self.go_home_callback,
            callback_group=self.callback_group
        )
        
        self.sleep_service = self.create_service(
            Trigger, 'go_sleep', self.go_sleep_callback,
            callback_group=self.callback_group
        )
        
        self.ready_service = self.create_service(
            Trigger, 'go_ready', self.go_ready_callback,
            callback_group=self.callback_group
        )
        
        self.gripper_open_service = self.create_service(
            Trigger, 'gripper_open', self.gripper_open_callback,
            callback_group=self.callback_group
        )
        
        self.gripper_close_service = self.create_service(
            Trigger, 'gripper_close', self.gripper_close_callback,
            callback_group=self.callback_group
        )
        
        # Initialize connection
        self.initialize_hardware()
        
        # Start joint state publisher
        self.state_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_joint_states
        )
        
        self.get_logger().info('PincherX100 Controller Node initialized')
    
    def _load_joint_limits(self):
        """Load joint limits from parameters."""
        # Default limits (full range)
        default_limits = {
            'base': {'min': 0, 'max': 4095},
            'shoulder': {'min': 0, 'max': 4095},
            'elbow': {'min': 0, 'max': 4095},
            'wrist': {'min': 0, 'max': 4095},
            'gripper': {'min': 0, 'max': 4095},
        }
        
        self.joint_limits = {}
        for joint in self.joint_names:
            self.declare_parameter(f'limits.{joint}.min', default_limits[joint]['min'])
            self.declare_parameter(f'limits.{joint}.max', default_limits[joint]['max'])
            
            self.joint_limits[joint] = {
                'min': self.get_parameter(f'limits.{joint}.min').value,
                'max': self.get_parameter(f'limits.{joint}.max').value,
            }
        
        # Preset poses
        self.declare_parameter('poses.home', [2048, 2048, 2048, 2048, 2048])
        self.declare_parameter('poses.sleep', [2048, 1024, 512, 2048, 2048])
        self.declare_parameter('poses.ready', [2048, 2500, 2200, 2048, 2048])
        self.declare_parameter('poses.gripper_open', 2448)
        self.declare_parameter('poses.gripper_closed', 1648)
        
        self.pose_home = self.get_parameter('poses.home').value
        self.pose_sleep = self.get_parameter('poses.sleep').value
        self.pose_ready = self.get_parameter('poses.ready').value
        self.gripper_open_pos = self.get_parameter('poses.gripper_open').value
        self.gripper_closed_pos = self.get_parameter('poses.gripper_closed').value
    
    def initialize_hardware(self):
        """Initialize connection to Dynamixel servos."""
        self.get_logger().info(f'Connecting to {self.port} at {self.baudrate} baud...')
        
        success, message = self.dxl.connect()
        if not success:
            self.get_logger().error(f'Failed to connect: {message}')
            return False
        
        self.get_logger().info('Connected to servos')
        
        # Verify all servos are present
        for idx, servo_id in enumerate(self.servo_ids):
            if not self.dxl.ping_servo(servo_id):
                self.get_logger().warn(f'Servo {servo_id} ({self.joint_names[idx]}) not responding')
            else:
                self.get_logger().info(f'Servo {servo_id} ({self.joint_names[idx]}) found')
        
        # Configure servos
        self.get_logger().info('Configuring servos...')
        for idx, servo_id in enumerate(self.servo_ids):
            joint_name = self.joint_names[idx]
            
            # Disable torque for configuration
            self.dxl.disable_torque(servo_id)
            
            # Set limits
            limits = self.joint_limits[joint_name]
            self.dxl.set_position_limits(servo_id, limits['min'], limits['max'])
            
            # Set profile velocity and acceleration
            self.dxl.set_profile_velocity(servo_id, self.profile_velocity)
            self.dxl.set_profile_acceleration(servo_id, self.profile_acceleration)
            
            # Set power limits
            self.dxl.set_pwm_limit(servo_id, self.pwm_limit)
            self.dxl.set_current_limit(servo_id, self.current_limit)
            
            # Enable torque
            self.dxl.enable_torque(servo_id)
            
            self.get_logger().info(f'  {joint_name}: limits [{limits["min"]}, {limits["max"]}]')
        
        # Read initial positions
        positions = self.dxl.sync_read_positions(self.servo_ids)
        if positions:
            with self.state_lock:
                self.current_positions = positions
            self.get_logger().info(f'Initial positions: {positions}')
        
        self.publish_status('READY')
        self.get_logger().info('Hardware initialization complete')
        return True
    
    def publish_joint_states(self):
        """Publish current joint states."""
        # Read positions from servos
        positions = self.dxl.sync_read_positions(self.servo_ids)
        
        if positions:
            with self.state_lock:
                self.current_positions = positions
                
                # Read velocities and efforts (currents)
                for idx, servo_id in enumerate(self.servo_ids):
                    vel = self.dxl.read_velocity(servo_id)
                    if vel is not None:
                        # Convert to rad/s (approximate)
                        self.current_velocities[idx] = self.dynamixel_velocity_to_rad_s(vel)
                    
                    effort = self.dxl.read_current(servo_id)
                    if effort is not None:
                        # Convert to Nm (approximate)
                        self.current_efforts[idx] = self.dynamixel_current_to_nm(effort)
            
            # Create and publish joint state message
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [self.dynamixel_to_rad(p) for p in self.current_positions]
            msg.velocity = self.current_velocities
            msg.effort = self.current_efforts
            
            self.joint_state_pub.publish(msg)
    
    def position_command_callback(self, msg):
        """Handle position commands."""
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(f'Invalid command length: {len(msg.data)}')
            return
        
        # Convert radians to Dynamixel positions
        positions = [self.rad_to_dynamixel(rad) for rad in msg.data]
        
        # Apply joint limits
        positions = self.apply_joint_limits(positions)
        
        # Send to servos
        if self.dxl.sync_write_positions(self.servo_ids, positions):
            self.get_logger().debug(f'Position command executed: {positions}')
        else:
            self.get_logger().warn('Failed to execute position command')
    
    def velocity_command_callback(self, msg):
        """Handle velocity commands (converts to position control)."""
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(f'Invalid command length: {len(msg.data)}')
            return
        
        # Simple velocity control: position += velocity * dt
        dt = 1.0 / self.publish_rate
        
        with self.state_lock:
            current_pos = list(self.current_positions)
        
        new_positions = []
        for idx, vel_rad_s in enumerate(msg.data):
            # Convert velocity to position change
            vel_dxl = self.rad_s_to_dynamixel_velocity(vel_rad_s)
            delta_pos = vel_dxl * dt
            new_pos = current_pos[idx] + delta_pos
            new_positions.append(int(new_pos))
        
        # Apply joint limits
        new_positions = self.apply_joint_limits(new_positions)
        
        # Send to servos
        if self.dxl.sync_write_positions(self.servo_ids, new_positions):
            self.get_logger().debug(f'Velocity command executed')
        else:
            self.get_logger().warn('Failed to execute velocity command')
    
    def trajectory_callback(self, msg):
        """Handle trajectory messages (simple execution)."""
        self.get_logger().info(f'Executing trajectory with {len(msg.points)} points')
        
        with self.trajectory_lock:
            if self.executing_trajectory:
                self.get_logger().warn('Already executing trajectory, ignoring new command')
                return
            self.executing_trajectory = True
        
        try:
            self._execute_trajectory(msg)
        finally:
            with self.trajectory_lock:
                self.executing_trajectory = False
    
    def execute_trajectory_action(self, goal_handle):
        """Execute trajectory via action server."""
        self.get_logger().info('Executing trajectory action')
        
        trajectory = goal_handle.request.trajectory
        
        with self.trajectory_lock:
            if self.executing_trajectory:
                goal_handle.abort()
                return FollowJointTrajectory.Result()
            self.executing_trajectory = True
        
        self.publish_status('EXECUTING_TRAJECTORY')
        
        try:
            success = self._execute_trajectory(trajectory, goal_handle)
            
            if success:
                goal_handle.succeed()
                self.publish_status('READY')
                return FollowJointTrajectory.Result()
            else:
                goal_handle.abort()
                self.publish_status('ERROR')
                return FollowJointTrajectory.Result()
        finally:
            with self.trajectory_lock:
                self.executing_trajectory = False
    
    def _execute_trajectory(self, trajectory, goal_handle=None):
        """Internal trajectory execution."""
        start_time = self.get_clock().now()
        
        for point_idx, point in enumerate(trajectory.points):
            # Check if preempted (for action server)
            if goal_handle and not goal_handle.is_active:
                self.get_logger().warn('Trajectory execution preempted')
                return False
            
            # Calculate target positions
            if len(point.positions) != len(self.joint_names):
                self.get_logger().error(f'Invalid point positions length: {len(point.positions)}')
                return False
            
            positions = [self.rad_to_dynamixel(rad) for rad in point.positions]
            positions = self.apply_joint_limits(positions)
            
            # Wait for the time_from_start
            target_time = start_time + rclpy.duration.Duration(
                seconds=point.time_from_start.sec,
                nanoseconds=point.time_from_start.nanosec
            )
            
            while self.get_clock().now() < target_time:
                time.sleep(0.01)
            
            # Send command
            if not self.dxl.sync_write_positions(self.servo_ids, positions):
                self.get_logger().error('Failed to send trajectory point')
                return False
            
            # Publish feedback (for action server)
            if goal_handle:
                feedback = FollowJointTrajectory.Feedback()
                feedback.actual.positions = [self.dynamixel_to_rad(p) for p in self.current_positions]
                feedback.desired.positions = list(point.positions)
                goal_handle.publish_feedback(feedback)
            
            self.get_logger().debug(f'Trajectory point {point_idx + 1}/{len(trajectory.points)} executed')
        
        self.get_logger().info('Trajectory execution complete')
        return True
    
    def go_home_callback(self, request, response):
        """Service callback to move to home position."""
        self.get_logger().info('Moving to home position')
        
        if self.dxl.sync_write_positions(self.servo_ids, self.pose_home):
            response.success = True
            response.message = 'Moved to home position'
        else:
            response.success = False
            response.message = 'Failed to move to home position'
        
        return response
    
    def go_sleep_callback(self, request, response):
        """Service callback to move to sleep position."""
        self.get_logger().info('Moving to sleep position')
        
        if self.dxl.sync_write_positions(self.servo_ids, self.pose_sleep):
            response.success = True
            response.message = 'Moved to sleep position'
        else:
            response.success = False
            response.message = 'Failed to move to sleep position'
        
        return response
    
    def go_ready_callback(self, request, response):
        """Service callback to move to ready position."""
        self.get_logger().info('Moving to ready position')
        
        if self.dxl.sync_write_positions(self.servo_ids, self.pose_ready):
            response.success = True
            response.message = 'Moved to ready position'
        else:
            response.success = False
            response.message = 'Failed to move to ready position'
        
        return response
    
    def gripper_open_callback(self, request, response):
        """Service callback to open gripper."""
        self.get_logger().info('Opening gripper')
        
        gripper_id = self.servo_ids[4]  # Assuming gripper is 5th joint
        if self.dxl.write_position(gripper_id, self.gripper_open_pos):
            response.success = True
            response.message = 'Gripper opened'
        else:
            response.success = False
            response.message = 'Failed to open gripper'
        
        return response
    
    def gripper_close_callback(self, request, response):
        """Service callback to close gripper."""
        self.get_logger().info('Closing gripper')
        
        gripper_id = self.servo_ids[4]  # Assuming gripper is 5th joint
        if self.dxl.write_position(gripper_id, self.gripper_closed_pos):
            response.success = True
            response.message = 'Gripper closed'
        else:
            response.success = False
            response.message = 'Failed to close gripper'
        
        return response
    
    def apply_joint_limits(self, positions):
        """Apply joint limits to positions."""
        limited_positions = []
        for idx, pos in enumerate(positions):
            joint_name = self.joint_names[idx]
            limits = self.joint_limits[joint_name]
            limited_pos = max(limits['min'], min(limits['max'], pos))
            if limited_pos != pos:
                self.get_logger().debug(
                    f'{joint_name} position {pos} limited to {limited_pos}'
                )
            limited_positions.append(limited_pos)
        return limited_positions
    
    def publish_status(self, status):
        """Publish status message."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    # Conversion functions
    def dynamixel_to_rad(self, position):
        """Convert Dynamixel position to radians."""
        # XL430: 0-4095 = 0-360 degrees
        return (position / 4096.0) * 2.0 * math.pi
    
    def rad_to_dynamixel(self, radians):
        """Convert radians to Dynamixel position."""
        position = int((radians / (2.0 * math.pi)) * 4096.0)
        return max(0, min(4095, position))
    
    def dynamixel_velocity_to_rad_s(self, velocity):
        """Convert Dynamixel velocity to rad/s."""
        # XL430: velocity unit is 0.229 rpm
        rpm = velocity * 0.229
        return (rpm / 60.0) * 2.0 * math.pi
    
    def rad_s_to_dynamixel_velocity(self, rad_s):
        """Convert rad/s to Dynamixel velocity units."""
        rpm = (rad_s / (2.0 * math.pi)) * 60.0
        return int(rpm / 0.229)
    
    def dynamixel_current_to_nm(self, current):
        """Convert Dynamixel current to Nm (approximate)."""
        # XL430: current unit is 2.69 mA
        # Torque constant is approximately 1.5 Nm/A at 12V
        amps = (current * 2.69) / 1000.0
        return amps * 1.5
    
    def shutdown(self):
        """Shutdown node and disable servos."""
        self.get_logger().info('Shutting down...')
        
        # Disable torque for all servos
        for servo_id in self.servo_ids:
            self.dxl.disable_torque(servo_id)
        
        # Disconnect
        self.dxl.disconnect()
        
        self.get_logger().info('Shutdown complete')


def main(args=None):
    rclpy.init(args=args)
    
    node = ArmControllerNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



