#!/usr/bin/env python3
"""
Approach Controller Node - Uses RSyncDetectionList and RoboSync
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from object_location_interfaces.msg import RSyncDetectionList, DetectedItem, RoboSync,LocationList,ItemLocation,RSyncLocationList

class ApproachControllerNode(Node):

    MAX_MSG = 10

    DEFAULT_QOS = 10 # QoSProfile(
    #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
    #     durability=QoSDurabilityPolicy.VOLATILE,
    #     depth=MAX_MSG
    # )
    DEFAULT_MISSION_TOPIC = '/mission/state'
    DEFAULT_STATUS_TOPIC = '/visual_servo/status'
    DEFAULT_TARGET_CLASS = 'bottle'
    DEFAULT_TARGET_DISTANCE = 0.4  # meters
    DEFAULT_MAX_LINEAR_SPEED = .7  # m/s
    DEFAULT_MAX_ANGULAR_SPEED = .4  # rad/s
    DEFAULT_KP_ANGULAR = 0.004
    DEFAULT_KP_LINEAR = 0.5
    DEFAULT_IMAGE_WIDTH = 320
    DEFAULT_DISTANCE_TOLERANCE = 0.05  # meters
    DEFAULT_ANGULAR_TOLERANCE = 1  # degree
    DEFAULT_ENABLED = False
    DEFAULT_TIMEOUT = 2.0  # seconds

    APPROACH_MSG = 'APPROACHING'
    IDLE_MSG = 'IDLE'
    DETECTING_MSG = 'DETECTING'
    PICKING_MSG = 'PICKING'
    DONE_MSG = 'DONE'

    ALIGNED_STATUS = 'aligned'
    NOT_ALIGNED_STATUS = 'aligning'

    def __init__(self):
        super().__init__('approach_controller_node')
        
        # Parameters
        self.declare_parameter('target_class', 'bottle')
        self.declare_parameter('target_distance', 0.4)
        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('kp_angular', 0.004)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('angular_tolerance', 30)
        self.declare_parameter('enabled', True)
        self.declare_parameter('timeout', 2.0)
        
        self.__qos = self.DEFAULT_QOS
        self.__mission_topic = self.DEFAULT_MISSION_TOPIC
        self.__status_topic = self.DEFAULT_STATUS_TOPIC 

        self.target_class = self.DEFAULT_TARGET_CLASS
        
        self.enabled = self.DEFAULT_ENABLED
        
        self.distance_tolerance = self.DEFAULT_DISTANCE_TOLERANCE
        self.angular_tolerance = self.DEFAULT_ANGULAR_TOLERANCE
        self.target_distance = self.DEFAULT_TARGET_DISTANCE
        self.max_linear_speed = self.DEFAULT_MAX_LINEAR_SPEED
        self.max_angular_speed = self.DEFAULT_MAX_ANGULAR_SPEED
        self.kp_angular = self.DEFAULT_KP_ANGULAR
        self.kp_linear = self.DEFAULT_KP_LINEAR
        
        self.image_width = self.DEFAULT_IMAGE_WIDTH
        self.timeout = self.DEFAULT_TIMEOUT
        self.item_location = None
        self.item_details = None
        
        
        self.image_center_x = self.image_width / 2
        self.bridge = CvBridge()
        
        # State
        self.current_detection = None
        self.current_depth_image = None
        self.last_detection_time = self.get_clock().now()
        self.approaching = False
        self.target_reached = False
        self.travel_distance = 0

        self.__timer_interval = 0.1  # seconds
        self.countdown_timer_forward = 0
        self.countdown_timer_turn = 0
        self.x_vel = 0.5
        self.z_vel = 0.1
        
        # Subscribers
        self.__detection_sub = self.create_subscription(
            RSyncLocationList,
            '/objects/locations',
            self.detection_callback,
            self.__qos
        )

        self.__mission_sub = self.create_subscription(
            String,
            self.__mission_topic,
            self.__mission_callback,
            self.__qos
        )

        

        
        # Publisher
        self.__cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel', 
            self.__qos
        )

        self.__status_pub = self.create_publisher(
            String,
            self.__status_topic,
            self.__qos
        )
        
        # Control loop
        self.control_timer = self.create_timer(self.__timer_interval, self.control_loop)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Approach Controller Started')
        self.get_logger().info(f'Target: {self.target_class} | Distance: {self.target_distance}m')
        self.get_logger().info('='*60)

    #-----------------------------------------------------------------------------------------------
    def __mission_callback(self, msg:String):
        """ Handle mission state messages to enable/disable approach controller"""
        if msg.data == self.DETECTING_MSG:
            self.enabled = False
            self.stop_robot()
            self.get_logger().info('Approach Controller Disabled for DETECTING')
        elif msg.data == self.PICKING_MSG:
            self.enabled = False
            self.stop_robot()
            self.get_logger().info('Approach Controller Disabled for PICKING')
        elif msg.data == self.DONE_MSG:
            self.enabled = False
            self.stop_robot()
            self.get_logger().info('Approach Controller Disabled for DONE')
        elif msg.data == self.IDLE_MSG:
            self.enabled = False
            self.stop_robot()
            self.get_logger().info('Approach Controller Disabled')
        elif msg.data == self.APPROACH_MSG:
            self.enabled = True
            self.target_reached = False
            self.get_logger().info('Approach Controller Enabled')

    #-----------------------------------------------------------------------------------------------
    def detection_callback(self, msg:RSyncDetectionList):
        """ Process detection messages to approach target object """
        if self.enabled == False:
            return
        
        try:
            target_found = False

            # List of items detected
            item_list = msg.locations.location_list
            
            # Check list of detected items for desired item
            for item in item_list:
                if item.name == self.target_class:
                    
                    # Store item details
                    self.update_item_details(item)

                    # Remember time of detection:
                    self.last_detection_time = self.get_clock().now()

                    # Calculate forward moving time
                    self.countdown_timer_forward = self.calculate_movement_time(
                        self.travel_distance,
                        self.x_vel
                    )

                    # Calculate turning time (Use absolute values since angular velocities can be negative)
                    self.countdown_timer_turn = self.calculate_movement_time(
                        abs(float(self.item_details.relative_yaw)),
                        abs(self.z_vel)
                    )

                    target_found = True
                    
                    # Notify user if located target
                    if not self.approaching:
                        self.get_logger().info(
                            f'🎯 {item.name} detected at ({item.distance:.2f}m, {item.relative_yaw:.2f})'
                        )
                    self.approaching = True
                    break
            
            # Notify user if target is lost
            if not target_found:
                if self.approaching:
                    self.get_logger().info(f'Lost {self.target_class}')
                self.current_detection = None
                self.approaching = False
                
        except Exception as e:
            self.get_logger().error(f'Detection error: {str(e)}')


    #-----------------------------------------------------------------------------------------------
    def set_target_reached(self):
        status_msg = String()
        status_msg.data = self.ALIGNED_STATUS
        self.__status_pub.publish(status_msg)

        self.get_logger().info(f'Target {self.target_class} reached at distance {self.item_details.distance:.2f}m')

        self.enabled = False
        self.approaching = False
        self.target_reached = True
        self.stop_robot()
    #-----------------------------------------------------------------------------------------------
    def update_item_details(self, item):
        self.item_details = item
        self.item_details.relative_yaw = np.deg2rad(self.item_details.relative_yaw) if self.item_details.relative_yaw is not None else 0
        self.x_vel = min(self.item_details.distance, self.max_linear_speed)
        self.z_vel = ((self.item_details.relative_yaw) *-1)/2
        self.travel_distance = (self.item_details.distance - self.target_distance)

    #-----------------------------------------------------------------------------------------------
    def calculate_movement_time(self, distance,velocity):
        if velocity <= 0:
            return 0
        return distance/velocity
    
    #-----------------------------------------------------------------------------------------------
    def approach_target(self, item, move_msg:TwistStamped):
        move_msg.twist.linear.x = self.x_vel

    #-----------------------------------------------------------------------------------------------
    def face_target(self, item, move_msg:TwistStamped):
        move_msg.twist.angular.z = self.z_vel
    #-----------------------------------------------------------------------------------------------
    def stop_robot(self):
        twist = TwistStamped()
        self.__cmd_vel_pub.publish(twist)


    #-----------------------------------------------------------------------------------------------
    def control_loop(self):
        if not self.enabled:
            return
        
        move_msg = TwistStamped()
        
        if self.countdown_timer_forward > 0:
            self.countdown_timer_forward -= self.__timer_interval
            self.approach_target(self.item_details, move_msg)
            print (f'Moving Forward: {self.countdown_timer_forward:.2f}s remaining')

        if self.countdown_timer_turn > 0:
            self.countdown_timer_turn -= self.__timer_interval
            self.face_target(self.item_details,move_msg)
            print (f'Turning: {self.countdown_timer_turn:.2f}s remaining')

        if self.approaching:
            if self.countdown_timer_forward <= 0 and self.countdown_timer_turn <= 0:
                self.set_target_reached()
            else:    
                self.__cmd_vel_pub.publish(move_msg)

#***************************************************************************************************
#***************************************************************************************************

def main(args=None):
    rclpy.init(args=args)
    node = ApproachControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
