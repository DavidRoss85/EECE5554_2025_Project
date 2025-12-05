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
    DEFAULT_TARGET_CLASS = 'person'
    DEFAULT_TARGET_DISTANCE = 0.0  # meters
    DEFAULT_MAX_LINEAR_SPEED = .2  # m/s
    DEFAULT_MAX_ANGULAR_SPEED = .05  # rad/s
    # DEFAULT_KP_ANGULAR = 0.004
    # DEFAULT_KP_LINEAR = 0.5
    DEFAULT_IMAGE_WIDTH = 320
    DEFAULT_DISTANCE_TOLERANCE = 0.1  # meters
    DEFAULT_ANGULAR_TOLERANCE = .5  # rads
    DEFAULT_ENABLED = True
    DEFAULT_TIMEOUT = 2.0 * 1e9  # seconds

    APPROACH_MSG = 'APPROACHING'
    IDLE_MSG = 'IDLE'
    DETECTING_MSG = 'DETECTING'
    PICKING_MSG = 'PICKING'
    DONE_MSG = 'DONE'

    ALIGNED_STATUS = 'aligned'
    NOT_ALIGNED_STATUS = 'aligning'

    def __init__(self):
        super().__init__('approach_controller_node')
    
        
        self.__qos = self.DEFAULT_QOS
        self.__mission_topic = self.DEFAULT_MISSION_TOPIC
        self.__status_topic = self.DEFAULT_STATUS_TOPIC 

        self.target_class = self.DEFAULT_TARGET_CLASS
        
        self.enabled = self.DEFAULT_ENABLED
        
        self.max_linear_speed = self.DEFAULT_MAX_LINEAR_SPEED
        self.max_angular_speed = self.DEFAULT_MAX_ANGULAR_SPEED
        # self.kp_angular = self.DEFAULT_KP_ANGULAR
        # self.kp_linear = self.DEFAULT_KP_LINEAR
        
        self.image_width = self.DEFAULT_IMAGE_WIDTH
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
        self.start_moving_time = None
        self.stop_moving_time = self.get_clock().now()
        self.__estimated_distance = 0.0

        self.__lock_in_commitment = False
        














        #=========================================================
        # Fresh variables:
        self.distance_tolerance = 0.5
        self.angular_tolerance = 0.27 #rads ~10 degrees
        self.target_distance = self.DEFAULT_TARGET_DISTANCE


        self.__minimum_calculatable_distance = 0.9  # meters
        self.__slow_down_distance = 1.1  # meters
        self.__sensor_travel_distance_left = None
        self.__estimated_distance_left = None
        self.x_vel = 0.15
        self.z_vel = 0.01
        self.__constant_turn_speed = 0.1  # rad/s
        self.__fast_approach_speed = 0.2  # m/s
        self.__slow_approach_speed = 0.1  # m/s

    
        self.__timer_interval = 0.1  # seconds
        self.countdown_timer_forward = 0
        self.countdown_timer_turn = 0
        self.timeout = 2.0  # seconds




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
        
        # if self.approaching:
        #     return
        
        
        # message_time = (msg.robo_sync.header.stamp.sec + msg.robo_sync.header.stamp.nanosec*1e-9)
        # stop_time = (self.stop_moving_time.nanoseconds*1e-9) if self.stop_moving_time is not None else float('inf')

        # if message_time < stop_time:
        #     # Ignore older messages while moving
        #     print(f'SStop Moving Time: {stop_time}s\n Stamped Time: {message_time}s')
        #     self.get_logger().info('Ignoring detection message while moving')
        #     return
            
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


                    # Calculate turning time (Use absolute values since angular velocities can be negative)
                    if abs(self.item_details.relative_yaw) > self.angular_tolerance:
                        self.countdown_timer_turn = self.calculate_movement_time(
                            abs(float(self.item_details.relative_yaw)),
                            abs(self.z_vel),
                        ) 

                    # Check if within calculatable distance
                    if self.__sensor_travel_distance_left >= self.__minimum_calculatable_distance and self.__lock_in_commitment == False:
                        # Decide speed based on distance left
                        if self.__sensor_travel_distance_left <= self.__slow_down_distance:
                            self.x_vel = self.__slow_approach_speed
                        else:
                            self.x_vel = self.__fast_approach_speed


                        target_found = True
                        # Calculate forward moving time
                        self.countdown_timer_forward = self.calculate_movement_time(
                            self.__sensor_travel_distance_left,
                            self.x_vel,
                        )

                        # Update estimated distance left if not set
                        if self.__estimated_distance_left is None:
                            self.__estimated_distance_left = self.__sensor_travel_distance_left

                    else:
                        self.__lock_in_commitment = True

                    
                    # Notify user if located target
                    # if not self.approaching:
                        # self.get_logger().info(
                        #     f'🎯 {item.name} detected at ({item.distance:.2f}m, {item.relative_yaw:.2f})'
                        # )
                    self.approaching = True
                    break
            
            # Notify user if target is lost
            if not target_found:
                # if self.approaching:
                #     self.get_logger().info(f'Lost {self.target_class}')
                self.current_detection = None
                self.approaching = False
                
        except Exception as e:
            self.get_logger().error(f'Detection error: {str(e)}')


    #-----------------------------------------------------------------------------------------------
    def update_item_details(self, item):
        self.item_details = item
        self.item_details.relative_yaw = np.deg2rad(self.item_details.relative_yaw) if self.item_details.relative_yaw is not None else 0

        self.__sensor_travel_distance_left = (self.item_details.distance - self.target_distance)
        
        self.z_vel = (self.item_details.relative_yaw/abs(self.item_details.relative_yaw) * self.__constant_turn_speed) *-1

        # print(f'Received Item Details:\n Distance={self.item_details.distance:.2f}m,\n Relative Yaw={np.rad2deg(self.item_details.relative_yaw):.2f}deg')

    #-----------------------------------------------------------------------------------------------
    def calculate_movement_time(self, distance,velocity):
        if velocity <= 0:
            return 0
        return distance/velocity
    
    #-----------------------------------------------------------------------------------------------
    def approach_target(self, item, move_msg:TwistStamped):
        move_msg.twist.linear.x = self.x_vel
        self.get_logger().info(f'Setting fwd spd {self.x_vel:.2f} m/s')

    #-----------------------------------------------------------------------------------------------
    def face_target(self, item, move_msg:TwistStamped):
        move_msg.twist.angular.z = self.z_vel
    #-----------------------------------------------------------------------------------------------
    def set_target_reached(self):
        status_msg = String()
        status_msg.data = self.ALIGNED_STATUS
        self.__status_pub.publish(status_msg)

        self.get_logger().info(f'Target {self.target_class} reached at distance {self.item_details.distance:.2f}m')
        self.target_reached = True
        self.stop_robot()
    #-----------------------------------------------------------------------------------------------
    def stop_robot(self):
        twist = TwistStamped()
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        self.__cmd_vel_pub.publish(twist)

        self.log_stop_moving_time()
        # self.enabled = False
        self.approaching = False
        self.__lock_in_commitment = False
        self.get_logger().info('\n\n\n\n****************Robot Stopped****************\n\n\n\n')

    #-----------------------------------------------------------------------------------------------
    def log_start_moving_time(self):
        # self.stop_moving_time = self.get_clock().now()
        if self.start_moving_time is None:
                self.start_moving_time = self.get_clock().now()
    
    #-----------------------------------------------------------------------------------------------
    def log_stop_moving_time(self):
        self.stop_moving_time = self.get_clock().now()
    #-----------------------------------------------------------------------------------------------
    def control_loop(self):
        if not self.enabled or self.target_reached:
            return
        
        # if self.last_detection_time is not None and not self.__lock_in_commitment:
        #     time_since_last_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds * 1e-9
        #     if time_since_last_detection > self.timeout:
        #         self.get_logger().info('No recent detections. Stopping robot.')
        #         self.stop_robot()
        #         return
            
        move_msg = TwistStamped()
        
        # TURNING MOVEMENT:
        if self.countdown_timer_turn > 0:
            self.log_start_moving_time()
            self.countdown_timer_turn -= self.__timer_interval
            self.face_target(self.item_details,move_msg)
            print (f'Turning: {self.countdown_timer_turn:.4f}s remaining')

        else:

            # FORWARD MOVEMENT:
            if self.countdown_timer_forward > 0:
                self.log_start_moving_time()
                self.countdown_timer_forward -= self.__timer_interval

                # Update estimated distance left
                self.__estimated_distance_left -= self.x_vel * self.__timer_interval

                # if self.__estimated_distance_left < .5:
                #     self.__lock_in_commitment = True
                # Recalculate forward moving time
                # self.countdown_timer_forward = self.calculate_movement_time(
                #     self.__estimated_distance_left,
                #     self.x_vel,
                # )
                self.approach_target(self.item_details, move_msg)
                print (f'Moving Forward: {self.countdown_timer_forward:.4f}s remaining, Estimated Distance Left: {self.__estimated_distance_left:.4f}m')


        if self.approaching:
            print( "Still approaching...")
            if self.countdown_timer_forward <= 0 and self.countdown_timer_turn <= 0:
                self.set_target_reached()
            else:    
                self.__cmd_vel_pub.publish(move_msg)
    #-----------------------------------------------------------------------------------------------
    def get_alpha_filter_distance(self,estimated_distance, sensor_distance,alpha=0.5):
        new_estimated_distance = 0.0
        if estimated_distance is None:
            estimated_distance = sensor_distance
        else:
            new_estimated_distance = alpha * estimated_distance + (1 - alpha) * sensor_distance

        return new_estimated_distance

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
