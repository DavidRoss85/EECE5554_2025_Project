#!/usr/bin/env python3
"""
Approach Controller Node - Uses RSyncDetectionList and RoboSync
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from object_location_interfaces.msg import RSyncDetectionList, DetectedItem, RoboSync,LocationList,ItemLocation,RSyncLocationList

class ApproachControllerNode(Node):

    MAX_MSG = 10

    DEFAULT_QOS = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=MAX_MSG
    )

    DEFAULT_TARGET_CLASS = 'bottle'
    DEFAULT_TARGET_DISTANCE = 0.4  # meters
    DEFAULT_MAX_LINEAR_SPEED = 0.15  # m/s
    DEFAULT_MAX_ANGULAR_SPEED = 0.5  # rad/s
    DEFAULT_KP_ANGULAR = 0.004
    DEFAULT_KP_LINEAR = 0.5
    DEFAULT_IMAGE_WIDTH = 320
    DEFAULT_DISTANCE_TOLERANCE = 0.05  # meters
    DEFAULT_ANGULAR_TOLERANCE = 1  # degree
    DEFAULT_ENABLED = True
    DEFAULT_TIMEOUT = 2.0  # seconds

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

        self.target_class = self.DEFAULT_TARGET_CLASS
        self.target_distance = self.DEFAULT_TARGET_DISTANCE
        self.max_linear_speed = self.DEFAULT_MAX_LINEAR_SPEED
        self.max_angular_speed = self.DEFAULT_MAX_ANGULAR_SPEED
        self.kp_angular = self.DEFAULT_KP_ANGULAR
        self.kp_linear = self.DEFAULT_KP_LINEAR
        self.image_width = self.DEFAULT_IMAGE_WIDTH
        self.distance_tolerance = self.DEFAULT_DISTANCE_TOLERANCE
        self.angular_tolerance = self.DEFAULT_ANGULAR_TOLERANCE
        self.enabled = self.DEFAULT_ENABLED
        self.timeout = self.DEFAULT_TIMEOUT
        self.item_location = None
        
        self.image_center_x = self.image_width / 2
        self.bridge = CvBridge()
        
        # State
        self.current_detection = None
        self.current_depth_image = None
        self.last_detection_time = self.get_clock().now()
        self.approaching = False
        self.target_reached = False
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            RSyncLocationList,
            '/objects/locations',
            self.detection_callback,
            self.__qos
        )
        
        # self.sync_sub = self.create_subscription(
        #     RoboSync,
        #     '/sync/robot/state',
        #     self.sync_callback,
        #     10
        # )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Approach Controller Started')
        self.get_logger().info(f'Target: {self.target_class} | Distance: {self.target_distance}m')
        self.get_logger().info('='*60)
    
    def detection_callback(self, msg:RSyncDetectionList):
        try:
            target_found = False
            item_list = msg.locations.location_list
            for item in item_list:
                if item.name == self.target_class:
                    # if len(item.xywh) >= 2:
                    center_x = 1#float(item.xywh[0])
                    center_y = 1#float(item.xywh[1])
                        
                    self.current_detection = {
                        'center_x': 1,#center_x,
                        'center_y': 1,#center_y,
                        'name': item.name,
                        'confidence': 1,#item.confidence,
                        'distance': 1,#item.distance
                    }

                    self.item_location = {
                        'name': item.name,
                        'index': item.index,
                        'distance': item.distance,
                        'yaw': item.relative_yaw
                    }

                    
                    self.last_detection_time = self.get_clock().now()
                    target_found = True
                    
                    if not self.approaching:
                        self.get_logger().info(
                            f'🎯 {item.name} detected at ({center_x:.0f}, {center_y:.0f})'
                        )
                    self.approaching = True
                    break
            
            if not target_found:
                if self.approaching:
                    self.get_logger().info(f'Lost {self.target_class}')
                self.current_detection = None
                self.approaching = False
                
        except Exception as e:
            self.get_logger().error(f'Detection error: {str(e)}')
    
    def sync_callback(self, msg):
        try:
            self.current_depth_image = msg.depth_image
        except Exception as e:
            self.get_logger().error(f'Sync error: {str(e)}')
    
    def get_distance_from_depth(self, center_x, center_y):
        if self.current_depth_image is None:
            return None
        
        try:
            depth_array = self.bridge.imgmsg_to_cv2(
                self.current_depth_image,
                desired_encoding='passthrough'
            )
            
            height, width = depth_array.shape
            cx = int(np.clip(center_x, 0, width - 1))
            cy = int(np.clip(center_y, 0, height - 1))
            
            # Sample 5x5 region
            sample_size = 5
            x_start = max(0, cx - sample_size)
            x_end = min(width, cx + sample_size)
            y_start = max(0, cy - sample_size)
            y_end = min(height, cy + sample_size)
            
            depth_region = depth_array[y_start:y_end, x_start:x_end]
            valid_depths = depth_region[(depth_region > 0) & (~np.isnan(depth_region))]
            
            if len(valid_depths) > 0:
                median_depth = np.median(valid_depths)
                distance = median_depth / 1000.0
                
                if 0.2 < distance < 5.0:
                    return distance
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'Depth error: {str(e)}')
            return None
    
    def control_loop(self):
        if not self.enabled:
            return
        
        time_since = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        
        if time_since > self.timeout:
            if self.approaching:
                self.get_logger().info('⏱️ Timeout, stopping')
                self.stop_robot()
                self.approaching = False
                self.target_reached = False
            return
        
        if self.item_location is not None and self.approaching:
            # distance = self.get_distance_from_depth(
            #     self.current_detection['center_x'],
            #     self.current_detection['center_y']
            # )
            distance = self.item_location['distance']
            
            if distance is not None:
                self.approach_target(self.item_location)
    
    def approach_target(self, item):
        move_msg = TwistStamped()
        
        # Angular control1
        # error_x = target_x - self.image_center_x
        angular_vel = -self.kp_angular * np.radians(item['yaw'])
        angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
        
        # Distance control
        distance_error = item['distance'] - self.target_distance
        

        # if abs(distance_error) <= self.distance_tolerance:# and abs(item['yaw']) <= self.angular_tolerance:
        move_msg.twist.linear.x = 0.5

        #     if not self.target_reached:
        #         # self.get_logger().info('✅ TARGET REACHED!')
        #         # self.target_reached = True
            
        #         twist.linear.x = 0.0
        #         twist.angular.z = angular_vel * 0.3
        # else:
        #     self.target_reached = False
            
        #     if abs(distance_error) > self.distance_tolerance:
        #         linear_vel = self.kp_linear * distance_error
                
        #         if abs(error_x) > 100:
        #             linear_vel *= 0.3
        #         elif abs(error_x) > 50:
        #             linear_vel *= 0.5
                
        #         linear_vel = np.clip(linear_vel, -self.max_linear_speed, self.max_linear_speed)
        #     else:
        #         linear_vel = 0.0
            
        #     twist.linear.x = linear_vel
        #     twist.angular.z = angular_vel
            
        #     self.get_logger().info(
        #         f'→ D={distance:.2f}m | err={distance_error:+.2f}m | '
        #         f'angle_err={error_x:+.0f}px | v={linear_vel:.2f} w={angular_vel:.2f}',
        #         throttle_duration_sec=1.0
        #     )
        
        self.cmd_vel_pub.publish(move_msg)
    
    def stop_robot(self):
        twist = TwistStamped()
        self.cmd_vel_pub.publish(twist)

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
