#New starts

import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from .navigator_node import NavigatorSetupNode

from object_location_interfaces.msg import(
  RSyncDetectionList, 
  DetectedItem, 
  RoboSync,
  LocationList,
  ItemLocation as ItemData,
  RSyncLocationList  
) 


class ApproachControllerNode(Node):

    def __init__(self):
        super().__init__('approach_controller_node')
        self.get_logger().info('Initializing Approach Controller Node...')

        # ROS2 settings
        self.__qos = 10
        self.__location_topic = '/objects/locations'
        self.__velocity_topic = '/cmd_vel'

        # location data
        self.__target_item_id = 'bottle'  # Example target item ID
        self.__timer_inerval = 0.1  # seconds
        self.__fast_approach_speed = 1.0  # m/s
        self.__slow_approach_speed = 0.1  # m/s
        self.__fast_approach_turn_rate = 0.5  # rad/s
        self.__slow_approach_turn_rate = 0.1  # rad/s
        self.__filter_alpha = 0.5  # Alpha filter coefficient

        
        self.__target_distance = 0.4  # meters
        self.__fast_approach_distance = 1.1  # meters
        self.__dead_reckoning_distance = 0.8  # meters
        self.__angular_tolerance = np.deg2rad(10.0)  # radians

        self.__fast_approach = True
        self.__last_sensor_reading_time = None
        self.__begin_approach = True
        self.__target_info = None
        self.__last_target_info = None
        self.__last_reckoning_update_time = None
        self.__locked_target = False
        self.__enable_approach = True

        # Subscribers and Publishers
        self.__location_subscription = self.create_subscription(
            RSyncLocationList,
            self.__location_topic,
            self.__location_received_callback,
            self.__qos
        )

        self.__velocity_publisher = self.create_publisher(
            TwistStamped,
            self.__velocity_topic,
            self.__qos
        )

        self.__control_timer = self.create_timer(self.__timer_inerval, self.__control_loop)

    def __location_received_callback(self, msg: RSyncLocationList):
        """Callback function for location topic subscription"""

        # print("Location message received.")

        if not msg.locations:
            self.get_logger().info('No locations received.')
            return

        # Process locations to find the target item
        target = None
        for item in msg.locations.location_list:
            if item.name == self.__target_item_id:
                target = item
                break
            else:
                continue

        if target is not None:
            self.__update_target_info(target)
        #     self.get_logger().info(f'Target item "{self.__target_item_id}" found at position:\n\t distance: {item.distance} mm\n\t relative_yaw: {item.relative_yaw} deg')
        # else:
        #     print(f"... Searching for target item '{self.__target_item_id}' ...")
    #--------------------------------------------------------------------------
    def __stop_robot(self):
        """Stops the robot by publishing zero velocities"""
        self.get_logger().info('Stopping robot.')
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0
        self.__velocity_publisher.publish(stop_msg)
    #--------------------------------------------------------------------------
    def __goal_reached(self):
        """Handle actions when goal is reached"""
        self.get_logger().info('Goal reached. Stopping robot.')
        self.__stop_robot()


        self.__begin_approach = False
        self.__target_info = None
        self.__last_target_info = None
        self.__locked_target = False
    #--------------------------------------------------------------------------
    def __update_target_info(self, target: ItemData):
        """Update target information based on new location data"""
        self.__target_info = target
        current_time = self.get_clock().now()

        predicted_distance = target.distance
        if self.__last_target_info is not None and self.__last_sensor_reading_time is not None:
            predicted_distance = self.__predict_distance(
                self.__last_target_info.distance,
                self.__fast_approach_speed if self.__fast_approach else self.__slow_approach_speed,
                current_time
            )

        filtered_distance = self.__apply_alpha_filter(
            predicted_distance,
            target.distance,
            alpha=self.__filter_alpha
        )

        target.distance = filtered_distance

        if target.distance > self.__dead_reckoning_distance and not self.__locked_target:
            self.__last_target_info = target

        self.__last_sensor_reading_time = current_time
    
    #--------------------------------------------------------------------------
    def __approach_target(self, target: ItemData):
        """Begin approach maneuver towards the target location"""
        print("Approaching target...")
        speed = 0.0
        turn_rate = 0.0

        if target.distance > self.__fast_approach_distance:
            speed = self.__fast_approach_speed
            turn_rate = self.__fast_approach_turn_rate
            self.__fast_approach = True
        else:
            speed = self.__slow_approach_speed
            turn_rate = self.__slow_approach_turn_rate
            self.__fast_approach = False

        if target.distance >= self.__dead_reckoning_distance and not self.__locked_target:
            twist_msg = self.__calculate_velocity_using_sensor_data(target, speed, turn_rate)
        else:
            if self.__last_target_info is not None:
                print(f"LOCKED IN!!")
                self.__locked_target = True
                twist_msg = self.__calculate_velocity_using_dead_reckoning()

        if self.__last_target_info.distance <= self.__target_distance:
                    self.__goal_reached()
                    return      


        self.__velocity_publisher.publish(twist_msg)

    #--------------------------------------------------------------------------
    # The speed will be determined based on the distance to the target, hence fast_approach_speed and slow_approach_speed are defined.
    # The robot will also need to center itself on the target as it approaches.
    # Because the target distance received is not reliable closer than about .8 meters,
    #  we will use fast approach speed until we are within 1 meter, then switch to slow approach speed.
    # Also, once entering slow approach, we will need to lock in our destination and ignore sensor readings for disance.
    # At then end of the approach, one more attempt to center on the target will be made.

    #--------------------------------------------------------------------------
    def __calculate_velocity_using_sensor_data(self, target: ItemData,speed: float=0.0, turn_rate: float=0.0) -> TwistStamped:
        """Approach target using real-time sensor data"""
        print(f'Sensor Data - Distance: {target.distance} m, Relative Yaw: {target.relative_yaw} deg')


        yaw_rad = np.deg2rad(target.relative_yaw)   # Convert degrees to radians
        turn_rate = self.__calculate_angular_velocity(turn_rate, yaw_rad)
        
        
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = speed
        twist_msg.twist.angular.z = turn_rate

        return twist_msg
    
    #--------------------------------------------------------------------------
    def __calculate_velocity_using_dead_reckoning(self) -> TwistStamped:
        """Dead reckoning approach towards target using last known location data"""
        if self.__last_target_info is None:
            self.get_logger().info('No last known target information available for dead reckoning.')
            return TwistStamped()  # Return zero velocity
        
        self.__fast_approach = False

        target = self.__last_target_info
        speed = self.__slow_approach_speed
        turn_rate = self.__slow_approach_turn_rate

        yaw_rad = np.deg2rad(target.relative_yaw)   # Convert degrees to radians
        turn_rate = self.__calculate_angular_velocity(turn_rate, yaw_rad)

        # No angular velocity if within tolerance
        if abs(turn_rate) <= self.__angular_tolerance:
            turn_rate = 0.0  
        
        if turn_rate != 0.0:
            speed = 0.0  # Only turn if we need to adjust yaw

        #Create stamped twist message
        twist_msg = TwistStamped()
        stamp_time = self.get_clock().now()
        twist_msg.header.stamp = stamp_time.to_msg()

        print(f'Dead Reckoning - Distance: {target.distance} m, Relative Yaw: {target.relative_yaw} deg, Speed: {speed} m/s, Turn Rate: {turn_rate} rad/s')
        self.__update__dead_reckoning_target(speed, turn_rate, stamp_time)
        
        # Set velocities
        twist_msg.twist.angular.z = turn_rate   # angular velocity
        twist_msg.twist.linear.x = speed    # linear velocity

        return twist_msg
    #--------------------------------------------------------------------------
    def __calculate_angular_velocity(self, turn_rate: float, relative_yaw: float) -> float:
        """Calculate angular velocity based on turn rate and relative yaw"""
        angular_velocity = -1 * turn_rate * (relative_yaw / abs(relative_yaw)) if relative_yaw != 0 else 0.0
        return angular_velocity
    
    #--------------------------------------------------------------------------
    def __update__dead_reckoning_target(self, speed: float, turn_rate: float, current_time ):
        """Update last known target information for dead reckoning"""
        if self.__last_target_info is not None:

            if self.__last_reckoning_update_time is None:
                self.__last_reckoning_update_time = current_time
                return
            
            elapsed_time = (current_time - self.__last_reckoning_update_time).nanoseconds / 1e9  # Convert to seconds
            self.__last_target_info.distance -= speed * elapsed_time
            self.__last_target_info.relative_yaw -= np.rad2deg(turn_rate) * elapsed_time
            self.__last_reckoning_update_time = current_time

    #--------------------------------------------------------------------------
    # We need the robot to calculate it's total distance moved towards the target based on speed and time elapsed.
    # This will allow the robot to estimate it's distance to the target even when the sensor data.
    # We will use the sensor data and apply an alpha filter to smooth out the distance readings.

    # prediction  = the estimate (starts at initial distance) - speed * time
    # measurement = the sensor reading (new distance to target)
    # estimate = prediction + alpha * (measurement - predition)

    # if we calculate speed estimate:
    # new speed = old speed + (beta * (measured speed - old speed))
    #--------------------------------------------------------------------------
    def __predict_distance(self, previous_distance: float, speed: float, current_time: float) -> float:
        """Predict distance to target based on previous distance, speed, and elapsed time"""
        elapsed_time = (current_time - self.__last_sensor_reading_time).nanoseconds / 1e9  # Convert to seconds
        predicted_distance = previous_distance - (speed * elapsed_time)
        return predicted_distance
    #--------------------------------------------------------------------------
    def __apply_alpha_filter(self, predicted_distance: float, measured_distance: float, alpha: float) -> float:
        """Apply alpha filter to smooth distance measurements"""
        estimate = predicted_distance + alpha * (measured_distance - predicted_distance)
        return estimate

    #--------------------------------------------------------------------------
    def __control_loop(self):
        """Main control loop for approach controller"""

        if self.__target_info is not None:
            if self.__begin_approach:
                self.__approach_target(self.__target_info)

#***************************************************************************************************
#***************************************************************************************************

def main(args=None):
    rclpy.init(args=args)
    node = ApproachControllerNode()
    navigator = NavigatorSetupNode()
    
    try:
        # print("Spinning navigator node once to initialize...")
        # rclpy.spin_once(navigator, timeout_sec=14.0)  # Spin navigator once to initialize
        print("Starting approach controller node...")
        rclpy.spin(node)
    finally:
        node.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()