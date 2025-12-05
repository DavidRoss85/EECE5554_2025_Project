#!/usr/bin/env python3

import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import  PoseStamped,Pose2D
# TurtleBot4 Navigator imports (These are available via the turtlebot4 common and turtlebot4_navigation packages)
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator




class NavigatorSetupNode(Node):

    DEFAULT_GOAL_TOPIC = '/navigate/goal'
    def __init__(self):
        super().__init__('navigator_setup_node')
        self.get_logger().info('Initializing Navigator Setup Node...')

        self.__initial_x = 1.0
        self.__initial_y = 0.1
        self.__initial_yaw = 0
        self.__initialized = False
        self.__qos = 10
        self.__navigating = False

        # # TF2 Buffer and Listener
        # self.__tf_buffer = Buffer()
        # self.__tf_listener = TransformListener(self.__tf_buffer, self)
        # self.get_logger().info('TF2 Listener initialized.')

        self.__navigator = TurtleBot4Navigator()

        # Subscribe to goal topic
        self.__goal_subscription = self.create_subscription(
            Pose2D,
            self.DEFAULT_GOAL_TOPIC,
            self.__goal_received_callback,
            self.__qos
        )
        self.__initialize()
    #-----------------------------------------------------------------------------
    def __goal_received_callback(self, msg: Pose2D):
        """Callback function for goal topic subscription"""

        if self.__navigating:
            self.get_logger().info('Already navigating to a goal, ignoring new goal.')
            return
        
        self.get_logger().info('Goal received, initializing navigator setup...')
        x = msg.x
        y = msg.y
        yaw = int(msg.theta)  
        self.move_to_pose(x, y, yaw)
    #-----------------------------------------------------------------------------
    def move_to_pose(self, x, y, yaw):
        """Moves the robot to the specified pose (x, y, yaw)"""
        self.__navigating = True
        goal_pose =self.__navigator.getPoseStamped([x, y], yaw)
        goal_pose.header.stamp = self.__navigator.get_clock().now().to_msg()

        # initial_pose = self.__navigator.getPoseStamped([0, 0], 0.0)
        # self.__navigator.setInitialPose(initial_pose)

        self.__navigator.info('Waiting for Nav2 to become active')
        self.__navigator.waitUntilNav2Active(localizer='slam_toolbox')
        self.__navigator.info(f'Moving to pose: x={x}, y={y}, yaw={yaw}')

        self.__navigator.goToPose(goal_pose)
        self.__navigating = False
    #----------------------------------------------------------------------------------
    def __initialize(self):
        """Sets the initial pose of the robot after undocking"""
        
        if self.__initialized:
            return
        print("Initializing...")
        # Undock if docked
        if self.__navigator.getDockedStatus():
            self.__navigator.info('Undocking')
            self.__navigator.undock()

        # time.sleep(2.0)  # Wait for undocking to complete
        # self.destroy_node()
    #----------------------------------------------------------------------------------
    def __set_initial_pose(self):
        """Sets the initial pose of the robot"""
        initial_pose = self.__navigator.getPoseStamped([self.__initial_x, self.__initial_y], self.__initial_yaw)
        initial_pose.header.stamp = self.__navigator.get_clock().now().to_msg()
        self.__navigator.setInitialPose(initial_pose)

        self.__navigator.info('Waiting for Nav2 to become active')
        self.__navigator.waitUntilNav2Active(localizer='slam_toolbox')
        self.get_logger().info('NavigatorSetupNode initialization complete.')

        self.__initialized = True

    #----------------------------------------------------------------------------------
    def navigator(self):
        """Returns the TurtleBot4Navigator instance"""
        return self.__navigator

#******************************************************************************
def main(args=None):

    rclpy.init(args=args)
    navigator_setup_node = NavigatorSetupNode()
    try:
        rclpy.spin(navigator_setup_node)

    finally:
        navigator_setup_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
