# This node performs uses the detected objects and synchronized depth images
# to compute the relative yaw and distance of each detected item.
# It publishes this information as an RSyncLocationList message.

# Math Imports:
import math
import numpy as np

# ROS2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from object_location_interfaces.msg import (
    RoboSync as RSync,
    DetectedItem,
    DetectionList,
    RSyncDetectionList,
    ItemLocation,
    LocationList,
    RSyncLocationList,
)

# OpenCV imports
import cv2      #pip3 install opencv-python
from cv_bridge import CvBridge


class DistanceNode(Node):
    """
    DistanceNode subscribes to synchronized detection + depth messages
    and computes the relative yaw and distance of each detected item.

    Output is published as an RSyncLocationList message containing one
    ItemLocation entry per detected object.
    """

    # Default ROS topic names and settings
    DEFAULT_PUBLISH_TOPIC = '/objects/locations'
    DEFAULT_DETECTIONS_TOPIC = '/objects/detections'
    MAX_MSG = 10

    # Image + depth settings
    DEFAULT_IMAGE_ENCODING = 'passthrough'
    DEFAULT_DEPTH_MIN = 1
    DEFAULT_DEPTH_MAX = 65000
    METER_DEPTH_FACTOR = 1
    MM_DEPTH_FACTOR = 1000
    DEFAULT_USING_GAZEBO = True

    # Camera intrinsics (unused right now, but available)
    DEFAULT_FOCAL_LENGTH = 870.0
    DEFAULT_BASELINE = 0.075
    DEFAULT_CAMERA_PIXEL_WIDTH = 320
    DEFAULT_CAMERA_PIXEL_HEIGHT = 240
    DEFAULT_CAMERA_FOV_ANGLE = 78.24

    def __init__(self):
        """
        Node initialization.
        Sets up parameters, subscriptions, and publishers.
        """
        super().__init__('distance_node')
        self.get_logger().info('Initializing Distance Node')

        # Local configuration variables
        self.__detections_topic = self.DEFAULT_DETECTIONS_TOPIC
        self.__locations_topic = self.DEFAULT_PUBLISH_TOPIC
        self.__max_msg = self.MAX_MSG
        self.__camera_width_res = self.DEFAULT_CAMERA_PIXEL_WIDTH         # Width in pixels of the image used for yaw calculation
        self.__camera_angle_width = self.DEFAULT_CAMERA_FOV_ANGLE        # Horizontal field of view of the camera (degrees)
        self.__image_encoding = self.DEFAULT_IMAGE_ENCODING
        self.__depth_min = self.DEFAULT_DEPTH_MIN
        self.__depth_max = self.DEFAULT_DEPTH_MAX
        self.__using_gazebo = self.DEFAULT_USING_GAZEBO
        self.__depth_factor = self.METER_DEPTH_FACTOR if self.__using_gazebo else self.MM_DEPTH_FACTOR

        # Placeholder for parameter server imports
        self.__load_parameters()

        # OpenCV CVBridge conversion helper
        self.__bridge = CvBridge()
        self.get_logger().info(f'CV Bridge loaded')

        try:
            # Subscribe to detection messages that include YOLO bounding boxes + synchronized depth image
            self.__detection_sub = self.create_subscription(
                RSyncDetectionList,
                self.__detections_topic,
                self.__process_detections,
                self.__max_msg
            )
            self.get_logger().info(f'Subscribed to detection topic: {self.__detections_topic}')

            # Publisher that outputs distance/yaw for each detected object
            self.__locations_pub = self.create_publisher(
                RSyncLocationList,
                self.__locations_topic,
                self.__max_msg
            )
            self.get_logger().info(f'Publisher created on topic: {self.__locations_topic}')

            self.get_logger().info('Successfully initialized Distance Node.')

        except Exception as e:
            self.__handle_error(e, '__init__()', 'Failed to initialize Distance Node')
            self.destroy_node()

    #----------------------------------------------------------------------------------
    def __load_parameters(self):
        """
        Placeholder for pulling ROS2 parameters.
        Currently unused, but structured for future integration.
        """
        pass

    #----------------------------------------------------------------------------------
    def __process_detections(self, message: RSyncDetectionList):
        """
        Main callback: receives RSyncDetectionList containing
        (1) YOLO detection list
        (2) synchronized depth image from same timestamp

        For each detected item:
            • Extract center pixel from YOLO bounding box
            • Read depth from depth image
            • Compute relative yaw from pixel x coordinate
        Then publishes an RSyncLocationList containing these results.
        """

        # Extract YOLO detections and depth image
        detection_list = message.detections.item_list
        depth_image = message.robo_sync.depth_image
        locations_list = []

        # Convert ROS Image → OpenCV
        cv_image = self.__bridge.imgmsg_to_cv2(depth_image, self.__image_encoding)

        # Clip depth range to expected values
        depth_map = np.clip(cv_image, self.__depth_min, self.__depth_max)

        # Loop through each detected item and compute relative pose information
        for item in detection_list:
            relative_location = ItemLocation()
            relative_location.name = item.name
            relative_location.index = item.index

            # YOLO bounding box: (xc, yc, width, height)
            xc, yc, w, h = item.xywh

            # Optional: Find closest depth value within bounding box (Only x axis used for now)
            closest_point_on_x = float(self.__depth_max) # Large initial value
            xl = int(xc - w/2)
            xr = int(xc + w/2)
            for n in range(xl , xr ):
                # Note: numpy uses [row=y, col=x]
                # Depth lookup at bounding-box center
                closest_point_on_x = min(closest_point_on_x, depth_map[yc,n])

            relative_location.distance = float(closest_point_on_x) / self.__depth_factor  # Distance in m
            # print(f"Item: {item.name}, xc: {xc}, yc: {yc}, w: {w}, h: {h}, xl: {xl}, xr: {xr}")
            # print(f"Item: {item.name}, CLOSESET Distance: {relative_location.distance:.2f} m")
            # relative_location.distance = float(depth_map[yc, xc]) / self.__depth_factor 
            # print(f"Item: {item.name}, CENTER Distance: {relative_location.distance:.2f} m")
            # Compute angular offset based on horizontal pixel location
            relative_location.relative_yaw = float(
                self.__calculate_yaw_from_pixels(
                    self.__camera_width_res,
                    self.__camera_angle_width,
                    xc
                )
            )
            # print(f"Item: {item.name}, Distance: {relative_location.distance:.2f} mm, Yaw: {relative_location.relative_yaw:.2f}°")

            locations_list.append(relative_location)

        # Bundle computed locations and republish synchronized message
        rsync_msg = self.__generate_location_message(locations_list, message.robo_sync)
        self.__locations_pub.publish(rsync_msg)

    #----------------------------------------------------------------------------------
    def __generate_location_message(self, locations_list, robo_sync):
        """
        Build and return an RSyncLocationList message ready for publishing.
        This preserves the original RoboSync timestamp bundle so downstream
        nodes receive consistent synchronized data.
        """
        rsync_msg = RSyncLocationList()
        list_msg = LocationList()

        list_msg.location_list = locations_list
        rsync_msg.robo_sync = robo_sync
        rsync_msg.locations = list_msg

        return rsync_msg

    #----------------------------------------------------------------------------------
    def __calculate_yaw_from_pixels(self, camera_x_res, camera_angle_width, x):
        """
        Compute the relative yaw angle (in degrees) from pixel position 'x'.

        The formula assumes:
            • Center pixel = 0° yaw
            • Left side = +FOV/2
            • Right side = -FOV/2

        Args:
            camera_x_res (int): horizontal resolution
            camera_angle_width (float): full horizontal field of view (degrees)
            x (int): pixel coordinate of detected object center

        Returns:
            float: relative yaw angle in degrees
        """

        # Normalize pixel position to range [-1, +1]
        offset = (x - (camera_x_res / 2)) / (camera_x_res / 2)

        # Convert normalized pixel offset → yaw angle
        yaw = offset * (camera_angle_width / 2)

        return yaw

    #----------------------------------------------------------------------------------
    def __handle_error(self, error, function_name, custom_message=''):
        """
        Print standardized error messages using ROS2 logging system.
        """
        self.get_logger().error(f'Error in {function_name}: {str(error)}. {custom_message}')


#----------------------------------------------------------------------------------
def main(args=None):
    """
    Standard ROS2 node entry point.
    """
    rclpy.init()
    distance_node = DistanceNode()
    rclpy.spin(distance_node)
    distance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
