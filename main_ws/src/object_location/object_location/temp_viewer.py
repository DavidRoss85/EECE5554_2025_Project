# ROS2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from object_location_interfaces.msg import (
    RoboSync as RSync,
    DetectedItem,
    DetectionList,
    RSyncDetectionList,
    ItemLocation,
    LocationList,
    RSyncLocationList,
)
import math
import numpy as np
import matplotlib.pyplot as plt

from .helpers import find_a_star_path, quaternion_to_yaw, inflate_obstacles,transform_2d
import cv2
from cv_bridge import CvBridge

class TempViewer(Node):
    def __init__(self):
        super().__init__('temp_viewer')
        self.get_logger().info('Initializing Temp Viewer Node')

        # self.__image_subscription = self.create_subscription(
        #     Image,
        #     '/oakd/rgb/preview/image_raw',
        #     self.image_callback,
        #     10
        # )

        # self.__overlay_subscription = self.create_subscription(
        #     OccupancyGrid,
        #     '/grid/overlay',
        #     self.overlay_callback,
        #     10
        # )

        # self.__map_subscription = self.create_subscription(
        #     OccupancyGrid,
        #     '/map',
        #     self.map_callback,
        #     10
        # )

        self.__detection_vision = self.create_subscription(
            RSyncDetectionList,
            '/object/detections',
            self.detection_vision_callback,
            10
        )

        self.__cv_bridge = CvBridge()
        cv2.namedWindow("Temp Viewer", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Overlay Map", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Base Map", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detections", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        cv_image = self.__cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Temp Viewer", cv_image)
        cv2.waitKey(1)

    def overlay_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        data=data.transpose()
        data[data == 0] = 100
        data[data == 101] = 0

        cv2.imshow("Overlay Map", data * 2)  # Scale for better visibility
        cv2.waitKey(1)

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        data=data.transpose()
        data[data == -1] = 50  # Set unknown cells to mid-gray for visualization
        # data[data == 100] = 1    # Free space to black
        # data[data == 0] = 100  # Occupied space to whited
        # data[data == 1] = 0  # Unknown space to gray

        cv2.imshow("Base Map", data * 2)  # Scale for better visibility
        cv2.waitKey(1)
        # For debugging purposes, we can print the shape of the map
        # self.get_logger().info(f'Received map of size: {data.shape}')

    def detection_vision_callback(self,msg):
        print("Detection Vision Callback")
        cv_image = self.__cv_bridge.imgmsg_to_cv2( msg.detections.image_annotated)
        cv2.imshow("Detections", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    temp_viewer_node = TempViewer()
    rclpy.spin(temp_viewer_node)
    temp_viewer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()