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

        self.__latest_map = None
        self.__latest_overlay = None

        self.__image_subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )

        self.__overlay_subscription = self.create_subscription(
            OccupancyGrid,
            '/grid/overlay',
            self.overlay_callback,
            10
        )

        self.__map_subscription = self.create_subscription(
            OccupancyGrid,
            '/grid/occupancy',
            self.map_callback,
            10
        )

        self.__detection_vision = self.create_subscription(
            RSyncDetectionList,
            '/objects/detections',
            self.detection_vision_callback,
            10
        )
        self.__navigation_subscription = self.create_subscription(
            OccupancyGrid,
            '/grid/navigation',
            self.path_callback,
            10
        )


        self.__cv_bridge = CvBridge()
        cv2.namedWindow("Temp Viewer", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Overlay Map", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Base Map", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detections", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Path Viewer", cv2.WINDOW_NORMAL)

    #----------------------------------------------------------------------------------
    def path_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        data = data.T

        # Create grayscale image
        img = np.zeros_like(data, dtype=np.uint8)


        img[data == 100] = 0     # occupied = black
        img[data == 0] = 255     # free = white
        img[(data != 0) & (data != 100)] = 127   # unknown = gray

        cv2.imshow("Path Viewer", img)
        cv2.waitKey(1)
    def image_callback(self, msg):
        cv_image = self.__cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Temp Viewer", cv_image)
        cv2.waitKey(1)
    #----------------------------------------------------------------------------------
    def overlay_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        data = data.T

        # # Prepare color image (BGR)
        # overlay = np.zeros((width, height, 3), dtype=np.uint8)

        # # objects = bright greenda
        # overlay[data == 250] = (0, 255, 0)

        # # inflated obstacles = yellow
        # overlay[data == 100] = (0, 255, 255)
        
        # self.__latest_overlay = overlay
        cv2.imshow("Overlay Map", data)
        # self.__update_combined_view()
        cv2.waitKey(1)
    #--------------------------------------------------------------------------------
    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        data = data.T

        # Create grayscale image
        img = np.zeros_like(data, dtype=np.uint8)


        img[data == 100] = 0     # occupied = black
        img[data == 0] = 255     # free = white
        img[(data != 0) & (data != 100)] = 127   # unknown = gray

        self.__latest_map = img
        cv2.imshow("Base Map", img)
        # self.__update_combined_view()
        cv2.waitKey(1)
    

    def __update_combined_view(self):
        if self.__latest_map is None or self.__latest_overlay is None:
            return

        # Convert grayscale map to BGR
        map_bgr = cv2.cvtColor(self.__latest_map, cv2.COLOR_GRAY2BGR)

        print("MAP SHAPE:     ", map_bgr.shape)
        print("OVERLAY SHAPE: ", self.__latest_overlay.shape)

        # Blend overlay on top of occupancy map
        combined = cv2.addWeighted(map_bgr, 1.0, self.__latest_overlay, 1.0, 0)

        cv2.imshow("Combined View", combined)
        cv2.waitKey(1)


    def detection_vision_callback(self,msg):
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