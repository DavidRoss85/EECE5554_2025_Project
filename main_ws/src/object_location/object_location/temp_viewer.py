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

from .helpers import find_a_star_path, quaternion_to_yaw, inflate_obstacles,transform_2d, YOLO_CLASSES
import cv2
from cv_bridge import CvBridge

class TempViewer(Node):
    def __init__(self):
        super().__init__('temp_viewer')
        self.get_logger().info('Initializing Temp Viewer Node')

        self.__latest_map = None
        self.__latest_overlay = None
        
        self.__combined_view_timer = self.create_timer(
            0.5,
            self.__update_combined_view
        )

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
        # cv2.namedWindow("Overlay Map", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("Base Map", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detections", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("Path Viewer", cv2.WINDOW_NORMAL)

    #----------------------------------------------------------------------------------
    def path_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        data = data.T

        self.__latest_map = {
            'info':msg.info,
            'data':np.array(msg.data, dtype=np.uint16).reshape((height, width)).T
        }
        
        return
    
        # self.__update_combined_view()

        # Create grayscale image
        img = np.zeros_like(data, dtype=np.uint8)


        img[data == 100] = 0     # occupied = black
        img[data == 0] = 255     # free = white
        # img[(data != 0) & (data != 100)] = 255 - img   # unknown = gray
        mask = (data != 0) & (data != 100)
        img[mask] = 255 - img[mask]

        cv2.imshow("Path Viewer", img)
        cv2.waitKey(1)
    #----------------------------------------------------------------------------------
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

        self.__latest_overlay = {
            'info':msg.info,
            'data':np.array(msg.data, dtype=np.uint16).reshape((height, width)).T
        }
        
        # self.__update_combined_view()
        # # Prepare color image (BGR)
        # overlay = np.zeros((width, height, 3), dtype=np.uint8)

        # # objects = bright greenda
        # overlay[data == 250] = (0, 255, 0)

        # # inflated obstacles = yellow
        # overlay[data == 100] = (0, 255, 255)
        
        # self.__latest_overlay = overlay
        # cv2.imshow("Overlay Map", data)
        # self.__update_combined_view()
        # cv2.waitKey(1)
    #--------------------------------------------------------------------------------
    def map_callback(self, msg):

        # if self.__latest_map is not None and self.__latest_overlay is not None:
        #     if self.__latest_map['info'].width == self.__latest_overlay['info'].width and \
        #        self.__latest_map['info'].height == self.__latest_overlay['info'].height:
        # self.__update_combined_view()
        return
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
    
    #------------------------------------------------------------------------------------
    def __update_combined_view(self):
        
        if self.__latest_map is None or self.__latest_overlay is None:
            return
        
        if self.__latest_map['data'] is None or self.__latest_overlay['data'] is None:
            return
        
        if self.__latest_map['info'].width != self.__latest_overlay['info'].width or \
            self.__latest_map['info'].height != self.__latest_overlay['info'].height:
            return
        
        width = self.__latest_map['info'].width
        height = self.__latest_map['info'].height

        overlay = self.__latest_overlay['data'].astype(np.uint8)
        map = self.__latest_map['data'].astype(np.uint8)

        # Convert grayscale map to BGR
        map_bgr = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)
        overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_GRAY2BGR)

        map_free_mask = map == 0
        map_occupied_mask = map == 100
        map_unknown_mask = map == 50
        map_robot_mask = map == 99

        map_bgr[map_free_mask] = [255, 255, 255]      # Free = White
        map_bgr[map_occupied_mask] = [0, 0, 0]          # Occupied = Black
        map_bgr[map_unknown_mask] = [127, 127, 127]      # Unknown = Gray
        map_bgr[map_robot_mask] = [10,10,10]    

        # Create mask where overlay is NOT zero
        mask = overlay != 0      # shape (H, W)
        mask3 = np.stack([mask]*3, axis=-1)   # make it 3-channel

        overlay_bgr[mask] = [255, 0, 255]   # Red for high overlay values

        # Overwrite only where mask is True
        combined = map_bgr.copy()
        combined[mask3] = overlay_bgr[mask3]


        # Blend overlay on top of occupancy map
        # combined = cv2.addWeighted(map_bgr, 1.0, overlay_bgr, 1.0, 0)
        cv2.namedWindow("Combined View", cv2.WINDOW_NORMAL)

        clusters = self.find_object_clusters(overlay)

        for class_code, instances in clusters.items():
            txt_lookup_index = max(0, class_code - 101)
            class_name = YOLO_CLASSES[txt_lookup_index]

            for inst in instances:
                cx, cy = inst["centroid"]

                cv2.putText(
                    combined,
                    class_name,
                    (cx, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )


        cv2.imshow("Combined View", combined)
        cv2.waitKey(100)

    #----------------------------------------------------------------------------------

    def find_object_clusters(self,overlay):
        # Dictionary: class_id → list of clusters
        # Each cluster contains (component_mask, centroid)
        clusters = {}

        # Get all unique class codes except zero
        class_codes = np.unique(overlay)
        class_codes = class_codes[class_codes != 0]

        for code in class_codes:
            # Binary mask for this class
            mask = (overlay == code).astype(np.uint8)

            # Run connected component labeling
            num_labels, labels = cv2.connectedComponents(mask)

            # Each label represents an instance
            # label 0 is background
            objs = []

            for comp_id in range(1, num_labels):
                comp_mask = (labels == comp_id)

                ys, xs = np.where(comp_mask)

                # Centroid
                cx = int(np.mean(xs))
                cy = int(np.mean(ys))

                objs.append({
                    "mask": comp_mask,
                    "centroid": (cx, cy),
                    "code": code
                })

            clusters[code] = objs

        return clusters

    #----------------------------------------------------------------------------------
    def detection_vision_callback(self,msg):
        cv_image = self.__cv_bridge.imgmsg_to_cv2( msg.detections.image_annotated)
        cv2.imshow("Detections", cv_image)
        cv2.waitKey(1)

    #----------------------------------------------------------------------------------
            

#*******************************************************************************
#*******************************************************************************
# Main function
def main(args=None):
    rclpy.init(args=args)
    temp_viewer_node = TempViewer()
    rclpy.spin(temp_viewer_node)
    temp_viewer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()