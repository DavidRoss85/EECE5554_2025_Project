# Math Imports:
import math
import numpy as np

# ROS2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from object_location_interfaces.msg import RoboSync as RSync, DetectedItem, DetectionList, RSyncDetectionList,\
    ItemLocation, LocationList, RSyncLocationList


# OpenCV imports
import cv2      #pip3 install opencv-python
from cv_bridge import CvBridge

class DistanceNode(Node):

    DEFAULT_PUBLISH_TOPIC = '/objects/locations'
    DEFAULT_DETECTIONS_TOPIC = '/objects/detections'
    MAX_MSG = 10

    DEFAULT_IMAGE_ENCODING = 'passthrough' #'bgr8'


    DEFAULT_DEPTH_MIN = 0
    DEFAULT_DEPTH_MAX = 255

    DEFAULT_FOCAL_LENGTH = 870.0
    DEFAULT_BASELINE = 0.075

    def __init__(self):
        super().__init__('distance_node')
        self.get_logger().info('Initializing Distance Node')

        # Variables:
        self.__detections_topic = self.DEFAULT_DETECTIONS_TOPIC
        self.__locations_topic = self.DEFAULT_PUBLISH_TOPIC
        self.__max_msg = self.MAX_MSG
        self.__camera_width_res = 100
        self.__camera_angle_width = 130
        self.__image_encoding = self.DEFAULT_IMAGE_ENCODING
        self.__depth_min = self.DEFAULT_DEPTH_MIN
        self.__depth_max = self.DEFAULT_DEPTH_MAX
        
        self.__load_parameters()    # Load external parameters

        self.__bridge = CvBridge()
        self.get_logger().info(f'CV Bridge loaded')
        try:
            self.__detection_sub = self.create_subscription(
                RSyncDetectionList,
                self.__detections_topic,
                self.__process_detections,
                self.__max_msg
            )
            self.get_logger().info(f'Subscribed to detection topic: {self.__detections_topic}')
            self.__locations_pub = self.create_publisher(
                RSyncLocationList,
                self.__locations_topic,
                self.__max_msg
            )
            self.get_logger().info(f'Publisher created on topic: {self.__locations_topic}')

            self.get_logger().info('Successfully initialized Distance Node.')
        except Exception as e:
            self.__handle_error(e,'__init__()','Failed to initialize Distance Node')
            self.destroy_node()

    #----------------------------------------------------------------------------------
    def __load_parameters(self):
        pass
    #----------------------------------------------------------------------------------
    def __process_detections(self,message: RSyncDetectionList):
        print('Detected Object')
        detection_list = message.detections.item_list
        depth_image = message.robo_sync.depth_image
        locations_list =[]

        cv_image = self.__bridge.imgmsg_to_cv2(depth_image,self.__image_encoding)
        depth_map = np.clip(cv_image,self.__depth_min,self.__depth_max)
        
        # Calculate yaw and distance for each item and make a list:
        for item in detection_list:
            relative_location = ItemLocation()
            relative_location.name = item.name
            xc,yc,w,h = item.xywh   #Get center pixel of item
            relative_location.distance = float(depth_map[yc,xc])   #Reverse xc,yc for np arrays
            
            # Calculate relative yaw based on pixel location and known camera Angle of View
            relative_location.relative_yaw = float(
                self.__calculate_yaw_from_pixels(
                    self.__camera_width_res, self.__camera_angle_width, xc
                )
            )
            locations_list.append(relative_location)
        
        # Publish list of items
        rsync_msg = self.__generate_location_message(locations_list,message.robo_sync)
        self.__locations_pub.publish(rsync_msg)

    #----------------------------------------------------------------------------------
    def __generate_location_message(self, locations_list, robo_sync):
        rsync_msg = RSyncLocationList()
        list_msg = LocationList()
        list_msg.location_list = locations_list
        rsync_msg.robo_sync = robo_sync
        rsync_msg.locations = list_msg

    #----------------------------------------------------------------------------------
    def __calculate_yaw_from_pixels(self,camera_x_res, camera_angle_width, x):
        """
        Compute relative yaw angle (in radians) of an object based on its pixel x-coordinate.
        """
        
        # Pixel offset from center (negative=right, positive=left depending on convention)
        offset = (x - (camera_x_res / 2)) / (camera_x_res / 2)

        # Convert offset into angle
        yaw = offset * (camera_angle_width / 2)

        return yaw

    #----------------------------------------------------------------------------------
    def __handle_error(self, error, function_name, custom_message=''):
        self.get_logger().error(f'Error in {function_name}: {str(error)}. {custom_message}')



def main(args=None):
    rclpy.init()
    distance_node = DistanceNode()
    rclpy.spin(distance_node)
    distance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()