
# ROS2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from yolo_interfaces.msg import YoloFrame, YoloItem
from object_detection_interfaces.msg import RoboSync as RSync, DetectionList


# OpenCV imports
import cv2      #pip3 install opencv-python
from cv_bridge import CvBridge

# YOLO library:
from ultralytics import YOLO #pip3 install typeguard ultralytics
#Ultralytics glitch when attempting to build. Use export to ensure proper import:
# export PYTHONPATH=</path/to/your/virtual/environment>/lib/python3.12/site-packages:$PYTHONPATH
# export PYTHONPATH=~/Desktop/EECE5554/EECE5554_2025_Project/venv/lib/python3.12/site-packages:$PYTHONPATH
# export PYTHONPATH=/home/david-ross/gitRepos/EECE5554_2025_Project/venv/lib/python3.12/site-packages:$PYTHONPATH




class DetectionNode(Node):

    # Class Constants:

    # ROS2
    DEFAULT_SYNC_TOPIC = '/sync/robot/state'
    DEFAULT_PUBLISH_TOPIC = '/objects/detections'
    MAX_MSG = 10

    # YOLO
    DEFAULT_YOLO_MODEL_PATH = 'yolov8n.pt'  # Use yolov8n.pt for nano model
    DEFAULT_CONFIDENCE_THRESHOLD = 0.8

    #OpenCV
    DEFAULT_IMAGE_ENCODING = 'passthrough'#'bgr8'  # OpenCV uses BGR format

    # Default values:
    DEFAULT_MAX = 10
    DEFAULT_THRESHOLD = 0.5
    DEFAULT_FEED_SHOW = True
    DEFAULT_SHOULD_PUBLISH = True
    DEFAULT_LINE_THICKNESS = 2
    DEFAULT_FONT_SCALE = 0.5
    DEFAULT_BGR_FONT_COLOR = (0,255,0)
    DEFAULT_BGR_BOX_COLOR = (255,0,0)

    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info('Initializing DetectionNode...')

        #Variables:
        self.__model_name = self.DEFAULT_YOLO_MODEL_PATH
        self.__sync_topic = self.DEFAULT_SYNC_TOPIC
        self.__publish_topic = self.DEFAULT_PUBLISH_TOPIC
        self.__max_msg = self.MAX_MSG

        self.__bgr_font_color = self.DEFAULT_BGR_FONT_COLOR
        self.__bgr_box_color = self.DEFAULT_BGR_BOX_COLOR
        self.__font_scale = self.DEFAULT_FONT_SCALE
        self.__line_thickness = self.DEFAULT_LINE_THICKNESS

        self.__pure_image = None    #Stores the pure image returned from the camera
        self.__annotated_image = None   #Stores the image with boxes and identifiers
        self.__detection_threshold = self.DEFAULT_THRESHOLD   # Threshold for detecting items
        self.__detected_list = []   #Stores a list of detected items
        self.__wanted_list = [] # Update this list to filter detections

        self.__load_parameters()    #Load external parameters

        self.__bridge = CvBridge()
        self.get_logger().info(f'CV Bridge loaded')

        self.__yolo_model = YOLO(self.__model_name)
        self.get_logger().info(f'YOLO model loaded from: {self.__model_name}')

        try:
            # Subscriber
            self.__sub = self.create_subscription(
                RSync,
                self.__sync_topic,
                self.__sync_callback,
                self.__max_msg
            )
            self.get_logger().info(f'Subscribed to sync topic: {self.__sync_topic}')

            # Publisher
            self.__pub = self.create_publisher(
                YoloFrame,
                self.__publish_topic,
                10
            )
            self.get_logger().info(f'Publisher created on topic: {self.__publish_topic}')

            self.get_logger().info('Detection Node initialized and ready.')
        except Exception as e:
            self.__handle_error(e,'__init__()','Failed to initialize Detection Node')
            self.destroy_node()
    
    #----------------------------------------------------------------------------------
    def __load_parameters(self):
        """Loads external parameters """
        #Enter code here to load parameters
        pass

    #----------------------------------------------------------------------------------
    def __handle_error(self, error, function_name, custom_message=''):
        self.get_logger().error(f'Error in {function_name}: {str(error)}. {custom_message}')


def main(args=None):
    rclpy.init()
    detection_node = DetectionNode()
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()