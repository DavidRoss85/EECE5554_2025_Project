#This node performs object detection using a YOLO model on incoming RGB images
# and publishes detected objects along with annotated images.

# Math Imports:
import math
import numpy as np

# ROS2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from object_location_interfaces.msg import RoboSync as RSync, DetectedItem, DetectionList, RSyncDetectionList, RSyncDetectionList


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
    YOLO_MODEL_LIST = ['yolov8n.pt', 'yolov8s.pt', 'yolov8m.pt']
    DEFAULT_YOLO_MODEL_PATH = YOLO_MODEL_LIST[2]  # Use yolov8n.pt for nano model
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
    DEFAULT_WANTED_LIST = ['bottle']  # Empty list means all items are wanted

    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info('Initializing Detection Node...')

        #Variables:
        self.__model_name = self.DEFAULT_YOLO_MODEL_PATH
        self.__image_encoding = self.DEFAULT_IMAGE_ENCODING
        self.__sync_topic = self.DEFAULT_SYNC_TOPIC
        self.__publish_topic = self.DEFAULT_PUBLISH_TOPIC
        self.__max_msg = self.MAX_MSG

        self.__bgr_font_color = self.DEFAULT_BGR_FONT_COLOR
        self.__bgr_box_color = self.DEFAULT_BGR_BOX_COLOR
        self.__font_scale = self.DEFAULT_FONT_SCALE
        self.__line_thickness = self.DEFAULT_LINE_THICKNESS

        self.__show_cv_feed = self.DEFAULT_FEED_SHOW
        self.__pure_image = None    #Stores the pure image returned from the camera
        self.__annotated_image = None   #Stores the image with boxes and identifiers
        self.__detection_threshold = self.DEFAULT_THRESHOLD   # Threshold for detecting items
        self.__detected_list = []   #Stores a list of detected items
        self.__wanted_list = self.DEFAULT_WANTED_LIST # Update this list to filter detections
        self.__still_in_function = False

        self.__load_parameters()    #Load external parameters

        self.__bridge = CvBridge()
        self.get_logger().info(f'CV Bridge loaded')

        self.__model = YOLO(self.__model_name)
        self.get_logger().info(f'YOLO model loaded from: {self.__model_name}')
        self.get_logger().info(f'Model confidence threshold set to: {self.__detection_threshold}')
        self.get_logger().info(f'Number of items in dataset: {len(self.__model.names)}')

        try:
            # Subscriber
            self.__rsync_sub = self.create_subscription(
                RSync,
                self.__sync_topic,
                self.__process_detections,
                self.__max_msg
            )
            self.get_logger().info(f'Subscribed to sync topic: {self.__sync_topic}')

            # Publisher
            self.__detection_pub = self.create_publisher(
                RSyncDetectionList,
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
    def __process_detections(self,message:RSync):
        print("Processing Detections...")
        rgb_image = message.rgb_image

        # Convert ros2 message to image:
        cv_image = self.__bridge.imgmsg_to_cv2(rgb_image,self.__image_encoding)
        # Pass frame through model and return results:
        results = self.__model(cv_image, verbose=False)[0]
        # Save original image:
        self.__pure_image = cv_image
        self.__annotated_image = cv_image
        #Reset list:
        self.__detected_list=[]
        
        
        # Get items and label if meet criteria
        count = 0
        if results.boxes is not None:
            for box in results.boxes:
                count +=1
                if self.__meets_critera(box):
                    # Get box coordinates:
                    x1,y1,x2,y2 = map(int,box.xyxy[0].tolist()) #Convert tensor to list
                    box_coords = [x1,y1,x2,y2]

                    xc,yc,w,h = map(int,box.xywh[0].tolist()) #Grab center of box
                    box_center = [xc,yc,w,h]

                    # Get box name:
                    item_name = self.__model.names[int(box.cls)]    # Convert item index to name

                    # Create an object to hold detected information
                    detected_object = DetectedObject(
                        xyxy=box_coords,
                        xywh=box_center,
                        name=item_name,
                        confidence=float(box.conf),
                        index=int(box.cls)
                    )
                    
                    # Add to list:
                    self.__detected_list.append(detected_object)
                    
                    # Add bounding box to annotated frame
                    self.__annotated_image = self.__add_bounding_box_to_image(
                        self.__annotated_image,
                        xyxy = box_coords,
                        bgr = self.__bgr_box_color,
                        thickness= self.__line_thickness
                    )

                    # Add text with item's name/type to annotated frame
                    self.__annotated_image=  self.__add_text_to_image(
                        self.__annotated_image,
                        x = box_coords[0],
                        y = box_coords[1],
                        text = item_name,
                        bgr = self.__bgr_font_color
                    )
                    # if count >= 10:
                    #     break
                # for i in range(100000000):
                #     pass

        # Publish message if there are detections:
        if len(self.__detected_list) > 0:
            pub_msg = RSyncDetectionList()
            pub_msg.header.stamp = rclpy.time.Time().to_msg()
            pub_msg.robo_sync =  message
            pub_msg.detections = self.__generate_detection_list()
            self.__publish_data(pub_msg)

        # Show feed if flag is set
        if self.__show_cv_feed:
            cv2.imshow("Detections",self.__annotated_image)
            cv2.waitKey(1)


    #----------------------------------------------------------------------------------
    def __generate_detection_list(self):
        """
        Generates a message to publish based on information
        stored within the object
        returns: DetectionList
        """
        pub_image_raw = self.__bridge.cv2_to_imgmsg(self.__pure_image)
        pub_image_annotated = self.__bridge.cv2_to_imgmsg(self.__annotated_image)
        pub_item_list = []
        for item in self.__detected_list:
            pub_item = DetectedItem()
            pub_item.name = item.name
            pub_item.xyxy = item.xyxy
            pub_item.xywh = item.xywh
            pub_item.confidence = item.confidence
            pub_item.index = item.index

            pub_item_list.append(pub_item)

        pub_frame_message = DetectionList()
        pub_frame_message.image_raw = pub_image_raw
        pub_frame_message.image_annotated = pub_image_annotated
        pub_frame_message.item_list = pub_item_list

        return pub_frame_message
    #----------------------------------------------------------------------------------
    def __meets_critera(self, box):
        """
        Criteria for identifying an object
        """
        # If confidence is over threshold
        if box.conf < self.__detection_threshold:
            return False
        
        # If there's a list of items to look for then filter by list
        if len(self.__wanted_list) > 0 and self.__model.names[int(box.cls)] not in self.__wanted_list:
            return False
        
        return True
    #----------------------------------------------------------------------------------
    def __add_bounding_box_to_image(self, image, xyxy=[0,0,0,0],bgr:tuple=(0,0,0),thickness:int=1):
        """Draw a rectangle at the given coordinates"""
        cv2.rectangle(
            image, 
            (xyxy[0],xyxy[1]),
            (xyxy[2],xyxy[3]),
            bgr,
            thickness
        )
        return image
    #----------------------------------------------------------------------------------
    def __add_text_to_image(self, image, x:int=0, y:int=0, text='',bgr:tuple=(0,0,0),font=cv2.FONT_HERSHEY_SIMPLEX,thickness=1):
        """Modify image with text at location"""
        cv2.putText(
            image,
            text,
            (x,y),
            font,
            self.__font_scale,
            bgr,
            thickness=thickness,
            lineType=cv2.LINE_AA
        )
        return image
 
    #----------------------------------------------------------------------------------
    def __publish_data(self,message:RSyncDetectionList):
        self.__detection_pub.publish(message)

    #----------------------------------------------------------------------------------
    def __handle_error(self, error, function_name, custom_message=''):
        self.get_logger().error(f'Error in {function_name}: {str(error)}. {custom_message}')


#*******************************************************
# Generic class to store detected objects data 
# (THIS IS POINTLESS... MAKE NOTE TO REPLACE WITH a normal DetectedItem())
class DetectedObject:
    def __init__(self, xyxy:list=[0,0,0,0], xywh:list =[0,0,0,0], name:str='',confidence:float=0.0, index:int=0):
        self.xyxy = xyxy
        self.xywh = xywh
        self.name = name
        self.confidence = confidence
        self.index = index



def main(args=None):
    rclpy.init()
    detection_node = DetectionNode()
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()