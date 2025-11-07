
# ROS2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from yolo_interfaces.msg import YoloFrame, YoloItem

# OpenCV imports
import cv2      #pip3 install opencv-python
from cv_bridge import CvBridge

# YOLO library:
from ultralytics import YOLO #pip3 install typeguard ultralytics
#Ultralytics glitch when attempting to build. Use export to ensure proper import:
# export PYTHONPATH=</path/to/your/virtual/environment>/lib/python3.12/site-packages:$PYTHONPATH
# export PYTHONPATH=~/Desktop/EECE5554/EECE5554_2025_Project/sandbox_venv/lib/python3.12/site-packages:$PYTHONPATH
# export PYTHONPATH=/home/david-ross/gitRepos/EECE5554_2025_Project/sandbox_venv/lib/python3.12/site-packages:$PYTHONPATH

# Global Constants:
YOLO_MODEL_LIST = ['yolov8n.pt', 'yolov8s.pt', 'yolov8m.pt']
DEFAULT_YOLO_MODEL = 'yolov8s.pt'
DEFAULT_IMAGE_TOPIC = '/oakd/rgb/preview/image_raw'
DEFAULT_PUBLISH_TOPIC = '/yolo_detection'
DEFAULT_IMAGE_CONVERSION = 'passthrough' #'bgr8'

class YoloFrame():
    def __init__(self):
        pass
class YoloItem():
    def __init__(self):
        pass

class TurtleBotYoloDetector(Node):
    # Default values:
    __DEFAULT_MAX = 10
    __DEFAULT_THRESHOLD = 0.5
    __DEFAULT_FEED_SHOW = True
    __DEFAULT_SHOULD_PUBLISH = True
    __DEFAULT_LINE_THICKNESS = 2
    __DEFAULT_FONT_SCALE = 0.5
    #----------------------------------------------------------------------------------
    def __init__(self, model:str= DEFAULT_YOLO_MODEL, image_topic:str=DEFAULT_IMAGE_TOPIC, show_feed:bool=__DEFAULT_FEED_SHOW, publish:bool=__DEFAULT_SHOULD_PUBLISH,pub_topic:str=DEFAULT_PUBLISH_TOPIC):
        super().__init__('turtle_object_detector')
        # Model Variables:
        self.__model_name = model   # Name of Model
        self.__bridge = CvBridge()  # Image Conversion
        self.__model = YOLO(self.__model_name)  #  Load YOLO Model

        self.__bgr_font_color = (0,255,0)
        self.__bgr_box_color = (255,0,0)
        self.__font_scale = self.__DEFAULT_FONT_SCALE
        self.__line_thickness = self.__DEFAULT_LINE_THICKNESS
        self.__pure_image = None    #Stores the pure image returned from the camera
        self.__annotated_image = None   #Stores the image with boxes and identifiers
        self.__detection_threshold = self.__DEFAULT_THRESHOLD   # Threshold for detecting items
        self.__detected_list = []   #Stores a list of detected items
        self.__wanted_list = [] # Update this list to filter detections
        self.__show_cv_feed = show_feed # Set to True to view camera feed live

        # Node Variables
        self.__max_msgs = self.__DEFAULT_MAX
        self.__start_message = 'Starting Object Detection Node.'
        self.__image_topic = image_topic    #Subscription topic for image
        self.__should_publish_info = publish    # Set to true to publish data
        self.__publish_topic = pub_topic    # Topic to publish to
        self.__image_subscription = None    # Subscription object
        self.__object_publisher = None  # Publisher object
        
        # Set up parameters:
        self.__setup_topics()
        self.__setup_windows()

        self.get_logger().info(self.__start_message)

    #----------------------------------------------------------------------------------
    def __setup_windows(self):
        """Sets window values"""
        if self.__show_cv_feed:
            cv2.namedWindow("this",cv2.WINDOW_NORMAL)
            # cv2.resizeWindow("this", 1920,1080)

    #----------------------------------------------------------------------------------
    def __setup_topics(self):
        """Sets the topics for publishing and subscribing"""
        # Sub topic:
        self.__image_subscription = self.create_subscription(
            Image,
            self.__image_topic,
            self.__process_image,
            self.__max_msgs
        )

        # Pub topic
        if self.__should_publish_info:
            self.__object_publisher = self.create_publisher(
                YoloFrame, 
                self.__publish_topic,
                self.__max_msgs
            )
        
    #----------------------------------------------------------------------------------
    def __publish_data(self, message:YoloFrame):
        """Publishes the specified message"""
        self.__object_publisher.publish(message)
    #----------------------------------------------------------------------------------
    def __process_image(self,message:Image):
        """
        Takes an image in ROS2 message format and processes it 
        through the YOLO model. If flags are set, it will publish
        the data, or show on screen
        """
        # Convert ros2 message to image:
        cv_image = self.__bridge.imgmsg_to_cv2(message,DEFAULT_IMAGE_CONVERSION)
        # Pass frame through model and return results:
        results = self.__model(cv_image, verbose=False)[0]
        # Save original image:
        self.__pure_image = cv_image
        self.__annotated_image = cv_image
        #Reset list:
        self.__detected_list=[]
        

        # Get items and label if meet criteria
        if results.boxes is not None:
            for box in results.boxes:
                if self.__meets_critera(box):
                    # Get box coordinates:
                    x1,y1,x2,y2 = map(int,box.xyxy[0].tolist()) #Convert tensor to list
                    box_coords = [x1,y1,x2,y2]
                    # Get box name:
                    item_name = self.__model.names[int(box.cls)]    # Convert item index to name

                    # Create an object to hold detected information
                    detected_object = YoloObject(
                        box_coords,
                        item_name,
                        float(box.conf)
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

        # Publish info if flag is set
        if self.__should_publish_info:
            message = self.__generate_publish_message()
            self.__publish_data(message)

        # Show feed if flag is set
        if self.__show_cv_feed:
            cv2.imshow("this",self.__annotated_image)
            cv2.waitKey(1)

    #----------------------------------------------------------------------------------
    def __generate_publish_message(self):
        """
        Generates a message to publish based on information
        stored within the object
        returns: YoloFrame
        """
        pub_image_raw = self.__bridge.cv2_to_imgmsg(self.__pure_image)
        pub_image_annotated = self.__bridge.cv2_to_imgmsg(self.__annotated_image)
        pub_item_list = []
        for item in self.__detected_list:
            pub_item = YoloItem()
            pub_item.name = item.name
            pub_item.xyxy = item.xyxy
            pub_item.confidence = item.confidence

            pub_item_list.append(pub_item)

        pub_frame_message = YoloFrame()
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
    def shutdown_clean(self):
        cv2.destroyAllWindows()
    #----------------------------------------------------------------------------------
    #Getters:
    def get_raw_image(self):
        return self.__pure_image
    
    def get_annotated_image(self):
        return self.__annotated_image
    
    def get_detected_list(self):
        return self.__detected_list
    #----------------------------------------------------------------------------------
    #Setters:
    def set_publish_data(self,value:bool):
        self.__should_publish_info=value

    def set_publish_topic(self,topic:str):
        self.__publish_topic = topic

    def set_subscribe_topic(self, topic:str):
        self.__image_topic = topic

    def set_start_message(self, message:str):
        self.__start_message = message

    def set_show_cv_feed(self, value:bool):
        self.__show_cv_feed = value

    def set_wanted_list(self, list_of_items:list=[]):
        """provide a list of items as strings that the model should detect"""
        self.__wanted_list= list_of_items

    def set_detection_threshold(self, confidence:float=0.5):
        self.__detection_threshold = confidence

    def set_box_thickness(self, thickness=1):
        self.__line_thickness = thickness

    def set_box_color_rgb(self, r=0,g=0,b=0):
        b = b if b>=0 and b<=255 else 0
        g = g if g>=0 and g<=255 else 0
        r = r if r>=0 and r<=255 else 0
        self.__bgr_box_color = (b,g,r)

    def set_font_color_rgb(self, r=0,g=0,b=0):
        b = b if b>=0 and b<=255 else 0
        g = g if g>=0 and g<=255 else 0
        r = r if r>=0 and r<=255 else 0
        self.__bgr_font_color = (b,g,r)

    def set_font_scale(self, scale:float=0.5):
        self.__font_scale= scale

#*******************************************************
# Generic class to store detected objects data
class YoloObject:
    def __init__(self, xyxy:list=[0,0,0,0],name:str='',confidence:float=0.0):
        self.xyxy = xyxy
        self.name = name
        self.confidence = confidence



def main(args=None):
    rclpy.init(args=args)
    detector = TurtleBotYoloDetector(publish=False)
    rclpy.spin(detector)
    detector.shutdown_clean()
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        