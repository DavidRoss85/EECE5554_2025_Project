
# ROS2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# OpenCV imports
import cv2      #pip3 install opencv-python
from cv_bridge import CvBridge

# YOLO library:
from ultralytics import YOLO #pip3 install typeguard ultralytics
#Ultralytics glitch when attempting to build. Use export to ensure proper import:
# export PYTHONPATH=</path/to/your/virtual/environment>/lib/python3.12/site-packages:$PYTHONPATH

# export PYTHONPATH=/home/david-ross/gitRepos/EECE5554_2025_Project/sandbox_venv/lib/python3.12/site-packages:$PYTHONPATH

# Global Constants:
YOLO_MODEL_LIST = ['yolov8n.pt', 'yolov8s.pt', 'yolov8m.pt']
DEFAULT_YOLO_MODEL = 'yolov8s.pt'
DEFAULT_IMAGE_TOPIC = '/oakd/rgb/preview/image_raw'
DEFAULT_IMAGE_CONVERSION = 'bgr8'

class TurtleBotYoloDetector(Node):
    __DEFAULT_MAX = 10
    __DEFAULT_THRESHOLD = 0.5
    __DEFAULT_FEED_SHOW = True
    __DEFAULT_LINE_THICKNESS = 2
    __DEFAULT_FONT_SCALE = 0.5
    #----------------------------------------------------------------------------------
    def __init__(self, model:str= DEFAULT_YOLO_MODEL, image_topic:str=DEFAULT_IMAGE_TOPIC, show_feed:bool=__DEFAULT_FEED_SHOW):
        super().__init__('turtle_object_detector')
        self.__model_name = model   # Name of Model
        self.__bridge = CvBridge()  # Image Conversion
        self.__model = YOLO(self.__model_name)  #  Load YOLO Model
        self.__image_topic = image_topic    #Subscription topic for image
        self.__max_msgs = self.__DEFAULT_MAX
        self.__start_message = 'Starting Object Detection Node.'

        self.__bgr_font_color = (0,255,0)
        self.__bgr_box_color = (255,0,0)
        self.__font_scale = self.__DEFAULT_FONT_SCALE
        self.__line_thickness = self.__DEFAULT_LINE_THICKNESS
        self.__pure_image = None    #Stores the pure image returned from the camera
        self.__annotated_image = None   #Stores the image with boxes and identifiers
        self.__detection_threshold = self.__DEFAULT_THRESHOLD   # Threshold for detecting items
        self.__detected_list = []   #Stores a list of detected items
        self.__wanted_list = []
        self.__show_cv_feed = show_feed

        self.__image_subscription = self.create_subscription(
            Image,
            self.__image_topic,
            self.__process_image,
            self.__max_msgs
        )
        cv2.namedWindow("this",cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("this", 1920,1080)
        self.get_logger().info(self.__start_message)

    #----------------------------------------------------------------------------------
    def __process_image(self,message:Image):
        # Convert ros2 message to image:
        cv_image = self.__bridge.imgmsg_to_cv2(message)
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
        if self.__show_cv_feed:
            cv2.imshow("this",self.__annotated_image)
            cv2.waitKey(1)

    #----------------------------------------------------------------------------------
    def __meets_critera(self, box):
        # If confidence is over threshold
        if box.conf < self.__detection_threshold:
            return False
        
        # If there's a list of items to look for then filter by list
        if len(self.__wanted_list) > 0 and self.__model.names[int(box.cls)] not in self.__wanted_list:
            return False
        
        return True
    #----------------------------------------------------------------------------------
    def __add_bounding_box_to_image(self, image, xyxy=[0,0,0,0],bgr:tuple=(0,0,0),thickness:int=1):
        #Draw a rectangle at the given coordinates
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
        #Modify image with text at location
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
    #Getters:
    def get_pure_image(self):
        return self.__pure_image
    def get_annotated_image(self):
        return self.__annotated_image

#*******************************************************
# Generic class to store detected objects data
class YoloObject:
    def __init__(self, xyxy:list=[0,0,0,0],name:str='',confidence:float=0.0):
        self.xyxy = xyxy
        self.name = name
        self.confidence = confidence



def main(args=None):
    rclpy.init(args=args)
    detector = TurtleBotYoloDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        

# Clean up
# cap.release()
# cv2.destroyAllWindows()