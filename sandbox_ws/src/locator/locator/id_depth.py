
# ROS2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolo_interfaces.msg import YoloFrame, YoloItem

# OpenCV imports
import cv2      #pip3 install opencv-python
from cv_bridge import CvBridge

# YOLO library:
from ultralytics import YOLO #pip3 install typeguard ultralytics
#Ultralytics glitch when attempting to build. Use export to ensure proper import:
# export PYTHONPATH=</path/to/your/virtual/environment>/lib/python3.12/site-packages:$PYTHONPATH
# export PYTHONPATH=~/Desktop/EECE5554/EECE5554_2025_Project/venv/lib/python3.12/site-packages:$PYTHONPATH
# export PYTHONPATH=/home/david-ross/gitRepos/EECE5554_2025_Project/venv/lib/python3.12/site-packages:$PYTHONPATH


import math
import numpy as np

# Global Constants:
YOLO_MODEL_LIST = ['yolov8n.pt', 'yolov8s.pt', 'yolov8m.pt']
DEFAULT_YOLO_MODEL = 'yolov8s.pt'
DEFAULT_IMAGE_TOPIC = '/oakd/rgb/preview/image_raw'
DEFAULT_PUBLISH_TOPIC = '/yolo_detection'
DEFAULT_IMAGE_CONVERSION = 'passthrough' #'bgr8'


DEFAULT_GRID_TOPIC = '/map'
TURTLEBOT_WIDTH_METERS = 0.36
TURTLEBOT_ARROW_SCALE = 1.2

DEPTH_TOPIC = '/oakd/stereo/image_raw' #'/oakd/rgb/preview/depth'
MAX_MSG = 10
DEFAULT_IMAGE_CONVERSION = 'passthrough' #'bgr8'

FOCAL_LENGTH = 870.0
BASELINE = 0.075


class DepthAssign(Node):
    def __init__(self):
        super().__init__('depther')
        self.__bridge = CvBridge()
        self.__model = YOLO(DEFAULT_YOLO_MODEL)

        self.__disparity_sub = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.__process_depth,
            MAX_MSG
        )

        self.__rgb_sub = self.create_subscription(
            Image,
            DEFAULT_IMAGE_TOPIC,
            self.__process_image,
            MAX_MSG
        )

        self.__depth_map = None
        self.__depth_min = 0
        self.__depth_max = 1
        cv2.namedWindow("this",cv2.WINDOW_NORMAL)
        cv2.namedWindow("depth",cv2.WINDOW_NORMAL)

    def __process_depth(self, msg:Image):
        cv_image = self.__bridge.imgmsg_to_cv2(msg,DEFAULT_IMAGE_CONVERSION)
        self.__depth_map = cv_image #np.clip(cv_image,0,100)
        depth_array = np.array(cv_image)
        self.__depth_min = min(self.__depth_min,depth_array.min())
        self.__depth_max = max(self.__depth_max, depth_array.max())
        print(f"min: {self.__depth_min}, max: {self.__depth_max}")


    def __process_image(self,msg:Image):
        if self.__depth_map is None:
            print("Image recd")
            # cv_image = self.__bridge.imgmsg_to_cv2(msg,DEFAULT_IMAGE_CONVERSION)
            # cv2.imshow("this",cv_image)
            # cv2.waitKey(1)
            return
        
        cv_image = self.__bridge.imgmsg_to_cv2(msg,DEFAULT_IMAGE_CONVERSION)
        depth_image = np.clip(self.__depth_map,0,65535)
        depth_image = cv2.normalize(depth_image,None,0,100,cv2.NORM_MINMAX).astype(np.uint8)
        results = self.__model(cv_image,verbose=False)[0]
        detected_list=[]
        annotated_image = cv_image #results.plot()

        # Get items and label if meet criteria
        if results.boxes is not None:
            for box in results.boxes:
                if box.conf > 0.5:
                    # Get box coordinates:
                    x1,y1,x2,y2 = map(int,box.xyxy[0].tolist()) #Convert tensor to list
                    box_coords = [x1,y1,x2,y2]

                    xc,yc,w,h = map(int,box.xywh[0].tolist()) #Grab center of box
                    
                    # Get box name:
                    item_name = self.__model.names[int(box.cls)]    # Convert item index to name

                    # if item_name == 'person':
                    # print(f"Person: {xc},{yc}")
                    # # Remember to flip coords for np arrays (x=yc, y=xc)
                    # print(f"Depth: {self.__depth_map[yc,xc] if self.__depth_map is not None else 0}")
                    depth_image[yc,xc] = 255
                    
                    # Add text with item's name/type to annotated frame
                    annotated_image =  self.__add_text_to_image(
                        cv_image,
                        x = xc,
                        y = yc,
                        text = f"{self.__depth_map[yc,xc]:.2f}m away",
                        bgr = (0,255,0)
                    )
        

        
        cv2.imshow("this",annotated_image)
 
        cv2.imshow("depth", depth_image)
        cv2.waitKey(1)

    def __calculate_depth(self,f,b,d):
        if d <= 0: return 0
        return (f*b)/d
    #----------------------------------------------------------------------------------
    def __add_text_to_image(self, image, x:int=0, y:int=0, text='',bgr:tuple=(0,0,0),font=cv2.FONT_HERSHEY_SIMPLEX,thickness=1):
        """Modify image with text at location"""
        fontscale = 0.5
        cv2.putText(
            image,
            text,
            (x,y),
            font,
            fontscale,
            bgr,
            thickness=thickness,
            lineType=cv2.LINE_AA
        )
        return image
def main(args=None):
    rclpy.init(args=args)
    detector = DepthAssign()
    rclpy.spin(detector)
    detector.shutdown_clean()
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        