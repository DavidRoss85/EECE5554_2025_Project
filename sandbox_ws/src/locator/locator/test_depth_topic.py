
# ROS2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


# OpenCV imports
import cv2      #pip3 install opencv-python
from cv_bridge import CvBridge

import math
import numpy as np

# Global Constants:

DEFAULT_IMAGE_CONVERSION = 'passthrough' #'bgr8'

DEFAULT_IMAGE_TOPIC = '/oakd/rgb/image_raw'
# DEPTH_TOPIC = '/oakd/right/image_raw' #'/oakd/stereo/image_raw/compressedDepth' #'/oakd/rgb/preview/depth'
DEPTH_TOPIC = '/oakd/stereo/image_raw' #'/oakd/rgb/preview/depth'
# DEPTH_TOPIC = '/oakd/left/image_raw' #'/oakd/stereo/image_raw/compressedDepth' #'/oakd/rgb/preview/depth'


MAX_MSG = 10



class DepthAssign(Node):
    def __init__(self):
        super().__init__('depther')
        self.__bridge = CvBridge()

        self.__disparity_sub = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.__process_depth,
            MAX_MSG
        )

        # self.__rgb_sub = self.create_subscription(
        #     Image,
        #     DEFAULT_IMAGE_TOPIC,
        #     self.__process_image,
        #     MAX_MSG
        # )

        self.__depth_map = None

    def __process_depth(self, msg:Image):
        print("Received depth data")
        # return
        # cv_image = self.__bridge.imgmsg_to_cv2(msg,DEFAULT_IMAGE_CONVERSION)
        # cv2.imshow("depth",cv_image)
        # cv2.waitKey(1)

        # return 
        cv_image = self.__bridge.imgmsg_to_cv2(msg,DEFAULT_IMAGE_CONVERSION)
        self.__depth_map = np.clip(cv_image,0,65000)
        depth_image = np.clip(self.__depth_map,0,65000)
        depth_image = cv2.normalize(depth_image,None,0,255,cv2.NORM_MINMAX).astype(np.uint8)
 
        cv2.imshow("depth", depth_image)
        cv2.waitKey(1)


    def __process_image(self,msg:Image):
        # print("Image Received")
        
        cv_image = self.__bridge.imgmsg_to_cv2(msg,DEFAULT_IMAGE_CONVERSION)
        cv2.imshow("this",cv_image)
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
        