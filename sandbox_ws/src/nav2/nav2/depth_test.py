
# ROS2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from yolo_interfaces.msg import YoloFrame, YoloItem

# OpenCV imports
import cv2      #pip3 install opencv-python
from cv_bridge import CvBridge

# Math imports
import numpy as np

IMAGE_TOPIC = '/oakd/stereo/image_raw' # '/oakd/rgb/preview/depth'
MAX_MSG = 10
DEFAULT_IMAGE_CONVERSION = 'passthrough' #'bgr8'

FOCAL_LENGTH = 870.0
BASELINE = 0.075



class DisparityShow(Node):
    def __init__(self):
        super().__init__('disparity_show_node')
        self.__bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.__process_disparity,
            MAX_MSG
        )

        cv2.namedWindow("Disparity Demo",cv2.WINDOW_NORMAL)

    def __process_disparity(self, msg:Image):

        cv_image = self.__bridge.imgmsg_to_cv2(msg,DEFAULT_IMAGE_CONVERSION)
        disp_vis = np.clip(cv_image,0,100)
        disp_vis = cv2.normalize(disp_vis, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        
        h,w = disp_vis.shape
        u,v = w//2, int(h/3)

        point_depth = cv_image[v,u]
        print(f"Depth a center pixel: {point_depth:.7f}")
        print(f"{disp_vis.dtype}, {disp_vis.min()}, {disp_vis.max()}")
        disp_vis[v,u] = 255
        cv2.imshow("Disparity Demo",disp_vis)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init()
    disparity = DisparityShow()
    rclpy.spin(disparity)

    disparity.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()