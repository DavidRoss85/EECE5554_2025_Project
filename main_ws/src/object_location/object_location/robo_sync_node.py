#This node will synchronize messages from multiple topics
#  and republish them as a single synchronized message.

# ROS2 Imports
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from message_filters import ApproximateTimeSynchronizer, Subscriber #pip3 install message-filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from object_location_interfaces.msg import RoboSync as RSync

#Testing:
from geometry_msgs.msg import PoseWithCovarianceStamped

from .helpers import USING_GAZEBO

class RoboSyncNode(Node):

    #Class Constants
    DEFAULT_USING_GAZEBO = USING_GAZEBO
    DEFAULT_IMAGE_TOPIC = '/oakd/rgb/preview/image_raw'
    DEFAULT_DEPTH_TOPIC = '/oakd/stereo/image_raw'
    DEFAULT_DEPTH_TOPIC_GAZEBO = '/oakd/rgb/preview/depth'
    DEFAULT_DYNAMIC_TRANSFORM_TOPIC = '/tf'
    DEFAULT_STATIC_TRANSFORM_TOPIC = '/tf_static'
    DEFAULT_PUBLISH_TOPIC = '/sync/robot/state'
    MAX_MSG = 10
    DEFAULT_SLOP = 0.1
    DEFAULT_SIMULATE_FRAME_LOSS = 0 #frames
    DEFAULT_SIMULATE_LAG_TIME = 0   # seconds


    def __init__(self):
        super().__init__('robo_sync_node')
        self.get_logger().info('Initializing RoboSyncNode...')

        self.__rgb_image = None
        self.__depth_image = None
        self.__robot_pose = None
        self.__image_topic = self.DEFAULT_IMAGE_TOPIC
        self.__depth_topic = self.DEFAULT_DEPTH_TOPIC if not self.DEFAULT_USING_GAZEBO else self.DEFAULT_DEPTH_TOPIC_GAZEBO
        self.__publish_topic = self.DEFAULT_PUBLISH_TOPIC
        self.__max_msg = self.MAX_MSG
        self.__slop = self.DEFAULT_SLOP
        self.__simulate_frame_loss = self.DEFAULT_SIMULATE_FRAME_LOSS
        self.__simulated_lag_time = self.DEFAULT_SIMULATE_LAG_TIME
        
        self.__load_parameters()

        self.__frame_counter = 0
        self.__msg_queue = []
        self.__lag_timer = self.create_timer(
            self.__simulated_lag_time,
            self.__publish_late_message
        )
        try:
            # Subscribers
            self.__image_sub = Subscriber(self, Image, self.__image_topic)
            self.get_logger().info(f'Subscribed to RGB Image topic: {self.__image_topic}')
            
            self.__depth_sub = Subscriber(self, Image, self.__depth_topic)
            self.get_logger().info(f'Subscribed to Depth Image topic: {self.__depth_topic}')
            
            # TF2 Buffer and Listener
            self.__tf_buffer = Buffer()
            self.__tf_listener = TransformListener(self.__tf_buffer, self)
            self.get_logger().info('TF2 Listener initialized.')

            # Synchronizer
            self.__sync = ApproximateTimeSynchronizer(
                [self.__image_sub, self.__depth_sub],
                queue_size=self.__max_msg,
                slop=self.__slop
            )
            self.get_logger().info('ApproximateTimeSynchronizer initialized.')
            self.__sync.registerCallback(self.__sync_callback)
            self.get_logger().info('Callback registered with synchronizer.')
            
            # Publisher
            self.__pub = self.create_publisher(
                RSync,
                self.__publish_topic,
                self.__max_msg
            )


            self.get_logger().info(f'Publisher created on topic: {self.__publish_topic}')
            
            self.get_logger().info('RoboSync Node initialized and ready.')
        except Exception as e:
            self.__handle_error(e,'__init__()','Failed to initialize RoboSync Node.')
            self.destroy_node()
    #----------------------------------------------------------------------------------
    def __load_parameters(self):
        """Loads external parameters """
        #Enter code here to load parameters
        pass
    #----------------------------------------------------------------------------------
    def __sync_callback(self, image_msg, depth_msg):
        
        # Simulates Frame loss by skipping messages
        self.__frame_counter += 1
        if self.__frame_counter <= self.__simulate_frame_loss:
            return
        else:
            self.__frame_counter = 0
        
        # Get images and robot pose
        self.__rgb_image = image_msg
        self.__depth_image = depth_msg
        self.__get_robot_pose()

        sync_msg = RSync()
        sync_msg.header.stamp = self.__rgb_image.header.stamp
        sync_msg.rgb_image = self.__rgb_image
        sync_msg.depth_image = self.__depth_image

        if self.__robot_pose is not None:
            sync_msg.robot_pose = self.__robot_pose

        # When perfected, should publish only when all three messages are available
        if self.__simulated_lag_time <= 0:
            self.__pub.publish(sync_msg)
            self.get_logger().info('Published synchronized message.')
        else:
            self.__msg_queue.append(sync_msg)
    #----------------------------------------------------------------------------------
    def __publish_late_message(self):
        # Simulates a delayed message
        if len(self.__msg_queue) > 0:
            self.__pub.publish(self.__msg_queue.pop(0))
            self.get_logger().info('Published delayed synchronized message.')

    #----------------------------------------------------------------------------------
    def __get_robot_pose(self):
        try:
            # Get transform map → base_link
            trans = self.__tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.__robot_pose = trans
        except Exception as e:
            self.__handle_error(e,'__get_robot_pose()','Error performing transform')

            
    #----------------------------------------------------------------------------------
    def __handle_error(self, error, function_name, custom_message=''):
        self.get_logger().error(f'Error in {function_name}: {str(error)}. {custom_message}')


def main(args=None):
    rclpy.init(args=args)
    robo_sync_node = RoboSyncNode()

    try:
        rclpy.spin(robo_sync_node)
    except KeyboardInterrupt:
        print('Shutting down RoboSyncNode...')
    finally:
        robo_sync_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    