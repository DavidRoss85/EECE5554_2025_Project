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

class RoboSyncNode(Node):

    #Class Constants
    DEFAULT_IMAGE_TOPIC = '/oakd/rgb/preview/image_raw'
    DEFAULT_DEPTH_TOPIC = '/oakd/stereo/image_raw'#/oakd/rgb/preview/depth'
    DEFAULT_DYNAMIC_TRANSFORM_TOPIC = '/tf'
    DEFAULT_STATIC_TRANSFORM_TOPIC = '/tf_static'
    DEFAULT_PUBLISH_TOPIC = '/sync/robot/state'
    MAX_MSG = 10
    DEFAULT_SLOP = 0.1


    def __init__(self):
        super().__init__('robo_sync_node')
        self.get_logger().info('Initializing RoboSyncNode...')

        self.__rgb_image = None
        self.__depth_image = None
        self.__robot_pose = None
        self.__image_topic = self.DEFAULT_IMAGE_TOPIC
        self.__depth_topic = self.DEFAULT_DEPTH_TOPIC
        self.__publish_topic = self.DEFAULT_PUBLISH_TOPIC
        self.__max_msg = self.MAX_MSG
        self.__slop = self.DEFAULT_SLOP
        
        self.__load_parameters()
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
        self.__rgb_image = image_msg
        self.__depth_image = depth_msg
        self.__get_robot_pose()

        if self.__robot_pose is not None:
            sync_msg = RSync()
            sync_msg.rgb_image = self.__rgb_image
            sync_msg.depth_image = self.__depth_image
            sync_msg.robot_pose = self.__robot_pose

        # When perfected, should publish only when all three messages are available
        self.__pub.publish(sync_msg)
        #self.get_logger().info('Published synchronized message.')

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


    def set_intial_pose(self):
        #Testing:
        posepub = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            self.__max_msg
        )
        self.set_intial_pose()
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0

        posepub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    robo_sync_node = RoboSyncNode()
    rclpy.spin(robo_sync_node)
    robo_sync_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    