# ROS2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

import numpy as np
import matplotlib.pyplot as plt


DEFAULT_GRID_TOPIC = '/map'

class GridReader(Node):
    # Default values:
    __DEFAULT_MAX = 10

    #----------------------------------------------------------------------------------
    def __init__(self, sub_topic:str=DEFAULT_GRID_TOPIC):
        super().__init__('grid_reader')
        self.__sub_topic = sub_topic
        self.__max_msg = self.__DEFAULT_MAX

        self.__map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.__sub_topic,
            self.__interpret_map,
            self.__max_msg
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Matplotlib setup
        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.map_data = None
        self.map_info = None

        # Timer to update robot position
        # self.timer = self.create_timer(0.2, self.update_robot_marker)

    #----------------------------------------------------------------------------------
    def __interpret_map(self, msg):
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_data[self.map_data == -1] = 50  # unknown → gray
        self.map_info = msg.info

        self.ax.clear()
        self.ax.imshow(self.map_data, cmap='gray', origin='lower')
        plt.draw()
        plt.pause(0.01)
        self.update_robot_marker()

    def update_robot_marker(self):
        if self.map_info is None:
            return
        
        try:
            # Get transform map → base_link
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            # Convert world coordinates → map indices
            origin_x = self.map_info.origin.position.x
            origin_y = self.map_info.origin.position.y
            resolution = self.map_info.resolution

            j = int((x - origin_x) / resolution)
            i = int((y - origin_y) / resolution)

            # Redraw with robot marker
            self.ax.clear()
            self.ax.imshow(self.map_data, cmap='gray', origin='lower')
            self.ax.plot(j, i, 'ro', markersize=8)  # red circle marker
            plt.draw()
            plt.pause(0.01)

        except Exception as e:
            self.get_logger().warn(f"No transform available yet: {e}")



#********************************************************************
def main(args=None):
    rclpy.init(args=args)
    map_reader = GridReader()
    rclpy.spin(map_reader)
    # detector.shutdown_clean()
    map_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        