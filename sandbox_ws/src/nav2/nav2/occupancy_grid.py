# ROS2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

import math
import numpy as np
import matplotlib.pyplot as plt

from .helpers import find_a_star_path, quaternion_to_yaw, inflate_obstacles

DEFAULT_GRID_TOPIC = '/map'
TURTLEBOT_WIDTH_METERS = 0.36
TURTLEBOT_ARROW_SCALE = 1.2

class GridReader(Node):
    # Default values:
    __DEFAULT_MAX = 10
    __DEFAULT_DRAW_INTERVAL = .5
    __DEFAULT_GOAL = [10,15]
    __DEFAULT_FINDING = True
    __DEFAULT_PATH_LINE_COLOR = 'lime'
    __DEFAULT_PATH_LINE_WIDTH = 2
    __DEFAULT_PATH_LINE_STYLE = '-'
    __DEFAULT_ARROW_HEAD_WIDTH = 5
    __DEFAULT_ARROW_HEAD_LENGTH = 1

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
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)

        # Matplotlib setup
        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.__map_data = None
        self.__map_info = None
        self.__location_goal = self.__DEFAULT_GOAL
        self.__robot_width = TURTLEBOT_WIDTH_METERS
        self.__robot_arrow_length = TURTLEBOT_WIDTH_METERS * TURTLEBOT_ARROW_SCALE
        self.__draw_interval = self.__DEFAULT_DRAW_INTERVAL
        self.__map_draw_timer = self.create_timer(self.__draw_interval,self.__plot_map)
        self.__path_line_color = self.__DEFAULT_PATH_LINE_COLOR
        self.__path_line_width = self.__DEFAULT_PATH_LINE_WIDTH
        self.__path_linestyle = self.__DEFAULT_PATH_LINE_STYLE
        self.__arrow_head_width = self.__DEFAULT_ARROW_HEAD_WIDTH
        self.__arrow_head_length = self.__DEFAULT_ARROW_HEAD_LENGTH
        self.__finding_path = self.__DEFAULT_FINDING


        self.get_logger().info('Up and running')
    #----------------------------------------------------------------------------------
    def __interpret_map(self, msg):
        self.__map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.__map_data[self.__map_data == -1] = 50  # unknown → gray
        self.__map_info = msg.info
        self.__save_map_data()

    #----------------------------------------------------------------------------------
    def __save_map_data(self):
        np.savetxt('./mapdata.txt',self.__map_data,delimiter=',',fmt='%.0f')

    #----------------------------------------------------------------------------------
    def __fetch_origin_and_resolution(self):
        origin_x = self.__map_info.origin.position.x
        origin_y = self.__map_info.origin.position.y
        resolution = self.__map_info.resolution
        return [origin_x,origin_y,resolution]
    #----------------------------------------------------------------------------------
    def __convert_world_to_grid(self, x, y):
        
        origin_x, origin_y, resolution = self.__fetch_origin_and_resolution()

        i = int((y - origin_y) / resolution)  # row
        j = int((x - origin_x) / resolution)  # col

        return [i,j]
    #----------------------------------------------------------------------------------   
    def __convert_grid_to_world(self, i, j):
        origin_x, origin_y, resolution = self.__fetch_origin_and_resolution()

        x = origin_x + j * resolution
        y = origin_y + i * resolution
        return [x, y]

    #----------------------------------------------------------------------------------
    def __fetch_robot_world_location(self):
        DEFAULT = [0,0,0,[0,0,0,0]]

        if self.__map_info is None:
            return DEFAULT
        
        try:
            # Get transform map → base_link
            trans = self.__tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            q = trans.transform.rotation
            return [x,y,z,q]
        
        except Exception as e:
            self.__handle_error(e,'update_robot_marker()','No transform available yet')
            return DEFAULT
        
    #----------------------------------------------------------------------------------
    def __plot_map(self):
        if self.__map_data is None:
            return
        self.ax.clear()

        self.ax.imshow(self.__map_data, cmap='gray_r', origin='lower')
        # self.ax.imshow(inflated_map, cmap='gray_r', origin='lower')
        
        self.__update_robot_marker()

        x,y,z,q = self.__fetch_robot_world_location()

        if self.__finding_path:
            ox,oy,res = self.__fetch_origin_and_resolution()
            inflated_map = inflate_obstacles(self.__map_data,self.__robot_width,res,51)
            
            i,j = self.__convert_world_to_grid(x,y)
            k,l = self.__convert_world_to_grid(self.__location_goal[0],self.__location_goal[1])

            # path = find_a_star_path([i,j], [k,l],self.__map_data,51)
            path = find_a_star_path([i,j], [k,l],inflated_map,51,1.5)
        
            self.__plot_path_on_map(path)


        plt.draw()
        plt.pause(0.01)
    #----------------------------------------------------------------------------------
    def __update_robot_marker(self):
        if self.__map_info is None:
            return
        
        # Robot's location in grid, not converted to world coords:
        x, y, z, q = self.__fetch_robot_world_location()
        #Get robot pose/orientation:
        yaw = quaternion_to_yaw(q)
        # Convert world coordinates to grid
        ox,oy,resolution = self.__fetch_origin_and_resolution()
        i,j= self.__convert_world_to_grid(x,y)

        # Redraw robot marker
        self.ax.plot(j, i, 'ro', markersize=(self.__robot_width / resolution))  # red circle marker
        # Draw direction arrow (scaled to grid)
        arrow_length = self.__robot_arrow_length / resolution  # in grid cells
        dj = arrow_length * math.cos(yaw)
        di = arrow_length * math.sin(yaw)
        self.ax.arrow(j, i, dj, di, head_width=self.__arrow_head_width, head_length=self.__arrow_head_length, fc='r', ec='r')


    def __handle_error(self,e:Exception, function_location:str='',msg:str='',show_exception:bool=False):
        self.get_logger().warn(f'An error occured in {function_location}\n{msg}\n{e if show_exception else ""}')

    #----------------------------------------------------------------------------------
    def __plot_path_on_map(self, path):
        """
        Plots an A* path (list of [x, y] world coords) on the current occupancy map.
        Draws a green line connecting all path points.
        """
        if self.__map_data is None or self.__map_info is None:
            # self.get_logger().warn("Map not available.")
            return
        if not path:
            self.get_logger().warn("Empty path.")
            return

        # Separate for plotting
        i_vals = [p[0] for p in path]
        j_vals = [p[1] for p in path]

        self.ax.plot(j_vals, i_vals, color=self.__path_line_color, linewidth=self.__path_line_width, linestyle=self.__path_linestyle)

        return




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
        