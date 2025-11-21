# This node reads the occupancy grid from ROS2, combines it with TF2 data to determine
# the robot's position. It will also use the RsyncLocationList from the detection node to
# plot detected objects on the map build a navigation map, and object overlay to
# go with the occupancy grid.

# ROS2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from object_location_interfaces.msg import (
    RoboSync as RSync,
    DetectedItem,
    DetectionList,
    RSyncDetectionList,
    ItemLocation,
    LocationList,
    RSyncLocationList,
)
import math
import numpy as np
import matplotlib.pyplot as plt

from .helpers import find_a_star_path, quaternion_to_yaw, inflate_obstacles,transform_2d
import cv2


class MapGenerator(Node):
    # Default values:
    DEFAULT_OCCUPANCY_SUB_TOPIC = '/map'
    DEFAULT_LOCATIONS_SUB_TOPIC = '/objects/locations'
    DEFAULT_OVERLAY_PUBLISH_TOPIC = '/grid/overlay'
    DEFAULT_OCCUPANCY_PUBLISH_TOPIC = '/grid/occupancy'
    DEFAULT_NAVIGATION_PUBLISH_TOPIC = '/grid/navigation'
    MAX_MSG = 10
    
    DEFAULT_ROBOT_WIDTH_METERS = 0.36
    DEFAULT_ROBOT_POSE_QUATERNION = [0,0,0,[0,0,0,0]]

    #----------------------------------------------------------------------------------
    def __init__(self):
        super().__init__('map_node')
        self.__grid_sub_topic = self.DEFAULT_OCCUPANCY_SUB_TOPIC
        self.__locations_sub_topic = self.DEFAULT_LOCATIONS_SUB_TOPIC

        self.__overlay_map_topic = self.DEFAULT_OVERLAY_PUBLISH_TOPIC
        self.__occupancy_map_topic = self.DEFAULT_OCCUPANCY_PUBLISH_TOPIC
        self.__navigation_map_topic = self.DEFAULT_NAVIGATION_PUBLISH_TOPIC

        self.__max_msg = self.MAX_MSG

        self.__map_data = None
        self.__map_info = None
        self.__items_grid = np.zeros((1,1)) # Placeholder until map received
        self.__navigation_grid = np.zeros((1,1)) # Placeholder until map received
        self.__last_robot_pose = self.DEFAULT_ROBOT_POSE_QUATERNION
        self.__robot_width = self.DEFAULT_ROBOT_WIDTH_METERS
        
        self.__locations_subscriber = self.create_subscription(
            RSyncLocationList,
            self.__locations_sub_topic,
            self.__interpret_locations,
            self.__max_msg
        )
        
        self.__grid_subscriber = self.create_subscription(
            OccupancyGrid,
            self.__grid_sub_topic,
            self.__interpret_grid,
            self.__max_msg
        )

        # Publishers
        self.__overlay_publisher = self.create_publisher(
            OccupancyGrid,
            self.__overlay_map_topic,
            self.__max_msg
        )

        # TF2 Buffer and Listener for robot pose
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)



        #Test area:
        # cv2.namedWindow("Occupancy",cv2.WINDOW_NORMAL)

        self.get_logger().info('Up and running')
    #----------------------------------------------------------------------------------
    def __interpret_grid(self, msg):
        if self.__map_data is not None:
            # If map already exists
            # Check for map size changes
            if msg.info.width != self.__map_info.width or msg.info.height != self.__map_info.height:
                
                #If size changed, shift overlays accordingly
                self.get_logger().info('Map size change detected, adjusting overlays')
                old_info = self.__map_info  # Store old info
                new_info = msg.info # Get new info
                self.__shift_location_overlay(old_info, new_info)   # Shift overlays
        else:
            # First time receiving map
            self.get_logger().info('Occupancy grid received')
            self.__items_grid = np.zeros((msg.info.height, msg.info.width))  # Initialize overlay grid
            self.__navigation_grid = np.zeros((msg.info.height, msg.info.width))  # Initialize navigation grid    

        # Store new map data
        self.__map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.__map_data[self.__map_data == -1] = 50  # Unknown cells to 50
        self.__map_info = msg.info
        self.__save_map_data()

    #----------------------------------------------------------------------------------
    def __shift_location_overlay(self,old_info,new_info):
        #Calculate shifts
        h_shift = new_info.height - old_info.height
        w_shift = new_info.width - old_info.width
        
        # Create new overlay and copy old data into shifted position
        new_overlay = np.zeros((new_info.height,new_info.width))
        new_overlay[h_shift:new_info.height, w_shift:new_info.width] = self.__items_grid
        
        # Update internal overlay
        self.__items_grid = new_overlay

    #----------------------------------------------------------------------------------
    def __interpret_locations(self, msg: RSyncLocationList):
        if self.__map_data is None:
            self.get_logger().info('No map data yet, cannot plot locations')
            return

        # Get robot world location and yaw
        my_pose = self.__fetch_robot_world_location()
        rx, ry, rz, rq = my_pose
        r_yaw = quaternion_to_yaw(rq) # In radians
        
        item_world_locations = []
        # Loop through each detected item and plot on overlay
        for item in msg.locations.location_list:
            item_dist = item.distance
            item_yaw = math.radians(item.relative_yaw)
            # Calculate x and y offset from robot
            y_offset = math.cos(item_yaw)/item_dist
            x_offset = math.sin(item_yaw)/item_dist
            # Calculate world coordinates of item
            item_x,item_y = transform_2d(
                rx, ry, r_yaw,
                x_offset, y_offset
            )
            item_world_locations.append((item.name,item_x,item_y))

        # Update overlay grid
        self.__add_to_location_overlay(item_world_locations)
        
        # Pulish updated overlay
        overlay_msg = self.__generate_overlay_message()
        self.__overlay_publisher.publish(overlay_msg)
    
    #----------------------------------------------------------------------------------
    def __add_to_location_overlay(self, item_world_locations):
        for item in item_world_locations:
            name, x, y = item

            #STUB:
            #DO I NEED TO CONVERT UNITS HERE? COME BACK TO THIS
            i,j = self.__convert_world_to_grid(x,y)
            
            if 0 <= i < self.__items_grid.shape[0] and 0 <= j < self.__items_grid.shape[1]:
                self.__items_grid[i,j] = 101  # Mark detected item on overlay

    #----------------------------------------------------------------------------------
    def __generate_overlay_message(self):
        """Build and return an OccupancyGrid message for the overlay map."""

        overlay_msg = OccupancyGrid()
        overlay_msg.header.stamp = self.get_clock().now().to_msg()
        overlay_msg.header.frame_id = 'overlay_map'
        overlay_msg.info = self.__map_info
        overlay_msg.data = self.__items_grid.flatten().astype(np.int8).tolist()
        return overlay_msg
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
        DEFAULT = self.__last_robot_pose

        if self.__map_info is None:
            return DEFAULT
        
        try:
            # Get transform map → base_link
            trans = self.__tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            q = trans.transform.rotation
            self.__last_robot_pose = [x,y,z,q]
            return [x,y,z,q]
        
        except Exception as e:
            self.__handle_error(e,'update_robot_marker()','No transform available yet')
            return DEFAULT
        
    #----------------------------------------------------------------------------------
    def __plot_map(self):
        if self.__map_data is None:
            return
        inverted_map = cv2.transpose(self.__map_data,1)
        cv2.imshow("Occupancy",inverted_map)
        cv2.waitKey(1)

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
    def __handle_error(self,e:Exception, function_location:str='',msg:str='',show_exception:bool=False):
        self.get_logger().warn(f'An error occured in {function_location}\n{msg}\n{e if show_exception else ""}')

#********************************************************************




def main(args=None):
    rclpy.init(args=args)
    map_reader = MapGenerator()
    rclpy.spin(map_reader)
    # detector.shutdown_clean()
    map_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        