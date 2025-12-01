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

from .helpers import (
    find_a_star_path, quaternion_to_yaw, 
    inflate_obstacles, inflate_obstacles2, inflate_gaussian_multiclass,
    transform_2d
)
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

    DEFAULT_OBSTACLE_MARKER_VALUE = 101  # Value to mark detected objects on overlay grid
    DEFAULT_ROBOT_MARKER_VALUE = 99  # Value to mark robot path on navigation grid
    DEFAULT_INFLATION_METERS = 0.01  # Meters to inflate detected objects on overlay
    DEFAULT_ROBOT_POSE_UPDATE_INCREMENT = 0.5

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
        self.__path_map = None
        self.__occupancy_number_start = self.DEFAULT_OBSTACLE_MARKER_VALUE
        self.__robot_marker_number = self.DEFAULT_ROBOT_MARKER_VALUE
        self.__object_inflation_factor = self.DEFAULT_INFLATION_METERS  # Additive for how much to inflate detected objects on overlay (meters)

        self.__items_grid = np.zeros((1,1)) # Placeholder until map received
        self.__navigation_grid = np.zeros((1,1)) # Placeholder until map received
        self.__last_robot_pose = self.DEFAULT_ROBOT_POSE_QUATERNION
        self.__robot_width = self.DEFAULT_ROBOT_WIDTH_METERS
        
        self.__robot_pose_update_increment = self.DEFAULT_ROBOT_POSE_UPDATE_INCREMENT

        self.__pose_timer = self.create_timer(
            self.__robot_pose_update_increment,
            self.__plot_robot_path
        )

        self.__locations_subscriber = self.create_subscription(
            RSyncLocationList,
            self.__locations_sub_topic,
            self.__interpret_locations,
            self.__max_msg
        )
        
        self.__grid_subscriber = self.create_subscription(
            OccupancyGrid,
            self.__grid_sub_topic,
            self.__interpret_occupancy_grid,
            self.__max_msg
        )

        # Publishers
        self.__overlay_publisher = self.create_publisher(
            OccupancyGrid,
            self.__overlay_map_topic,
            self.__max_msg
        )
        self.__occupancy_publisher = self.create_publisher(
            OccupancyGrid,
            self.__occupancy_map_topic,
            self.__max_msg
        )
        self.__navigation_publisher = self.create_publisher(
            OccupancyGrid,
            self.__navigation_map_topic,
            self.__max_msg
        )

        # TF2 Buffer and Listener for robot pose
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)



        #Test area:
        # cv2.namedWindow("Occupancy",cv2.WINDOW_NORMAL)

        self.get_logger().info('Up and running')
    #----------------------------------------------------------------------------------
    def __interpret_occupancy_grid(self, msg):
        if self.__map_data is None:
            # First time receiving map
            self.get_logger().info('Occupancy grid received')

            # Initalize Overlay and Navigation Arrays to Match Occupancy Grid Array Size:
            self.__items_grid = np.zeros((msg.info.height, msg.info.width))  # Initialize overlay grid
            self.__navigation_grid = np.zeros((msg.info.height, msg.info.width))  # Initialize navigation grid    

        else:
            # If map already exists, check for map size changes
            if msg.info.width != self.__map_info.width or msg.info.height != self.__map_info.height:
                
                #Size changed, shift overlay accordingly
                self.get_logger().info('Map size change detected, adjusting overlays')
                old_info = self.__map_info  # Store old info
                new_info = msg.info # Get new info

                self.__shift_location_overlay(old_info, new_info)   # Shift overlays
                self.__overlay_publisher.publish(self.__generate_overlay_message())

        # Store new map data
        self.__map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.__map_data[self.__map_data == -1] = 50  # Unknown cells to 50
        self.__map_info = msg.info

        new_msg = OccupancyGrid()
        new_msg.header = msg.header
        new_msg.info = msg.info
        new_msg.data = self.__map_data.flatten().astype(np.int8).tolist()
        self.__occupancy_publisher.publish(msg)  # Republish occupancy grid as-is
    #----------------------------------------------------------------------------------

    #----------------------------------------------------------------------------------
    def __shift_location_overlay(self, old_info:OccupancyGrid.info, new_info:OccupancyGrid.info):
        """
        If the occupancy grid size changes, so does the position of values in the array. 
        The origin is kept track of in the map_info. This is done automatically by the
        slam toolbox, but not our generated data. We take the old origin and new 
        origin and compare the two to shift the values in the overlay accordingly.
        """

        # Map resolution
        map_resolution = new_info.resolution

        # Old and new origins
        old_origin = old_info.origin.position
        new_origin = new_info.origin.position

        # Calculate shift of coordinates in meters
        x_shift_m = old_origin.x - new_origin.x
        y_shift_m = old_origin.y - new_origin.y

        # Convert to cell shift (integer grid cells)
        x_shift_c = int(round(x_shift_m / map_resolution))
        y_shift_c = int(round(y_shift_m / map_resolution))

        # Old overlay grid
        old_overlay = self.__items_grid
        old_h, old_w = old_overlay.shape

        # New overlay grid to fill
        new_overlay = np.zeros((new_info.height, new_info.width))
        new_h, new_w = new_overlay.shape

        # Determine where to place the old data inside the new grid
        new_x_start = max(x_shift_c, 0)
        new_y_start = max(y_shift_c, 0)

        old_x_start = max(-x_shift_c, 0)
        old_y_start = max(-y_shift_c, 0)

        # Determine overlapping region sizes
        copy_w = min(old_w - old_x_start, new_w - new_x_start)
        copy_h = min(old_h - old_y_start, new_h - new_y_start)

        # Only copy if there is a valid overlap
        if copy_w > 0 and copy_h > 0:
            new_overlay[new_y_start:new_y_start + copy_h,
                        new_x_start:new_x_start + copy_w] = \
            old_overlay[old_y_start:old_y_start + copy_h,
                        old_x_start:old_x_start + copy_w]

        # Update internal overlay
        self.__items_grid = new_overlay

    #----------------------------------------------------------------------------------
    def __interpret_locations(self, msg: RSyncLocationList):
        if self.__map_data is None:
            self.get_logger().info('No map data yet, cannot plot locations')
            return

        # Get World coordinates of robot - x,y and yaw
        my_pose = msg.robo_sync.robot_pose
        rx, ry = my_pose.transform.translation.x, my_pose.transform.translation.y
        rq = my_pose.transform.rotation
        r_yaw = quaternion_to_yaw(rq) # In radians
        
        # self.__plot_robot_path(my_pose)

        item_world_locations = []
        # Loop through each detected item and plot on overlay
        for item in msg.locations.location_list:
            item_dist = item.distance   # in meters
            item_yaw = math.radians(item.relative_yaw)
            # print(f"Item {item.name}: distance={item.distance:.2f}m, relative_yaw={item_yaw:.2f}deg")
            # print(f"Robot Pose: x={rx:.2f}m, y={ry:.2f}m, yaw={math.degrees(r_yaw):.2f}deg")

            # # Calculate x and y offset from robot
            # x_offset = math.cos(item_yaw)*item_dist #Meters
            # y_offset = math.sin(item_yaw)*item_dist #Meters

            # # Translate robot coordinates to World coordinates in meters
            # item_x,item_y = transform_2d(
            #     rx, ry, r_yaw,
            #     x_offset, y_offset
            # )

            # Alternative calculation (Remember to Come back to this):
            actual_yaw = r_yaw - item_yaw
            item_x = rx + math.cos(actual_yaw)*item_dist
            item_y = ry + math.sin(actual_yaw)*item_dist
            
            item_world_locations.append((item.name, item.index,item_x,item_y))

        # Update overlay grid (Grid conversion is done inside function)
        self.__add_to_location_overlay(item_world_locations)
        
        # Pulish updated overlay
        overlay_msg = self.__generate_overlay_message()
        self.__overlay_publisher.publish(overlay_msg)
    
    #----------------------------------------------------------------------------------
    def __add_to_location_overlay(self, item_world_locations):
        """
        Item locations are given in meters. So must be converted to array coords before appending
        """
        for item in item_world_locations:
            name, index, x, y = item
            # Converts the x,y world coordinates in meters to array cell coordinates
            i,j = self.__convert_world_to_grid(x,y)
            print(f'Plotting item {name} at grid cell (row={i}, col={j})')
            
            # Verify that new coordinate is not out of bounds:
            height, width = self.__items_grid.shape
            if 0 <= i < height and 0 <= j < width:
                self.__items_grid[i,j] = self.__occupancy_number_start + index # Mark detected item on overlay

    #----------------------------------------------------------------------------------
    
    def __plot_robot_path(self, pose=None, map=None):
        if self.__map_info is None:
            return
        
        if map is None:
            map = np.copy(self.__map_data if self.__map_data is not None else np.zeros((1,1)))
        
        rx,ry,rq = 0,0,0
        if pose is not None:
            rx, ry = pose.transform.translation.x, pose.transform.translation.y
            rq = pose.transform.rotation
        else:
            rx, ry, _, rq = self.__fetch_robot_world_location()

        r_yaw = quaternion_to_yaw(rq) # In radians

        i,j = self.__convert_world_to_grid(rx,ry)
        map[i,j] = self.__robot_marker_number  # Mark robot path on map

        #Make a circular vicinity around robot
        for theta in range(1,360,1):
            for r in range (1,5):
                di = round(r*math.sin(theta))
                dj = round(r*math.cos(theta))
                map[i+di,j+dj]= self.__robot_marker_number -1  # Mark robot vicinity on map

        #Make a line in front of robot
        for ilen in range(1,10):
            di = round(ilen * math.sin(r_yaw))
            dj = round(ilen * math.cos(r_yaw))
            map[i+di,j+dj] = self.__robot_marker_number # Mark robot forward path on map

            
        self.__path_map = map
        new_msg = OccupancyGrid()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = 'path'
        new_msg.info = self.__map_info
        new_msg.data = map.flatten().astype(np.int8).tolist()
        self.__navigation_publisher.publish(new_msg)

    #----------------------------------------------------------------------------------
    def __fuse_navigation_overlay(self):
            pass
    #----------------------------------------------------------------------------------
    def __generate_navigation_message(self):
        """Build and return an OccupancyGrid message for the navigation map."""

        self.__fuse_navigation_overlay()

        navigation_msg = OccupancyGrid()
        navigation_msg.header.stamp = self.get_clock().now().to_msg()
        navigation_msg.header.frame_id = 'map'
        navigation_msg.info = self.__map_info
        navigation_msg.data = self.__navigation_grid.flatten().astype(np.int8).tolist()
        return navigation_msg
    #----------------------------------------------------------------------------------
    def __generate_overlay_message(self):
        """Build and return an OccupancyGrid message for the overlay map."""

        overlay_msg = OccupancyGrid()
        # Set time stamp
        overlay_msg.header.stamp = self.get_clock().now().to_msg()
        overlay_msg.header.frame_id = 'overlay_map'


        # Inflate points to be bigger on map for visualization
        items_grid = inflate_obstacles2(
            self.__items_grid, 
            inflation_factor = self.__object_inflation_factor, 
            resolution = self.__map_info.resolution, 
            threshold = self.__occupancy_number_start
        )

        # Get dimensions
        overlay_msg.info = self.__map_info
        height, width = items_grid.shape[:2]
        overlay_msg.info.height = height
        overlay_msg.info.width = width
        
        # Flatten array for messaging
        overlay_msg.data = items_grid.flatten().astype(np.int8).tolist()
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

        i = round((y - origin_y) / resolution)  # row
        j = round((x - origin_x) / resolution)  # col

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
        