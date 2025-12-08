#!/bin/bash
echo "Launching Viewers..."
echo "Sourcing ROS2 files..."
source /opt/ros/jazzy/setup.bash
source ../../main_ws/install/setup.bash
echo "Running python script..."
ros2 run object_location temp_viewer