#!/bin/bash
echo "Launching ROS2 Bridge"
source /opt/ros/jazzy/setup.bash
source ../../main_ws/install/setup.bash
ros2 run rosbridge_server rosbridge_websocket