#!/bin/bash

source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py nav2:=false slam:=true localization:=false rviz:=false