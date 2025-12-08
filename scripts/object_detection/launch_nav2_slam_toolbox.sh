#!/bin/bash

source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot4_navigation slam.launch.py sync:=false sim_time:=false