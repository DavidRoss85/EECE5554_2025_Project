#!/bin/bash
echo "Launching SLAM Toolbox and Object Location Pipeline Nodes..."
sleep 1
echo "Making launch scripts executable..."
chmod +x ./launch_sync_node.sh ./launch_detection_node.sh ./launch_distance_node.sh ./launch_map_node.sh ./launch_temp_viewer.sh
echo "Launching SLAM Toolbox..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch slam_toolbox online_async_launch.py"
sleep 10
echo "Launching Object Location Pipeline Nodes..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_sync_node.sh"
sleep 1
echo "Launching Detection Node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_detection_node.sh"
sleep 1
echo "Launching Distance Node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_distance_node.sh"
sleep 1
echo "Launching Map Node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_map_node.sh"
sleep 1
echo "Launching Temp Viewer..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_temp_viewer.sh"