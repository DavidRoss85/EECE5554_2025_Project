#!/bin/bash
echo "Starting semantic map sequence..."
echo "Updating permissions..."
chmod +x ./launch_sync_node.sh ./launch_detection_node.sh ./launch_distance_node.sh ./launch_map_node.sh ./launch_temp_viewer.sh ./launch_approach_controller.sh
sleep 3
echo "Launching simulation separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_turtlebot_simulation.sh; exec bash"
echo "Waiting for simulation to start..."
sleep 15
echo "Launching Robosync node in separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_sync_node.sh; exec bash"
sleep 2
echo "Launching Detection Node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_detection_node.sh; exec bash"
sleep 2
echo "Launching Location node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_distance_node.sh; exec bash"
sleep 2
echo "Launching Mapping node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_map_node.sh; exec bash"
sleep 2
echo "Launching viewer..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_temp_viewer.sh; exec bash"
echo "Startup complete."
sleep 2
echo "Launching approach controller"
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_approach_controller.sh; exec bash"
