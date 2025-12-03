#!/bin/bash
echo "Launching Front End"
echo "Updating permissions..."
chmod +x ./launch_dashboard.sh ./launch_rosbridge.sh ./launch_state_machine.sh
sleep 3
echo "Launching Rosbridge in separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_rosbridge.sh; exec bash"
sleep 2
echo "Launching State Machine in separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_state_machine.sh; exec bash"
sleep 2
echo "Starting Dashboard..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_dashboard.sh; exec bash"
sleep 2
echo "Front End launch complete."