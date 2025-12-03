echo "Launching Detection node"
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash
current_dir_name="$PWD"
# export PYTHONPATH=</path/to/your/virtual/environment>/lib/python3.12/site-packages:$PYTHONPATH
if [[ "$current_dir_name" == "/home/david-ross/gitRepos/EECE5554_2025_Project/main_ws/scripts" ]]; then
    export PYTHONPATH=/home/david-ross/gitRepos/EECE5554_2025_Project/venv/lib/python3.12/site-packages:$PYTHONPATH
else
    export PYTHONPATH=~/Desktop/EECE5554/EECE5554_2025_Project/venv/lib/python3.12/site-packages:$PYTHONPATH
    # export PYTHONPATH=~/Desktop/cs5335/venv/lib/python3.12/site-packages:$PYTHONPATH
fi
ros2 run object_location detection_node