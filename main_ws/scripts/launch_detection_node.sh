echo "Launching Detection node"
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash
# export PYTHONPATH=</path/to/your/virtual/environment>/lib/python3.12/site-packages:$PYTHONPATH
export PYTHONPATH=~/Desktop/EECE5554/EECE5554_2025_Project/venv/lib/python3.12/site-packages:$PYTHONPATH
# export PYTHONPATH=/home/david-ross/gitRepos/EECE5554_2025_Project/venv/lib/python3.12/site-packages:$PYTHONPATH
ros2 run object_location detection_node