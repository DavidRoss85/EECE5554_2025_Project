echo "Launching Detection node"
source /opt/ros/jazzy/setup.bash
source ../../main_ws/install/setup.bash
current_dir_name="$PWD"

# export PYTHONPATH=</path/to/your/virtual/environment>/lib/python3.12/site-packages:$PYTHONPATH
export PYTHONPATH="${current_dir_name}/../venv/lib/python3.12/site-packages:$PYTHONPATH"


ros2 run object_location detection_node