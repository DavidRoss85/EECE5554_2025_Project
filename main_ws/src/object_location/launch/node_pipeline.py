from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd='export PYTHONPATH=~/Desktop/EECE5554/EECE5554_2025_Project/venv/lib/python3.12/site-packages:$PYTHONPATH'
        ),
        ExecuteProcess(
            cmd='ros2 launch slam_toolbox online_async_launch.py'
        ),
        Node(
            package='object_location',
            executable='robo_sync_node',
            name='robo_sync_node',
        ),
        Node(
            package='object_location',
            executable='detection_node',
            name='detection_node',
        ),
        Node(
            package='object_location',
            executable='distance_node',
            name='distance_node',
        ),
        Node(
            package='object_location',
            executable='temp_viewer',
            name='temp_viewer',
        ),
        Node(
            package='object_location',
            executable='map_node',
            name='map_node',
        ),
    ])