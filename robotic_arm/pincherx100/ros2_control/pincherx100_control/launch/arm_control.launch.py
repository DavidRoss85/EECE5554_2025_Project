#!/usr/bin/env python3
"""
Launch file for PincherX100 arm control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description."""
    
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('pincherx100_control'),
            'config',
            'arm_config.yaml'
        ]),
        description='Path to arm configuration file'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Dynamixel servos'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='1000000',
        description='Baudrate for Dynamixel communication'
    )
    
    # Arm controller node
    arm_controller_node = Node(
        package='pincherx100_control',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
            }
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        port_arg,
        baudrate_arg,
        arm_controller_node,
    ])



