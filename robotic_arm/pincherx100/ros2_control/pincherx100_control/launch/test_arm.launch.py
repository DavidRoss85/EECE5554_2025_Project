#!/usr/bin/env python3
"""
Launch file for testing PincherX100 arm.
Launches both the controller and test node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description."""
    
    # Include arm control launch
    arm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pincherx100_control'),
                'launch',
                'arm_control.launch.py'
            ])
        ])
    )
    
    # Test node
    test_joints_node = Node(
        package='pincherx100_control',
        executable='test_joints',
        name='joint_tester',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        arm_control_launch,
        test_joints_node,
    ])



