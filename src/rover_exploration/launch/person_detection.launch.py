#!/usr/bin/env python3
"""
Launch file for BlazePose person detection
Runs pose detection on RealSense camera feed at 3Hz
"""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    rover_exploration_dir = get_package_share_directory('rover_exploration')
    
    # Configuration file
    blazepose_config = str(Path(rover_exploration_dir, 'config/blazepose_params.yaml'))
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    
    # BlazePose Detector Node
    blazepose_node = Node(
        package='rover_exploration',
        executable='blazepose_detector',
        name='blazepose_detector',
        output='screen',
        parameters=[blazepose_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        emulate_tty=True
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    
    # Add nodes
    ld.add_action(blazepose_node)
    
    return ld

