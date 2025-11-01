#!/usr/bin/env python3

"""
View LiDAR Hardware Launch File
Launches RViz to visualize LiDAR data from the physical robot

This assumes the robot driver is ALREADY RUNNING with:
  ros2 launch roverrobotics_driver mini.launch.py
  
Usage:
  ros2 launch rover_exploration view_lidar_hardware.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    exploration_dir = get_package_share_directory('rover_exploration')
    
    # Launch argument for use_sim_time
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (false for hardware)'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # RViz configuration file path
    rviz_config_path = os.path.join(exploration_dir, 'config', 'lidar_view.rviz')
    
    # Launch RViz with custom config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments and nodes
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(rviz_node)
    
    return ld

