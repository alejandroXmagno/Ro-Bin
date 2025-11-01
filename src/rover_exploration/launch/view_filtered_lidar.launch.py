#!/usr/bin/env python3

"""
View Filtered LiDAR Launch File
Launches RViz to visualize both original and filtered LiDAR data side-by-side

This assumes:
1. The robot driver is running (publishing /scan)
2. The filtered_lidar_scanner is running (publishing /scan_filtered)
  
Usage:
  ros2 launch rover_exploration view_filtered_lidar.launch.py
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
    rviz_config_path = os.path.join(exploration_dir, 'config', 'filtered_lidar_view.rviz')
    
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

