#!/usr/bin/env python3

"""
Hardware Navigation Visualization Launch File
Opens RViz2 with comprehensive navigation visualization

Shows:
- LiDAR scans (original /scan in RED, filtered /scan_filtered in GREEN)
- SLAM-generated map
- Global path planning (green line)
- Local path planning (yellow line)
- Local costmap (obstacles and inflation)
- Global costmap (optional)
- Robot model and footprint
- Current goal pose
- TF frames
- Exploration markers

Usage:
  ros2 launch rover_exploration view_hardware_navigation.launch.py
  
Requires:
- Robot driver running (mini.launch.py)
- Navigation stack running (hardware_navigation_stack.launch.py)
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
    rviz_config_path = os.path.join(exploration_dir, 'config', 'hardware_nav_visualization.rviz')
    
    # Launch RViz with navigation visualization config
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

