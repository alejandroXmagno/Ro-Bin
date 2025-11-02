#!/usr/bin/env python3
"""
Launch file for autonomous exploration with RealSense D435i depth camera
Combines SLAM, Nav2, Autonomous Explorer, and RealSense depth sensing
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    # Get package directories
    rover_exploration_dir = get_package_share_directory('rover_exploration')
    roverrobotics_driver_dir = get_package_share_directory('roverrobotics_driver')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration files
    exploration_config = str(Path(rover_exploration_dir, 'config/exploration_params.yaml'))
    realsense_config = str(Path(rover_exploration_dir, 'config/realsense_navigation.yaml'))
    nav2_config = str(Path(rover_exploration_dir, 'config/nav2_params_exploration.yaml'))
    slam_config = str(Path(roverrobotics_driver_dir, 'config/slam_configs/mapper_params_online_async.yaml'))
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) time'
    )
    
    # RealSense Camera Node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense',
        namespace='',
        parameters=[realsense_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Depth Image to LaserScan Converter
    # Converts depth camera data to 2D laser scan format for Nav2
    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/camera/scan')
        ],
        parameters=[realsense_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # Nav2 Bringup (without SLAM since we're using SLAM Toolbox)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(nav2_bringup_dir, 'launch/navigation_launch.py'))
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_config,
            'autostart': 'true'
        }.items()
    )
    
    # Autonomous Explorer Node
    autonomous_explorer = Node(
        package='rover_exploration',
        executable='autonomous_explorer',
        name='autonomous_explorer',
        parameters=[exploration_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    
    # Add nodes
    ld.add_action(realsense_node)
    ld.add_action(depthimage_to_laserscan)
    ld.add_action(slam_toolbox_node)
    ld.add_action(nav2_bringup)
    ld.add_action(autonomous_explorer)
    
    return ld

