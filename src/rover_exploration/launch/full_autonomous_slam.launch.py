#!/usr/bin/env python3

"""
Full Autonomous SLAM and Exploration Launch File
Launches the mini rover with LIDAR SLAM, navigation, and autonomous exploration
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    rover_driver_dir = get_package_share_directory('roverrobotics_driver')
    exploration_dir = get_package_share_directory('rover_exploration')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav_params_file = LaunchConfiguration('nav_params_file')
    exploration_params_file = LaunchConfiguration('exploration_params_file')
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    declare_slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(rover_driver_dir, 'config/slam_configs', 
                                   'mapper_params_online_async.yaml'),
        description='Full path to SLAM parameters file'
    )
    
    declare_nav_params_arg = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(exploration_dir, 'config', 'nav2_params_exploration.yaml'),
        description='Full path to navigation parameters file'
    )
    
    declare_exploration_params_arg = DeclareLaunchArgument(
        'exploration_params_file',
        default_value=os.path.join(exploration_dir, 'config', 'exploration_params.yaml'),
        description='Full path to exploration parameters file'
    )
    
    # Launch the mini rover (robot driver, URDF, accessories including LIDAR)
    mini_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_driver_dir, 'launch', 'mini.launch.py')
        )
    )
    
    # Launch SLAM toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_driver_dir, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )
    
    # Launch Nav2 navigation stack
    nav_backend_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_driver_dir, 'launch', 'nav2_backend.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
            'autostart': 'true'
        }.items()
    )
    
    # Launch autonomous exploration node (delayed to ensure other nodes are ready)
    exploration_node = Node(
        package='rover_exploration',
        executable='autonomous_explorer',
        name='autonomous_explorer',
        output='screen',
        parameters=[
            exploration_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Delay exploration start by 10 seconds to ensure SLAM and Nav2 are ready
    delayed_exploration = TimerAction(
        period=10.0,
        actions=[exploration_node]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_slam_params_arg)
    ld.add_action(declare_nav_params_arg)
    ld.add_action(declare_exploration_params_arg)
    
    # Add launch files and nodes
    ld.add_action(mini_launch)
    ld.add_action(slam_launch)
    ld.add_action(nav_backend_launch)
    ld.add_action(delayed_exploration)
    
    return ld



