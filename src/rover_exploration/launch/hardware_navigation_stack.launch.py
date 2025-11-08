#!/usr/bin/env python3

"""
Hardware Navigation Stack Launch File
Launches navigation components for the PHYSICAL ROBOT with FILTERED LiDAR

Prerequisites:
1. Robot driver must be running: ros2 launch roverrobotics_driver mini.launch.py
2. Filtered LiDAR must be running: ./run_filtered_lidar.sh <angles_file>

Usage:
  ros2 launch rover_exploration hardware_navigation_stack.launch.py

This will:
- Use /scan_filtered for obstacle detection
- Start SLAM for mapping
- Launch Nav2 for navigation
- Optionally start autonomous exploration
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
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
    enable_exploration = LaunchConfiguration('enable_exploration')
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (false for physical robot)'
    )
    
    declare_slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(rover_driver_dir, 'config/slam_configs', 
                                   'mapper_params_online_async_filtered.yaml'),
        description='Full path to SLAM parameters file (uses /scan_filtered)'
    )
    
    declare_nav_params_arg = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(exploration_dir, 'config', 'nav2_params_hardware_filtered.yaml'),
        description='Full path to navigation parameters file (uses /scan_filtered)'
    )
    
    declare_exploration_params_arg = DeclareLaunchArgument(
        'exploration_params_file',
        default_value=os.path.join(exploration_dir, 'config', 'exploration_params.yaml'),
        description='Full path to exploration parameters file'
    )
    
    declare_enable_exploration_arg = DeclareLaunchArgument(
        'enable_exploration',
        default_value='false',
        description='Enable autonomous exploration (set to true for autonomous mode)'
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
    # Note: Nav2 controller_server publishes to /cmd_vel by default
    # Delay Nav2 startup to allow SLAM to initialize and publish map frame first
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
    
    # Delay Nav2 launch by 5 seconds to allow SLAM to initialize map frame
    delayed_nav_backend = TimerAction(
        period=5.0,
        actions=[nav_backend_launch]
    )
    
    # Velocity smoother node - smooths navigation commands to prevent wheel jitter
    # Subscribes to /cmd_vel_nav (from relay) and /cmd_vel_explorer (from autonomous explorer)
    # Publishes smoothed commands to /cmd_vel
    # Uses same acceleration limiting logic as joystick control
    velocity_smoother_node = Node(
        package='roverrobotics_input_manager',
        executable='velocity_smoother.py',
        name='velocity_smoother',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel_nav',  # From relay (original Nav2 output)
            'exploration_input_topic': '/cmd_vel_explorer',  # From autonomous explorer (takes priority)
            'output_topic': '/cmd_vel',     # To robot driver
            'odom_topic': '/odom_raw',
            'max_linear_acceleration': 5.0,  # m/s^2 (matching robot driver)
            'max_angular_acceleration': 30.0,  # rad/s^2
            'control_frequency': 30.0,  # Hz (matching robot driver)
            'speed_scale_factor': 0.25  # Quarter speed
        }],
        respawn=True,
        respawn_delay=1
    )
    
    # Topic relay to forward Nav2's cmd_vel to a different topic for velocity smoother
    # This prevents feedback loop where smoother subscribes to its own output
    # Nav2 controller_server -> /cmd_vel -> relay -> /cmd_vel_nav -> smoother -> /cmd_vel
    cmd_vel_relay_node = Node(
        package='roverrobotics_input_manager',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_nav'
        }],
        output='screen',
        respawn=True,
        respawn_delay=1
    )
    
    # Launch autonomous exploration node (optional, delayed)
    exploration_node = Node(
        package='rover_exploration',
        executable='autonomous_explorer',
        name='autonomous_explorer',
        output='screen',
        parameters=[
            exploration_params_file,
            {'use_sim_time': use_sim_time}
        ],
        condition=conditions.IfCondition(enable_exploration)
    )
    
    # Delay exploration start by 15 seconds to ensure SLAM and Nav2 are ready
    delayed_exploration = TimerAction(
        period=15.0,
        actions=[exploration_node]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_slam_params_arg)
    ld.add_action(declare_nav_params_arg)
    ld.add_action(declare_exploration_params_arg)
    ld.add_action(declare_enable_exploration_arg)
    
    # Add navigation components
    ld.add_action(slam_launch)
    ld.add_action(delayed_nav_backend)  # Delayed to allow SLAM to initialize
    ld.add_action(cmd_vel_relay_node)
    ld.add_action(velocity_smoother_node)
    ld.add_action(delayed_exploration)
    
    return ld

