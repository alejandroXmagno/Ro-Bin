#!/usr/bin/env python3
"""
Launch visual servoing person tracker with autonomous exploration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    # Get package directory
    pkg_dir = Path(get_package_share_directory('rover_exploration'))
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Visual servoing tracker - takes over when person detected
    visual_servoing = Node(
        package='rover_exploration',
        executable='visual_servoing_tracker',
        name='visual_servoing_tracker',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Autonomous explorer - handles exploration when no person detected
    # NOTE: Visual servoing has priority on /cmd_vel
    autonomous_explorer = Node(
        package='rover_exploration',
        executable='autonomous_explorer',
        name='autonomous_explorer',
        output='screen',
        parameters=[
            str(pkg_dir / 'config' / 'exploration_params.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        visual_servoing,
        autonomous_explorer
    ])
