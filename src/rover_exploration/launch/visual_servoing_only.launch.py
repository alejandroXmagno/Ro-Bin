#!/usr/bin/env python3
"""
Launch ONLY visual servoing person tracker
Simple behavior: rotate to find people, approach when found
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Visual servoing tracker - handles everything
    visual_servoing = Node(
        package='rover_exploration',
        executable='visual_servoing_tracker',
        name='visual_servoing_tracker',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        visual_servoing
    ])
