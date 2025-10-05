from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_action_server',
            executable='action_server',
            output='screen'),
        Node(
            package='my_action_server',
            executable='waveshare_controller',
            output='screen'),
    ])