from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project_localization',
            executable='save_spots',
            output='screen'),
    ])