import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    spots_yaml_file = os.path.join(get_package_share_directory('project_path_planning'), 'config', 'spots_data.yaml')
    
    spot_name = LaunchConfiguration('spot_name')
    spot_name_arg = DeclareLaunchArgument(
        'spot_name',
        default_value='corner2'
    )
    return LaunchDescription([
        spot_name_arg,
        Node(
            package = 'project_path_planning',
            name = 'Nav_To_Pose_Action_Client',
            executable = 'navigate_to_pose_client',
            parameters = [
                        {'spot_name': spot_name},
                        spots_yaml_file]),
    ])