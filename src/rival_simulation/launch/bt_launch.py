import os
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
# for including other launch file
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
# for DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('rival_simulation'),
        'params',
        'config_path.yaml'
    )
    map_points = os.path.join(
        get_package_share_directory('bt_app_2025'),
        'params',
        'map_points.yaml'
    )
    return LaunchDescription([
        Node(
            package = 'rival_simulation',
            executable = 'rival_main',
            name = 'rival_main',
            parameters = [
                config_path, 
                map_points, 
                {"frame_id": "base_footprint"}, 
                {"tree_name": "EasyMainTree"}
            ]
        )
    ])
