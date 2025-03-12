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
        get_package_share_directory('bt_app_2025'),
        'params',
        'config_path.yaml'
    )
    mission_set = os.path.join(
        get_package_share_directory('bt_app_2025'),
        'params',
        'mission_set.yaml'
    )
    return LaunchDescription([
        Node(
            parameters=[
                config_path, 
                mission_set, 
                {"tree_name": "MainTree"}, 
                {"Robot_name": "Tongue"}  # Invisible or Tongue
            ],
            remappings=[
                ("/map", "/map"),
                ("/base_link", "/final_pose")
            ],
            package = 'bt_app_2025',
            executable = 'bt_m',
            name = 'bt_m'
        ),
        Node(
            parameters=[
                {"start_point": "0"} # 0 to 5
            ],
            package = 'startup',
            executable = 'startup',
            name = 'startup'
        )
        # TimerAction(
        #     period = 1.0,
        #     actions = [Node(
        #         package = 'bt_app_test',
        #         executable = 'bt_ros2',
        #         name = 'bt_ros2',
        #     )],
        # )
    ])
