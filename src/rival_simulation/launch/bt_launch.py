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

def generate_launch_description():
    return LaunchDescription([
        # params = os.path.join(
        #     get_package_share_directory('rival_simulation'),
        #     'params',
        #     'config_path.yaml'
        # ),
        Node(
            # parameters=[LaunchConfiguration('param_file')],
            parameters=[{"tree_name": "MainTree"}],
            package = 'rival_simulation',
            executable = 'rival_main',
            name = 'rival_main',
            parameters = ["../params/config_path.yaml"],
            # parameters = [params]
        )
    ])
