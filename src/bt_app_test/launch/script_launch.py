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
    params = os.path.join(
        get_package_share_directory('bt_app_test'),
        'params',
        'script.yaml'
    )
    return LaunchDescription([
        Node(
            package = 'bt_app_test',
            executable = 'main_script',
            name = 'main_script',
            parameters = [params]
        )
    ])
