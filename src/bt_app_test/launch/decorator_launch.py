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
    pkg_dir = os.path.join('/home/ros/Eurobot-2025-Main/src/bt_app_test')
    config_path_arg = DeclareLaunchArgument(
        'params',
        default_value=os.path.join(pkg_dir, 'params', 'config_path.yaml'),
        description='Full path to parameter YAML file'
    )
    config_path = LaunchConfiguration('params')
    bt = Node(
        package = 'bt_app_test',
        executable = 'bt_ros2',
        name = 'bt_ros2',
        parameters = [config_path, {"tree_name": "Decorator-Simple-Demo"}]
    )
    return LaunchDescription([
        config_path_arg,
        bt
    ])
