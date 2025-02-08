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
        DeclareLaunchArgument(
            'param_file',
            default_value=os.path.join(
                os.path.dirname(__file__), '../params/config_path.yaml'
            ),
            description='Path to the parameter file'
        ),
        Node(
            # parameters=[LaunchConfiguration('param_file')],
            parameters=[{"tree_name": "NavTest"}],
            package = 'bt_app_2025',
            executable = 'bt_m',
            name = 'bt_m'
        ),
        Node(
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
