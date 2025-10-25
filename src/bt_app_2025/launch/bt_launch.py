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


    pkg_dir = os.path.join('/home/ros/Eurobot-2025-Main/src/bt_app_2025')

    config_path_arg = DeclareLaunchArgument(
        'params0',
        default_value=os.path.join(pkg_dir, 'params', 'config_path.yaml'),
        description='Full path to parameter YAML file'
    )
    finisher_arg = DeclareLaunchArgument(
        'params1',
        default_value=os.path.join(pkg_dir, 'params', 'finisher.yaml'),
        description='Full path to parameter YAML file'
    )
    nav_parameters_arg = DeclareLaunchArgument(
        'params2',
        default_value=os.path.join(pkg_dir, 'params', 'nav_parameters.yaml'),
        description='Full path to parameter YAML file'
    )
    map_points_arg = DeclareLaunchArgument(
        'params3',
        default_value = os.path.join(pkg_dir, 'params', 'map_points.yaml'),
        description = 'map points'
    )

    config_path = LaunchConfiguration('params0')
    finisher = LaunchConfiguration('params1')
    nav_parameters = LaunchConfiguration('params2')
    map_points = LaunchConfiguration('params3')

    bt_m_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                parameters=[
                    config_path,
                    finisher,
                    nav_parameters,
                    map_points,
                    {"frame_id": "base_footprint"},
                    {"tree_name": "MainTree"},
                    # {"plan_script": [1, 2, 17]}
                ],
                package = 'bt_app_2025',
                executable = 'bt_m',
                name = 'bt_m'
            )
        ]
    )
    startup_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                parameters=[
                    config_path,
                    map_points,
                    {"Robot_name": "Panda"},  # Raccoon or Panda
                    {"plan_code": 1}, # 10e1: plan (start from 1), 10e0: color(0 for yellow, 1 for blue)
                ],
                package = 'startup',
                executable = 'startup',
                name = 'startup'
            )
        ]
    )
    firmware_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '-b', '115200', '-D', '/dev/mission']
    )
    return LaunchDescription([
        bt_m_node,
        startup_node,
        config_path_arg,
        finisher_arg,
        nav_parameters_arg,
        map_points_arg,
        firmware_node
        # TimerAction(
        #     period = 1.0,
        #     actions = [Node(
        #         package = 'bt_app_test',
        #         executable = 'bt_ros2',
        #         name = 'bt_ros2',
        #     )],
        # )
    ])
