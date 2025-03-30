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

    config_path = LaunchConfiguration('params0')
    finisher = LaunchConfiguration('params1')
    nav_parameters = LaunchConfiguration('params2')

    bt_m_node = Node(
        parameters=[
            config_path, 
            finisher, 
            nav_parameters,
            {"frame_id": "base_link"},
            {"tree_name": "MainTree"}, 
            {"Robot_name": "SCX"}  # Invisible or Tongue
        ],
        package = 'bt_app_2025',
        executable = 'bt_m',
        name = 'bt_m'
    )
    startup_node = Node(
        parameters=[
            {"start_point": 3} # 0 to 5
        ],
        package = 'startup',
        executable = 'startup',
        name = 'startup'
    )
    firmware_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '-b', '115200', '-D', '/dev/mission']
    )
    return LaunchDescription([
        config_path_arg,
        finisher_arg,
        nav_parameters_arg,
        bt_m_node,
        startup_node,
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
