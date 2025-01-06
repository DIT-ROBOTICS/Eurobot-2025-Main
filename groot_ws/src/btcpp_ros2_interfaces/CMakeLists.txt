cmake_minimum_required(VERSION 3.16)
project(btcpp_ros2_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(btcpp_ros2_interfaces
    "msg/NodeStatus.msg"
    "msg/Localization.msg"
    "action/ExecuteTree.action"
    "action/Sleep.action"
    "action/Navigation.action"
    "action/NavMission.action"
    DEPENDENCIES geometry_msgs action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
