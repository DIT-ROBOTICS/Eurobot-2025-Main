#!/bin/bash -e

# Enter the ROS workspace
cd $ROS_WS

# Mark directories as safe for Git
git config --global --add safe.directory $ROS_WS/src/uros/micro-ROS-Agent
git config --global --add safe.directory $ROS_WS/src/uros/micro_ros_msgs

echo -e "\033[1;32m--------------- [ micro-ROS Build Start ] ----------------\033[0m"
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y

source ./install/local_setup.bash

echo -e "\033[1;32m--------------- [ micro-ROS Agent Creat ] ------------------\033[0m"
ros2 run micro_ros_setup create_agent_ws.sh
echo -e "\033[1;32m--------------- [ micro-ROS Build Agent ] ------------------\033[0m"
ros2 run micro_ros_setup build_agent.sh

colcon build --packages-select micro_ros_setup
colcon build --packages-select micro_ros_agent

echo -e "\033[1;32m--------------- [ micro-ROS Build Complete ] ---------------\033[0m"
