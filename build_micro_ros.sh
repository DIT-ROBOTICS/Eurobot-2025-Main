#!/bin/bash

# Exit immediately if a command fails

chmod +x build_micro_ros.sh

set -e

rosdep update
# Source the ROS 2 workspace setup file
source /home/user/Eurobot-2025-Main-ws/install/local_setup.bash

# Run micro-ROS setup scripts
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

# Build the workspace using colcon
colcon build --packages-select micro_ros_setup
colcon build --packages-select micro_ros_agent

echo "Build process completed successfully!"
