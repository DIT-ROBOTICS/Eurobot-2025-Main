#!/bin/bash

# Set the ROS workspace
if [ -z "$ROS_WS" ]; then
    echo "ROS_WS is not set. Setting to default"
    ROS_WS=/home/ros/Eurobot-2025-Main
fi
cd $ROS_WS

# Install micro-ROS
if [ ! -d "$ROS_WS/src/micro_ros" ]; then
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros
fi


# . /opt/ros/humble/setup.sh && colcon build --symlink-install

source $ROS_WS/install/setup.bash
# ros2 run robot_status_monitor robot_status_node

while true; do
    sleep 60
done

exec "$@"