#!/bin/bash -e

USER_ID=${LOCAL_USER_ID:-1000}
GROUP_ID=${LOCAL_GROUP_ID:-1000}
USERNAME=ros

# Update ros user's UID and GID to match external user
if [ "$(id -u $USERNAME)" != "$USER_ID" ] || [ "$(id -g $USERNAME)" != "$GROUP_ID" ]; then
    echo "Updating $USERNAME UID and GID to match external user..."
    usermod -u $USER_ID $USERNAME
    groupmod -g $GROUP_ID $USERNAME
    chown -R $USERNAME:$USERNAME /home/$USERNAME
fi

# Enter the ROS workspace
cd $ROS_WS

# Change user to ros
if [ "$(id -u)" != "$USER_ID" ]; then
    echo "Switching to user $USERNAME..."
    exec gosu $USERNAME "$0" "$@"
fi

# Install micro-ROS
if [ ! -d "$ROS_WS/src/micro_ros" ]; then
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros
fi

# Source ROS Environment
if [ -f "$ROS_WS/install/setup.bash" ]; then
    echo "Sourcing ROS workspace setup..."
    source "$ROS_WS/install/setup.bash"
fi

# Auto build workspace
if [ "${AUTO_BUILD}" = "true" ]; then
    echo "Running colcon build..."
    source /opt/ros/humble/setup.bash
    cd "$ROS_WS"
    
    if [ ! -d "$ROS_WS/install/micro_ros_msgs" ]; then
        echo "Building workspace excluding micro_ros_msgs..."
        colcon build --symlink-install --packages-skip micro_ros_msgs
    else
        echo "micro-ROS already built. Skipping build step for micro_ros_msgs."
    fi

    if [ ! -d "$ROS_WS/src/uros" ]; then
        echo "Building all packages..."
        colcon build --symlink-install
        echo "Installing micro-ROS agent..."
        ./scripts/build_micro_ros.sh
    fi
fi

# Auto run program
if [ "${AUTO_RUN}" = "true" ]; then
    echo "Launching main program..."
    source "$ROS_WS/install/setup.bash"
    ros2 launch bt_app_2025 bt_launch.py
    # exit 0
fi

# If no command is provided, start an idle loop to keep the container alive
if [ $# -eq 0 ]; then
    echo "No command provided. Starting idle loop to keep container alive..."
    while true; do sleep 60; done
else
    echo "Executing command as $USERNAME: $@"
    # exec gosu $USERNAME "$@"
    exec "$@"
fi
