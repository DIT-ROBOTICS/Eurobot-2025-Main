# ========== ROS ==========
# Source ROS environment
source /opt/ros/humble/setup.bash
# Source workspace environment
# Note: If you have not built your workspace yet, the following command will fail
source /home/user/Eurobot-2025-Main-ws/install/setup.bash

# ========== Groot ==========
# Check if the AppImage is exist
if [ -f $GROOT_APPIMAGE ]; then
    echo "Groot AppImage found: $GROOT_APPIMAGE"
else
    echo "Groot AppImage not found: $GROOT_APPIMAGE"
fi
# Add alias for groot
alias groot='$GROOT_APPIMAGE'