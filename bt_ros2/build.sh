#!/bin/bash
set -e

# --- Configuration ---
# PLEASE EDIT THIS PATH if your ROS 2 installation is in a different location.
ROS2_DISTRO="humble"
ROS2_SETUP_PATH="/opt/ros/${ROS2_DISTRO}/setup.bash"

# The root of your colcon workspace (where the 'src' directory is)
# This script assumes the workspace root is the 'gitprj' directory.
COLCON_WS_ROOT="/Users/kdi/workspace/gitprj"

# --- Script ---
if [ ! -f "$ROS2_SETUP_PATH" ]; then
    echo "Error: ROS 2 setup file not found at $ROS2_SETUP_PATH"
    echo "Please update the ROS2_SETUP_PATH variable in this script."
    exit 1
fi

echo "Navigating to workspace root: $COLCON_WS_ROOT"
cd "$COLCON_WS_ROOT"

echo "Sourcing ROS 2 environment: ${ROS2_SETUP_PATH}"
source "${ROS2_SETUP_PATH}"

echo "Building dependencies (my_robot_interfaces)..."
colcon build --packages-up-to my_robot_interfaces

echo "Building bt_ros2 package..."
colcon build --packages-select bt_ros2

echo "Build finished successfully."
echo "To run the node, navigate to the workspace, source it, and run:"
echo "cd \"${COLCON_WS_ROOT}\""
echo "source \"install/setup.bash\""
echo "ros2 run bt_ros2 bt_executor"
