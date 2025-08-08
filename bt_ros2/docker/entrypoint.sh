#!/bin/bash
set -e

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Source the workspace setup file
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

# Execute the command passed to the container (e.g., ros2 run ...)
exec "$@"
