# Use the ROS 2 Humble base image
FROM ros:humble-ros-base

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    colcon-common-extensions \
    ros-humble-behaviortree-cpp-v3 \
    && rm -rf /var/lib/apt/lists/*

# Create a colcon workspace and a source directory
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

# Copy the necessary packages into the workspace
# This assumes the Docker build context is the 'gitprj' directory
COPY ./bt_ros2 ./bt_ros2
COPY ./my_robot_interfaces ./my_robot_interfaces

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Set up the entrypoint to source the workspace automatically
COPY ./bt_ros2/docker/entrypoint.sh /ros2_ws/entrypoint.sh
RUN chmod +x /ros2_ws/entrypoint.sh
ENTRYPOINT ["/ros2_ws/entrypoint.sh"]

# The default command to run when the container starts
CMD ["ros2", "run", "bt_ros2", "bt_executor"]
