# Dockerfile for ROS2 Humble on Raspberry Pi with PincherX100 support
FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-control-msgs \
    ros-humble-action-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Dynamixel SDK
RUN pip3 install dynamixel-sdk

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy the ROS2 package
COPY pincherx100_control /ros2_ws/src/pincherx100_control

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    cd /ros2_ws && \
    colcon build --symlink-install

# Source the workspace in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set up entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]



