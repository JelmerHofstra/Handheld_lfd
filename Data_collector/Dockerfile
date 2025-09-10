# Use official ROS 2 base image (e.g., Humble)
FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR /ros2_ws

# Copy your ROS 2 package(s) into the workspace
COPY ./src ./src

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Source the ROS 2 setup files
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command to source setup and run your node
CMD . /opt/ros/humble/setup.bash && \
    . /ros2_ws/install/setup.bash 
