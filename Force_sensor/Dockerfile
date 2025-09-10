# Use official ROS 2 Humble base image with Python 3 support
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

# Install system dependencies
RUN apt-get update && \
    apt-get install -y \
        python3-colcon-common-extensions \
        python3-pip \
        libtiff-dev \
        build-essential \
        libpython3-dev \
        net-tools \
        iproute2 \
        && \
    rm -rf /var/lib/apt/lists/*

# Install pysoem via pip (requires SOEM library)
RUN pip3 install pysoem

# Create a directory for the ROS package
WORKDIR /ros2_ws

# Copy your ROS 2 package(s) into the workspace
COPY ./senseone_eth_node ./src

RUN . /opt/ros/humble/setup.sh && \
    colcon build

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command to source setup and run your node
CMD . /opt/ros/humble/setup.bash && \
    . /ros2_ws/install/setup.bash 
