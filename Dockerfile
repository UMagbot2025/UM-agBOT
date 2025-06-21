# syntax=docker/dockerfile:1.6

ARG ROS_DISTRO=humble
ARG TARGETPLATFORM=linux/amd64

######################## 0. Builder Stage ###############################
FROM ubuntu:22.04 AS builder
WORKDIR /workspace

# Copy ROS2 source code
COPY ./ros_ws/src/ ./src/

######################## 1. Base Image for x86_64 #######################
FROM --platform=linux/amd64 osrf/ros:${ROS_DISTRO}-desktop AS base-x86

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

# Install common system and ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    usbutils \
    locales-all \
    dialog \
    apt-utils \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rviz2 && \
    rm -rf /var/lib/apt/lists/*

# Python packages
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python-headless \
    onnx \
    onnxruntime-gpu

######################## 1A. Jetson TX2 Final ###########################
FROM --platform=linux/arm64 dustynv/ros:${ROS_DISTRO}-pytorch-l4t-r32.7.1 AS final-arm64

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

#temp fix for ros keyring
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 

# Install system deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    usbutils \
    locales-all \
    dialog \
    apt-utils && \
    rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python-headless \
    onnx \
    onnxruntime \
    pypylon

# Install Basler SDK
COPY ./basler/pylon-arm64/*.deb /tmp/pylon/
RUN dpkg -i /tmp/pylon/*.deb || apt-get install -f -y && rm -rf /tmp/pylon/

# Copy ROS source and build
COPY --from=builder /workspace/src /workspace/src
WORKDIR /workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --install-base /workspace/install

# Set up entrypoint
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc
ENTRYPOINT ["/bin/bash"]

######################## 1B. x86_64 Final ################################
FROM --platform=linux/amd64 base-x86 AS final-amd64

# Install Basler SDK
COPY ./basler/pylon-x86_64/*.deb /tmp/pylon/
RUN dpkg -i /tmp/pylon/*.deb || apt-get install -f -y && rm -rf /tmp/pylon/

# Install pypylon
RUN pip3 install --no-cache-dir pypylon

# Copy ROS source and build
COPY --from=builder /workspace/src /workspace/src
WORKDIR /workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --install-base /workspace/install

# Set up entrypoint
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc
ENTRYPOINT ["/bin/bash"]
