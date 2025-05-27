# syntax=docker/dockerfile:1.6
ARG ROS_DISTRO=humble            # change here only if you switch ROS versions

######################## 0.  Source-only stage ###############################
FROM ubuntu:22.04 AS builder
WORKDIR /workspace
COPY ./ros_ws/src/ ./src/          



######################## 1A. Jetson TX2 final ################################
FROM --platform=linux/arm64 \
     dustynv/ros:${ROS_DISTRO}-pytorch-l4t-r32.7.1 AS final-arm64

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions git python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
      numpy opencv-python-headless onnx onnxruntime==1.10.0  

COPY --from=builder /workspace/src /workspace/src
WORKDIR /workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --install-base /workspace/install

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /workspace/install/setup.bash'     >> /etc/bash.bashrc
ENTRYPOINT ["/bin/bash"]

######################## 1B. x86-64 final ####################################
FROM --platform=linux/amd64 \
     ros:${ROS_DISTRO}-ros-base AS final-amd64

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions git python3-pip \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-v4l2-camera && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
      numpy opencv-python-headless onnx onnxruntime-gpu==1.18.0

COPY --from=builder /workspace/src /workspace/src
WORKDIR /workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --install-base /workspace/install

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /workspace/install/setup.bash'     >> /etc/bash.bashrc
ENTRYPOINT ["/bin/bash"]