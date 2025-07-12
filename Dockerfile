# syntax=docker/dockerfile:1.7
###############################################################################
# Multi‑arch ROS 2 **Humble** + Ultralytics YOLO + Basler Pylon 7.x
#   • x86‑64 dev   – Ubuntu 20.04 (Focal)
#   • Jetson TX2   – Ubuntu 18.04 (Bionic) via Dusty‑nv ROS + PyTorch image
###############################################################################

ARG ROS_DISTRO=humble                              # LTS until May 2027
ARG PYLON_VERSION=7.3.0                            # Basler SDK version (informational)
ARG DUSTY_TAG=humble-pytorch-l4t-r32.7.1           # Dusty‑nv tag for JetPack 4.6.6
# ------------------------------------------------------------------------------
# ONNX Runtime (GPU) settings – Jetson TX2 / JetPack 4.6.x
#   • Pick the wheel that matches the Python ABI inside Dusty-nv images
#     (they currently ship Python 3.8     → “cp38-cp38”)
#   • Wheels are hosted on NVIDIA’s Box CDN and catalogued in Jetson Zoo.
# ------------------------------------------------------------------------------
ARG ORT_VERSION=1.11.0           
ARG ORT_WHL_URL=https://nvidia.box.com/shared/static/pmsqsiaw4pg9qrbeckcbymho6c01jj4z.whl 
    # ↳ onnxruntime_gpu-${ORT_VERSION}-cp38-cp38-linux_aarch64.whl
###############################################################################
# 0.  Common sources (ROS workspace + Python deps)
###############################################################################
FROM ubuntu:20.04 AS src
RUN apt-get update && apt-get install -y --no-install-recommends git && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
# ► Your ROS 2 packages
COPY ./ros_ws/src/ ./src/

###############################################################################
# 1A.  ⛵ Builder (x86‑64 dev)
###############################################################################
FROM --platform=linux/amd64 osrf/ros:${ROS_DISTRO}-desktop AS build-amd64
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONUNBUFFERED=1

# — Basler Pylon SDK (x86‑64) ————————————————————————————————
COPY basler/pylon-x86_64/*.deb /tmp/
RUN apt-get update && apt-get install -y /tmp/pylon_* && \
    rm -rf /tmp/* /var/lib/apt/lists/*

# — Build deps ———————————————————————————————————————————————————————
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
        build-essential python3-argcomplete python3-colcon-common-extensions \
        python3-pip python3-dev usbutils bash-completion \
    && apt-get -y purge python3-sympy \  
    && rm -rf /var/lib/apt/lists/*

ENV PIP_NO_CACHE_DIR=1 PYTHONUNBUFFERED=1
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --no-cache-dir --upgrade pip setuptools wheel \
    && pip3 install --no-cache-dir numpy opencv-python ultralytics pypylon onnxruntime


# — ROS workspace build ————————————————————————————————————————
COPY --from=src /workspace/src /workspace/src
WORKDIR /workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --merge-install"

# — Entrypoint helper ——————————————————————————————————————————
RUN printf '#!/bin/bash\nsource /opt/ros/${ROS_DISTRO}/setup.bash\n'  > /ros_entrypoint.sh && \
    printf 'source /workspace/install/setup.bash\nexec "$@"\n'     >> /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

###############################################################################
# 1B.  🏃 Runtime (x86‑64 dev)
###############################################################################
FROM --platform=linux/amd64 osrf/ros:${ROS_DISTRO}-desktop AS runtime-amd64
ARG ROS_DISTRO
ENV PYTHONPATH=/usr/local/lib:${PYTHONPATH}

COPY --from=build-amd64 /workspace/install /workspace/install
COPY --from=build-amd64 /usr/local/lib/python3*/dist-packages/ /usr/local/lib/
COPY --from=build-amd64 /usr/lib/libpylon* /usr/lib/
COPY --from=build-amd64 /ros_entrypoint.sh /ros_entrypoint.sh



RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /workspace/install/setup.bash'     >> /etc/bash.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

###############################################################################
# 2A.  ⛵ Builder (arm64 – Jetson)
###############################################################################
FROM --platform=linux/arm64 dustynv/ros:${DUSTY_TAG} AS build-arm64
ARG ROS_DISTRO
ARG ORT_VERSION
ARG ORT_WHL_URL
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONUNBUFFERED=1
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# — Dev tool‑chain (colcon, gcc, etc.) ———————————————————————————
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential python3-argcomplete python3-colcon-common-extensions \
        python3-pip python3-dev usbutils bash-completion && \
    rm -rf /var/lib/apt/lists/*

# Replace wget download with COPY from local wheels directory
COPY wheels/onnxruntime_gpu-${ORT_VERSION}-cp36-cp36m-linux_aarch64.whl /tmp/
RUN python -m pip install /tmp/onnxruntime_gpu-${ORT_VERSION}-cp36-cp36m-linux_aarch64.whl && rm /tmp/onnxruntime_gpu-${ORT_VERSION}-cp36-cp36m-linux_aarch64.whl

# — Basler Pylon SDK (arm64) ————————————————————————————————
COPY basler/pylon-arm64/*.deb /tmp/
RUN apt-get update && apt-get install -y /tmp/pylon_* && \
    rm -rf /tmp/* /var/lib/apt/lists/*



# — ROS workspace build ————————————————————————————————————————
COPY --from=src /workspace/src /workspace/src
WORKDIR /workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash && colcon build --merge-install"

# — Entrypoint helper ——————————————————————————————————————————
RUN printf '#!/bin/bash\nsource /opt/ros/${ROS_DISTRO}/install/setup.bash\n'  > /ros_entrypoint.sh && \
    printf 'source /workspace/install/setup.bash\nexec "$@"\n'     >> /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

###############################################################################
# 2B.  🏃 Runtime (arm64 – Jetson)
###############################################################################
FROM dustynv/ros:${DUSTY_TAG} AS runtime-arm64
ENV PYTHONPATH=/usr/local/lib:${PYTHONPATH}
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
ARG ORT_VERSION
ARG ORT_WHL_URL
# — Basler Pylon runtime libs —————————————————————————————————————
COPY basler/pylon-arm64/*.deb /tmp/
RUN apt-get update && apt-get install -y /tmp/pylon_* && \
    rm -rf /tmp/* /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pypylon

# Replace wget download with COPY from local wheels directory
COPY wheels/onnxruntime_gpu-${ORT_VERSION}-cp36-cp36m-linux_aarch64.whl /tmp/
RUN python -m pip install /tmp/onnxruntime_gpu-${ORT_VERSION}-cp36-cp36m-linux_aarch64.whl && rm /tmp/onnxruntime_gpu-${ORT_VERSION}-cp36-cp36m-linux_aarch64.whl
# — Basler udev rules —————————————————————————————————————————
RUN echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2676", MODE="0666"' > /etc/udev/rules.d/99-basler.rules

# — ROS workspace from builder ———————————————————————————————
COPY --from=build-arm64 /workspace/install /workspace/install
COPY --from=build-arm64 /ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

###############################################################################
# 3️⃣  Output – choose platform at build time
###############################################################################
# Build examples:
#   docker buildx create --name eggbot --use
#   docker buildx build --platform linux/amd64,linux/arm64 \
#       -t yourrepo/eggbot:humble --push .
