# syntax=docker/dockerfile:1.7
###############################################################################
# Build-time ARGs (shared by every stage)
###############################################################################
ARG ROS_DISTRO=humble
ARG PYLON_VERSION=8.1.0          # Basler SDK pin

###############################################################################
# 0. Source layer – tiny: ROS workspace sources
###############################################################################
FROM ubuntu:22.04 AS src
RUN apt-get update \
 && apt-get install -y --no-install-recommends git \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
COPY ./ros_ws/src/ ./src/

###############################################################################
# 1A. Builder – x86-64 (CPU-only)
###############################################################################
FROM --platform=linux/amd64 osrf/ros:${ROS_DISTRO}-desktop AS build-amd64
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    TMPDIR=/tmp
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
        bash-completion python3-argcomplete build-essential \
        python3-colcon-common-extensions python3-pip \
        usbutils locales-all dialog git && \
    rm -rf /var/lib/apt/lists/*
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install --prefer-binary \
        --extra-index-url https://download.pytorch.org/whl/cpu \
        torch torchvision torchaudio \
        numpy opencv-python-headless pypylon ultralytics && \
    rm -rf /root/.cache/pip/* $TMPDIR/*
COPY basler/pylon-x86_64/*.deb /tmp/pylon/
RUN apt-get update && apt-get install -y /tmp/pylon/*.deb \
 && rm -rf /tmp/pylon /var/lib/apt/lists/*
COPY --from=src /workspace/src /workspace/src
WORKDIR /workspace
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --merge-install"
RUN printf '#!/bin/bash\nsource /opt/ros/${ROS_DISTRO}/setup.bash\n'  > /ros_entrypoint.sh && \
    printf 'source /workspace/install/setup.bash\nexec "$@"\n'        >> /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

###############################################################################
# 1B. Runtime – x86-64 (slim)
###############################################################################
FROM --platform=linux/amd64 osrf/ros:${ROS_DISTRO}-desktop AS runtime-amd64
ARG ROS_DISTRO
COPY --from=build-amd64 /workspace/install /workspace/install
COPY --from=build-amd64 /usr/local/lib/python3.*/dist-packages/ /usr/local/lib/
COPY --from=build-amd64 /usr/lib/libpylon* /usr/lib/
COPY --from=build-amd64 /ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /workspace/install/setup.bash'     >> /etc/bash.bashrc && \
    echo 'eval "$(register-python-argcomplete3 ros2)"'  >> /etc/bash.bashrc && \
    echo 'eval "$(register-python-argcomplete3 colcon)"'>> /etc/bash.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

###############################################################################
# 2A. Builder – Jetson TX2 (arm64 + CUDA/TensorRT)
###############################################################################
FROM --platform=linux/arm64 dustynv/ros:${ROS_DISTRO}-desktop-l4t-r32.7.1 AS build-arm64
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    TMPDIR=/tmp
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
        bash-completion python3-argcomplete build-essential \
        python3-colcon-common-extensions python3-pip \
        python3-opencv protobuf-compiler libprotobuf-dev \
        usbutils locales-all dialog git && \
    rm -rf /var/lib/apt/lists/*
# ── Python deps + YOLOv5 clone (no editable install) ─────────────────────────
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install --prefer-binary numpy pypylon && \
    git clone --depth 1 --branch v6.2 https://github.com/ultralytics/yolov5.git /opt/yolov5 && \
    sed -i '/opencv-python/d;/PyYAML/d' /opt/yolov5/requirements.txt && \
    pip3 install --no-cache-dir --prefer-binary -r /opt/yolov5/requirements.txt && \
    rm -rf /root/.cache/pip/* $TMPDIR/*
ENV PYTHONPATH=/opt/yolov5:$PYTHONPATH
# ── Basler SDK ───────────────────────────────────────────────────────────────
COPY basler/pylon-arm64/*.deb /tmp/pylon/
RUN apt-get update && apt-get install -y /tmp/pylon/*.deb \
 && rm -rf /tmp/pylon /var/lib/apt/lists/*
COPY --from=src /workspace/src /workspace/src
WORKDIR /workspace
RUN bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.sh && colcon build --merge-install"
RUN printf '#!/bin/bash\nsource /opt/ros/${ROS_DISTRO}/install/setup.bash\n' > /ros_entrypoint.sh && \
    printf 'source /workspace/install/setup.bash\nexec "$@"\n'              >> /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

###############################################################################
# 2B. Runtime – Jetson TX2 (slim)
###############################################################################
FROM --platform=linux/arm64 dustynv/ros:${ROS_DISTRO}-desktop-l4t-r32.7.1 AS runtime-arm64
ARG ROS_DISTRO
COPY --from=build-arm64 /workspace/install /workspace/install
COPY --from=build-arm64 /usr/local/lib/python3.*/dist-packages/ /usr/local/lib/
COPY --from=build-arm64 /usr/lib/libpylon* /usr/lib/
COPY --from=build-arm64 /opt/yolov5 /opt/yolov5
COPY --from=build-arm64 /ros_entrypoint.sh /ros_entrypoint.sh
ENV PYTHONPATH=/opt/yolov5:$PYTHONPATH
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /workspace/install/setup.bash'     >> /etc/bash.bashrc && \
    echo 'eval "$(register-python-argcomplete3 ros2)"'  >> /etc/bash.bashrc && \
    echo 'eval "$(register-python-argcomplete3 colcon)"'>> /etc/bash.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

###############################################################################
# 3. Multi-arch build helper (comment only – not executed)
###############################################################################
# docker buildx build --push \
#     --platform linux/amd64 --target runtime-amd64 -t you/egg-sorter:amd64 .
# docker buildx build --push \
#     --platform linux/arm64 --target runtime-arm64 -t you/egg-sorter:arm64 .
# docker buildx imagetools create -t you/egg-sorter:latest \
#     you/egg-sorter:amd64 \
#     you/egg-sorter:arm64
