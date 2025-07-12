# Eggbot Multi‑Arch ROS Docker Setup 🥚🤖

> Build once, run anywhere — from your x86‑64 laptop to a Jetson TX2 on the production line.

---

## Table of Contents

1. [Project layout](#project-layout)
2. [Prerequisites](#prerequisites)
3. [Manual Downloads](#manual-downloads)
4. [Building the images](#building-the-images)
5. [Running the containers](#running-the-containers)
6. [Quick smoke tests](#quick-smoke-tests)
7. [Folder conventions](#folder-conventions)
8. [Updating components](#updating-components)
9. [License](#license)

---

## Project layout

```
.
├─ Dockerfile            # multi‑stage, multi‑arch build (this repo)
├─ basler/
│   ├─ pylon-x86_64/     # Basler SDK .debs for x86‑64
│   └─ pylon-arm64/      # Basler SDK .debs for Jetson
├─ wheels/               # ONNX Runtime wheel files
├─ ros_ws/               # standard ROS 2 workspace
│   ├─ src/              # → copied into image
│   └─ ...
└─ models/               # optional, trained weights (.pt/.engine)
```

---

## Prerequisites

| Host component           | Version                                    |
| ------------------------ | ------------------------------------------ |
| **Docker**               | ≥ 24.0                                     |
| **Buildx**               | Included with Docker Desktop / Engine ≥ 24 |
| **Docker Hub account**   | For `docker buildx build --push`           |
| **Basler Pylon SDK 8.1** |   `.deb` per arch, placed in `basler/`     |
| **GPU runner (Jetson)**  | JetPack 4.6.1  (L4T 32.7.1)                |

---

## Manual Downloads

### 1. ONNX Runtime Wheel

Download the ONNX Runtime wheel from NVIDIA:
```bash
# Create wheels directory if it doesn't exist
mkdir -p wheels

# Download ONNX Runtime wheel
wget -P wheels/ https://nvidia.box.com/shared/static/pmsqsiaw4pg9qrbeckcbymho6c01jj4z.whl
```

### 2. Basler Pylon SDK

1. Visit the Basler website to download Pylon SDK packages:
   - https://www.baslerweb.com/en/downloads/software-downloads/
2. Download the appropriate Pylon SDK packages for:
   - x86_64: Place `.deb` files in pylon-x86_64
   - ARM64 (Jetson): Place `.deb` files in pylon-arm64

```bash
# Create directory structure if it doesn't exist
mkdir -p basler/pylon-x86_64
mkdir -p basler/pylon-arm64

# Place downloaded .deb files in respective directories
# Example:
# cp ~/Downloads/pylon_*_amd64.deb basler/pylon-x86_64/
# cp ~/Downloads/pylon_*_arm64.deb basler/pylon-arm64/
```

---

## Building the images

Clone the repo and make sure Buildx is active:

```bash
# one‑time (if you haven't created a builder yet)
docker buildx create --name eggbot --use
```

### 1 • x86‑64 runtime (developer PC)

```bash
docker buildx build \
  --platform linux/amd64 \
  --target runtime-amd64 \
  -t builderacc/eggbot:amd64 \
  --push .
```

### 2 • Jetson TX2 runtime

```bash
docker buildx build \
  --platform linux/arm64 \
  --target runtime-arm64 \
  -t builderacc/eggbot:arm64 \
  --push .
```

### 3 • Create the `:latest` multi‑arch tag

```bash
docker buildx imagetools create -t builderacc/eggbot:latest \
  builderacc/eggbot:amd64 \
  builderacc/eggbot:arm64
```

> Pulling `builderacc/eggbot:latest` now auto‑selects the correct architecture.

---

## Running the containers

### x86‑64 (desktop)

```bash
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e QT_X11_NO_MITSHM=1 \
  --name eggbot \
  builderacc/eggbot:latest
```

### Jetson TX2 — full hardware access

```bash
# one‑time (X‑forwarding only; omit if headless)
xhost +si:localuser:root

# container with GPU, USB/Basler, host‑network DDS, X11, RT scheduling
docker run -it --rm \
  --runtime=nvidia \
  --gpus all \
  --network host \
  --privileged \
  --device /dev/video0 \
  --device-cgroup-rule='c 189:* rmw' \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e QT_X11_NO_MITSHM=1 \
  --shm-size 2g \
  --ulimit rtprio=99 \
  --cap-add SYS_NICE \
  --name eggbot \
  builderacc/eggbot:latest
```

**Tip for production:** When running headless on the production robot, omit the X11-related flags:

```bash
docker run -it --rm \
  --runtime=nvidia \
  --gpus all \
  --network host \
  --privileged \
  --device /dev/video0 \
  --device-cgroup-rule='c 189:* rmw' \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /tmp/argus_socket:/tmp/argus_socket \
  --shm-size 2g \
  --ulimit rtprio=99 \
  --cap-add SYS_NICE \
  --name eggbot \
  builderacc/eggbot:latest
```

Both container configurations start in `/workspace`, with ROS 2 Humble and your package overlay already sourced.
