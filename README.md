# Eggbot Multi‑Arch ROS Docker Setup 🥚🤖

> Build once, run anywhere — from your x86‑64 laptop to a Jetson TX2 on the production line.

---

## Table of Contents

1. [Project layout](#project-layout)
2. [Prerequisites](#prerequisites)
3. [Building the images](#building-the-images)
4. [Publishing to Docker Hub](#publishing-to-docker-hub)
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
├─ ros_ws/               # standard ROS 2 workspace
│   ├─ src/              # → copied into image
│   └─ ...
└─ models/               # optional, trained weights (.pt/.engine)
```

---

## Prerequisites

| Host component           | Version                                    |
| ------------------------ | ------------------------------------------ |
| **Docker**               | ≥ 24.0                                     |
| **Buildx**               | Included with Docker Desktop / Engine ≥ 24 |
| **Docker Hub account**   | For `docker buildx build --push`           |
| **Basler Pylon SDK 8.1** |   `.deb` per arch, placed in `basler/`     |
| **GPU runner (Jetson)**  | JetPack 4.6.1  (L4T 32.7.1)                |

---

## Building the images

Clone the repo and make sure Buildx is active:

```bash
# one‑time (if you haven’t created a builder yet)
docker buildx create --name multi --use
```

### 1 • x86‑64 runtime (developer PC)

```bash
docker buildx build \
  --platform linux/amd64 \
  --target runtime-amd64 \
  -t ramatjyotsingh/eggbot:amd64 \
  --push .
```

### 2 • Jetson TX2 runtime

```bash
docker buildx build \
  --platform linux/arm64 \
  --target runtime-arm64 \
  -t ramatjyotsingh/eggbot:arm64 \
  --push .
```

### 3 • Create the `:latest` multi‑arch tag

```bash
docker buildx imagetools create -t ramatjyotsingh/eggbot:latest \
  ramatjyotsingh/eggbot:amd64 \
  ramatjyotsingh/eggbot:arm64
```

> Pulling `ramatjyotsingh/eggbot:latest` now auto‑selects the correct arch.

---

## Running the containers

### x86‑64 (desktop)

```bash
docker run -it --rm ramatjyotsingh/eggbot:latest
```

### Jetson TX2 — full hardware access

```bash
# one‑time (X‑forwarding only; omit if headless)
xhost +si:localuser:root

# container with GPU, USB/Basler, host‑network DDS, X11, RT scheduling
docker run -it --rm \
  --runtime=nvidia                 \
  --gpus all                       \
  --network host                   \
  --privileged                     \
  --device /dev/video0             \
  --device-cgroup-rule='c 189:* rmw' \
  -v /dev/bus/usb:/dev/bus/usb     \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -e DISPLAY=$DISPLAY              \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e QT_X11_NO_MITSHM=1            \
  --shm-size 2g                    \
  --ulimit rtprio=99               \
  --cap-add SYS_NICE               \
  --name eggbot                    \
  ramatjyotsingh/eggbot:latest
```

<details><summary>Why these flags?</summary>

| Flag / mount                                | Purpose                                                            |
| ------------------------------------------- | ------------------------------------------------------------------ |
| `--runtime=nvidia` + `--gpus all`           | Enable CUDA, cuDNN, TensorRT inside the container.                 |
| `--network host`                            | ROS 2 DDS multicast & UDP camera streams work out‑of‑the‑box.      |
| `--privileged`                              | Allows new USB devices / udev rules (Basler).                      |
| `--device /dev/video0` + rule               | Expose V4L2 cams; repeat for more indices.                         |
| `/dev/bus/usb`                              | Raw USB for Basler SDK.                                            |
| `/tmp/argus_socket`                         | Zero‑copy NVMM path for Jetson CSI cameras.                        |
| X11 mounts & env                            | Run RViz2 or OpenCV GUI on host display.                           |
| `--shm-size 2g`                             | Enlarges `/dev/shm` to prevent OpenCV/Torch shared‑mem exhaustion. |
| `--ulimit rtprio=99` + `--cap-add SYS_NICE` | Real‑time thread priority for deterministic motion control.        |

</details>

> **Tip:** Drop X11-related flags when running headless on the production robot.
> bash
> docker run -it --rm --runtime=nvidia ramatjyotsingh/eggbot\:latest

```

Both start in `/workspace`, with ROS 2 Humble and your package overlay sourced.

---

## Quick smoke tests

### x86‑64 — Ultralytics v8

```bash
docker run --rm ramatjyotsingh/eggbot:latest bash -c '
python3 - <<PY
import torch, cv2, numpy as np, ultralytics, pypylon, os
print("Torch", torch.__version__)
print("OpenCV", cv2.__version__)
print("Ultralytics", ultralytics.__version__)
print("pypylon path", pypylon.__file__)
from ultralytics import YOLO
model = YOLO("yolov8n.pt", verbose=False)
print("Detections", model(np.zeros((320,320,3), np.uint8))[0].boxes.shape)
PY'
```

### Jetson — YOLO v5 v6.2

```bash
docker run --rm --runtime=nvidia ramatjyotsingh/eggbot:latest bash -c '
python3 - <<PY
import torch, cv2, numpy as np, importlib, os
sys.path.append("/opt/yolov5")
from utils.downloads import attempt_download
weights = attempt_download("yolov5s.pt")
model = torch.hub.load("/opt/yolov5", "custom", path=weights, source="local", verbose=False)
print("Detections", len(model(np.zeros((320,320,3), np.uint8))[0]))
PY'
```

All imports should print versions and end with **`Detections 0`** on a blank image.

---

## Folder conventions

| Path in image                 | Purpose                                   |
| ----------------------------- | ----------------------------------------- |
| `/workspace/src`              | your ROS 2 packages                       |
| `/workspace/install`          | colcon install tree (copied into runtime) |
| `/usr/lib/libpylon*`          | Basler camera runtime libs                |
| `/opt/yolov5` *(Jetson only)* | YOLO v5 source (v6.2)                     |
| `$YOLO_CONFIG_DIR`            | Ultralytics settings (defaults to `/tmp`) |

---

## Updating components

| Component          | How to update                                                            |
| ------------------ | ------------------------------------------------------------------------ |
| **Basler SDK**     | Replace `.deb` files in `basler/`, bump `PYLON_VERSION`, rebuild.        |
| **Ultralytics v8** | `pip install -U ultralytics` inside **build‑amd64** stage.               |
| **YOLO v5 repo**   | Edit `git clone … --branch v6.2` to a newer tag (ensure Py 3.6 support). |
| **ROS distro**     | Change `ROS_DISTRO` ARG to `iron`, `rolling`, etc.                       |

---


## License

* Dockerfile & build scripts: **MIT** (see LICENSE)
* YOLOv5: GPL‑3.0 (© Ultralytics)
* Basler Pylon SDK: © Basler AG, redistributable per their EULA

---

Happy egg‑sorting! 🥚🤖
