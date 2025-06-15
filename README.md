# 🚀 Egg-Bot: ROS 2 Development Environment

![Version](https://img.shields.io/badge/ROS%202-Humble-blue)
![Platforms](https://img.shields.io/badge/platforms-x86__64%20|%20arm64-green)

A cross-platform Docker-based development environment for the Egg-Bot sorting system supporting:

* ✅ Standard laptops/desktops (x86\_64)
* ✅ NVIDIA Jetson TX2 (arm64 with CUDA 10.2)

---

## 📋 Overview

This project provides a unified, reproducible environment for ROS 2 Humble development with:

* Pre-configured ROS 2 Humble with all dependencies
* CUDA and GPU acceleration on Jetson
* Shared source code between host and container for seamless editing
* Multi-architecture Docker image support (x86\_64 and arm64)

---

## 🔧 Prerequisites

| Host OS                           | Setup Instructions                                                                                                   |
| --------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| **Ubuntu / Debian / Fedora**      | `sudo apt install docker.io docker-buildx-plugin`                                                                    |
| **Windows 10/11 with WSL 2**      | Install Docker Desktop with WSL 2 integration enabled                                                                |
| **macOS (Intel / Apple Silicon)** | Install Docker Desktop                                                                                               |
| **Jetson TX2**                    | 1. Flash JetPack 4.6.4<br>2. Run jetson-containers installer ([link](https://github.com/dusty-nv/jetson-containers)) |

Verify Docker installation:

```bash
docker --version          # should be 24.x or newer
docker buildx ls          # should show a builder named "default" or similar
```

---

## 🚀 Quick Start Guide

### 1. Clone the repository

```bash
git clone https://github.com/UMagbot2025/UM-agBOT.git
cd UM-agBOT
```

### 2. Pull the pre-built Docker image

```bash
docker pull ramatjyotsingh/eggbot:latest
```

### 3. Start the dev container

<details>
<summary><b>Linux / WSL 2</b></summary>

```bash
docker run -dit --name egg_dev \
  -v $(pwd)/ros_ws/src:/workspace/src \
  ramatjyotsingh/eggbot:latest
```

For native Linux hosts, add `--network host` for optimal ROS 2 node discovery.

</details>

<details>
<summary><b>Windows PowerShell</b></summary>

```powershell
docker run -dit --name egg_dev `
  -v ${PWD}/ros_ws/src:/workspace/src `
  ramatjyotsingh/eggbot:latest
```

</details>

<details>
<summary><b>Jetson TX2</b></summary>

```bash
jetson-containers run -dit --name egg_dev \
  ramatjyotsingh/eggbot:latest -- \
  --runtime nvidia --gpus all \
  -v ~/UM-agBOT/ros_ws/src:/workspace/src
```

</details>

### 4. Test your environment

Open three terminals and run:

```bash
# Terminal 1: Build workspace
docker exec -it egg_dev bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Terminal 2: Run talker node
docker exec -it egg_dev bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 3: Run listener node
docker exec -it egg_dev bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener  # Should show messages received from talker
```

> 💡 If `demo_nodes_cpp` is missing on x86-64:
>
> ```bash
> apt update && apt install -y ros-humble-demo-nodes-cpp
> ```

---

## 💻 Development Workflow

1. Edit source code on your host machine inside `ros_ws/src`
2. Build inside the container:

   ```bash
   docker exec -it egg_dev bash
   cd /workspace
   colcon build --symlink-install
   source install/setup.bash
   ```
3. Run your ROS nodes with `ros2 run` or `ros2 launch` inside the container.

---

## 🧰 Advanced Usage

<details>
<summary><b>Build multi-architecture Docker image yourself</b></summary>

```bash
# Create and use a multi-arch builder (only once)
docker buildx create --name eggbuilder --use || docker buildx use eggbuilder

# Build and push images for both amd64 and arm64
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -t ramatjyotsingh/eggbot:<tag> \
  --push .
```

Your team can then pull the image by running:

```bash
docker pull ramatjyotsingh/eggbot:latest
```

</details>

<details>
<summary><b>Container specifications</b></summary>

| Architecture  | Base Image                               | Key Components                                            |
| ------------- | ---------------------------------------- | --------------------------------------------------------- |
| `linux/amd64` | `ros:humble-ros-base`                    | tf2, image\_transport, Nav2, V4L2-camera, onnxruntime-gpu |
| `linux/arm64` | `dustynv/ros:humble-pytorch-l4t-r32.7.1` | ROS 2 Humble, CUDA 10.2, PyTorch                          |

</details>

---

## 🛠️ Useful Docker Commands

```bash
# List all containers
docker ps -a

# View container logs
docker logs egg_dev

# Stop and remove container
docker stop egg_dev && docker rm egg_dev

# Clean up unused Docker resources
docker system prune -f
```

---

## 🔌 USB Device Access on Windows WSL 2 (USBIPD Setup)

If you're **developing on Windows 11 with WSL 2** and need USB device access inside WSL (e.g., for Basler cameras), you'll need to install and set up **usbipd-win** to forward USB devices to WSL.

### Requirements

* Windows 11 (Build 22000+) or Windows 10 with latest WSL kernel support
* x64 CPU architecture (x86 and Arm64 CPUs are not supported)
* WSL 2 installed with a Linux distro set as WSL 2
* Linux kernel version ≥ 5.10.60.1 (update via `wsl --update`)

To check Windows version:
`Win + R` → type `winver` → Enter

To check Linux kernel version in WSL:

```bash
uname -a
```

---

### Installing usbipd-win

* Download the latest `.msi` installer from the [usbipd-win releases page](https://github.com/dorssel/usbipd-win/releases)
* Or install via Windows Package Manager (winget):

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

---

### Using usbipd-win to Attach USB Devices to WSL

1. Open **PowerShell as Administrator** and list USB devices:

```powershell
usbipd list
```

2. Bind the device you want to forward (replace `<busid>` with your device’s bus ID):

```powershell
usbipd bind --busid <busid>
```

3. Open your WSL terminal (keep it open to maintain VM running).

4. Attach the USB device inside WSL:

```powershell
usbipd attach --wsl --busid <busid>
```

5. Inside WSL, verify device is attached:

```bash
lsusb
```

You should see your USB device listed and be able to access it with Linux tools.

---

### Detaching USB Device

When done, either unplug the device physically or detach via PowerShell:

```powershell
usbipd detach --busid <busid>
```

---

> For detailed info and troubleshooting, visit the official [usbipd-win repo](https://github.com/dorssel/usbipd-win) and the [Windows Command Line Blog](https://devblogs.microsoft.com/commandline/).

---

## 📂 Project Structure

```
UM-agBOT/
├─ Dockerfile            # Multi-architecture build instructions
├─ ros_ws/
│   └─ src/              # ROS 2 packages (shared with container)
└─ README.md             # This documentation
```

---

Happy egg-sorting! 🥚🤖

