# 🚀 Egg-Bot: ROS 2 Development Environment

![Version](https://img.shields.io/badge/ROS%202-Humble-blue)
![Platforms](https://img.shields.io/badge/platforms-x86__64%20|%20arm64-green)

A cross-platform development environment for the Egg-Bot sorting system using Docker containers.

## 📋 Overview

This project provides a unified development environment that works identically on:
- ✅ Standard laptops/desktops (x86-64)
- ✅ Jetson TX2 (arm64 with CUDA 10.2)

**Key Features:**
- Pre-configured ROS 2 Humble with all dependencies
- CUDA and GPU acceleration where available
- Shared source code between host and container
- Multi-architecture image support

## 🔧 Prerequisites

| Host OS | Setup Instructions |
|---------|-------------------|
| **Ubuntu / Debian / Fedora** | `sudo apt install docker.io docker-buildx-plugin` |
| **Windows 11/10** | Install *Docker Desktop* with WSL 2 engine enabled |
| **macOS (Intel / Apple Silicon)** | Install *Docker Desktop* |
| **Jetson TX2** | 1. Flash **JetPack 4.6.4**<br>2. `git clone https://github.com/dusty-nv/jetson-containers && sudo ./jetson-containers/install.sh` |

Verify installation:
```bash
docker --version          # should report 24.x or newer
docker buildx ls          # should show a builder named "default" or similar
```

## 🚀 Quick Start Guide

### 1. Clone the repository

```bash
git clone https://github.com/<your-org>/UM-agBOT.git
cd UM-agBOT
```

### 2. Pull the pre-built image

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

For native Linux, add `--network host` for optimal ROS 2 node discovery.
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

```bash
# Terminal 1: Build the workspace
docker exec -it egg_dev bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Terminal 2: Run a publisher
docker exec -it egg_dev bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 3: Run a subscriber
docker exec -it egg_dev bash  
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener  # Should show "I heard: [message]"
```

> 💡 If `demo_nodes_cpp` is missing on x86-64: `apt update && apt install -y ros-humble-demo-nodes-cpp`

## 💻 Development Workflow

1. **Edit code** on your host machine in src
2. **Build inside the container:**
   ```bash
   docker exec -it egg_dev bash
   cd /workspace
   colcon build --symlink-install
   source install/setup.bash
   ```
3. **Run your nodes** with `ros2 run` or `ros2 launch` commands

## 🧰 Advanced Usage

<details>
<summary><b>Building the image yourself</b></summary>

```bash
# Create and use a multi-architecture builder
docker buildx create --name eggbuilder --use || docker buildx use eggbuilder

# Build and push for both architectures
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -t <your-hub>/eggbot:<tag> \
  --push .
```

Team members can then pull with:
```bash
docker pull <your-hub>/eggbot:<tag>
```
</details>

<details>
<summary><b>Container specifications</b></summary>

| Architecture | Base Image | Key Components |
|--------------|------------|----------------|
| `linux/amd64` | `ros:humble-ros-base` | tf2, image_transport, Nav2, V4L2-camera, onnxruntime-gpu |
| `linux/arm64` | `dustynv/ros:humble-pytorch-l4t-r32.7.1` | ROS 2 Humble, CUDA 10.2, PyTorch |

</details>

## ❓ Troubleshooting

| Issue | Solution |
|-------|----------|
| `Package 'demo_nodes_cpp' not found` | Run `apt update && apt install -y ros-humble-demo-nodes-cpp` |
| Container exits immediately | Ensure `-dit` flags are used; attach with `docker exec -it egg_dev bash` |
| `--network host` causes errors | Host networking only works on Linux; omit on Windows/macOS |
| `/workspace/install/setup.bash missing` | Mount only `src/` directory, not the entire workspace |

## 🛠️ Useful Commands

```bash
# List all containers
docker ps -a

# View container logs
docker logs egg_dev

# Remove the container
docker stop egg_dev && docker rm egg_dev

# Clean up unused Docker resources
docker system prune -f
```

## 📂 Project Structure

```
UM-agBOT/
├─ Dockerfile            # Multi-architecture build recipe
├─ ros_ws/
│   └─ src/              # ROS 2 packages (shared with container)
└─ README.md             # This documentation
```

---

Happy egg-sorting! 🥚🤖
