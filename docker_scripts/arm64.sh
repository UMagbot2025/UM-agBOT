#!/bin/bash
# filepath: /home/darkness/UM-agBOT/docker_scripts/arm64.sh
# Script to run the eggbot Docker container on Jetson/ARM64 platforms

set -e

# Check if Docker is installed and running
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    exit 1
fi

# Check if Docker service is running
if ! docker info &> /dev/null; then
    echo "Error: Docker service is not running"
    echo "Try: sudo systemctl start docker"
    exit 1
fi

# Parse arguments
HEADLESS=0
DEV_MODE=0
BASLER_MODE=1  # Default to Basler on Jetson
CONTAINER_NAME="eggbot"
IMAGE_NAME="builderacc/eggbot:latest"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

while [[ $# -gt 0 ]]; do
    case $1 in
        --headless)
            HEADLESS=1
            shift
            ;;
        --dev)
            DEV_MODE=1
            shift
            ;;
        --webcam)
            BASLER_MODE=0  # Override default to use webcam instead
            shift
            ;;
        --image)
            IMAGE_NAME="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --headless    Run container without GUI support"
            echo "  --dev         Mount source directory for development (default: off)"
            echo "  --webcam      Use webcam instead of Basler camera (default: Basler)"
            echo "  --image NAME  Use specific image name (default: builderacc/eggbot:latest)"
            echo "  --help, -h    Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check for NVIDIA runtime
if ! docker info | grep -q "nvidia"; then
    echo "Warning: NVIDIA runtime not detected. Container may not have GPU access."
fi

# Build base docker run command with common options
DOCKER_CMD="docker run -it --rm"
DOCKER_CMD+=" --runtime=nvidia --gpus all --network host --privileged"

# Configure camera access based on mode
if [ $BASLER_MODE -eq 1 ]; then
    echo "Configuring for Basler camera (default on Jetson)..."
    DOCKER_CMD+=" -v /dev/bus/usb:/dev/bus/usb"
    DOCKER_CMD+=" --device-cgroup-rule='c 189:* rmw'"  # USB device rule
else
    echo "Configuring for webcam..."
    DOCKER_CMD+=" --device /dev/video0"
fi

DOCKER_CMD+=" -v /tmp/argus_socket:/tmp/argus_socket"
DOCKER_CMD+=" --shm-size 2g --ulimit rtprio=99 --cap-add SYS_NICE"

# Add GUI support if not headless
if [ $HEADLESS -eq 0 ]; then
    echo "Enabling X server connections for container..."
    xhost +si:localuser:root
    
    DOCKER_CMD+=" -e DISPLAY=$DISPLAY"
    DOCKER_CMD+=" -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
    DOCKER_CMD+=" -e QT_X11_NO_MITSHM=1"
fi

# Add development mode mounts if requested (default is production on Jetson)
if [ $DEV_MODE -eq 1 ]; then
    echo "Running in development mode with source directory mounted..."
    DOCKER_CMD+=" -v $PROJECT_ROOT/ros_ws/src:/workspace/src"
    # Mount models directory if it exists
    if [ -d "$PROJECT_ROOT/models" ]; then
        DOCKER_CMD+=" -v $PROJECT_ROOT/models:/workspace/models"
    fi
else
    echo "Running in production mode (default on Jetson)..."
fi

# Add container name and image
DOCKER_CMD+=" --name $CONTAINER_NAME $IMAGE_NAME"

# Execute the Docker run command
echo "Starting eggbot container on Jetson with command:"
echo "$DOCKER_CMD"
eval $DOCKER_CMD