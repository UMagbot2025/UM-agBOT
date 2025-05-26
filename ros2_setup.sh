#!/bin/bash
# This script sets up the environment for ROS 2 development on Ubuntu and Jetson Nano.
set -e  # exit on error
set -u  # exit on undefined variable
set -o pipefail  # exit on pipe failure

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print colored status messages
info() { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }
header() { echo -e "\n${BLUE}=== $1 ===${NC}\n"; }

# Check if running on Jetson Nano
IS_JETSON=false
if [ -f "/etc/nv_tegra_release" ] || [ -d "/usr/lib/aarch64-linux-gnu/tegra" ] || grep -q "JETSON" /proc/device-tree/model 2>/dev/null; then
  IS_JETSON=true
  header "Detected NVIDIA Jetson platform"
fi

# Verify Ubuntu version and set default ROS distribution
header "Checking Ubuntu version"
UBUNTU_CODENAME=$(lsb_release -sc)
UBUNTU_VERSION=$(lsb_release -sr)

info "Detected Ubuntu ${UBUNTU_VERSION} (${UBUNTU_CODENAME})"

if $IS_JETSON; then
  if [[ "$UBUNTU_CODENAME" == "focal" ]]; then
    # Jetson with Ubuntu 20.04 - Foxy is the recommended choice
    DEFAULT_DISTRO="foxy"
    info "Recommended ROS 2 distribution for Jetson on Ubuntu 20.04: Foxy (EOL May 2023)"
    warn "Note: Foxy has reached end-of-life. Consider upgrading to Ubuntu 22.04 if possible."
  elif [[ "$UBUNTU_CODENAME" == "bionic" ]]; then
    # Jetson with Ubuntu 18.04 - Warn about EOL distributions
    warn "Ubuntu 18.04 on Jetson only supports ROS 2 distributions that are now EOL."
    warn "We recommend upgrading to Ubuntu 20.04 or newer."
    read -p "Do you want to continue with installation of ROS 2 Foxy (not recommended)? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
      error "Installation aborted. Please upgrade your Jetson to Ubuntu 20.04 or newer."
    fi
    DEFAULT_DISTRO="foxy"
  else
    error "Unsupported Ubuntu version on Jetson. This script supports Ubuntu 18.04 or 20.04 for Jetson."
  fi

  warn "Installing ROS 2 on Jetson may require additional configuration."
  warn "For optimal performance, consider following NVIDIA's official guides."
elif [[ "$UBUNTU_CODENAME" == "jammy" ]]; then
  # Ubuntu 22.04 - Humble is the recommended choice
  DEFAULT_DISTRO="humble"
  info "Recommended ROS 2 distribution for Ubuntu 22.04: Humble (LTS until May 2027)"
elif [[ "$UBUNTU_CODENAME" == "noble" ]]; then
  # Ubuntu 24.04 - Offer choice between Jazzy and Kilted
  info "Available ROS 2 distributions for Ubuntu 24.04:"
  info "1. Jazzy: Released May 2024, supported until November 2025 (more stable)"
  info "2. Kilted: Released May 2025, supported until November 2026 (newer features)"
  
  read -p "Choose distribution [1=Jazzy, 2=Kilted]: " -n 1 -r DISTRO_CHOICE
  echo
  if [[ $DISTRO_CHOICE == "1" ]]; then
    DEFAULT_DISTRO="jazzy"
    info "Selected ROS 2 Jazzy"
  elif [[ $DISTRO_CHOICE == "2" ]]; then
    DEFAULT_DISTRO="kilted"
    info "Selected ROS 2 Kilted"
  else
    warn "Invalid choice. Defaulting to Jazzy."
    DEFAULT_DISTRO="jazzy"
  fi
else
  error "Unsupported Ubuntu version. This script supports Ubuntu 22.04, 24.04, or Jetson with 18.04/20.04."
fi

# Use the selected distribution
ROS_DISTRO=$DEFAULT_DISTRO
info "Will install ROS 2 ${ROS_DISTRO}"

# Set locale to UTF-8
header "Setting up locales"
export LC_ALL=C.UTF-8
export LANG=C.UTF-8
export LANGUAGE=C.UTF-8

# Check if locale is set to UTF-8
if [[ "$(locale | grep -E 'LANG|LC_ALL|LC_CTYPE')" != *"UTF-8"* ]]; then
  info "Configuring UTF-8 locale..."
  sudo apt update && sudo apt install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
fi

# Check if ROS 2 is already installed
if dpkg -l | grep -q "ros-$ROS_DISTRO-desktop"; then
  warn "ROS 2 $ROS_DISTRO appears to be already installed. Some steps may be skipped."
fi

# Install prerequisites
header "Installing prerequisites"
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y

# Setup sources for ROS 2 using manual GPG key method
header "Setting up ROS 2 repositories"

# Add the ROS 2 GPG key
info "Adding ROS 2 GPG key..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository
info "Adding ROS 2 repository to sources list..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists
sudo apt update

# Install development tools and dependencies
header "Installing development tools and dependencies"
sudo apt update && sudo apt install -y \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

# Install additional development tools
sudo apt install -y \
  libbullet-dev \
  python3-vcstool

# For Jetson Nano, install additional CUDA-related dependencies
if $IS_JETSON; then
  header "Installing Jetson-specific dependencies"
  
  # Check if CUDA is already installed
  if [ -d "/usr/local/cuda" ]; then
    info "CUDA appears to be already installed at /usr/local/cuda"
  else
    warn "CUDA not found. Please install NVIDIA JetPack for proper functionality."
  fi

  # Install additional packages for Jetson
  sudo apt install -y \
    build-essential \
    cmake \
    python3-numpy \
    python3-opencv
    
  # Set environment variables
  export CUDA_HOME=/usr/local/cuda
  info "Setting CUDA_HOME environment variable to $CUDA_HOME"
fi

# Ask user before proceeding with source build
read -p "Do you want to build ROS 2 from source? This will take significant time and disk space. (y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
  # Setup workspace for building ROS 2 from source
  WORKSPACE_PATH=~/ros2_${ROS_DISTRO}
  header "Building ROS 2 ${ROS_DISTRO} from source"
  info "Creating workspace at $WORKSPACE_PATH..."
  
  mkdir -p $WORKSPACE_PATH/src
  cd $WORKSPACE_PATH
  
  info "Downloading ROS 2 $ROS_DISTRO source code..."
  vcs import --input https://raw.githubusercontent.com/ros2/ros2/$ROS_DISTRO/ros2.repos src

  # Update system and install dependencies
  info "Updating system packages..."
  sudo apt upgrade -y

  # Initialize rosdep if not already done
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    info "Initializing rosdep..."
    sudo rosdep init
  fi
  
  info "Updating rosdep..."
  rosdep update
  
  # Set skip keys based on ROS 2 version
  if [[ "$ROS_DISTRO" == "foxy" ]]; then
    SKIP_KEYS="fastcdr rti-connext-dds-5.3.1 urdfdom_headers"
  else
    SKIP_KEYS="fastcdr rti-connext-dds-7.3.0 urdfdom_headers"
  fi
  
  info "Installing ROS 2 dependencies with rosdep..."
  rosdep install --from-paths src --ignore-src -y --skip-keys "$SKIP_KEYS"

  # For Jetson Nano, use a more conservative build configuration
  BUILD_ARGS="--symlink-install"
  if $IS_JETSON; then
    warn "Building on Jetson may take a very long time and could overheat the device."
    warn "Consider using cooling solutions and/or build only essential packages."
    info "Using memory-efficient build options for Jetson..."
    BUILD_ARGS="--symlink-install --parallel 2 --executor sequential"
  fi

  # Build the code
  info "Building ROS 2 from source (this may take a while)..."
  colcon build $BUILD_ARGS

  info "ROS 2 ${ROS_DISTRO} built successfully!"
  info "To use ROS 2, source the setup file in each terminal:"
  info "  source $WORKSPACE_PATH/install/setup.bash"
  
  # Offer to add sourcing to .bashrc
  read -p "Would you like to add ROS 2 sourcing to your .bashrc file? (y/N) " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "source $WORKSPACE_PATH/install/setup.bash" >> ~/.bashrc
    info "Added ROS 2 sourcing to your .bashrc file"
  fi
else
  # Install ROS 2 from binary packages
  header "Installing ROS 2 ${ROS_DISTRO} from binary packages"

  if $IS_JETSON; then
    # For Jetson, prefer base packages as desktop may be too heavy
    info "Installing base packages for Jetson (no GUI tools)..."
    sudo apt install -y ros-$ROS_DISTRO-ros-base
    
    # Offer to install desktop packages with warning
    read -p "Would you like to install desktop GUI packages? This may impact performance on Jetson. (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
      info "Installing desktop packages (may take a while)..."
      sudo apt install -y ros-$ROS_DISTRO-desktop
    fi
  else
    # For regular systems, install full desktop
    sudo apt install -y ros-$ROS_DISTRO-desktop
  fi

  info "ROS 2 $ROS_DISTRO installation complete!"
  info "To use ROS 2, source the setup file in each terminal:"
  info "  source /opt/ros/$ROS_DISTRO/setup.bash"
  
  # Offer to add ROS sourcing to .bashrc
  read -p "Would you like to add ROS 2 sourcing to your .bashrc file? (y/N) " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    info "Added ROS 2 sourcing to your .bashrc file"
  fi
fi

# Install additional useful ROS 2 packages
if ! $IS_JETSON || [[ $REPLY =~ ^[Yy]$ ]]; then  # Only ask for regular systems or Jetsons that opted for desktop
  read -p "Do you want to install additional commonly used ROS 2 packages? (y/N) " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    header "Installing additional ROS 2 packages"
    
    ADDITIONAL_PKGS="ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-robot-state-publisher"
    
    # Only install GUI tools if not on Jetson or if user specifically chose to install desktop
    if ! $IS_JETSON || [[ $REPLY =~ ^[Yy]$ ]]; then
      ADDITIONAL_PKGS="$ADDITIONAL_PKGS ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-rqt ros-$ROS_DISTRO-rqt-common-plugins"
    fi
    
    # Add gazebo based on ROS distribution
    ADDITIONAL_PKGS="$ADDITIONAL_PKGS ros-$ROS_DISTRO-gazebo-ros-pkgs"
    
    sudo apt install -y $ADDITIONAL_PKGS
  fi
fi

# Jetson-specific final tips
if $IS_JETSON; then
  header "Jetson-Specific Configuration Tips"
  info "1. Make sure to regularly monitor temperature to prevent throttling"
  info "2. Consider enabling JetsonHacks' Jetson Stats: sudo pip3 install jetson-stats"
  info "3. For camera usage, install: sudo apt install ros-$ROS_DISTRO-v4l2-camera"
  info "4. For improved performance, minimize GUI usage when running ROS applications"
fi

header "ROS 2 ${ROS_DISTRO} setup completed successfully!"
info "Distribution: ROS 2 ${ROS_DISTRO}"
if [[ "$ROS_DISTRO" == "humble" ]]; then
  info "Support status: Long-Term Support (LTS) until May 2027"
elif [[ "$ROS_DISTRO" == "jazzy" ]]; then
  info "Support status: Regular release, supported until November 2025"
elif [[ "$ROS_DISTRO" == "kilted" ]]; then
  info "Support status: Regular release, supported until November 2026"
elif [[ "$ROS_DISTRO" == "foxy" ]]; then
  info "Support status: EOL since May 2023, but commonly used on Jetson"
fi
info "For tutorials, visit: https://docs.ros.org/en/${ROS_DISTRO}/Tutorials.html"