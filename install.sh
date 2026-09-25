#!/bin/bash

set -e

echo "=========================================="
echo " Ubuntu 24.04 + ROS 2 Jazzy + Gazebo"
echo " + VRX Installation"
echo "=========================================="

# ============================================================
# 1. Ubuntu preparation
# ============================================================

echo "[1/7] Preparing Ubuntu..."

sudo apt update
sudo apt upgrade -y

sudo apt install -y \
    curl \
    wget \
    git \
    gnupg \
    lsb-release \
    software-properties-common \
    build-essential

sudo add-apt-repository universe -y
sudo apt update


# ============================================================
# 2. Install ROS 2 Jazzy
# ============================================================

echo "[2/7] Installing ROS 2 Jazzy..."

sudo apt install -y curl software-properties-common

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F "tag_name" \
    | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb \
"https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update

sudo apt install -y \
    ros-jazzy-desktop \
    ros-dev-tools

sudo apt install -y python3-rosdep

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update


# ============================================================
# 3. Install Gazebo Harmonic
# ============================================================

echo "[3/7] Installing Gazebo Harmonic..."

sudo apt install -y \
    lsb-release \
    curl \
    gnupg

sudo curl \
    https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update

sudo apt install -y gz-harmonic


# ============================================================
# 4. ROS 2 <-> Gazebo integration
# ============================================================

echo "[4/7] Installing ROS-Gazebo integration..."

sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-xacro \
    python3-sdformat14


# ============================================================
# 5. Create VRX workspace
# ============================================================

echo "[5/7] Creating VRX workspace..."

mkdir -p ~/vrx_ws/src

cd ~/vrx_ws/src

if [ ! -d "vrx" ]; then
    git clone https://github.com/osrf/vrx.git
else
    echo "VRX repository already exists, skipping clone."
fi


# ============================================================
# 6. Install VRX dependencies and build
# ============================================================

echo "[6/7] Building VRX..."

cd ~/vrx_ws

# Temporarily source ROS so rosdep and colcon can find it
source /opt/ros/jazzy/setup.bash

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y

colcon build --merge-install


# ============================================================
# 7. Automatically source ROS + VRX
# ============================================================

echo "[7/7] Configuring ~/.bashrc..."

# Remove old automatically-added lines if they exist
sed -i '/# ROS 2 Jazzy/d' ~/.bashrc
sed -i '/# VRX workspace/d' ~/.bashrc
sed -i 's|source /opt/ros/jazzy/setup.bash||g' ~/.bashrc
sed -i 's|source ~/vrx_ws/install/setup.bash||g' ~/.bashrc

cat >> ~/.bashrc <<'EOF'

# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# VRX workspace
source ~/vrx_ws/install/setup.bash
EOF


# ============================================================
# Finish
# ============================================================

echo ""
echo "=========================================="
echo " INSTALLATION COMPLETE"
echo "=========================================="
echo ""

echo "ROS 2:"
ros2 --version || true

echo ""
echo "Gazebo:"
gz sim --versions || true

echo ""
echo "VRX packages:"
source ~/vrx_ws/install/setup.bash
ros2 pkg list | grep vrx | head -20 || true

echo ""
echo "=========================================="
echo " ROS 2 and VRX were added to ~/.bashrc"
echo " You no longer need to source them manually."
echo "=========================================="
echo ""

echo "Restart your terminal or run:"
echo ""
echo "    source ~/.bashrc"
echo ""
