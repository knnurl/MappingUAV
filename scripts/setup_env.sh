#!/usr/bin/env bash
# filepath: scripts/setup_env.sh
# One-shot environment provisioning for Gates 1-4. Idempotent where practical.
# Run as the normal user, not root.
#
# ADAPTED from the handover spec on 2026-07-04 (all deviations reported):
# - ROS 2 Humble was NOT preinstalled on this Jetson (spec assumed it was).
#   Section 0b installs ros-humble-desktop per operator decision.
# - Wifi interface on this Jetson is wlP1p1s0, not wlan0 (cyclonedds.xml).
# - ~/.colcon/defaults.yaml enforces parallel-workers 2 + symlink-install for
#   EVERY colcon call, including the one inside livox_ros_driver2/build.sh.
set -euo pipefail

WS=~/colcon_ws
mkdir -p "$WS/src" "$WS/scripts"

# ---------------------------------------------------------------------------
# 0. System sanity
# ---------------------------------------------------------------------------
if [ "$(lsb_release -rs)" != "22.04" ]; then
  echo "FATAL: expected Ubuntu 22.04"; exit 1
fi

# ---------------------------------------------------------------------------
# 0b. ROS 2 Humble (desktop) — added because /opt/ros was empty on this unit
# ---------------------------------------------------------------------------
if [ ! -f /opt/ros/humble/setup.bash ]; then
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl gnupg lsb-release
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt-get update
  sudo apt-get install -y ros-humble-desktop ros-dev-tools
fi
source /opt/ros/humble/setup.bash

sudo apt-get update
sudo apt-get install -y \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  libpcl-dev libeigen3-dev \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep \
  can-utils net-tools linux-tools-generic

# ---------------------------------------------------------------------------
# 1. Middleware: CycloneDDS. rmw_zenoh is intentionally NOT used, see spec header.
# ---------------------------------------------------------------------------
# Kernel UDP buffers for CycloneDDS under point cloud load (persist across boot):
sudo tee /etc/sysctl.d/60-cyclonedds.conf > /dev/null <<'EOF'
net.core.rmem_max=8388608
net.core.rmem_default=8388608
EOF
sudo sysctl --system > /dev/null

# Shell environment (append once):
if ! grep -q "RMW_IMPLEMENTATION" ~/.bashrc; then
cat >> ~/.bashrc <<'EOF'
# --- drone stack environment ---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/colcon_ws/src/drone_bringup/config/cyclonedds.xml
export ROS_DOMAIN_ID=42
export MAKEFLAGS="-j4"
source /opt/ros/humble/setup.bash
[ -f $HOME/colcon_ws/install/setup.bash ] && source $HOME/colcon_ws/install/setup.bash
EOF
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export MAKEFLAGS="-j4"

# ---------------------------------------------------------------------------
# 2. Micro XRCE-DDS Agent v2.x (pairs with Humble-era FastDDS 2.x and default
#    PX4 client build). Firmware confirmed v1.15.x stock: agent v2.x is correct.
# ---------------------------------------------------------------------------
if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  [ -d /tmp/xrce-agent ] || git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce-agent
  cmake -S /tmp/xrce-agent -B /tmp/xrce-agent/build
  cmake --build /tmp/xrce-agent/build -j4
  sudo cmake --install /tmp/xrce-agent/build
  sudo ldconfig /usr/local/lib/
fi

# ---------------------------------------------------------------------------
# 3. Livox SDK2 (system-wide, required by livox_ros_driver2)
# ---------------------------------------------------------------------------
if [ ! -f /usr/local/lib/liblivox_lidar_sdk_shared.so ] && [ ! -f /usr/local/lib/liblivox_lidar_sdk_static.a ]; then
  [ -d /tmp/livox-sdk2 ] || git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/livox-sdk2
  cmake -S /tmp/livox-sdk2 -B /tmp/livox-sdk2/build
  cmake --build /tmp/livox-sdk2/build -j4
  sudo cmake --install /tmp/livox-sdk2/build
  sudo ldconfig
fi

# ---------------------------------------------------------------------------
# 4. Source repos (idempotent; already cloned 2026-07-04)
# ---------------------------------------------------------------------------
cd "$WS/src"

# px4_msgs: branch MUST match flashed firmware. Firmware confirmed v1.15.x via QGC.
PX4_MSGS_BRANCH="release/1.15"
[ -d px4_msgs ] || git clone -b "$PX4_MSGS_BRANCH" https://github.com/PX4/px4_msgs.git

# Livox ROS 2 driver:
[ -d livox_ros_driver2 ] || git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# FAST-LIO2, official ROS2 branch:
[ -d FAST_LIO ] || git clone -b ROS2 --recursive https://github.com/hku-mars/FAST_LIO.git

# ---------------------------------------------------------------------------
# 5. Build. livox_ros_driver2's build.sh generates its ROS2 package.xml and
#    runs colcon on the whole workspace; ~/.colcon/defaults.yaml caps its
#    parallelism (spec rule 2). Run it ONCE, then normal colcon builds work.
# ---------------------------------------------------------------------------
if [ ! -f "$WS/src/livox_ros_driver2/package.xml" ]; then
  cd "$WS/src/livox_ros_driver2"
  ./build.sh humble
fi

cd "$WS"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init || true
rosdep update || true
rosdep install --from-paths src --ignore-src -y || true
colcon build --symlink-install --parallel-workers 2

# ---------------------------------------------------------------------------
# 6. Serial access + performance mode
# ---------------------------------------------------------------------------
sudo usermod -aG dialout "$USER"
# Disable the Jetson serial getty that squats on /dev/ttyTHS1:
sudo systemctl stop nvgetty 2>/dev/null || true
sudo systemctl disable nvgetty 2>/dev/null || true
# Max clocks (re-run after reboot or add to a systemd unit later):
sudo nvpmodel -m 0 || true
sudo jetson_clocks || true

# zram swap headroom for builds (Jetson usually ships zram enabled; ensure it):
sudo systemctl enable nvzramconfig 2>/dev/null || true

echo "setup_env.sh complete. LOG OUT AND BACK IN for dialout group to apply."
