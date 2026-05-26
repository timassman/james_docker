#!/bin/bash
# Sets up the Jetson Orin Nano (JetPack 6.x / Ubuntu 22.04) as the host for
# running the robojames Docker container.
# Safe to run multiple times — steps check before acting where possible.
#
# What this sets up:
#   1. Xbox controller (xpad kernel module)
#   2. Docker Engine
#   3. NVIDIA Container Toolkit  -> GPU / CUDA access from Docker
#   4. udev rules                -> USB access for OAK-D, Xbox, Dynamixel
#   5. VSCode (optional, for debugging on the Jetson itself)
#   6. Persistent boot logging
#
# After setup, load the Docker image built on the laptop:
#   On laptop:  docker save robojames-jetson | gzip > robojames-jetson.tar.gz
#               scp robojames-jetson.tar.gz <user>@<jetson-ip>:~/
#   On Jetson:  docker load < robojames-jetson.tar.gz
#   Run:        docker run --runtime=nvidia --gpus all --rm -it robojames-jetson bash

set -euo pipefail

[[ "$(uname -m)" == "aarch64" ]] || { echo "Run this script on the Jetson, not the laptop."; exit 1; }

# Always resolve paths relative to the script location, not the working directory.
# This allows running the script from any directory: ./setup_jetson.sh or ~/james_docker/setup_jetson.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ********************************************************
# * Swap file (prevents OOM on docker load / heavy tasks)*
# ********************************************************

SWAP_TARGET_GB=8
SWAP_TARGET_BYTES=$(( SWAP_TARGET_GB * 1024 * 1024 * 1024 ))

if [ -f /swapfile ]; then
    SWAP_ACTUAL=$(stat -c%s /swapfile)
    if [ "$SWAP_ACTUAL" -lt "$SWAP_TARGET_BYTES" ]; then
        echo "[INFO] Swap too small, recreating as ${SWAP_TARGET_GB}GB..."
        sudo swapoff /swapfile 2>/dev/null || true
        sudo rm /swapfile
        sudo fallocate -l "${SWAP_TARGET_GB}G" /swapfile
        sudo chmod 600 /swapfile
        sudo mkswap /swapfile
    else
        echo "[INFO] Swap file already ${SWAP_TARGET_GB}GB or larger."
    fi
else
    echo "[INFO] Creating ${SWAP_TARGET_GB}GB swap file..."
    sudo fallocate -l "${SWAP_TARGET_GB}G" /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
fi

if ! swapon --show | grep -q /swapfile; then
    sudo swapon /swapfile
fi

if ! grep -q '/swapfile' /etc/fstab; then
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab > /dev/null
fi

# ********************************************************
# * Xbox controller (xpad kernel module)                 *
# ********************************************************

# xpad is the standard Linux kernel driver for Xbox controllers, included in
# the stock JetPack kernel. No userspace driver, PPA, or kernel rebuild needed.
#
# Note: the Xbox 360 controller LED will keep blinking — this is normal. The
# controller expects a proprietary sync packet that only a real Xbox console sends.
# It has no effect on functionality.

# Remove xpad blacklist if it was set by a previous xboxdrv installation
sudo sed -i '/blacklist xpad/d' /etc/modprobe.d/blacklist.conf 2>/dev/null || true

# Load xpad now
sudo modprobe xpad

# Persist across reboots
if ! grep -q "^xpad$" /etc/modules; then
    echo "xpad" | sudo tee -a /etc/modules
fi

# Install jstest-gtk for controller testing
sudo apt-get install -y jstest-gtk

# ********************************************************
# * Docker Engine                                        *
# ********************************************************

# https://docs.docker.com/engine/install/ubuntu/

if command -v docker &>/dev/null; then
    echo "[INFO] Docker already installed: $(docker --version)"
else
    echo "[INFO] Installing Docker Engine..."

    for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do
        sudo apt-get remove -y "$pkg" 2>/dev/null || true
    done

    sudo apt-get update -qq
    sudo apt-get install -y ca-certificates curl

    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
        | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

    sudo apt-get update -qq
    sudo apt-get install -y \
        docker-ce docker-ce-cli containerd.io \
        docker-buildx-plugin docker-compose-plugin
fi

sudo groupadd docker 2>/dev/null || true
sudo usermod -aG docker "$USER"
# dialout: needed for Roomba serial port (/dev/ttyUSB0) and Dynamixel
sudo usermod -aG dialout "$USER"
sudo systemctl enable docker
sudo systemctl start docker

# ********************************************************
# * NVIDIA Container Toolkit (GPU / CUDA access)         *
# ********************************************************

# Required to use --runtime=nvidia --gpus all when running Docker containers.
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

if dpkg -s nvidia-container-toolkit &>/dev/null 2>&1; then
    echo "[INFO] NVIDIA Container Toolkit already installed."
else
    echo "[INFO] Installing NVIDIA Container Toolkit..."

    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
        | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
        | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
        | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null

    sudo apt-get update -qq
    sudo apt-get install -y nvidia-container-toolkit
fi

if ! grep -q '"nvidia"' /etc/docker/daemon.json 2>/dev/null; then
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
else
    echo "[INFO] NVIDIA Docker runtime already configured."
fi

# ********************************************************
# * udev rules                                           *
# ********************************************************

# Xbox controller — creates /dev/input/xbox-controller symlink used by run.sh
# Matches on Microsoft USB vendor ID (045e) so it works for any Xbox controller.
# The js* kernel filter ensures only the joystick node gets the symlink, not the
# separate event nodes that xpad also creates.
if ! grep -q "xbox-controller" /etc/udev/rules.d/10-xbox-controller.rules 2>/dev/null; then
    echo 'SUBSYSTEM=="input", ATTRS{idVendor}=="045e", KERNEL=="js*", MODE="0666", SYMLINK+="input/xbox-controller"' \
        | sudo tee /etc/udev/rules.d/10-xbox-controller.rules > /dev/null
fi

# OAK-D cameras (Luxonis vendor ID: 03e7 — covers both OAK-D Pro and OAK-D Lite)
#
# IMPORTANT — Jetson Orin Nano USB port warning:
# The four USB-A ports on the case are labelled "USB 3.0" but are physically wired
# to the USB 2.0 controller (480 Mbps max). This is a hardware limitation of the
# carrier board, not a cable or driver issue.
# Only the USB-C port provides real USB 3.0 (5 Gbps).
#
# - OAK-D Pro:  connect to the USB-C port (directly or via USB 3.0 hub).
#               Needs USB 3.0 for stereo depth + RGB at full resolution (~1-3 Gbps).
# - OAK-D Lite: USB 2.0 only by design — any USB-A port is fine.
if ! grep -q "03e7" /etc/udev/rules.d/80-movidius.rules 2>/dev/null; then
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
        | sudo tee /etc/udev/rules.d/80-movidius.rules > /dev/null
fi

# Roomba serial adapter — creates /dev/roomba symlink used by run.sh
# FTDI FT232 chip, serial number AC00MLAL (identified via udevadm info -a /dev/ttyUSB*)
if ! grep -q "AC00MLAL" /etc/udev/rules.d/10-roomba.rules 2>/dev/null; then
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{serial}=="AC00MLAL", SYMLINK+="roomba", MODE="0666"' \
        | sudo tee /etc/udev/rules.d/10-roomba.rules > /dev/null
fi

# OpenManipulator serial adapter — creates /dev/openmanipulator symlink used by run.sh
# FTDI FT232H chip, serial number FT7928TQ (identified via udevadm info -a /dev/ttyUSB*)
if ! grep -q "FT7928TQ" /etc/udev/rules.d/10-openmanipulator.rules 2>/dev/null; then
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{serial}=="FT7928TQ", SYMLINK+="openmanipulator", MODE="0666"' \
        | sudo tee /etc/udev/rules.d/10-openmanipulator.rules > /dev/null
fi

# Dynamixel USB serial: reduce latency from 16ms to 1ms, prevent ModemManager from capturing it
if ! grep -q "ftdi_sio" /etc/udev/rules.d/99-open-manipulator-cdc.rules 2>/dev/null; then
    echo 'KERNEL=="ttyUSB*", DRIVERS=="ftdi_sio", MODE="0666", ATTR{device/latency_timer}="1"' \
        | sudo tee /etc/udev/rules.d/99-open-manipulator-cdc.rules > /dev/null
fi

# USB autosuspend off (prevents USB devices from going to sleep)
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend > /dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger

# ********************************************************
# * VSCode (optional — useful for debugging on Jetson)   *
# ********************************************************

if ! command -v code &>/dev/null; then
    sudo apt-get install -y wget gpg apt-transport-https
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
        | gpg --dearmor > packages.microsoft.gpg
    sudo install -D -o root -g root -m 644 packages.microsoft.gpg \
        /etc/apt/keyrings/packages.microsoft.gpg
    rm -f packages.microsoft.gpg
    sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] \
https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
    sudo apt-get update -qq
    sudo apt-get install -y code
fi

# ********************************************************
# * Jetson performance mode                              *
# ********************************************************

# nvpmodel -m 0: maximum power/performance mode (all CPU+GPU cores, max frequency)
# jetson_clocks: lock clocks to maximum (prevents thermal throttling during operation)
# Both need to be re-applied after every reboot, so we install a systemd service.

sudo nvpmodel -m 0 2>/dev/null || echo "[WARN] nvpmodel not available (normal if not on Jetson hardware)"
sudo jetson_clocks 2>/dev/null || echo "[WARN] jetson_clocks not available"

if [ ! -f /etc/systemd/system/jetson-performance.service ]; then
    sudo tee /etc/systemd/system/jetson-performance.service > /dev/null << 'EOF'
[Unit]
Description=Set Jetson to maximum performance mode
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/nvpmodel -m 0
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl enable jetson-performance.service
    echo "[INFO] Jetson performance service installed (survives reboot)."
else
    echo "[INFO] Jetson performance service already installed."
fi

# ********************************************************
# * ROS2 DDS network tuning                             *
# ********************************************************

# ROS2 uses UDP multicast with large buffers for fast topic publishing (e.g. camera).
# Without this, messages can be dropped on high-bandwidth topics.
# https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html

ROS2_SYSCTL=/etc/sysctl.d/10-ros2-dds.conf
if [ ! -f "$ROS2_SYSCTL" ]; then
    sudo tee "$ROS2_SYSCTL" > /dev/null << 'EOF'
# ROS2 DDS tuning — prevents message loss on high-bandwidth topics (camera, pointcloud)
net.core.rmem_max=8388608
net.core.rmem_default=8388608
net.core.wmem_max=8388608
net.core.wmem_default=8388608
EOF
    sudo sysctl --system -q
    echo "[INFO] ROS2 DDS network tuning applied."
else
    echo "[INFO] ROS2 DDS tuning already configured."
fi

# ********************************************************
# * SSH server (for VS Code Remote SSH and scp)          *
# ********************************************************

if dpkg -s openssh-server &>/dev/null 2>&1; then
    echo "[INFO] SSH server already installed."
else
    sudo apt-get install -y openssh-server
    echo "[INFO] SSH server installed."
fi
sudo systemctl enable ssh
sudo systemctl start ssh 2>/dev/null || true

# Set hostname to jetson-orin if not already set
DESIRED_HOSTNAME="jetson-orin"
if [ "$(hostname)" != "$DESIRED_HOSTNAME" ]; then
    sudo hostnamectl set-hostname "$DESIRED_HOSTNAME"
    echo "[INFO] Hostname set to $DESIRED_HOSTNAME."
    echo "[WARN] Reboot required for hostname change to fully take effect."
else
    echo "[INFO] Hostname already set to $DESIRED_HOSTNAME."
fi

# ********************************************************
# * Other                                                *
# ********************************************************

sudo apt-get install -y nano

# Persist logs across reboots (useful for debugging crashes)
sudo mkdir -p /var/log/journal
if ! grep -q "Storage=persistent" /etc/systemd/journald.conf; then
    echo "Storage=persistent" | sudo tee -a /etc/systemd/journald.conf > /dev/null
fi

# ********************************************************
# * Done                                                 *
# ********************************************************

echo ""
echo "[INFO] Setup complete!"
echo ""
echo "Next steps:"
echo "  1. Log out and back in (docker + dialout group take effect)"
echo "     (or run: newgrp docker)"
echo ""
echo "  2. Add your laptop's SSH key to the Jetson (run on laptop):"
echo "       ssh-copy-id $(whoami)@jetson-orin"
echo ""
echo "  3. VS Code SSH config on laptop (~/.ssh/config):"
echo "       Host jetson-orin"
echo "         HostName jetson-orin"
echo "         User $(whoami)"
echo "         ForwardAgent yes"
echo ""
echo "  4. Transfer the Docker image from the laptop:"
echo "       On laptop:  docker save robojames-jetson | gzip > robojames-jetson.tar.gz"
echo "                   scp robojames-jetson.tar.gz $(whoami)@jetson-orin:~/"
echo "       On Jetson:  docker load < robojames-jetson.tar.gz"
echo ""
echo "  5. Run the container with GPU access:"
echo "       docker run --runtime=nvidia --gpus all --rm -it robojames-jetson bash"
echo "     Or with docker-compose:"
echo "       docker-compose up"
