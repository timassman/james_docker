#!/bin/bash
# Creates and starts the robojames Docker container on the Jetson Orin Nano.
# Safe to run from any directory.
#
# When to run this script:
#   - First time, to create the container from the image
#   - After deploying a new image with deploy_to_jetson.sh
#
# When NOT needed:
#   - After a reboot — Docker restarts the container automatically
#     (--restart unless-stopped + Docker enabled as systemd service)
#
# Devices:
#   /dev/ttyUSB0              — Roomba serial
#   /dev/ttyUSB1              — OpenManipulator (Dynamixel)
#   /dev/input/xbox-controller — Xbox controller (symlink set by udev rule)
#   /dev/bus/usb              — OAK-D Pro camera (USB)
#
# --restart unless-stopped   — auto-starts on Jetson reboot
# --net=host --pid=host      — needed for ROS2 topic communication across hosts
# --runtime=nvidia --gpus all — enables CUDA / Jetson GPU access

set -euo pipefail

IMAGE="robojames-jetson"
CONTAINER="robojames"

# Restart policy:
#   on-failure:3     — restart max 3 times on crash, then stop (safe during development)
#   unless-stopped   — always restart, survives reboot (use when stable in production)
RESTART_POLICY="on-failure:3"

# Remove existing stopped container with the same name so we can start fresh
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "[INFO] Removing existing container '$CONTAINER'..."
    docker rm -f "$CONTAINER"
fi

# Resolve xbox controller device (udev creates a symlink, docker needs the real path)
XBOX_DEVICE=""
if [ -e /dev/input/xbox-controller ]; then
    XBOX_DEVICE="--device=$(readlink -f /dev/input/xbox-controller)"
else
    echo "[WARN] Xbox controller not found at /dev/input/xbox-controller — skipping."
fi

echo "[INFO] Starting $IMAGE..."

docker run -it \
    --name "$CONTAINER" \
    --runtime=nvidia \
    --gpus all \
    --device=/dev/ttyUSB0 \
    --device=/dev/ttyUSB1 \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    $XBOX_DEVICE \
    --restart "$RESTART_POLICY" \
    --net=host \
    --pid=host \
    "$IMAGE"
    # Uncomment to launch ROS2 directly instead of dropping into a shell:
    # bash -c "ros2 launch james_bringup james.launch.py"
