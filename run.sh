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
# Devices (all optional — skipped with a warning if not connected):
#   /dev/roomba               — Roomba serial (symlink set by udev rule)
#   /dev/openmanipulator      — OpenManipulator Dynamixel (symlink set by udev rule)
#   /dev/input/xbox-controller — Xbox controller (symlink set by udev rule)
#   /dev/bus/usb              — OAK-D cameras (USB, dynamic cgroup rule)
#
# OAK-D USB port warning:
#   The four USB-A ports on the Jetson case are labelled "USB 3.0" but are physically
#   wired to the USB 2.0 controller (480 Mbps). Only the USB-C port is real USB 3.0.
#   Connect the OAK-D Pro to the USB-C port (directly or via USB 3.0 hub).
#   The OAK-D Lite is USB 2.0 only by design — any port works.
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

# Use Fast DDS LARGE_DATA mode: TCP for data, UDP for discovery.
# Required for reliable delivery of large topics (point clouds, depth images) to the
# monitoring laptop. Works correctly alongside docker0 and multiple network interfaces.
# Note: WiFi is not reliable enough for these message sizes — laptop needs a wired LAN
# connection to receive point clouds. The robot itself can run on WiFi autonomously.
FASTDDS_MOUNT="-e FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA"
echo "[INFO] Fast DDS LARGE_DATA mode enabled (TCP transport for large messages)."

echo "[INFO] Starting $IMAGE..."

# Resolve serial devices (optional — warn if not connected)
ROOMBA_DEVICE=""
if [ -e /dev/roomba ]; then
    ROOMBA_DEVICE="--device=/dev/roomba"
else
    echo "[WARN] Roomba not found at /dev/roomba — skipping."
fi

ARM_DEVICE=""
if [ -e /dev/openmanipulator ]; then
    ARM_DEVICE="--device=/dev/openmanipulator"
else
    echo "[WARN] OpenManipulator not found at /dev/openmanipulator — skipping."
fi

docker run -it \
    --name "$CONTAINER" \
    --runtime=nvidia \
    --gpus all \
    $ROOMBA_DEVICE \
    $ARM_DEVICE \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    $XBOX_DEVICE \
    $FASTDDS_MOUNT \
    --restart "$RESTART_POLICY" \
    --net=host \
    --pid=host \
    "$IMAGE"
    # Uncomment to launch ROS2 directly instead of dropping into a shell:
    # bash -c "ros2 launch james_bringup james.launch.py"
