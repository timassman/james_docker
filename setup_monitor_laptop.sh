#!/bin/bash
# Sets up the laptop as a monitoring station for the James robot.
# Configures ROS2 and Fast DDS so the laptop can see topics published
# by the Docker container on the Jetson.
# Safe to run multiple times.
#
# What this does:
#   1. Sources ROS2 Humble automatically in every new terminal
#   2. Enables Fast DDS LARGE_DATA mode (TCP transport for large messages)
#      Fixes message loss for large topics (images, point clouds) over LAN.
#   3. Sets ROS_DOMAIN_ID=0 (must match the Jetson container)

set -euo pipefail

BASHRC="$HOME/.bashrc"
MARKER="# === james robot ROS2 setup ==="

# ── Check ROS2 is installed ───────────────────────────────────────────────────
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[ERROR] ROS2 Humble not found at /opt/ros/humble/setup.bash"
    echo "        Install it from: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi
echo "[INFO] ROS2 Humble found."

# ── Add to ~/.bashrc (idempotent) ─────────────────────────────────────────────
if grep -q "$MARKER" "$BASHRC" 2>/dev/null; then
    echo "[INFO] ROS2 setup already present in ~/.bashrc — updating..."
    # Remove existing block and re-add
    sed -i "/$MARKER/,/# === end james robot ===/d" "$BASHRC"
fi

cat >> "$BASHRC" << 'BASHRC_BLOCK'
# === james robot ROS2 setup ===
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# Fast DDS LARGE_DATA mode: uses TCP for large messages, UDP for discovery.
# Required for reliable delivery of large topics (point clouds, depth images) over LAN.
# Note: WiFi is not reliable enough for these message sizes — use a wired connection.
# Must match the setting in run.sh on the Jetson.
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
# === end james robot ===
BASHRC_BLOCK

echo "[INFO] Added ROS2 + Fast DDS config to ~/.bashrc"

# ── ROS2 DDS network tuning ──────────────────────────────────────────────────
# Increases UDP socket buffers for DDS discovery traffic.
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

# ── Apply to current shell ────────────────────────────────────────────────────
# Temporarily disable unbound variable check — ROS2 setup.bash references
# AMENT_TRACE_SETUP_FILES which may not be set in the current shell context.
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=0

echo ""
echo "[INFO] Setup complete!"
echo ""
echo "Reload your terminal or run:"
echo "  source ~/.bashrc"
echo ""
echo "Then test with:"
echo "  ros2 topic list    # should show Jetson topics when robot is running"
echo "  rviz2              # start RViz2 for visualization"
