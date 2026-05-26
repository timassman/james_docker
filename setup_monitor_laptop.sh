#!/bin/bash
# Sets up the laptop as a monitoring station for the James robot.
# Configures ROS2 and Fast DDS so the laptop can see topics published
# by the Docker container on the Jetson, even when docker0 is present.
# Safe to run multiple times.
#
# What this does:
#   1. Sources ROS2 Humble automatically in every new terminal
#   2. Configures Fast DDS to use only LAN interfaces (excludes docker0)
#      IP is detected dynamically, so this survives IP changes and reboots.
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

# Configure Fast DDS to use only real LAN interfaces.
# Excludes docker0 and other virtual interfaces whose multicast traffic
# cannot reach the Jetson, which breaks ROS2 topic discovery.
# Runs dynamically so a new IP after DHCP renewal is picked up automatically.
_ros2_configure_dds() {
    # Collect all LAN IPs: exclude loopback, docker (172.x), bridge interfaces
    local ips
    ips=$(ip -4 addr show | grep 'inet ' \
        | grep -v '127\.0\.0\.1\|172\.\|docker\|br-\|virbr' \
        | awk '{print $2}' | cut -d/ -f1)

    [ -z "$ips" ] && return  # no LAN interface found, skip

    local whitelist=""
    for ip in $ips; do
        whitelist+="                <address>$ip</address>\n"
    done

    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_ros2.xml
    printf '<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>lan</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
%s            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="CustomParticipantProfile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>lan</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>\n' "$whitelist" > /tmp/fastdds_ros2.xml
}
_ros2_configure_dds
# === end james robot ===
BASHRC_BLOCK

echo "[INFO] Added ROS2 + Fast DDS config to ~/.bashrc"

# ── Apply to current shell ────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
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
