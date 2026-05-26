#!/bin/bash
# Smoke tests for the robojames-jetson Docker container.
# Run from the Jetson host:
#   docker exec -it $(docker ps -q -f ancestor=robojames-jetson) bash /tmp/test_container.sh
#
# Or copy into a running container and run:
#   docker cp test_container.sh <container_id>:/tmp/
#   docker exec -it <container_id> bash /tmp/test_container.sh

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0
WARN=0

pass() { echo -e "${GREEN}[PASS]${NC} $1"; (( PASS++ )); }
fail() { echo -e "${RED}[FAIL]${NC} $1"; (( FAIL++ )); }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; (( WARN++ )); }
section() { echo ""; echo "── $1 ──────────────────────────────────────"; }

echo "========================================"
echo " robojames-jetson container smoke tests"
echo "========================================"

# ── GPU / CUDA ────────────────────────────────────────────────────────────────
section "GPU / CUDA"

if nvidia-smi &>/dev/null; then
    GPU=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    pass "nvidia-smi: $GPU"
else
    fail "nvidia-smi not available — container started without --runtime=nvidia?"
fi

if python3 -c "import ctypes; ctypes.cdll.LoadLibrary('libcuda.so.1')" &>/dev/null; then
    pass "libcuda.so.1 loadable"
else
    warn "libcuda.so.1 not loadable (may be fine if GPU libs are mounted at runtime)"
fi

# ── ROS2 ─────────────────────────────────────────────────────────────────────
section "ROS2 environment"

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null
    pass "ROS2 Humble sourced"
else
    fail "/opt/ros/humble/setup.bash not found"
fi

if command -v ros2 &>/dev/null; then
    pass "ros2 CLI available"
else
    fail "ros2 CLI not found"
fi

if [ -f /home/james/ros2_ws/install/local_setup.bash ]; then
    source /home/james/ros2_ws/install/local_setup.bash 2>/dev/null
    pass "workspace overlay sourced"
else
    fail "workspace not found at /home/james/ros2_ws/install/"
fi

# ── ROS2 packages ─────────────────────────────────────────────────────────────
section "ROS2 packages"

check_pkg() {
    if ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        pass "ros2 pkg: $1"
    else
        fail "ros2 pkg: $1 (not found)"
    fi
}

check_pkg depthai_ros_driver
check_pkg depthai_bridge
check_pkg depthai_ros_msgs
check_pkg nav2_bringup
check_pkg rtabmap_ros
check_pkg moveit_ros_planning
check_pkg create_bringup
check_pkg open_manipulator_x_bringup
check_pkg open_manipulator_x_moveit_config
check_pkg dynamixel_hardware_interface
check_pkg james_bringup
check_pkg pcl_ros
check_pkg tf2_geometry_msgs
check_pkg vision_msgs

# ── Python packages ───────────────────────────────────────────────────────────
section "Python packages"

check_py() {
    local pkg=$1
    local import_name=${2:-$1}
    if python3 -c "import $import_name" 2>/dev/null; then
        VER=$(python3 -c "import $import_name; print(getattr($import_name, '__version__', 'ok'))" 2>/dev/null)
        pass "python3: $pkg ($VER)"
    else
        fail "python3: $pkg (import failed)"
    fi
}

check_py depthai
check_py open3d
check_py blobconverter
check_py ultralytics

# ── OAK-D camera detection ────────────────────────────────────────────────────
section "OAK-D Pro camera (USB)"

if python3 -c "import depthai as dai; d = dai.Device.getAllAvailableDevices(); print(len(d))" 2>/dev/null | grep -q "^[0-9]"; then
    COUNT=$(python3 -c "import depthai as dai; print(len(dai.Device.getAllAvailableDevices()))" 2>/dev/null)
    if [ "$COUNT" -gt 0 ]; then
        pass "OAK-D camera detected ($COUNT device(s))"
    else
        warn "OAK-D not detected — is the camera connected and /dev/bus/usb forwarded?"
    fi
else
    warn "Could not query OAK-D devices (depthai error)"
fi

# ── Serial devices ────────────────────────────────────────────────────────────
section "Serial devices"

if [ -e /dev/ttyUSB0 ]; then
    pass "/dev/ttyUSB0 present (Roomba)"
else
    warn "/dev/ttyUSB0 not found — Roomba not connected or device not forwarded"
fi

if [ -e /dev/ttyUSB1 ]; then
    pass "/dev/ttyUSB1 present (OpenManipulator)"
else
    warn "/dev/ttyUSB1 not found — arm not connected or device not forwarded"
fi

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
printf " Results: ${GREEN}%d passed${NC}  ${RED}%d failed${NC}  ${YELLOW}%d warnings${NC}\n" $PASS $FAIL $WARN
echo "════════════════════════════════════════"

[ "$FAIL" -eq 0 ] && exit 0 || exit 1
