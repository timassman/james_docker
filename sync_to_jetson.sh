#!/bin/bash
# Syncs a local repo from the laptop to ~/git/<repo> on the Jetson over SSH.
# Run this after making changes on the laptop to test them on the Jetson
# without needing to commit and push first.
#
# ~/git on the Jetson is mounted into the container (see run.sh), so the synced
# code is immediately visible there and can be built with colcon.
#
# Expects the repos side by side on the laptop, e.g.:
#   ~/Git-Workspace/james_docker      (this script)
#   ~/Git-Workspace/james_robot
#   ~/Git-Workspace/james_perception
#
# Usage:
#   ./sync_to_jetson.sh james_robot                        # sync to default host (jetson-orin)
#   ./sync_to_jetson.sh james_robot james@192.168.1.100    # override host
#
# Note: files deleted on the laptop are not deleted on the Jetson.

set -euo pipefail

REPO="${1:?Usage: $0 <repo> [host]   e.g. $0 james_robot}"
JETSON="${2:-jetson-orin}"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_DIR="$WORKSPACE_DIR/$REPO"
REMOTE_DIR="~/git/$REPO"

if [ ! -d "$LOCAL_DIR" ]; then
    echo "[ERROR] Repo not found: $LOCAL_DIR"
    exit 1
fi

# Simulation repos depend on Gazebo, which is not in the Jetson image
if [[ "$REPO" == *simulations* ]]; then
    echo "[ERROR] $REPO is laptop only (Gazebo is not installed on the Jetson)"
    exit 1
fi

echo "=== Syncing $LOCAL_DIR to $JETSON:$REMOTE_DIR ==="

rsync -av \
    --exclude=".git" \
    --exclude="build/" \
    --exclude="install/" \
    --exclude="log/" \
    "$LOCAL_DIR/" "$JETSON:$REMOTE_DIR/"

echo ""
echo "=== Done! Build on Jetson: ==="
echo "  docker exec -it robojames bash"
echo "  cd ~/git/$REPO"
echo "  colcon build --symlink-install --packages-select <package>"
echo "  source install/local_setup.bash"
