#!/bin/bash
# Syncs the local james_docker repo to the Jetson over SSH.
# Run this after making changes on the laptop to test them on the Jetson
# without needing to commit and push first.
#
# Usage:
#   ./sync_james.sh              # sync to default host (jetson-orin)
#   ./sync_james.sh james@192.168.1.100  # override host

set -euo pipefail

JETSON="${1:-jetson-orin}"
LOCAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_DIR="~/git/james_docker"

echo "=== Syncing to $JETSON:$REMOTE_DIR ==="

rsync -av --exclude=".git" "$LOCAL_DIR/" "$JETSON:$REMOTE_DIR/"

echo ""
echo "=== Done! ==="
echo "Test on Jetson:"
echo "  ssh $JETSON"
echo "  cd $REMOTE_DIR"
