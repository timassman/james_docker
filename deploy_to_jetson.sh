#!/bin/bash
# Transfers the robojames-jetson Docker image to the Jetson Orin Nano via SSH.
# No intermediate file needed — streams directly from laptop to Jetson.
#
# Usage:
#   ./deploy_to_jetson.sh                    # uses defaults below
#   ./deploy_to_jetson.sh james@jetson-orin  # override host

set -euo pipefail

IMAGE="robojames-jetson"
JETSON="${1:-james@jetson-orin}"

# Verify the image exists locally
if ! docker image inspect "$IMAGE" &>/dev/null; then
    echo "[ERROR] Image '$IMAGE' not found locally. Run ./build_on_laptop.sh first."
    exit 1
fi

IMAGE_SIZE=$(docker image inspect "$IMAGE" --format='{{.Size}}' | numfmt --to=iec 2>/dev/null || echo "unknown size")
echo "=== Deploying $IMAGE ($IMAGE_SIZE) to $JETSON ==="
echo ""
echo "Streaming image via SSH — no intermediate file needed."
echo "This may take several minutes depending on network speed."
echo ""

START=$SECONDS

# Install pv if not present (used for progress bar during transfer)
if ! command -v pv &>/dev/null; then
    echo "[INFO] Installing pv for progress display..."
    sudo apt-get install -y pv
fi

# Use pv for progress if available, otherwise stream directly.
# gzip compression reduces transfer size significantly (~3x for this image).
docker save "$IMAGE" \
    | pv --name "Compressing+Sending" --timer --rate --bytes \
    | gzip \
    | ssh "$JETSON" 'gunzip | docker load'

ELAPSED=$(( SECONDS - START ))
echo ""
echo "=== Transfer complete! ($(( ELAPSED / 60 ))m $(( ELAPSED % 60 ))s) ==="
echo ""
echo "Verify on Jetson:"
echo "  ssh $JETSON 'docker images robojames-jetson'"
echo ""
echo "Run on Jetson:"
echo "  ssh $JETSON 'docker run --runtime=nvidia --gpus all --rm -it robojames-jetson bash'"
