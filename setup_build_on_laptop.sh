#!/bin/bash
# Sets up the host to build ARM64 Docker images on an x86_64 laptop.
# Safe to run multiple times — all steps check before acting.
#
# What this sets up:
#   1. Swap file (16GB)       -> prevents OOM system freeze during heavy builds
#   2. qemu-user-static v7.0  -> lets the kernel run ARM64 binaries on x86_64
#                                 pinned to v7.0.0: newer versions cause segfaults
#                                 on kernels >6.8.0-49 (see docker/buildx#3170)
#   3. Docker Buildx          -> multi-platform image builder
#
# After setup: run ./build_on_laptop.sh to build the image.

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

[[ "$(uname -s)" == "Linux" ]]    || error "This script requires Linux."
[[ "$(uname -m)" == "x86_64" ]]   || error "This script is intended for an x86_64 machine."
command -v docker &>/dev/null      || error "Docker is not installed."

info "=== ARM64 build environment setup for Jetson Orin Nano ==="

# ── Step 1: swap file ─────────────────────────────────────────────────────────
info "Step 1/4 — Swap file (prevents OOM system freeze during build)..."

SWAP_TARGET_GB=16
SWAP_TARGET_BYTES=$(( SWAP_TARGET_GB * 1024 * 1024 * 1024 ))

if [ -f /swapfile ]; then
    SWAP_ACTUAL_BYTES=$(stat -c%s /swapfile)
    if [ "$SWAP_ACTUAL_BYTES" -lt "$SWAP_TARGET_BYTES" ]; then
        warn "Swap file exists but is too small ($(( SWAP_ACTUAL_BYTES / 1024 / 1024 / 1024 ))GB). Recreating as ${SWAP_TARGET_GB}GB..."
        sudo swapoff /swapfile 2>/dev/null || true
        sudo rm /swapfile
        sudo fallocate -l "${SWAP_TARGET_GB}G" /swapfile
        sudo chmod 600 /swapfile
        sudo mkswap /swapfile
    else
        info "Swap file already exists and is large enough ($(( SWAP_ACTUAL_BYTES / 1024 / 1024 / 1024 ))GB)."
    fi
else
    info "Creating ${SWAP_TARGET_GB}GB swap file..."
    sudo fallocate -l "${SWAP_TARGET_GB}G" /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
fi

if swapon --show | grep -q /swapfile; then
    info "Swap file already active."
else
    sudo swapon /swapfile
    info "Swap file activated."
fi

if grep -q '/swapfile' /etc/fstab; then
    info "Swap file already in /etc/fstab."
else
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab > /dev/null
    info "Swap file added to /etc/fstab (survives reboot)."
fi

SWAP_TOTAL=$(free -h | awk '/^Swap:/ {print $2}')
info "Total swap available: $SWAP_TOTAL"

# ── Step 2: install QEMU ──────────────────────────────────────────────────────
info "Step 2/4 — Installing QEMU user-static and binfmt support..."
sudo apt-get update -qq
sudo apt-get install -y \
    qemu-user-static \
    binfmt-support

# ── Step 3: register binfmt handlers ──────────────────────────────────────────
info "Step 3/4 — Registering QEMU binfmt handlers (pinned to v7.0.0)..."
# Pinned to v7.0.0: kernels >6.8.0-49 have a binfmt regression causing frequent
# ARM64 segfaults with newer QEMU versions. Re-running this is safe — it overwrites
# any existing handlers.
# See: https://github.com/docker/buildx/issues/3170
docker run --rm --privileged tonistiigi/binfmt:qemu-v7.0.0 --install all
info "QEMU binfmt handlers registered."

# ── Step 4: verify ────────────────────────────────────────────────────────────
info "Step 4/4 — Verifying ARM64 support..."
docker buildx version &>/dev/null || error "Docker Buildx not found. Install docker-buildx-plugin."

if docker run --rm --platform linux/arm64 arm64v8/ubuntu:22.04 uname -m 2>/dev/null | grep -q "aarch64"; then
    info "Smoke test passed: ARM64 container runs correctly on this machine."
else
    warn "Smoke test failed — QEMU binfmt may not be working correctly."
fi

echo ""
info "=== Setup complete! ==="
info "Current kernel : $(uname -r)"
info "Swap available : $(free -h | awk '/^Swap:/ {print $2}')"
info ""
info "Next step: run './build_on_laptop.sh' to build the Jetson image."
info "The build runs with low priority (nice/ionice) so your laptop stays usable."
