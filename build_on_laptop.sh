#!/bin/bash
# Builds the Jetson Orin Nano Docker image (linux/arm64) using Docker Buildx.
# Run setup_arm64_build.sh first if you haven't already.
#
# Recommendations for future improvements (when you have more time / build from source):
#
# - depthai-ros v3 (currently using v2.12.2):
#     v3 requires libdepthai 3.x which is not in the dustynv base image.
#     To use v3: add a Dockerfile step to build libdepthai 3.x from source first,
#     then change the git clone tag to v3.2.1-humble. Adds ~1-2h to build time.
#     See Dockerfile comment in the depthai-ros section.
#
# - rtabmap with CUDA (currently using apt, no CUDA):
#     The apt version works but does not use the Jetson GPU for feature extraction.
#     To enable: build rtabmap from source as an overlay image with -DWITH_CUDA=ON.
#     See the commented overlay instructions in the Dockerfile RTAB-Map section.
#
# - Open3D with CUDA (currently using CPU pip wheel):
#     The pip wheel is CPU-only. For GPU-accelerated point cloud processing:
#     build Open3D from source with -DBUILD_CUDA_MODULE=ON.
#     See the commented overlay instructions in the Dockerfile PCL section.
#
# - open_manipulator_x_gui (currently skipped):
#     Qt5 fails to compile under QEMU ARM64. Options:
#     a) Build directly on the Jetson (no QEMU), or
#     b) Set up proper cross-compilation with Qt5 sysroot.
#
# - Build on the Jetson itself (build_on_jetson.sh):
#     Eliminates all QEMU overhead and segfaults entirely.
#     Much faster and more stable, but risks OOM — add swap first (setup_jetson.sh).
#     The Jetson also stays tied up during the build.

set -euo pipefail

IMAGE_NAME="robojames-jetson"
PLATFORM="linux/arm64"
BASE_IMAGE="dustynv/ros:humble-ros-base-l4t-r36.3.0"

BUILD_START=$SECONDS

echo "=== Building Jetson Orin Nano Docker image ==="
echo "Platform  : $PLATFORM"
echo "Image     : $IMAGE_NAME"
echo "Base image: $BASE_IMAGE"
echo ""

# Run with low CPU/IO priority so the system stays responsive during the build.
# nice -n 10  : lower CPU priority (0=normal, 19=lowest)
# ionice -c 2 : best-effort IO class
# -n 7        : lowest priority within that class
# optionally add --no-cache to force a full rebuild
#
# Build args:
#   PURGE_NVIDIA_OPENCV=true  — the dustynv base image ships a CUDA-optimized OpenCV
#                               that conflicts at the file level with ROS2 packages
#                               (nav2, rqt) that expect the standard libopencv-dev.
#                               Purging it lets those packages install cleanly.
#                               The CUDA toolkit and all other NVIDIA libs remain intact.
#   COLCON_PARALLEL_WORKERS=1 — QEMU ARM64 emulation crashes under parallel load
#                               (multiple rosidl generators cause SIGILL/SIGSEGV).
#                               Set to 1 to serialise the build. Increase only if you
#                               have confirmed the build is stable on your machine.
nice -n 10 ionice -c 2 -n 7 \
docker buildx build \
    --platform "$PLATFORM" \
    --build-arg BASE_IMAGE="$BASE_IMAGE" \
    --build-arg PURGE_NVIDIA_OPENCV=true \
    --shm-size=2g \
    --tag "$IMAGE_NAME" \
    --load \
    --progress=plain \
    .

ELAPSED=$(( SECONDS - BUILD_START ))
echo ""
echo "=== Build complete! ($(( ELAPSED / 60 ))m $(( ELAPSED % 60 ))s) ==="
echo ""
echo "Test the image on your laptop (no Jetson needed):"
echo "  Interactively:          docker run --platform linux/arm64 --rm -it $IMAGE_NAME bash"
echo "  Check ROS2:             docker run --platform linux/arm64 --rm $IMAGE_NAME ros2 topic list"
echo "  Check depthai install:  docker run --platform linux/arm64 --rm $IMAGE_NAME python3 -c 'import depthai; print(depthai.__version__)'"
echo ""
echo "Note: running an ARM64 image on x86_64 uses QEMU — slow but works for smoke tests."
echo "GPU and OAK-D camera hardware can only be tested on the physical Jetson."
