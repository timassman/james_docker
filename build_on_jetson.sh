#!/bin/bash
# Builds the Docker image natively ON the Jetson Orin Nano (no QEMU, fastest method).
# Run this script directly on the Jetson, not on the laptop.
#
# Prerequisites: the repo must be present on the Jetson (git clone or rsync from laptop).
# Optionally add --no-cache to force a full rebuild.

# Build args:
#   PURGE_NVIDIA_OPENCV=true  — the dustynv base image ships a CUDA-optimized OpenCV
#                               that conflicts at the file level with ROS2 packages
#                               (nav2, rqt) that expect the standard libopencv-dev.
#                               Purging it lets those packages install cleanly.
#                               The CUDA toolkit and all other NVIDIA libs remain intact.
#   COLCON_PARALLEL_WORKERS=4 — native ARM64 build, no QEMU involved, so parallel
#                               workers don't cause crashes. 4 is safe on 8 GB RAM.
#                               Lower to 2 if you get OOM kills during depthai_bridge
#                               or moveit compilation.
docker build \
  --build-arg BASE_IMAGE=dustynv/ros:humble-ros-base-l4t-r36.3.0 \
  --build-arg PURGE_NVIDIA_OPENCV=true \
  --build-arg COLCON_PARALLEL_WORKERS=4 \
  --tag robojames-jetson \
  --progress=plain \
  .
