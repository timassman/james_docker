ARG ROS_DISTRO=humble

# Base image is configurable via --build-arg BASE_IMAGE:
#   Laptop (x86_64):           osrf/ros:humble-desktop  (default)
#   Jetson Orin Nano (ARM64):  dustynv/ros:humble-ros-base-l4t-r36.3.0  (CUDA-enabled)
# Laptop tags: https://hub.docker.com/r/osrf/ros/tags
# Jetson tags: https://hub.docker.com/r/dustynv/ros/tags?name=humble-ros-base-l4t
ARG BASE_IMAGE=osrf/ros:humble-desktop
FROM ${BASE_IMAGE}

# ARG must be re-declared after FROM to remain in scope
ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# ── All privileged operations run as root ────────────────────────────────────
# sudo is blocked in Docker Buildx's sandbox (nosuid mount), so all system-level
# operations run as root here. Only workspace and build steps run as the user.

# ********************************************************
# * Fix OpenCV conflict with CUDA base image             *
# ********************************************************

# The dustynv base ships CUDA-optimized OpenCV which conflicts at the file
# level with standard ROS2 packages (nav2, rqt) that require libopencv-dev.
# Purging it lets those packages install cleanly. The CUDA toolkit and all
# other NVIDIA libraries remain fully intact after this purge.
# Pass --build-arg PURGE_NVIDIA_OPENCV=true when using the dustynv base image.
ARG PURGE_NVIDIA_OPENCV=false
RUN if [ "$PURGE_NVIDIA_OPENCV" = "true" ]; then \
        apt-get purge -y '*opencv*' && apt-get autoremove -y; \
    fi

# ********************************************************
# * Fix ROS2 GPG key (may be stale in some base images)  *
# ********************************************************

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /tmp/ros.asc \
    && gpg --batch --no-tty --dearmor < /tmp/ros.asc > /usr/share/keyrings/ros-archive-keyring.gpg \
    && rm /tmp/ros.asc \
    && rm -f /etc/apt/sources.list.d/ros2.list \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# ********************************************************
# * Non-root user                                        *
# ********************************************************

ARG USERNAME=james
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -a -G dialout,video,plugdev $USERNAME

# ********************************************************
# * System dependencies                                  *
# ********************************************************

RUN apt-get update && apt-get install -y \
    curl \
    wget \
    cmake \
    git \
    nano \
    python3-pip \
    libusb-1.0-0-dev

# The dustynv base image ships a minimal ROS2 CLI that is missing these extensions.
# Without them, `ros2 topic`, `ros2 action` and `ros2 interface` are unavailable.
RUN apt-get install -y \
    ros-$ROS_DISTRO-ros2topic \
    ros-$ROS_DISTRO-ros2action \
    ros-$ROS_DISTRO-ros2interface

# ********************************************************
# * Roomba (create_robot)                                *
# ********************************************************

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-create-robot \
    ros-$ROS_DISTRO-create-bringup

# ********************************************************
# * Luxonis OAK-D Pro (DepthAI)                         *
# ********************************************************

# udev rule for USB access without root (Luxonis vendor ID: 03e7)
RUN mkdir -p /etc/udev/rules.d \
    && echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
    > /etc/udev/rules.d/80-movidius.rules

# Neural network workflow for OAK-D Pro bottle detection:
#
# 1. The OAK-D Pro runs models in .blob format (compiled for MyriadX VPU via OpenVINO).
#    Beer bottles are class "bottle" (class 39) in COCO — any COCO-trained YOLO works.
#
# 2. Get a pre-compiled YOLO blob from Luxonis model zoo:
#    https://models.luxonis.com/  (search: yolov6, yolov8, filter: object detection)
#    Or convert your own: https://tools.luxonis.com/
#
# 3. The depthai_ros_driver loads the model via a YAML config.
#    SpatialDetectionNetwork combines YOLO + stereo depth → gives (x,y,z) in meters directly.
#    No point cloud processing needed for basic bottle localization.
#
# 4. For future Jetson GPU inference (TensorRT):
#    The dustynv base already has TensorRT. Install ultralytics + export to TensorRT engine:
#    pip3 install ultralytics
#    yolo export model=yolov8n.pt format=engine device=0
#
# Use explicit PyPI index to bypass Jetson-specific pip index in dustynv base image
RUN pip3 install \
    depthai \
    blobconverter \
    ultralytics \
    --index-url https://pypi.org/simple/ \
    --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/ \
    --timeout 300 \
    --retries 5 \
    --no-cache-dir

# ********************************************************
# * Point Cloud Library (PCL)                            *
# ********************************************************

# ros-humble-perception-pcl: standard ROS2 PCL integration, CPU-based but well-optimized.
# Best documented and most widely used point cloud library in ROS2.
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-perception-pcl \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions

# Open3D: modern point cloud library with CUDA Tensor API, excellent documentation.
# CPU wheel installed here. For CUDA-accelerated Open3D on Jetson, build as overlay:
#
#   FROM robojames-jetson AS base
#   RUN apt-get install -y libeigen3-dev libgl1-mesa-dev libglib2.0-dev
#   RUN git clone --depth 1 --branch v0.18.0 https://github.com/isl-org/Open3D.git /tmp/open3d \
#       && cd /tmp/open3d && mkdir build && cd build \
#       && cmake .. -DBUILD_CUDA_MODULE=ON -DCMAKE_BUILD_TYPE=Release \
#                   -DBUILD_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF \
#       && make -j$(nproc) && make install && rm -rf /tmp/open3d
#
# Alternative for Jetson-native CUDA PCL: https://github.com/NVIDIA-AI-IOT/CuPCL
# (Jetson-specific, limited maintenance since 2022, source build only)
RUN pip3 install open3d \
    --index-url https://pypi.org/simple/ \
    --ignore-installed blinker

# ********************************************************
# * RTAB-Map                                             *
# ********************************************************

# Installed via apt: no CUDA support, but sufficient for RGB-D SLAM with OAK-D Pro.
#
# To enable CUDA-accelerated rtabmap in the future, build from source as an
# overlay image on top of this one:
#
#   FROM robojames-jetson AS base
#   RUN apt-get install -y libsqlite3-dev libpcl-dev
#   RUN git clone https://github.com/introlab/rtabmap.git /tmp/rtabmap \
#       && cd /tmp/rtabmap && mkdir build && cd build \
#       && cmake .. -DWITH_CUDA=ON -DCMAKE_BUILD_TYPE=Release \
#       && make -j$(nproc) && make install && rm -rf /tmp/rtabmap
#   RUN git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git ~/ros2_ws/src/rtabmap_ros \
#       && cd ~/ros2_ws && colcon build --symlink-install --packages-select rtabmap_ros
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rtabmap-ros

# ********************************************************
# * Manipulation & Perception                            *
# ********************************************************

# tf2-geometry-msgs: transform detections from camera frame to robot base frame
# vision-msgs: standard ROS2 message types for 2D/3D detections (used by depthai-ros spatial detections)
# moveit-visual-tools: visualize motion plans and grasp poses in RViz
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-moveit-visual-tools

# ********************************************************
# * Navigation2                                          *
# ********************************************************

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup

# ********************************************************
# * Robot arm (OpenManipulator-X)                        *
# ********************************************************

# apt dependencies per https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/
# ros-humble-gazebo-ros2-control omitted (simulation, not needed on Jetson)
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rqt* \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-position-controllers \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-gripper-controllers \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-moveit* \
    ros-$ROS_DISTRO-xacro

# ── Switch to non-root user for workspace and build steps ───────────────────

USER $USERNAME

# ********************************************************
# * General setup                                        *
# ********************************************************

RUN mkdir -p ~/ros2_ws/src

RUN echo -e "\nsource /opt/ros/$ROS_DISTRO/setup.bash\n" >> ~/.bashrc
RUN echo -e "source ~/ros2_ws/install/local_setup.bash\n" >> ~/.bashrc

# ********************************************************
# * Source packages                                      *
# ********************************************************

# depthai-ros: official ROS2 driver for OAK cameras
#
# Using v2.12.2-humble (not v3.x) because:
#   The dustynv base image ships libdepthai 2.x at /opt/ros/humble/include/depthai/.
#   depthai-ros v3.x requires libdepthai 3.x which added Platform.hpp and
#   PipelineAutoCalibrationMode — both missing from this base image.
#   v2.x supports all OAK-D Pro features needed for this project:
#   RGB + stereo depth + IMU + SpatialDetectionNetwork (neural network on VPU).
#   This has no impact on CUDA — the OAK-D uses its own MyriadX VPU for inference,
#   independent of the Jetson GPU.
#   To upgrade to v3.x later, libdepthai 3.x must be built from source first.
RUN cd ~/ros2_ws/src \
    && git clone --branch v2.12.2-humble --depth 1 \
       https://github.com/luxonis/depthai-ros.git

RUN cd ~/ros2_ws/src \
    && git clone https://github.com/timassman/james_robot.git

# Robot arm (OpenManipulator-X) — source packages per ROBOTIS docs
# https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/
RUN cd ~/ros2_ws/src \
    && git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
    && git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/open_manipulator.git \
    && git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git \
    && git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git

# ********************************************************
# * rosdep (runs as root to install system dependencies) *
# ********************************************************

USER root
RUN cd /home/$USERNAME/ros2_ws \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
       --skip-keys "gazebo_ros gazebo_ros_pkgs gazebo-ros"

# ********************************************************
# * Build                                                *
# ********************************************************

USER $USERNAME

# Number of parallel colcon workers during the build.
#
# Default is 1 because QEMU ARM64 emulation (used when cross-compiling on a
# laptop) is unstable under parallel load: multiple concurrent rosidl code
# generators frequently cause SIGILL / SIGSEGV crashes inside QEMU.
# Setting this to 1 serialises the build and makes retries reliable.
#
# When building natively on the Jetson (build_on_jetson.sh), there is no QEMU
# involved and crashes don't happen. Pass a higher value to speed up the build:
#   --build-arg COLCON_PARALLEL_WORKERS=4
#
# Rule of thumb for the Jetson Orin Nano (8 GB RAM):
#   4 workers   — fast, stays within RAM budget
#   8 workers   — may cause OOM on heavy C++ packages (depthai_bridge, moveit)
ARG COLCON_PARALLEL_WORKERS=1

# Each rosidl-heavy package gets its own RUN layer with retry logic.
# Docker caches successful layers, so a re-run of build_on_laptop.sh skips
# already-built packages and only retries failed ones — even across separate invocations.

# open_manipulator_x_gui is permanently skipped: Qt5 GUI fails under QEMU ARM64
# and is useless on a headless Jetson.

# Step 1: depthai_ros_msgs — consistently segfaults in rosidl type support generation
RUN cd ~/ros2_ws \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && for i in 1 2 3 4 5 6 7 8; do \
        colcon build --packages-select depthai_ros_msgs \
            --parallel-workers ${COLCON_PARALLEL_WORKERS} \
            --cmake-args -DBUILD_TESTING=OFF && break; \
        [ "$i" = "8" ] && echo "depthai_ros_msgs failed after 8 attempts" && exit 1; \
        echo "depthai_ros_msgs attempt $i failed, retrying..."; \
    done

# Step 2: dynamixel_interfaces — consistently segfaults in rosidl_typesupport_cpp
RUN cd ~/ros2_ws \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && for i in 1 2 3 4 5 6 7 8; do \
        colcon build --packages-select dynamixel_interfaces \
            --parallel-workers ${COLCON_PARALLEL_WORKERS} \
            --cmake-args -DBUILD_TESTING=OFF && break; \
        [ "$i" = "8" ] && echo "dynamixel_interfaces failed after 8 attempts" && exit 1; \
        echo "dynamixel_interfaces attempt $i failed, retrying..."; \
    done

# Step 3: fast packages with no complex workspace deps
RUN cd ~/ros2_ws && source /opt/ros/$ROS_DISTRO/setup.bash \
    && for i in 1 2 3 4 5 6 7 8; do \
        colcon build --packages-select depthai_descriptions dynamixel_sdk \
            --parallel-workers ${COLCON_PARALLEL_WORKERS} \
            --cmake-args -DBUILD_TESTING=OFF && break; \
        [ "$i" = "8" ] && exit 1; echo "retry $i..."; \
    done

# Step 4: depthai_bridge — heavyweight C++ bridge, own layer for caching
RUN cd ~/ros2_ws && source /opt/ros/$ROS_DISTRO/setup.bash \
    && for i in 1 2 3 4 5 6 7 8; do \
        colcon build --packages-select depthai_bridge \
            --parallel-workers ${COLCON_PARALLEL_WORKERS} \
            --cmake-args -DBUILD_TESTING=OFF && break; \
        [ "$i" = "8" ] && exit 1; echo "retry $i..."; \
    done

# Step 5: depthai_examples + depthai_ros_driver
# In v2.x, depthai_ros_driver has a dependency on depthai_examples (unlike v3).
RUN cd ~/ros2_ws && source /opt/ros/$ROS_DISTRO/setup.bash \
    && for i in 1 2 3 4 5 6 7 8; do \
        colcon build --packages-select depthai_examples depthai_ros_driver \
            --parallel-workers ${COLCON_PARALLEL_WORKERS} \
            --cmake-args -DBUILD_TESTING=OFF && break; \
        [ "$i" = "8" ] && exit 1; echo "retry $i..."; \
    done

# Step 6: dynamixel custom interfaces and hardware interface (depend on dynamixel_interfaces)
RUN cd ~/ros2_ws && source /opt/ros/$ROS_DISTRO/setup.bash \
    && for i in 1 2 3 4 5 6 7 8; do \
        colcon build --packages-select \
                dynamixel_sdk_custom_interfaces \
                dynamixel_hardware_interface \
            --parallel-workers ${COLCON_PARALLEL_WORKERS} \
            --cmake-args -DBUILD_TESTING=OFF && break; \
        [ "$i" = "8" ] && exit 1; echo "retry $i..."; \
    done

# Step 7: OpenManipulator description (base for all open_manipulator packages)
RUN cd ~/ros2_ws && source /opt/ros/$ROS_DISTRO/setup.bash \
    && for i in 1 2 3 4 5 6 7 8; do \
        colcon build --packages-select open_manipulator_x_description \
            --parallel-workers ${COLCON_PARALLEL_WORKERS} \
            --cmake-args -DBUILD_TESTING=OFF && break; \
        [ "$i" = "8" ] && exit 1; echo "retry $i..."; \
    done

# Step 8: OpenManipulator control and planning packages (depend on x_description)
RUN cd ~/ros2_ws && source /opt/ros/$ROS_DISTRO/setup.bash \
    && for i in 1 2 3 4 5 6 7 8; do \
        colcon build --packages-select \
                open_manipulator_x_bringup \
                open_manipulator_x_moveit_config \
                open_manipulator_x_playground \
                open_manipulator_x_teleop \
            --parallel-workers ${COLCON_PARALLEL_WORKERS} \
            --cmake-args -DBUILD_TESTING=OFF && break; \
        [ "$i" = "8" ] && exit 1; echo "retry $i..."; \
    done

# Step 9: everything remaining (james_robot, dynamixel_sdk_examples, etc.)
# open_manipulator_x_gui skipped: Qt5 fails under QEMU ARM64, useless on headless Jetson
# depthai_examples_v3 skipped: example programs, not needed on robot
RUN cd ~/ros2_ws && source /opt/ros/$ROS_DISTRO/setup.bash \
    && for i in 1 2 3 4 5 6 7 8; do \
        colcon build \
            --parallel-workers ${COLCON_PARALLEL_WORKERS} \
            --cmake-args -DBUILD_TESTING=OFF \
            --packages-skip open_manipulator_x_gui open_manipulator && break; \
        [ "$i" = "8" ] && exit 1; echo "retry $i..."; \
    done \
    && source ~/ros2_ws/install/local_setup.bash

# ********************************************************
# * Post install                                         *
# ********************************************************

COPY git_repos.sh /home/$USERNAME/ros2_ws/src/

ENV ROS_DISTRO=${ROS_DISTRO}

COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
