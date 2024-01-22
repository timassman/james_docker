ARG ROS_DISTRO=humble

# To use a docker base ROS2 image
# FROM arm64v8/ros:$ROS_DISTRO-perception-jammy

# To use the docker base image with rtabmap pre-installed (saves a lot of time)
# https://hub.docker.com/r/introlab3it/rtabmap_ros/tags
FROM introlab3it/rtabmap_ros:$ROS_DISTRO-latest

# ARGS cannot be used after a FROM, unless ARG witout value is used again
# https://docs.docker.com/engine/reference/builder/#understand-how-arg-and-from-interact
ARG ROS_DISTRO

SHELL [ "/bin/bash", "-c" ] # the source command is unknown to /bin/sh

# ********************************************************
# * Add non-root user                                    *
# ********************************************************

# https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user
ARG USERNAME=james
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

# ********************************************************
# * General setup                                        *
# ********************************************************

# create workspace
RUN mkdir -p ~/ros2_ws/src

# Add underlay to bashrc
RUN echo -e "\nsource /opt/ros/$ROS_DISTRO/setup.bash\n" >> ~/.bashrc

# Add overlay to bashrc
RUN echo -e "source ~/ros2_ws/install/local_setup.bash\n" >> ~/.bashrc

# Add user to dialout group
RUN sudo usermod -a -G dialout $(whoami)

# ********************************************************
# * Build Kinect drivers                                  *
# ********************************************************

# libfreenect needed for kinect v1
# https://github.com/fadlio/kinect_ros2/issues/5
RUN cd ~/ros2_ws/src \
    && git clone https://github.com/OpenKinect/libfreenect \
    && cd libfreenect \
    && mkdir build \
    && cd build \
    && cmake .. \
    && sudo ldconfig \
    && sudo make install

# libfreenect2 needed for kinect v2
# https://github.com/OpenKinect/libfreenect2?tab=readme-ov-file#linux
RUN sudo apt-get install libusb-1.0-0-dev -y \
    && sudo apt-get install libturbojpeg0-dev -y \
    && sudo apt-get install libglfw3-dev -y

RUN cd ~/ros2_ws/src \
    && git clone https://github.com/OpenKinect/libfreenect2.git \
    && cd libfreenect2 \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 \
    && make \
    && make install

# ********************************************************
# * RTAP-MAP                                             *
# ********************************************************

# Add RTAB-Map ROS2 package => Skipped, Already included in base image!
# From source: https://github.com/introlab/rtabmap_ros/tree/ros2?tab=readme-ov-file#from-source
# Delete rtabmap_demos folder, since it needs ros-humble-turtlebot3-gazebo
# that package is not yet built for arm64
# Building from source currently fails
# RUN cd ~/ros2_ws \
#    && git clone https://github.com/introlab/rtabmap.git src/rtabmap \
#    && git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros \
#    && rm -rf ~/ros2_ws/src/rtabmap_ros/rtabmap_demos

# From binaries: => Skipped, Already included in base image!
# Unable to locate package ros-humble-rtabmap-ros
# https://github.com/introlab/rtabmap_ros/tree/ros2?tab=readme-ov-file#binaries
# RUN apt-get update \
#     && sudo apt install ros-$ROS_DISTRO-rtabmap-ros -y

# ********************************************************
# * Navigation2                                          *
# ********************************************************

# https://navigation.ros.org/getting_started/index.html#installation
RUN sudo apt install ros-$ROS_DISTRO-navigation2 -y \
    && sudo apt install ros-$ROS_DISTRO-nav2-bringup -y

# ********************************************************
# * Roomba                                               *
# ********************************************************

# Add Roomba packages
# Forked from https://github.com/AutonomyLab/create_robot
RUN cd ~/ros2_ws/src \
    && git clone https://github.com/timassman/create_robot.git \
    && git clone https://github.com/AutonomyLab/libcreate.git \
    && git clone https://github.com/timassman/james_robot.git

# ********************************************************
# * Kinect                                               *
# ********************************************************

# Kinect1 for ROS2 package was forked from # https://github.com/fadlio/kinect_ros2
# by matlabbe to contain XYZRGB output
RUN cd ~/ros2_ws/src \
    && git clone https://github.com/matlabbe/kinect_ros2.git

# Could not find ROS2 package for Kinect2
# Therefore forked and modified the one for Kinect1
RUN cd ~/ros2_ws/src \
    && git clone https://github.com/timassman/kinect2_ros2.git

# ********************************************************
# * Robot arm                                            *
# ********************************************************

# Add Open Manipulator-X packages
# https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#install-ros-packages

RUN sudo apt install ros-$ROS_DISTRO-rqt* -y
# Building from source:
# https://docs.ros.org/en/humble/How-To-Guides/RQt-Source-Install.html
# RUN cd ~/ros2_ws \
#     && vcs import --force --input https://raw.githubusercontent.com/PickNikRobotics/rqt2_setup/master/rqt2.repos src

RUN sudo apt install ros-$ROS_DISTRO-joint-state-publisher -y
# Building from source:
# RUN cd ~/ros2_ws/src \
#     && git clone -b ros2 https://github.com/ros/joint_state_publisher.git

RUN cd ~/ros2_ws/src && git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
RUN cd ~/ros2_ws/src && git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
RUN cd ~/ros2_ws/src && git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/open_manipulator.git
RUN cd ~/ros2_ws/src && git clone -b ros2              https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
RUN cd ~/ros2_ws/src && git clone -b ros2              https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
RUN cd ~/ros2_ws/src && git clone -b ros2              https://github.com/ROBOTIS-GIT/robotis_manipulator.git

# ********************************************************
# * Build                                                *
# ********************************************************

RUN cd ~/ros2_ws \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

# export MAKEFLAGS="-j6" Can be ignored if you have a lot of RAM (>16GB)
# --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release
RUN cd ~/ros2_ws \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install \
    && source ~/ros2_ws/install/local_setup.bash

# ********************************************************
# * Install other packages                               *
# ********************************************************

RUN sudo apt install nano -y

# ********************************************************
# * Post install                                         *
# ********************************************************

# Copy script to update all repositories
COPY git_repos.sh ~/ros2_ws/src

# Somehow the base image does not have the ROS_DISTRO identifier
ENV ROS_DISTRO=${ROS_DISTRO}

# Add the overlay to the default ros_entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]