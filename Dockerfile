ARG ROS_DISTRO=humble

FROM arm64v8/ros:$ROS_DISTRO-perception-jammy
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

# Add the overlay to the default ros_entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# ********************************************************
# * Build Kinect driver                                  *
# ********************************************************

# Needed for kinect_ros2
# https://github.com/fadlio/kinect_ros2/issues/5
RUN cd ~/ros2_ws/src \
    && git clone https://github.com/OpenKinect/libfreenect \
    && cd libfreenect \
    && mkdir build \
    && cd build \
    && cmake .. \
    && sudo ldconfig \
    && sudo make install

# ********************************************************
# * Add ROS packages                                     *
# ********************************************************

# Add RTAB-Map ROS2 package
# From source: https://github.com/introlab/rtabmap_ros/tree/ros2?tab=readme-ov-file#from-source
# Delete rtabmap_demos folder, since it needs ros-humble-turtlebot3-gazebo
# that package is not yet built for arm64
# Building from source currently fails
# RUN cd ~/ros2_ws \
#    && git clone https://github.com/introlab/rtabmap.git src/rtabmap \
#    && git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros \
#    && rm -rf ~/ros2_ws/src/rtabmap_ros/rtabmap_demos

# From binaries: https://github.com/introlab/rtabmap_ros/tree/ros2?tab=readme-ov-file#binaries
RUN sudo apt install ros-$ROS_DISTRO-rtabmap-ros -y

# Install navigation packages
# https://navigation.ros.org/getting_started/index.html#installation
RUN sudo apt install ros-$ROS_DISTRO-navigation2 -y \
    && sudo apt install ros-$ROS_DISTRO-nav2-bringup -y

# Add Roomba packages
# Forked from https://github.com/AutonomyLab/create_robot
RUN cd ~/ros2_ws/src \
    && git clone https://github.com/timassman/create_robot.git \
    && git clone https://github.com/AutonomyLab/libcreate.git \
    && git clone https://github.com/timassman/james_robot.git

# Kinect1 for ROS2 package was forked from # https://github.com/fadlio/kinect_ros2
# by matlabbe to contain XYZRGB output
RUN cd ~/ros2_ws/src \
    && git clone https://github.com/matlabbe/kinect_ros2.git

# ********************************************************
# * Install dependencies and build                       *
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
