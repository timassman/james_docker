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
# * Add ROS packages                                     *
# ********************************************************

# Add Roomba packages
# Forked from https://github.com/AutonomyLab/create_robot
RUN cd ~/ros2_ws/src \
    && git clone https://github.com/timassman/create_robot.git \
    && git clone https://github.com/AutonomyLab/libcreate.git \
    && git clone https://github.com/timassman/james_robot.git

# ********************************************************
# * Install dependencies and build                       *
# ********************************************************

RUN cd ~/ros2_ws \
    && rosdep update \
    && rosdep install --from-paths src -y

RUN cd ~/ros2_ws \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install \
    && source ~/ros2_ws/install/local_setup.bash

# ********************************************************
# * Install other packages                               *
# ********************************************************

RUN sudo apt install nano -y
