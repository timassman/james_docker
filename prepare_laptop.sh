#!/bin/bash

ROS_DISTRO=humble

# ********************************************************
# * Open Manipulator-X - From source                     *
# ********************************************************

# Add Open Manipulator-X packages
# https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#install-ros-packages

#sudo apt install ros-$ROS_DISTRO-rqt* -y
# Building from source:
# https://docs.ros.org/en/humble/How-To-Guides/RQt-Source-Install.html
# RUN cd ~/ros2_ws \
#     && vcs import --force --input https://raw.githubusercontent.com/PickNikRobotics/rqt2_setup/master/rqt2.repos src

#sudo apt install ros-$ROS_DISTRO-joint-state-publisher -y
# Building from source:
# RUN cd ~/ros2_ws/src \
#     && git clone -b ros2 https://github.com/ros/joint_state_publisher.git

#cd ~/ros2_ws/src
#git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
#git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
#git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/open_manipulator.git
#git clone -b ros2              https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
#git clone -b ros2              https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
#git clone -b ros2              https://github.com/ROBOTIS-GIT/robotis_manipulator.git

# ********************************************************
# * Open Manipulator-X - Binaries                        *
# ********************************************************

# https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#software-setup

sudo apt install ros-$ROS_DISTRO-dynamixel-sdk ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-gripper-controllers ros-$ROS_DISTRO-moveit

# ********************************************************
# * Open Manipulator-X - Turtlebot3                      *
# ********************************************************

cd ~/ros2_ws/src
git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git

# ********************************************************
# * Build                                                *
# ********************************************************

cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source ~/ros2_ws/install/local_setup.bash