#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/james/ros2_ws/install/local_setup.bash"

exec "$@"