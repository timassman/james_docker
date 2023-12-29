#!/bin/bash

# Roomba:          /dev/ttyUSB0

# the new joy_node uses eventX instead of jsX to access the xbox controller
# https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md#technical-note-about-interfacing-with-joysticks-and-game-controllers-on-linux

# Xbox controller: /dev/input/event9 (found by `cat /proc/bus/input/devices`)

# The Kinect shows up as a new /dev/bus/usb device, the id can change, so forwarding all

# --restart unless-stopped makes sure it starts at boot of the Jetson

# --net=host --pid=host are needed to receive topic data on another host machine
# https://answers.ros.org/question/358453/ros2-docker-multiple-hosts/

# how to get the path of the xbox-controller symlink:
# https://stackoverflow.com/questions/53853845/docker-device-works-with-absolute-device-path-fails-with-symlink

docker run -it \
  --name robojames \
  --device=/dev/ttyUSB0 --device=$(readlink -f /dev/input/xbox-controller) --device=/dev/bus/usb \
  --restart unless-stopped \
  --net=host \
  --pid=host \
  robojames \
  sh -c "ros2 launch james_bringup james.launch.py"
  