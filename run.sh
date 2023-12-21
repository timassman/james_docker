#!/bin/bash

# Roomba:          /dev/ttyUSB0

# the new joy_node uses eventX instead of jsX to access the xbox controller
# https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md#technical-note-about-interfacing-with-joysticks-and-game-controllers-on-linux

# Xbox controller: /dev/input/event9 (found by `cat /proc/bus/input/devices`)

docker run -it \
  --name robojames \
  --device=/dev/ttyUSB0 --device=/dev/input/event9 \
  --restart unless-stopped \
  robojames \
  sh -c "ros2 launch james_bringup james.launch.py"
  