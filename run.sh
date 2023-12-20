#!/bin/bash

docker run -it \
  --name robojames \
  --device /dev/ttyUSB0:/dev/input/js0 \
  robojames