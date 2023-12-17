#!/bin/bash

docker run -it \
  --name robojames \
  --device /dev/ttyUSB0 \
  robojames