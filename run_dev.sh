#!/bin/bash

# Create container to run on your laptop for development, not on the jetson

docker run -it \
  --name robojames \
  --net=host \
  --pid=host \
  --platform=linux/arm64/v8 \
  robojames 
  