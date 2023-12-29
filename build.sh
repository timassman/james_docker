#!/bin/bash

# optionally add --no-cache argument

# build a new docker image "robojames" using the docker file in the current directory
docker build -t robojames \
  --progress=plain \
  .