#!/bin/bash

RUNTIME=""
DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)
if [[ $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
    RUNTIME="--gpus all"
else
    RUNTIME="--runtime=nvidia"
fi

docker run \
    -it --rm \
    --volume=$(pwd)/autoware-contents:/home/autoware/autoware-contents:ro \
    --volume=$(pwd)/ros-bridge:/home/autoware/ros-bridge:rw \
    --volume=$(pwd)/g29-wheel:/home/autoware/carla_ws/src/g29-wheel:rw \
    --env="DISPLAY=${DISPLAY}" \
    --privileged \
    --net=host \
    $RUNTIME \
    carla-autoware:latest

