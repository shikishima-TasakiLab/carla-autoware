#!/bin/bash

RUNTIME=""
DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)
if [[ $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
    RUNTIME="--gpus all"
else
    RUNTIME="--runtime=nvidia"
fi

gnome-terminal -- \
    docker run \
        -it --rm \
        --volume=$(pwd)/autoware-contents:/home/autoware/autoware-contents:ro \
        --volume=$(pwd)/ros-bridge:/home/autoware/ros-bridge:rw \
        --volume=$(pwd)/g29-wheel:/home/autoware/carla_ws/src/g29-wheel:rw \
        --volume=$(pwd)/ws/build:/home/autoware/carla_ws/build:rw \
        --volume=$(pwd)/ws/devel:/home/autoware/carla_ws/devel:rw \
        --env="DISPLAY=${DISPLAY}" \
        --privileged \
        --net=host \
        --name="carla-autoware" \
        $RUNTIME \
        carla-autoware:latest

gnome-terminal -- \
    docker run \
        -it --rm \
        -p 2000-2002:2000-2002 \
        --gpus all \
        carlasim/carla:0.9.10.1 \
        bash -c "DISPLAY= SDL_VIDEODRIVER=offscreen SDL_HINT_CUDA_DEVICE=0 ./CarlaUE4.sh -opengl"
