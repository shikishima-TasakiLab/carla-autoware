#!/bin/bash
CARLA_VERSION="0.9.10.1"

RUN_DIR=$(dirname $(readlink -f $0))

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
        -p 2000-2002:2000-2002 \
        ${RUNTIME} \
        carlasim/carla:${CARLA_VERSION} \
        bash -c "DISPLAY= SDL_VIDEODRIVER=offscreen SDL_HINT_CUDA_DEVICE=0 ./CarlaUE4.sh -opengl"

gnome-terminal -- \
    docker run \
        -it --rm \
        --volume=${RUN_DIR}/autoware-contents:/home/autoware/autoware-contents:ro \
        --volume=${RUN_DIR}/ros-bridge:/home/autoware/ros-bridge:rw \
        --volume=${RUN_DIR}/g29-wheel:/home/autoware/carla_ws/src/g29-wheel:rw \
        --volume=${RUN_DIR}/ws/build:/home/autoware/carla_ws/build:rw \
        --volume=${RUN_DIR}/ws/devel:/home/autoware/carla_ws/devel:rw \
        --volume=${RUN_DIR}/shared_dir:/home/autoware/shared_dir:rw \
        --env="DISPLAY=${DISPLAY}" \
        --privileged \
        --net=host \
        --name="carla-autoware" \
        ${RUNTIME} \
        carla-autoware:${CARLA_VERSION}
