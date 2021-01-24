#!/bin/bash
CARLA_VERSION="0.9.10.1"
G29_JS="/dev/input/js-g29"
G29_EVENT="/dev/input/event-g29"

RUN_DIR=$(dirname $(readlink -f $0))

RUNTIME=""
DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)
if [[ $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
    RUNTIME="--gpus all"
else
    RUNTIME="--runtime=nvidia"
fi

DOCKER_VOLUME="${DOCKER_VOLUME} --volume=${RUN_DIR}/autoware-contents:/home/autoware/autoware-contents:ro"
DOCKER_VOLUME="${DOCKER_VOLUME} --volume=${RUN_DIR}/ros-bridge:/home/autoware/carla_ws/src/ros-bridge:rw"
DOCKER_VOLUME="${DOCKER_VOLUME} --volume=${RUN_DIR}/g29-wheel:/home/autoware/carla_ws/src/g29-wheel:rw"
DOCKER_VOLUME="${DOCKER_VOLUME} --volume=${RUN_DIR}/ws/build:/home/autoware/carla_ws/build:rw"
DOCKER_VOLUME="${DOCKER_VOLUME} --volume=${RUN_DIR}/ws/devel:/home/autoware/carla_ws/devel:rw"
DOCKER_VOLUME="${DOCKER_VOLUME} --volume=${RUN_DIR}/shared_dir:/home/autoware/shared_dir:rw"

if [[ -c ${G29_EVENT} ]]; then
    DOCKER_VOLUME="${DOCKER_VOLUME} --volume=${G29_EVENT}:${G29_EVENT}:rw"
fi
if [[ -c ${G29_JS} ]]; then
    DOCKER_VOLUME="${DOCKER_VOLUME} --volume=${G29_JS}:${G29_JS}:rw"
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
        ${DOCKER_VOLUME} \
        --env="DISPLAY=${DISPLAY}" \
        --privileged \
        --net=host \
        --name="carla-autoware" \
        ${RUNTIME} \
        carla-autoware:${CARLA_VERSION}
