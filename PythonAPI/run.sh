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

docker run \
    -it --rm \
    --env="DISPLAY=${DISPLAY}" \
    --privileged \
    --net=host \
    --name="carla-pyapi" \
    ${RUNTIME} \
    carla-pyapi:${CARLA_VERSION}
