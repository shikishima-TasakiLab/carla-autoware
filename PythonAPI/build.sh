#!/bin/bash
CARLA_VERSION="0.9.10.1"

BUILD_DIR=$(dirname $(readlink -f $0))
TMP_LIST=(${CARLA_VERSION//./ })
CARLA_EGG_VERSION="${TMP_LIST[0]}.${TMP_LIST[1]}.${TMP_LIST[2]}"

docker build -t carla-pyapi:${CARLA_VERSION} --build-arg CARLA_VERSION="${CARLA_VERSION}" --build-arg CARLA_EGG_VERSION="${CARLA_EGG_VERSION}" ${BUILD_DIR}