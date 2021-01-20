#!/bin/bash
CARLA_VERSION="0.9.10.1"

BUILD_DIR=$(dirname $(readlink -f $0))

docker build -t carla-autoware:${CARLA_VERSION} -f Dockerfile ${BUILD_DIR}
