#!/bin/bash

set -e

PROGRESS=plain
VERSION=$(git rev-parse --short HEAD)
BUILD_TYPE=RelWithDebInfo

cd "$(dirname "${BASH_SOURCE[0]}")"./../

docker build --progress ${PROGRESS} \
             --build-arg TARGET=cpu \
             --build-arg BUILD_TYPE=${BUILD_TYPE} \
             --tag ghcr.io/au-agv/dyno:devel \
             --tag ghcr.io/au-agv/dyno:devel-${VERSION} \
             --tag ghcr.io/au-agv/dyno:devel-stable \
             --file docker/build/standalone/devel/Dockerfile \
             .

docker build --progress ${PROGRESS} \
             --build-arg TARGET=cpu \
             --build-arg BUILD_TYPE=${BUILD_TYPE} \
             --tag ghcr.io/au-agv/dyno \
             --tag ghcr.io/au-agv/dyno:runtime \
             --tag ghcr.io/au-agv/dyno:runtime-${VERSION} \
             --tag ghcr.io/au-agv/dyno:runtime-stable \
             --file docker/build/standalone/runtime/Dockerfile \
             .

if [ -f docker/build/standalone/devel/third_party/NVIDIA-OptiX-SDK-7.7.0-linux64-x86_64.sh ]; then
    docker build --progress ${PROGRESS} \
                --build-arg TARGET=cuda \
                --build-arg BUILD_TYPE=${BUILD_TYPE} \
                --tag ghcr.io/au-agv/dyno:devel-cuda \
                --tag ghcr.io/au-agv/dyno:devel-${VERSION}-cuda \
                --tag ghcr.io/au-agv/dyno:devel-stable-cuda \
                --file docker/build/standalone/devel/Dockerfile \
                .

    docker build --progress ${PROGRESS} \
                --build-arg TARGET=cuda \
                --build-arg BUILD_TYPE=${BUILD_TYPE} \
                --tag ghcr.io/au-agv/dyno:cuda \
                --tag ghcr.io/au-agv/dyno:runtime-cuda \
                --tag ghcr.io/au-agv/dyno:runtime-${VERSION}-cuda \
                --tag ghcr.io/au-agv/dyno:runtime-stable-cuda \
                --file docker/build/standalone/runtime/Dockerfile \
                .

    docker build --progress ${PROGRESS} \
                --build-arg BUILD_TYPE=${BUILD_TYPE} \
                --tag ghcr.io/au-agv/dyno:ros-devel \
                --tag ghcr.io/au-agv/dyno:ros-devel-${VERSION} \
                --tag ghcr.io/au-agv/dyno:ros-devel-stable \
                --file docker/build/ros/devel/Dockerfile \
                .

    docker build --progress ${PROGRESS} \
                --build-arg BUILD_TYPE=${BUILD_TYPE} \
                --tag ghcr.io/au-agv/dyno:ros \
                --tag ghcr.io/au-agv/dyno:ros-runtime \
                --tag ghcr.io/au-agv/dyno:ros-runtime-${VERSION} \
                --tag ghcr.io/au-agv/dyno:ros-runtime-stable \
                --file docker/build/ros/runtime/Dockerfile \
                .
fi